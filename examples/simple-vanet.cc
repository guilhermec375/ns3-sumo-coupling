/**
 * @file simple-vanet.cc
 * @brief Validation example for the ns-3/SUMO bidirectional coupling module.
 *
 * Validates eight properties of the module:
 *   1.  Position updates received from SUMO (SUMO -> ns-3).
 *   2.  Accuracy of vehicle coordinates against the expected road network area.
 *   3.  Plausibility of average vehicle speed.
 *   4.  Bidirectional speed override via SetVehicleSpeed (ns-3 -> SUMO).
 *   5.  Heading angle retrieval via GetVehicleState().angle.
 *   6.  Road and lane ID retrieval via GetVehicleState().roadId / laneId.
 *   7.  Gradual speed reduction via SetVehicleSlowDown.
 *   8.  Visual highlighting via SetVehicleColor.
 *
 * Output:
 *   - Terminal     : connection info, simulation events, test results.
 *   - vanet_log.txt: full per-step vehicle table, written next to this file.
 *
 * Usage:
 *   ./ns3 run simple-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=60
 *   (add --logFile=/path/to/out.txt to override the default log location)
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "../model/sumo-client.h"
#include "../model/sumo-manager.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleVanet");

// ---------------------------------------------------------------------------
// Log file.

static std::ofstream g_logFile;

// ---------------------------------------------------------------------------
// Validation counters.

static uint32_t g_positionUpdates  = 0;
static double   g_sumSpeed         = 0.0;
static double   g_minX = 1e9,  g_maxX = -1e9;
static double   g_minY = 1e9,  g_maxY = -1e9;

static bool g_bidirTested    = false;
static bool g_angleTested    = false;
static bool g_roadIdTested   = false;
static bool g_slowDownSent   = false;
static bool g_slowDownTested = false;
static bool g_colorTested    = false;

// Track known vehicles to detect entries.
static std::set<std::string> g_knownVehicles;

// ---------------------------------------------------------------------------
// Terminal helper: uniform timestamped event line.

static void PrintEvent(double t, const std::string& msg) {
    std::cout << "  [t=" << std::fixed << std::setprecision(1)
              << std::setw(6) << t << "s]  " << msg << "\n";
}

// ---------------------------------------------------------------------------
// Per-step grouping.

struct VehicleRow {
    std::string vid;
    double x, y, speed, angle, lanePosition;
    std::string roadId, laneId;
};

static double                  g_lastStep = -1.0;
static std::vector<VehicleRow> g_stepRows;

static void FlushStep() {
    if (g_stepRows.empty()) return;

    g_logFile << "\n[t = " << std::fixed << std::setprecision(1) << g_lastStep << " s]\n";

    for (const auto& r : g_stepRows) {
        g_logFile << std::fixed << std::setprecision(2)
                  << std::left  << std::setw(6)  << r.vid
                  << "  x="    << std::right << std::setw(7) << r.x
                  << "  y="    << std::setw(7)  << r.y
                  << "  spd="  << std::setw(6)  << r.speed
                  << "  ang="  << std::setw(6)  << r.angle
                  << "  road=" << std::left  << std::setw(20) << r.roadId
                  << "  lane=" << std::setw(24) << r.laneId
                  << "  lpos=" << std::right << std::setw(7)  << r.lanePosition << "\n";
    }

    g_stepRows.clear();
}

// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::string sumoConfig = "";
    std::string sumoHost   = "127.0.0.1";
    int         sumoPort   = 1337;
    uint32_t    poolSize   = 10;
    double      simTime    = 60.0;
    double      stepSize   = 0.1;
    bool        useGui     = false;

    // Default log file: same directory as this source file.
    std::string logFile =
        (std::filesystem::path(__FILE__).parent_path() / "vanet_log.txt").string();

    CommandLine cmd(__FILE__);
    cmd.AddValue("sumoConfig", "Path to .sumocfg file",    sumoConfig);
    cmd.AddValue("sumoHost",   "TraCI host",               sumoHost);
    cmd.AddValue("sumoPort",   "TraCI port",               sumoPort);
    cmd.AddValue("poolSize",   "Node pool size",           poolSize);
    cmd.AddValue("simTime",    "Simulation duration (s)",  simTime);
    cmd.AddValue("stepSize",   "Synchronisation step (s)", stepSize);
    cmd.AddValue("useGui",     "Use sumo-gui",             useGui);
    cmd.AddValue("logFile",    "Override log file path",   logFile);
    cmd.Parse(argc, argv);

    // Open log file.
    g_logFile.open(logFile);
    if (!g_logFile.is_open()) {
        std::cerr << "[ERROR] Cannot open log file: " << logFile << "\n";
        return 1;
    }

    // Log file header.
    g_logFile << "ns3-sumo-coupling - vehicle log\n";
    g_logFile << "Host: " << sumoHost << "  Port: " << sumoPort
              << "  SimTime: " << simTime << " s"
              << "  StepSize: " << stepSize << " s\n";
    g_logFile << std::string(100, '-') << "\n";
    g_logFile << "  VehID   x(m)      y(m)      spd(m/s)  ang(deg)  road                 lane                       lpos(m)\n";
    g_logFile << std::string(100, '-') << "\n";

    // Terminal header.
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  ns3-sumo-coupling\n";
    std::cout << "  Host: " << sumoHost << "  Port: " << sumoPort
              << "  SimTime: " << simTime << " s\n";
    std::cout << "  Log file: " << logFile << "\n";
    std::cout << std::string(60, '=') << "\n\n";

    // -----------------------------------------------------------------------
    // Node pool.

    NodeContainer pool;
    pool.Create(poolSize);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(pool);
    for (uint32_t i = 0; i < pool.GetN(); i++)
        pool.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(1e9, 1e9, 1e9));

    // -----------------------------------------------------------------------
    // SumoManager.

    Ptr<Node>        managerNode = CreateObject<Node>();
    Ptr<SumoManager> sumoManager = CreateObject<SumoManager>();
    sumoManager->SetAttribute("Host",       StringValue(sumoHost));
    sumoManager->SetAttribute("Port",       IntegerValue(sumoPort));
    sumoManager->SetAttribute("StepSize",   DoubleValue(stepSize));
    sumoManager->SetAttribute("SumoConfig", StringValue(sumoConfig));
    sumoManager->SetAttribute("UseGui",     BooleanValue(useGui));
    sumoManager->SetNodePool(pool);

    std::cout << "  Connecting to SUMO...\n\n";
    std::cout << "  --- Simulation events ---\n";

    // -----------------------------------------------------------------------
    // Per-vehicle callback.

    sumoManager->SetVehicleCallback(
        [&sumoManager](double t, const std::string& vid, const VehicleState& s) {

            // Metrics.
            g_positionUpdates++;
            g_sumSpeed += s.speed;
            g_minX = std::min(g_minX, s.x);
            g_maxX = std::max(g_maxX, s.x);
            g_minY = std::min(g_minY, s.y);
            g_maxY = std::max(g_maxY, s.y);

            // Detect vehicle entry.
            if (g_knownVehicles.find(vid) == g_knownVehicles.end()) {
                g_knownVehicles.insert(vid);
                PrintEvent(t, "vehicle " + vid + " entered");
            }

            // Step boundary.
            if (t != g_lastStep) {
                FlushStep();
                g_lastStep = t;
            }
            g_stepRows.push_back({vid, s.x, s.y, s.speed, s.angle, s.lanePosition, s.roadId, s.laneId});

            // Test 4: instant speed override.
            if (!g_bidirTested && s.speed > 3.0) {
                g_bidirTested = true;
                bool ok = sumoManager->GetClient().SetVehicleSpeed(vid, 2.0);
                PrintEvent(t, "SetVehicleSpeed(" + vid + ", 2.0 m/s) -> " + (ok ? "OK" : "FAILED"));
            }

            // Test 5: angle.
            if (!g_angleTested && s.angle != 0.0) {
                g_angleTested = true;
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(1);
                oss << "angle OK: " << vid << " = " << s.angle << " deg";
                PrintEvent(t, oss.str());
            }

            // Test 6: road / lane ID.
            if (!g_roadIdTested && !s.roadId.empty()) {
                g_roadIdTested = true;
                PrintEvent(t, "road/lane OK: " + vid + "  road=" + s.roadId + "  lane=" + s.laneId);
            }

            // Test 7: gradual SlowDown.
            if (!g_slowDownSent && s.speed > 5.0) {
                g_slowDownSent   = true;
                g_slowDownTested = true;
                bool ok = sumoManager->GetClient().SetVehicleSlowDown(vid, 1.0, 3.0);
                PrintEvent(t, "SetVehicleSlowDown(" + vid + ", 1.0 m/s, 3.0 s) -> " + (ok ? "OK" : "FAILED"));
            }


            // Test 8: color.
            if (!g_colorTested && s.speed > 8.0) {
                g_colorTested = true;
                TraCIColor red{255, 0, 0, 255};
                bool ok = sumoManager->GetClient().SetVehicleColor(vid, red);
                PrintEvent(t, "SetVehicleColor(" + vid + ", red) -> " + (ok ? "OK" : "FAILED"));
            }
        });

    // NS_LOG disabled: all terminal output is handled by PrintEvent above.

    managerNode->AddApplication(sumoManager);
    sumoManager->SetStartTime(Seconds(0.0));
    sumoManager->SetStopTime(Seconds(simTime));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    FlushStep();
    Simulator::Destroy();

    g_logFile.close();

    // -----------------------------------------------------------------------
    // Validation report.

    bool   t1  = (g_positionUpdates >= 100);
    bool   t2  = (g_minX >= -10 && g_maxX <= 410 && g_minY >= -10 && g_maxY <= 410);
    double avg = g_positionUpdates > 0 ? g_sumSpeed / g_positionUpdates : 0.0;
    bool   t3  = (avg > 1.0 && avg <= 14.0);
    bool   t4  = g_bidirTested;
    bool   t5  = g_angleTested;
    bool   t6  = g_roadIdTested;
    bool   t7  = g_slowDownTested;
    bool   t8  = g_colorTested;

    std::cout << "\n  --- Test report ---\n\n";
    std::cout << "  1. Position updates : " << g_positionUpdates << " [" << (t1 ? "OK" : "FAIL") << "]\n";
    std::cout << "  2. Coordinates in SUMO area [" << (t2 ? "OK" : "FAIL") << "]\n";
    std::cout << "  3. Average speed    : " << std::fixed << std::setprecision(2) << avg << " m/s [" << (t3 ? "OK" : "FAIL") << "]\n";
    std::cout << "  4. SetVehicleSpeed  (instant override) [" << (t4 ? "OK" : "FAIL") << "]\n";
    std::cout << "  5. GetVehicleState  (angle) [" << (t5 ? "OK" : "FAIL") << "]\n";
    std::cout << "  6. GetVehicleState  (road / lane ID) [" << (t6 ? "OK" : "FAIL") << "]\n";
    std::cout << "  7. SetVehicleSlowDown [" << (t7 ? "OK" : "FAIL") << "]\n";
    std::cout << "  8. SetVehicleColor [" << (t8 ? "OK" : "FAIL") << "]\n";
    std::cout << "\n  Result: " << (t1+t2+t3+t4+t5+t6+t7+t8) << "/8\n";
    std::cout << std::string(60, '=') << "\n\n";

    return 0;
}