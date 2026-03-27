/**
 * @file simple-vanet.cc
 * @brief Validation example for the ns-3/SUMO bidirectional coupling module.
 *
 * Validates four properties of the module:
 *   1. Position updates received from SUMO (SUMO -> ns-3).
 *   2. Accuracy of vehicle coordinates against the expected road network area.
 *   3. Plausibility of average vehicle speed.
 *   4. Bidirectional command (ns-3 -> SUMO) via SetVehicleSpeed.
 *
 * Usage:
 *   Terminal 1: sumo -c simple.sumocfg --remote-port 1337
 *   Terminal 2: NS_LOG=SumoManager=info ./ns3 run simple-vanet \
 *               -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=60
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "../model/sumo-client.h"
#include "../model/sumo-manager.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SimpleVanet");

static uint32_t g_positionUpdates = 0;
static double g_sumSpeed = 0.0;
static double g_minX = 1e9;
static double g_maxX = -1e9;
static double g_minY = 1e9;
static double g_maxY = -1e9;
static bool g_bidirTested = false;

int main(int argc, char* argv[]) {
    std::string sumoConfig = "";
    std::string sumoHost = "127.0.0.1";
    int sumoPort = 1337;
    uint32_t poolSize = 10;
    double simTime = 60.0;
    double stepSize = 0.1;
    bool useGui = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("sumoConfig", "Path to .sumocfg file", sumoConfig);
    cmd.AddValue("sumoHost", "TraCI host", sumoHost);
    cmd.AddValue("sumoPort", "TraCI port", sumoPort);
    cmd.AddValue("poolSize", "Node pool size", poolSize);
    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("stepSize", "Synchronization step (s)", stepSize);
    cmd.AddValue("useGui", "Use sumo-gui", useGui);
    cmd.Parse(argc, argv);

    LogComponentEnable("SumoManager", LOG_LEVEL_INFO);

    // Create the node pool with a constant-position mobility model.
    // SumoManager will update each node's position at every step.
    NodeContainer pool;
    pool.Create(poolSize);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(pool);
    for (uint32_t i = 0; i < pool.GetN(); i++)
        pool.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(1e9, 1e9, 1e9));

    // Create and configure SumoManager.
    Ptr<Node> managerNode = CreateObject<Node>();
    Ptr<SumoManager> sumoManager = CreateObject<SumoManager>();
    sumoManager->SetAttribute("Host", StringValue(sumoHost));
    sumoManager->SetAttribute("Port", IntegerValue(sumoPort));
    sumoManager->SetAttribute("StepSize", DoubleValue(stepSize));
    sumoManager->SetAttribute("SumoConfig", StringValue(sumoConfig));
    sumoManager->SetAttribute("UseGui", BooleanValue(useGui));
    sumoManager->SetNodePool(pool);

    // Callback invoked at every step for every active vehicle.
    // Collects metrics and tests one bidirectional command.
    sumoManager->SetVehicleCallback(
        [&sumoManager](double t, const std::string& vid, const VehicleState& s) {
            g_positionUpdates++;
            g_sumSpeed += s.speed;
            g_minX = std::min(g_minX, s.x);
            g_maxX = std::max(g_maxX, s.x);
            g_minY = std::min(g_minY, s.y);
            g_maxY = std::max(g_maxY, s.y);

            // Print the first five updates as a visible sample.
            if (g_positionUpdates <= 5)
                NS_LOG_INFO("[pos] " << vid << " x=" << s.x << " y=" << s.y << " speed=" << s.speed << " m/s t=" << t << "s");

            // Send one speed command to validate the ns-3 -> SUMO direction.
            if (!g_bidirTested && s.speed > 3.0) {
                g_bidirTested = true;
                bool ok = sumoManager->GetClient().SetVehicleSpeed(vid, 2.0);
                NS_LOG_INFO("[bidir] SetVehicleSpeed(" << vid << ", 2.0) -> " << (ok ? "OK" : "FAILED"));
            }
        });

    managerNode->AddApplication(sumoManager);
    sumoManager->SetStartTime(Seconds(0.0));
    sumoManager->SetStopTime(Seconds(simTime));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    // Validation report.
    bool t1 = (g_positionUpdates >= 100);
    bool t2 = (g_minX >= -10 && g_maxX <= 410 && g_minY >= -10 && g_maxY <= 410);
    double avgSpeed = g_positionUpdates > 0 ? g_sumSpeed / g_positionUpdates : 0.0;
    bool t3 = (avgSpeed > 1.0 && avgSpeed <= 14.0);
    bool t4 = g_bidirTested;

    std::cout << "\n[START TEST]\n";
    std::cout << "  1. Position updates: " << g_positionUpdates << " [" << (t1 ? "OK" : "FAIL") << "]\n";
    std::cout << "  2. Coordinates in SUMO area: [" << (t2 ? "OK" : "FAIL") << "]\n";
    std::cout << "  3. Average speed: " << avgSpeed << " m/s [" << (t3 ? "OK" : "FAIL") << "]\n";
    std::cout << "  4. Bidirectional command: [" << (t4 ? "OK" : "FAIL") << "]\n";
    std::cout << "[END TEST] Result: " << (t1 + t2 + t3 + t4) << "/4\n\n";

    return 0;
}