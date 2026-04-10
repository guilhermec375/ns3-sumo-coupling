/**
 * @file minimal-vanet.cc
 * @brief Minimal ns-3/SUMO coupling example.
 *
 * Usage:
 *   ./ns3 run minimal-vanet -- --sumoHost=127.0.0.1 --sumoPort=1337 --simTime=20
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "../model/sumo-manager.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <unordered_map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MinimalVanet");

int
main(int argc, char* argv[])
{
    std::string sumoConfig = "";
    std::string sumoHost = "127.0.0.1";
    int sumoPort = 1337;
    uint32_t poolSize = 10;
    double simTime = 20.0;
    double stepSize = 0.1;
    bool useGui = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("sumoConfig", "Path to .sumocfg file", sumoConfig);
    cmd.AddValue("sumoHost", "TraCI host", sumoHost);
    cmd.AddValue("sumoPort", "TraCI port", sumoPort);
    cmd.AddValue("poolSize", "Node pool size", poolSize);
    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("stepSize", "Synchronisation step (s)", stepSize);
    cmd.AddValue("useGui", "Use sumo-gui", useGui);
    cmd.Parse(argc, argv);

    NodeContainer pool;
    pool.Create(poolSize);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(pool);

    Ptr<Node> managerNode = CreateObject<Node>();
    Ptr<SumoManager> sumoManager = CreateObject<SumoManager>();

    sumoManager->SetAttribute("Host", StringValue(sumoHost));
    sumoManager->SetAttribute("Port", IntegerValue(sumoPort));
    sumoManager->SetAttribute("StepSize", DoubleValue(stepSize));
    sumoManager->SetAttribute("SumoConfig", StringValue(sumoConfig));
    sumoManager->SetAttribute("UseGui", BooleanValue(useGui));
    sumoManager->SetNodePool(pool);

    auto printActiveCount = [sumoManager](const char* label) {
        const auto active = sumoManager->GetClient().GetVehicleIds();
        std::cout << "### T=" << std::fixed << std::setprecision(1)
                  << Simulator::Now().GetSeconds() << "s "
                  << label << " ACTIVE VEHICLES=" << active.size() << " ###\n";
    };

    // Print once at start, then every 10 seconds.
    Simulator::Schedule(Seconds(0.0), [printActiveCount]() { printActiveCount("INITIAL:"); });
    for (double t = 10.0; t < simTime; t += 10.0)
    {
        Simulator::Schedule(Seconds(t), [printActiveCount]() { printActiveCount("PERIODIC:"); });
    }
    Simulator::Schedule(Seconds(simTime), [printActiveCount]() { printActiveCount("FINAL:"); });

    std::unordered_map<std::string, int64_t> lastPrintedSecondByVehicle;
    sumoManager->SetVehicleCallback(
        [&lastPrintedSecondByVehicle](double t, const std::string& vid, const VehicleState& s) {
            const int64_t sec = static_cast<int64_t>(std::floor(t));
            auto it = lastPrintedSecondByVehicle.find(vid);
            if (it == lastPrintedSecondByVehicle.end() || it->second != sec)
            {
                lastPrintedSecondByVehicle[vid] = sec;
                std::cout << "[t=" << std::fixed << std::setprecision(1) << t << "s] "
                          << vid << " x=" << s.x << " y=" << s.y << " v=" << s.speed << "\n";
            }
        });

    managerNode->AddApplication(sumoManager);
    sumoManager->SetStartTime(Seconds(0.0));
    sumoManager->SetStopTime(Seconds(simTime));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
