#pragma once

#include "sumo-client.h"
#include "ns3/application.h" // ns3::Application — base class for ns-3 apps
#include "ns3/node-container.h" // ns3::NodeContainer — pool of ns-3 nodes
#include "ns3/event-id.h" // ns3::EventId — handle to a scheduled event

#include <chrono> // std::chrono::milliseconds — typed duration for retry delay
#include <functional> // std::function — type-erased callback
#include <map> // std::map<string, Ptr<Node>>— vehicle-to-node mapping
#include <queue> // std::queue<Ptr<Node>> — FIFO pool of free nodes
#include <set> // std::set<string> — active vehicle ID set
#include <string>

namespace ns3 {

// ---------------------------------------------------------------------------
// SumoManager
//
// ns-3 Application that drives the ns-3/SUMO co-simulation.
//
// Responsibilities:
//   1. Optionally launch SUMO via system() and connect via TraCI.
//   2. At every simulation step: advance SUMO, read vehicle positions,
//      update the matching ns-3 node positions accordingly.
//   3. Manage a pool of pre-created ns-3 nodes mapped to SUMO vehicles.
//   4. Invoke a user-supplied callback for custom bidirectional logic
//      (e.g. sending speed commands back to SUMO).
//
// Usage:
//   auto mgr = CreateObject<SumoManager>();
//   mgr->SetAttribute("SumoConfig", StringValue("path/to/sim.sumocfg"));
//   mgr->SetNodePool(nodes);
//   mgr->SetVehicleCallback([&](double t, const std::string& id,
//                               const VehicleState& s) { ... });
//   node->AddApplication(mgr);
// ---------------------------------------------------------------------------
class SumoManager : public Application {
public:
    static TypeId GetTypeId(); // register attributes with the ns-3 type system

    SumoManager();
    ~SumoManager() override;

    // Assign the pool of pre-created ns-3 nodes.
    // Must be called before the simulation starts.
    // The pool must have at least as many nodes as the peak number of
    // simultaneous SUMO vehicles expected during the scenario.
    void SetNodePool(NodeContainer nodes);

    // Register a callback invoked at every step for every active vehicle.
    // Signature: void(double simTime, const std::string& vehicleId,
    //                 const VehicleState& state)
    using VehicleCallback =
        std::function<void(double, const std::string&, const VehicleState&)>;
    void SetVehicleCallback(VehicleCallback cb);

    // Direct access to the TraCI client for advanced commands that are not
    // wrapped by SumoManager (e.g. changing traffic-light phases).
    SumoClient& GetClient() { return m_client; }

protected:
    // Called by ns-3 at simulation time 0 (or when the app is started).
    void StartApplication() override;
    // Called by ns-3 when the simulation ends; cancels the step event.
    void StopApplication() override;

private:
    // Schedule one co-simulation step: advance SUMO, reconcile vehicle lists,
    // update ns-3 node positions, invoke the user callback, then re-schedule.
    void Step();

    // Take one node from the free pool and map it to vehicleId.
    // Returns nullptr if the pool is exhausted.
    Ptr<Node> AllocateNode(const std::string& vehicleId);

    // Unmap vehicleId, park its node at (1e9, 1e9, 1e9) to avoid radio
    // interference, and return it to the free pool.
    void ReleaseNode(const std::string& vehicleId);

    SumoClient m_client; // owns the TCP socket to SUMO

    // Attributes — configurable via ns-3 SetAttribute().
    std::string m_host{"127.0.0.1"};
    int m_port{1337};
    double m_stepSize{0.1}; // seconds per co-simulation step
    std::string m_sumoConfig{}; // path to .sumocfg; empty = SUMO already running
    bool m_useGui{false}; // true = launch sumo-gui, false = sumo (headless)

    // Retry policy for connecting after launching SUMO.
    // std::chrono::milliseconds makes the unit explicit in the type itself,
    // eliminating the ambiguity of the original "useconds_t 100'000".
    static constexpr int k_connectRetries = 20;
    static constexpr std::chrono::milliseconds k_connectRetryDelay{100};

    double m_currentTime{0.0}; // tracks wall-clock simulation time in seconds
    EventId m_stepEvent; // handle used to cancel the next Step() call

    // Node pool — vehicles enter and leave the simulation dynamically.
    std::queue<Ptr<Node>> m_freeNodes; // unassigned nodes
    std::map<std::string, Ptr<Node>> m_vehicleToNode; // vehicleId → ns-3 node
    std::set<std::string> m_activeVehicles; // IDs currently tracked

    VehicleCallback m_vehicleCallback; // user-supplied; empty = no callback
};

} // namespace ns3