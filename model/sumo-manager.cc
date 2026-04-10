#include "sumo-manager.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/vector.h"

#include <cstdlib> // system() — launch SUMO as a background shell command
#include <thread> // std::this_thread::sleep_for

NS_LOG_COMPONENT_DEFINE("SumoManager");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED(SumoManager);

// ===========================================================================
// ns-3 type registration
// ===========================================================================

TypeId SumoManager::GetTypeId() {
    static TypeId tid = TypeId("ns3::SumoManager")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<SumoManager>()
        // Each AddAttribute call exposes a parameter that can be set with
        // obj->SetAttribute("Name", Value(...)) without recompiling.
        .AddAttribute("Host",
            "Hostname or IP address of the SUMO TraCI server.",
            StringValue("127.0.0.1"),
            MakeStringAccessor(&SumoManager::m_host),
            MakeStringChecker())
        .AddAttribute("Port",
            "TCP port on which SUMO listens for TraCI connections.",
            IntegerValue(1337),
            MakeIntegerAccessor(&SumoManager::m_port),
            MakeIntegerChecker<int>(1, 65535))
        .AddAttribute("StepSize",
            "Duration of each co-simulation step in seconds.",
            DoubleValue(0.1),
            MakeDoubleAccessor(&SumoManager::m_stepSize),
            MakeDoubleChecker<double>(0.001, 10.0))
        .AddAttribute("SumoConfig",
            "Path to the .sumocfg file. If empty, SUMO must already be running.",
            StringValue(""),
            MakeStringAccessor(&SumoManager::m_sumoConfig),
            MakeStringChecker())
        .AddAttribute("UseGui",
            "If true, launch sumo-gui instead of the headless sumo binary.",
            BooleanValue(false),
            MakeBooleanAccessor(&SumoManager::m_useGui),
            MakeBooleanChecker());
    return tid;
}

// ===========================================================================
// Lifecycle
// ===========================================================================

SumoManager::SumoManager()
        : m_host("127.0.0.1"),
            m_port(1337),
            m_stepSize(0.1),
            m_useGui(false),
            m_currentTime(0.0)
{}

SumoManager::~SumoManager() {}

// ===========================================================================
// Configuration helpers
// ===========================================================================

void SumoManager::SetNodePool(NodeContainer nodes) {
    // Drain any existing pool before loading the new one.
    while (!m_freeNodes.empty()) m_freeNodes.pop();
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
        m_freeNodes.push(nodes.Get(i));
    NS_LOG_INFO("SumoManager: node pool set with " << nodes.GetN() << " nodes");
}

void SumoManager::SetVehicleCallback(VehicleCallback cb) {
    m_vehicleCallback = cb;
}

// ===========================================================================
// Application lifecycle — called by ns-3
// ===========================================================================

void SumoManager::StartApplication() {
    if (!m_sumoConfig.empty()) {
        // Launch SUMO as a background shell process via system().
        // system() is appropriate here: the config path comes from a
        // controlled project file (.sumocfg), not from arbitrary user input,
        // so shell-injection risk is not a concern.
        const std::string exe = m_useGui ? "sumo-gui" : "sumo";
        const std::string cmd = exe
            + " --configuration-file " + m_sumoConfig
            + " --remote-port " + std::to_string(m_port)
            + " --step-length " + std::to_string(m_stepSize)
            + " --no-step-log true"
            + " > /dev/null 2>&1 &"; // discard SUMO output; & = background
        NS_LOG_INFO("SumoManager: launching SUMO: " << cmd);
        (void)system(cmd.c_str()); // (void) suppresses the unused-return warning

        // Poll for a working TraCI connection instead of sleeping a fixed
        // worst-case delay. Stops as soon as SUMO is ready.
        bool connected = false;
        for (int attempt = 0; attempt < k_connectRetries; ++attempt) {
            // std::chrono::milliseconds makes the unit explicit; this
            // replaces the original usleep(100'000) where the unit (microseconds)
            // was implicit and easy to misread.
            std::this_thread::sleep_for(k_connectRetryDelay);
            if (m_client.Connect(m_host, m_port)) {
                connected = true;
                break; // stop polling immediately on success
            }
            NS_LOG_DEBUG("SumoManager: waiting for SUMO (attempt "
                         << attempt + 1 << "/" << k_connectRetries << ")");
        }
        if (!connected) {
            // .count() extracts the numeric value from the chrono duration.
            NS_FATAL_ERROR("SumoManager: SUMO did not start after "
                           << (k_connectRetries * k_connectRetryDelay.count())
                           << " ms — check the .sumocfg path and port");
            return;
        }
    } else {
        // SumoConfig is empty: assume SUMO was started externally.
        if (!m_client.Connect(m_host, m_port)) {
            NS_FATAL_ERROR("SumoManager: cannot connect to SUMO at "
                           << m_host << ":" << m_port);
            return;
        }
    }

    NS_LOG_INFO("SumoManager: connected to SUMO at " << m_host << ":" << m_port);
    m_currentTime = 0.0;
    m_stepEvent = Simulator::ScheduleNow(&SumoManager::Step, this);
}

void SumoManager::StopApplication() {
    Simulator::Cancel(m_stepEvent); // prevent the next Step() from firing
    m_client.Close();
    NS_LOG_INFO("SumoManager: disconnected from SUMO");
}

// ===========================================================================
// Step — the co-simulation loop body
//
// Called once per simulation step. Execution order:
//   1. Advance SUMO to the new time.
//   2. Register vehicles that entered the simulation this step.
//   3. Remove vehicles that are no longer in SUMO's active list.
//   4. Update ns-3 node positions; invoke the user callback.
//   5. Schedule the next Step() call.
// ===========================================================================

void SumoManager::Step() {
    m_currentTime += m_stepSize;

    if (!m_client.SimStep(m_currentTime)) {
        NS_LOG_ERROR("SumoManager: SimStep failed at t=" << m_currentTime);
        return; // do not re-schedule; the co-simulation is broken
    }

    // ------------------------------------------------------------------
    // 1. Register newly departed vehicles
    // ------------------------------------------------------------------
    for (const auto& vid : m_client.GetDepartedVehicleIds()) {
        if (vid.empty()) continue;
        m_activeVehicles.insert(vid);
        if (!AllocateNode(vid))
            NS_LOG_WARN("SumoManager: node pool exhausted, skipping vehicle " << vid);
        else
            NS_LOG_INFO("SumoManager: vehicle " << vid
                        << " entered at t=" << m_currentTime << "s");
    }

    // ------------------------------------------------------------------
    // 2. Reconcile with SUMO's authoritative vehicle list
    //
    // GetVehicleIds() is used instead of GetArrivedVehicleIds() because
    // VAR_ARRIVED_IDS only covers vehicles that completed their route
    // normally.  Vehicles removed by teleporting, rerouting failures, or
    // explicit TraCI removal do NOT appear in that list and would remain in
    // m_activeVehicles indefinitely, causing "vehicle not known" errors on
    // every subsequent GetVehicleState call.
    // ------------------------------------------------------------------
    auto knownIds = m_client.GetVehicleIds();
    std::set<std::string> knownSet(knownIds.begin(), knownIds.end());

    // Erase-during-iteration pattern for std::set: erase() returns the
    // iterator to the next element, so the loop variable stays valid.
    // A range-based for loop cannot be used here because erasing an element
    // invalidates the current iterator.
    for (auto it = m_activeVehicles.begin(); it != m_activeVehicles.end();) {
        if (knownSet.count(*it) == 0) {
            NS_LOG_INFO("SumoManager: vehicle " << *it
                        << " left at t=" << m_currentTime << "s");
            ReleaseNode(*it);
            m_vehicleToNode.erase(*it);
            it = m_activeVehicles.erase(it); // advance to next valid iterator
        } else {
            ++it;
        }
    }

    // ------------------------------------------------------------------
    // 3. Update positions and invoke the user callback
    //
    // At this point m_activeVehicles contains only IDs confirmed alive
    // by SUMO, so GetVehicleState() will never query an unknown vehicle.
    // ------------------------------------------------------------------
    for (const auto& vid : m_activeVehicles) {
        auto nodeIt = m_vehicleToNode.find(vid);
        if (nodeIt == m_vehicleToNode.end()) continue; // should never happen

        VehicleState state = m_client.GetVehicleState(vid);

        // Move the ns-3 node to the position reported by SUMO.
        auto mob = nodeIt->second->GetObject<MobilityModel>();
        if (mob) mob->SetPosition(Vector(state.x, state.y, 0.0));

        // Invoke user logic (e.g. send a message, change speed, log data).
        // The boolean check avoids a std::function call overhead when no
        // callback has been registered.
        if (m_vehicleCallback)
            m_vehicleCallback(m_currentTime, vid, state);
    }

    // Schedule the next step exactly m_stepSize seconds from now.
    m_stepEvent = Simulator::Schedule(Seconds(m_stepSize), &SumoManager::Step, this);
}

// ===========================================================================
// Node pool management
// ===========================================================================

// Dequeue one node from the free pool and record the vehicle-to-node mapping.
Ptr<Node> SumoManager::AllocateNode(const std::string& vehicleId) {
    if (m_freeNodes.empty()) return nullptr; // caller logs the warning

    // front() and pop() are separate operations in std::queue: pop() does not
    // return a value so that it can provide the strong exception guarantee
    // (if copying the value threw, the element would still be in the queue).
    Ptr<Node> node = m_freeNodes.front();
    m_freeNodes.pop();
    m_vehicleToNode[vehicleId] = node;
    return node;
}

// Unmap a vehicle, park its ns-3 node far outside the simulation area, and
// return the node to the free pool for reuse by the next arriving vehicle.
void SumoManager::ReleaseNode(const std::string& vehicleId) {
    auto it = m_vehicleToNode.find(vehicleId);
    if (it == m_vehicleToNode.end()) return; // already released or never allocated

    Ptr<Node> node = it->second;

    // Park at (1e9, 1e9, 1e9) metres — one billion metres from the origin.
    // This ensures the node does not participate in any radio propagation
    // model calculations while it is idle in the pool.
    auto mob = node->GetObject<MobilityModel>();
    if (mob) mob->SetPosition(Vector(1e9, 1e9, 1e9));

    m_freeNodes.push(node); // return to the back of the free queue
}

} // namespace ns3