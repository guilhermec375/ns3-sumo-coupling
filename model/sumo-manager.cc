#include "sumo-manager.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/vector.h"

#include <cstdlib>
#include <unistd.h>

NS_LOG_COMPONENT_DEFINE("SumoManager");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED(SumoManager);

TypeId SumoManager::GetTypeId() {
    static TypeId tid = TypeId("ns3::SumoManager")
        .SetParent<Application>()
        .SetGroupName("Applications")
        .AddConstructor<SumoManager>()
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
            "If true, launch sumo-gui instead of sumo.",
            BooleanValue(false),
            MakeBooleanAccessor(&SumoManager::m_useGui),
            MakeBooleanChecker());
    return tid;
}

// *******************************************************************************************************************

SumoManager::SumoManager()
    : m_host("127.0.0.1"),
      m_port(1337),
      m_stepSize(0.1),
      m_useGui(false),
      m_currentTime(0.0)
{}

SumoManager::~SumoManager() {}

// *******************************************************************************************************************

void SumoManager::SetNodePool(NodeContainer nodes) {
    while (!m_freeNodes.empty()) m_freeNodes.pop();
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
        m_freeNodes.push(nodes.Get(i));
    NS_LOG_INFO("SumoManager: node pool set with " << nodes.GetN() << " nodes");
}

void SumoManager::SetVehicleCallback(VehicleCallback cb) {
    m_vehicleCallback = cb;
}

// *******************************************************************************************************************

void SumoManager::StartApplication() {
    if (!m_sumoConfig.empty()) {
        const std::string exe = m_useGui ? "sumo-gui" : "sumo";
        const std::string cmd = exe
            + " --configuration-file " + m_sumoConfig
            + " --remote-port "        + std::to_string(m_port)
            + " --step-length "        + std::to_string(m_stepSize)
            + " --no-step-log true"
            + " > /dev/null 2>&1 &";
        NS_LOG_INFO("SumoManager: launching SUMO: " << cmd);
        (void)system(cmd.c_str());

        // Retry with a short delay instead of a single blind sleep.
        // This returns as soon as SUMO is ready rather than always
        // waiting the worst-case startup time.
        bool connected = false;
        for (int attempt = 0; attempt < k_connectRetries; ++attempt) {
            usleep(k_connectRetryDelay);
            if (m_client.Connect(m_host, m_port)) {
                connected = true;
                break;
            }
            NS_LOG_DEBUG("SumoManager: waiting for SUMO (attempt "
                         << attempt + 1 << "/" << k_connectRetries << ")");
        }
        if (!connected) {
            NS_FATAL_ERROR("SumoManager: SUMO did not start after "
                           << (k_connectRetries * k_connectRetryDelay / 1000)
                           << " ms — check the .sumocfg path and port");
            return;
        }
    } 
    else {
        // SUMO is assumed to be already running.
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
    Simulator::Cancel(m_stepEvent);
    m_client.Close();
    NS_LOG_INFO("SumoManager: disconnected from SUMO");
}

// *******************************************************************************************************************

void SumoManager::Step() {
    m_currentTime += m_stepSize;

    if (!m_client.SimStep(m_currentTime)) {
        NS_LOG_ERROR("SumoManager: SimStep failed at t=" << m_currentTime);
        return;
    }

    // -----------------------------------------------------------------------
    // 1. Register vehicles that entered the simulation in this step.
    for (const auto& vid : m_client.GetDepartedVehicleIds()) {
        if (vid.empty()) continue;
        m_activeVehicles.insert(vid);
        if (!AllocateNode(vid))
            NS_LOG_WARN("SumoManager: node pool exhausted, skipping vehicle " << vid);
        else
            NS_LOG_INFO("SumoManager: vehicle " << vid
                        << " entered at t=" << m_currentTime << "s");
    }

    // -----------------------------------------------------------------------
    // 2. Reconcile active vehicles against SUMO's authoritative vehicle list.
    //
    // Why GetVehicleIds() and not GetArrivedVehicleIds():
    //   VAR_ARRIVED_IDS only covers vehicles that completed their route
    //   normally.  Vehicles removed by teleporting, rerouting failures, or
    //   explicit TraCI removal do NOT appear in that list, so they would stay
    //   in m_activeVehicles forever and trigger "vehicle not known" errors on
    //   every subsequent GetVehicleState call.
    //   Comparing against the full active list is the only reliable way to
    //   detect all forms of vehicle removal.
    auto knownIds = m_client.GetVehicleIds();
    std::set<std::string> knownSet(knownIds.begin(), knownIds.end());

    for (auto it = m_activeVehicles.begin(); it != m_activeVehicles.end(); ) {
        if (knownSet.count(*it) == 0) {
            NS_LOG_INFO("SumoManager: vehicle " << *it
                        << " left at t=" << m_currentTime << "s");
            ReleaseNode(*it);
            m_vehicleToNode.erase(*it);
            it = m_activeVehicles.erase(it);
        } 
        else {
            ++it;
        }
    }

    // -----------------------------------------------------------------------
    // 3. Update position and invoke the user callback for every active vehicle.
    //    At this point m_activeVehicles contains only IDs that SUMO confirmed
    //    are still alive, so GetVehicleState will never query an unknown vehicle.
    for (const auto& vid : m_activeVehicles) {
        auto nodeIt = m_vehicleToNode.find(vid);
        if (nodeIt == m_vehicleToNode.end()) continue;

        VehicleState state = m_client.GetVehicleState(vid);

        auto mob = nodeIt->second->GetObject<MobilityModel>();
        if (mob) mob->SetPosition(Vector(state.x, state.y, 0.0));

        if (m_vehicleCallback)
            m_vehicleCallback(m_currentTime, vid, state);
    }

    m_stepEvent = Simulator::Schedule(Seconds(m_stepSize), &SumoManager::Step, this);
}

// *******************************************************************************************************************

Ptr<Node> SumoManager::AllocateNode(const std::string& vehicleId) {
    if (m_freeNodes.empty()) return nullptr;
    Ptr<Node> node = m_freeNodes.front();
    m_freeNodes.pop();
    m_vehicleToNode[vehicleId] = node;
    return node;
}

void SumoManager::ReleaseNode(const std::string& vehicleId) {
    auto it = m_vehicleToNode.find(vehicleId);
    if (it == m_vehicleToNode.end()) return;
    Ptr<Node> node = it->second;
    // Park the node far outside the simulation area so it does not
    // interfere with radio propagation while idle in the pool.
    auto mob = node->GetObject<MobilityModel>();
    if (mob) mob->SetPosition(Vector(1e9, 1e9, 1e9));
    m_freeNodes.push(node);
}

} // namespace ns3