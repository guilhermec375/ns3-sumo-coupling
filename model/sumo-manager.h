#ifndef SUMO_MANAGER_H
#define SUMO_MANAGER_H

#include "sumo-client.h"
#include "ns3/application.h"
#include "ns3/node-container.h"
#include "ns3/event-id.h"

#include <functional>
#include <map>
#include <queue>
#include <set>
#include <string>

namespace ns3 {

/**
 * @brief ns-3 Application that manages the ns-3/SUMO co-simulation.
 *
 * Responsibilities:
 *   - optionally launch SUMO and connect via TraCI (with retry logic)
 *   - at each step: advance SUMO, read positions, update ns-3 nodes
 *   - manage a pool of pre-created ns-3 nodes mapped to SUMO vehicles
 *   - invoke a user callback for custom bidirectional logic
 */
class SumoManager : public Application {
public:
    /**
     * @brief Register the type with the ns-3 type system.
     * @return TypeId of this class.
     */
    static TypeId GetTypeId();

    SumoManager();
    ~SumoManager() override;

    /**
     * @brief Assign the node pool that will be mapped to SUMO vehicles.
     *
     * Must be called before the simulation starts. The pool must contain
     * at least as many nodes as the maximum number of simultaneous vehicles
     * expected in the SUMO scenario.
     *
     * @param nodes NodeContainer with pre-created ns-3 nodes.
     */
    void SetNodePool(NodeContainer nodes);

    /**
     * @brief Set a callback invoked at every step for every active vehicle.
     *
     * Signature: void callback(double simTime, const std::string& vehicleId,
     *                          const VehicleState& state)
     *
     * Use this callback to implement custom logic, e.g. send commands back
     * to SUMO via GetClient().SetVehicleSpeed().
     */
    using VehicleCallback = std::function<void(double, const std::string&, const VehicleState&)>;
    void SetVehicleCallback(VehicleCallback cb);

    /**
     * @brief Direct access to the TraCI client for advanced commands.
     * @return Reference to the internal SumoClient.
     */
    SumoClient& GetClient() { return m_client; }

protected:
    void StartApplication() override;
    void StopApplication() override;

private:
    void Step();
    Ptr<Node> AllocateNode(const std::string& vehicleId);
    void ReleaseNode(const std::string& vehicleId);

    SumoClient m_client;

    // Configurable attributes (set via SetAttribute).
    std::string m_host;
    int         m_port;
    double      m_stepSize;
    std::string m_sumoConfig;
    bool        m_useGui;

    // How many times to retry connecting to SUMO after launching it,
    // and how long to wait between each attempt (microseconds).
    static constexpr int       k_connectRetries   = 20;
    static constexpr useconds_t k_connectRetryDelay = 100'000; // 100 ms

    double  m_currentTime;
    EventId m_stepEvent;

    std::queue<Ptr<Node>>             m_freeNodes;
    std::map<std::string, Ptr<Node>>  m_vehicleToNode;
    std::set<std::string>             m_activeVehicles;

    VehicleCallback m_vehicleCallback;
};

} // namespace ns3

#endif // SUMO_MANAGER_H