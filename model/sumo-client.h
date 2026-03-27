#ifndef SUMO_CLIENT_H
#define SUMO_CLIENT_H

#include <cstdint>
#include <string>
#include <vector>

namespace ns3 {

/**
 * @brief State of a SUMO vehicle, filled by GetVehicleState().
 */
struct VehicleState {
    double x     = 0.0; ///< X coordinate in meters (SUMO Cartesian system).
    double y     = 0.0; ///< Y coordinate in meters.
    double speed = 0.0; ///< Scalar speed in m/s.
};

/**
 * @brief Minimal TraCI client for ns-3/SUMO co-simulation.
 *
 * Implements the binary TraCI protocol over a TCP socket.
 * Has no dependency on SUMO source files.
 * Compatible with SUMO 1.8 and later.
 */
class SumoClient {
public:
    SumoClient();
    ~SumoClient();

    // Non-copyable.
    SumoClient(const SumoClient&)            = delete;
    SumoClient& operator=(const SumoClient&) = delete;

    /**
     * @brief Open a TCP connection to SUMO and perform the TraCI handshake.
     * @param host SUMO hostname or IP address.
     * @param port TraCI server port.
     * @return true on success.
     */
    bool Connect(const std::string& host, int port);

    /**
     * @brief Send CMD_CLOSE to SUMO and close the socket.
     */
    void Close();

    /**
     * @brief Check whether the socket is open.
     * @return true if connected.
     */
    bool IsConnected() const { return m_socket >= 0; }

    /**
     * @brief Advance the SUMO simulation to targetTime.
     * @param targetTime Target simulation time in seconds.
     * @return true if SUMO acknowledged the step successfully.
     */
    bool SimStep(double targetTime);

    /**
     * @brief Return the IDs of all active vehicles in SUMO.
     * @return Vector of vehicle ID strings.
     */
    std::vector<std::string> GetVehicleIds();

    /**
     * @brief Return the IDs of vehicles that entered the simulation in the last step.
     * @return Vector of vehicle ID strings.
     */
    std::vector<std::string> GetDepartedVehicleIds();

    /**
     * @brief Return the IDs of vehicles that left the simulation in the last step.
     *
     * Prefer this over calling GetVehicleIds() + set-difference at every step:
     * it requires no extra round-trip and lets SumoManager avoid an O(n) scan.
     *
     * @return Vector of vehicle ID strings.
     */
    std::vector<std::string> GetArrivedVehicleIds();

    /**
     * @brief Return the position and speed of a vehicle.
     *
     * Sends both queries back-to-back before reading any reply,
     * halving the number of network round-trips compared to two
     * sequential request/reply pairs.
     *
     * @param vehicleId SUMO vehicle ID.
     * @return VehicleState with x, y and speed. All zero if the vehicle is unknown.
     */
    VehicleState GetVehicleState(const std::string& vehicleId);

    /**
     * @brief Set the maximum speed of a vehicle (ns-3 to SUMO command).
     * @param vehicleId SUMO vehicle ID.
     * @param speed Maximum speed in m/s. Pass -1.0 to restore the original speed.
     * @return true if SUMO acknowledged the command.
     */
    bool SetVehicleSpeed(const std::string& vehicleId, double speed);

private:

    int m_socket; ///< TCP socket file descriptor. -1 when not connected.

    /// Internal representation of a single TraCI command parsed from a response.
    struct TraCICmd {
        uint8_t id;
        std::vector<uint8_t> data;
    };

    // -----------------------------------------------------------------------
    // TraCI protocol constants.
    // Source: https://sumo.dlr.de/docs/TraCI/Protocol.html

    // Commands sent by ns-3 to SUMO.
    static constexpr uint8_t CMD_GETVERSION      = 0x00;
    static constexpr uint8_t CMD_SIMSTEP         = 0x02;
    static constexpr uint8_t CMD_CLOSE           = 0x7F;
    static constexpr uint8_t CMD_GET_VEHICLE_VAR = 0xa4;
    static constexpr uint8_t CMD_SET_VEHICLE_VAR = 0xc4;
    static constexpr uint8_t CMD_GET_SIM_VAR     = 0xab;

    // Responses: command ID + 0x10.
    static constexpr uint8_t RESP_GET_VEHICLE_VAR = 0xb4;
    static constexpr uint8_t RESP_GET_SIM_VAR     = 0xbb;

    // Vehicle variable IDs.
    static constexpr uint8_t VAR_ID_LIST  = 0x00;
    static constexpr uint8_t VAR_POSITION = 0x42;
    static constexpr uint8_t VAR_SPEED    = 0x40;

    // Simulation variable IDs.
    static constexpr uint8_t VAR_DEPARTED_IDS = 0x74;
    static constexpr uint8_t VAR_ARRIVED_IDS  = 0x79;

    // Data type tags used in payloads.
    static constexpr uint8_t TYPE_DOUBLE     = 0x0b;
    static constexpr uint8_t TYPE_STRINGLIST = 0x0e;
    static constexpr uint8_t TYPE_POSITION2D = 0x01;

    // Status code returned by SUMO on success.
    static constexpr uint8_t RTYPE_OK = 0x00;

    // -----------------------------------------------------------------------
    // Transport layer.
    bool SendCmd(uint8_t cmdId, const std::vector<uint8_t>& payload);
    std::vector<uint8_t> RecvResponse();
    static std::vector<TraCICmd> ParseCommands(const std::vector<uint8_t>& buf);

    bool SendBytes(const void* buf, size_t len);
    bool RecvBytes(void* buf, size_t len);

    // -----------------------------------------------------------------------
    // Serializers / deserializers.
    static void PutInt32(std::vector<uint8_t>& b, int32_t v);
    static void PutDouble(std::vector<uint8_t>& b, double v);
    static void PutString(std::vector<uint8_t>& b, const std::string& s);

    static int32_t     GetInt32 (const std::vector<uint8_t>& b, size_t& pos);
    static double      GetDouble(const std::vector<uint8_t>& b, size_t& pos);
    static std::string GetString(const std::vector<uint8_t>& b, size_t& pos);

    // -----------------------------------------------------------------------
    // Shared helper used by GetVehicleIds / GetDepartedVehicleIds / GetArrivedVehicleIds.
    std::vector<std::string> GetStringListVar(uint8_t domain, uint8_t respId, uint8_t varId);
};

} // namespace ns3

#endif // SUMO_CLIENT_H