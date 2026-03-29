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
    double x            = 0.0; ///< X coordinate in meters (SUMO Cartesian system).
    double y            = 0.0; ///< Y coordinate in meters.
    double speed        = 0.0; ///< Scalar speed in m/s.
    double angle        = 0.0; ///< Heading angle in degrees (0 = North, clockwise).
    double acceleration = 0.0; ///< Longitudinal acceleration in m/s^2.
    std::string roadId;        ///< ID of the edge the vehicle is currently on.
    std::string laneId;        ///< ID of the lane the vehicle is currently on.
    double lanePosition = 0.0; ///< Distance from the start of the lane in meters.
};

/**
 * @brief RGBA color used by SetVehicleColor().
 */
struct TraCIColor {
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 0;
    uint8_t a = 255;
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

    // -----------------------------------------------------------------------
    // Connection management.

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

    // -----------------------------------------------------------------------
    // Simulation control.

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
     * Prefer this over GetVehicleIds() + set-difference: no extra round-trip.
     * Note: does NOT cover vehicles removed via teleporting or rerouting failures;
     * use GetVehicleIds() for authoritative reconciliation.
     */
    std::vector<std::string> GetArrivedVehicleIds();

    // -----------------------------------------------------------------------
    // Vehicle state retrieval (GET).

    /**
     * @brief Return the full vehicle state in a single batched request.
     *
     * Sends all variable queries back-to-back before reading any reply,
     * minimising TCP round-trips. Fills: x, y, speed, angle, acceleration,
     * roadId, laneId, lanePosition.
     *
     * @param vehicleId SUMO vehicle ID.
     * @return Fully populated VehicleState. All fields are zero/empty on error.
     */
    VehicleState GetVehicleState(const std::string& vehicleId);

    /** @return Heading angle of the vehicle in degrees (0 = North, clockwise). */
    double GetVehicleAngle(const std::string& vehicleId);

    /** @return Longitudinal acceleration of the vehicle in m/s^2. */
    double GetVehicleAcceleration(const std::string& vehicleId);

    /** @return ID of the road (edge) the vehicle is currently on. */
    std::string GetVehicleRoadId(const std::string& vehicleId);

    /** @return ID of the lane the vehicle is currently on. */
    std::string GetVehicleLaneId(const std::string& vehicleId);

    /**
     * @return Distance from the start of the current lane to the vehicle's
     *         front bumper in meters.
     */
    double GetVehicleLanePosition(const std::string& vehicleId);

    // -----------------------------------------------------------------------
    // Vehicle state commands (SET, ns-3 -> SUMO).

    /**
     * @brief Override the speed of a vehicle instantly.
     * @param vehicleId SUMO vehicle ID.
     * @param speed     Target speed in m/s. Pass -1.0 to restore autonomous control.
     * @return true if SUMO acknowledged the command.
     */
    bool SetVehicleSpeed(const std::string& vehicleId, double speed);

    /**
     * @brief Gradually reduce the vehicle speed to the given value.
     *
     * Implements CMD_SLOWDOWN (0x14): SUMO linearly interpolates the speed
     * reduction over the specified duration, which is more realistic than an
     * instant override.
     *
     * @param vehicleId SUMO vehicle ID.
     * @param speed     Target speed in m/s.
     * @param duration  Time in seconds to reach the target speed.
     * @return true if SUMO acknowledged the command.
     */
    bool SetVehicleSlowDown(const std::string& vehicleId, double speed, double duration);

    /**
     * @brief Change the colour of a vehicle in the SUMO GUI.
     *
     * Useful for visualising vehicle state (e.g. highlight vehicles that
     * received a message, or those above a speed threshold).
     *
     * @param vehicleId SUMO vehicle ID.
     * @param color     RGBA colour. Default yellow (255, 255, 0, 255).
     * @return true if SUMO acknowledged the command.
     */
    bool SetVehicleColor(const std::string& vehicleId, const TraCIColor& color);

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

    // GET vehicle variable IDs.
    static constexpr uint8_t VAR_ID_LIST      = 0x00;
    static constexpr uint8_t VAR_SPEED        = 0x40;
    static constexpr uint8_t VAR_POSITION     = 0x42;
    static constexpr uint8_t VAR_ANGLE        = 0x43;
    static constexpr uint8_t VAR_ROAD_ID      = 0x50;
    static constexpr uint8_t VAR_LANE_ID      = 0x51;
    static constexpr uint8_t VAR_LANE_POS     = 0x56;
    static constexpr uint8_t VAR_ACCELERATION = 0x72;

    // SET vehicle variable / command IDs.
    static constexpr uint8_t VAR_COLOR        = 0x45; ///< set color (RGBA)
    static constexpr uint8_t CMD_SLOWDOWN     = 0x14; ///< gradual speed reduction

    // Simulation variable IDs.
    static constexpr uint8_t VAR_DEPARTED_IDS = 0x74;
    static constexpr uint8_t VAR_ARRIVED_IDS  = 0x79;

    // Data type tags used in payloads.
    static constexpr uint8_t TYPE_DOUBLE      = 0x0b;
    static constexpr uint8_t TYPE_STRINGLIST  = 0x0e;
    static constexpr uint8_t TYPE_POSITION2D  = 0x01;
    static constexpr uint8_t TYPE_STRING      = 0x0c;
    static constexpr uint8_t TYPE_COLOR       = 0x11;
    static constexpr uint8_t TYPE_COMPOUND    = 0x0f;

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
    // Shared helpers — avoid boilerplate in individual getters.
    std::vector<std::string> GetStringListVar(uint8_t domain, uint8_t respId, uint8_t varId);
    double      GetDoubleVar (uint8_t varId, const std::string& vehicleId);
    std::string GetStringVar (uint8_t varId, const std::string& vehicleId);
};

} // namespace ns3

#endif // SUMO_CLIENT_H