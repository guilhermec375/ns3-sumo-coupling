#pragma once
// #pragma once: include this file only once per translation unit.
// Replaces the classical #ifndef / #define / #endif include-guard macros,
// which require a manually chosen unique name and are error-prone.

// Standard library headers — only what is actually needed in the interface.
#include <cstdint> // uint8_t, int32_t, uint64_t — fixed-width integers
#include <string> // std::string
#include <string_view> // std::string_view — non-owning string reference, no allocation
#include <vector> // std::vector<T> — dynamic byte buffer
#include <utility>

namespace ns3 {

// ---------------------------------------------------------------------------
// VehicleState
//
// Plain data aggregate filled by SumoClient::GetVehicleState().
// All numeric fields are default-initialised to 0.0 so that a
// default-constructed VehicleState{} is always safe to read.
// ---------------------------------------------------------------------------
struct VehicleState {
    double x = 0.0; // Cartesian X coordinate in metres (SUMO system).
    double y = 0.0; // Cartesian Y coordinate in metres.
    double speed = 0.0; // Scalar speed in m/s.
    double angle = 0.0; // Heading angle in degrees (0 = North, clockwise).
    double acceleration = 0.0; // Longitudinal acceleration in m/s².
    std::string roadId; // Current edge (road) ID — empty on error.
    std::string laneId; // Current lane ID — empty on error.
    double lanePosition = 0.0; // Distance from lane start to front bumper in metres.
};

// ---------------------------------------------------------------------------
// TraCIColor
//
// RGBA colour sent to SUMO via SetVehicleColor().
// Default: yellow (255, 255, 0) fully opaque (alpha = 255).
// ---------------------------------------------------------------------------
struct TraCIColor {
    uint8_t r = 255;
    uint8_t g = 255;
    uint8_t b = 0;
    uint8_t a = 255;
};

// ---------------------------------------------------------------------------
// SumoClient
//
// Minimal TraCI binary-protocol client for ns-3/SUMO co-simulation.
// Speaks the TraCI protocol over a raw TCP socket with no dependency on
// any SUMO source files. Compatible with SUMO 1.8 and later.
//
// Ownership model: non-copyable (a socket cannot be shared between two
// owners), not declared moveable (the object always lives as a member of
// SumoManager for the entire simulation run).
// ---------------------------------------------------------------------------
class SumoClient {
public:
    SumoClient();
    ~SumoClient(); // sends CMD_CLOSE to SUMO and closes the socket

    // Copying is disabled: two SumoClient objects must never share the same
    // file descriptor — the second destructor would call ::close() on an fd
    // already closed, causing undefined behaviour.
    SumoClient(const SumoClient&) = delete;
    SumoClient& operator=(const SumoClient&) = delete;

    // -----------------------------------------------------------------------
    // Connection management
    // -----------------------------------------------------------------------

    // Connect to SUMO and perform the TraCI version handshake.
    // Takes const std::string& instead of string_view because getaddrinfo()
    // internally requires a null-terminated C-string via .c_str().
    bool Connect(const std::string& host, int port);

    // Send CMD_CLOSE, read the acknowledgement, then close the socket.
    // Safe to call on an already-closed client.
    void Close();

    // Returns true when the underlying socket file descriptor is open.
    bool IsConnected() const { return m_socket.valid(); }

    // -----------------------------------------------------------------------
    // Simulation control
    // -----------------------------------------------------------------------

    // Advance the SUMO simulation to targetTime (seconds).
    // [[nodiscard]]: ignoring the return value silently skips error handling
    // and leaves ns-3 and SUMO out of sync.
    bool SimStep(double targetTime);

    // IDs of all vehicles currently active in SUMO.
    std::vector<std::string> GetVehicleIds();

    // IDs of vehicles that entered the simulation during the last step.
    std::vector<std::string> GetDepartedVehicleIds();

    // IDs of vehicles that completed their route during the last step.
    // Does NOT cover teleportation or rerouting failures — use GetVehicleIds()
    // for authoritative reconciliation.
    std::vector<std::string> GetArrivedVehicleIds();

    // -----------------------------------------------------------------------
    // Vehicle state retrieval (GET)
    // -----------------------------------------------------------------------

    // Retrieve the full vehicle state in a single pipelined request.
    // All 7 variable queries are sent back-to-back before any reply is read,
    // reducing 7 TCP round-trips to 1. Returns a zero/empty VehicleState on error.
    // std::string_view: the ID is forwarded only to PutString(); no .c_str() needed.
    VehicleState GetVehicleState(std::string_view vehicleId);

    double GetVehicleAngle(std::string_view vehicleId);
    double GetVehicleAcceleration(std::string_view vehicleId);
    std::string GetVehicleRoadId(std::string_view vehicleId);
    std::string GetVehicleLaneId(std::string_view vehicleId);
    double GetVehicleLanePosition(std::string_view vehicleId);

    // -----------------------------------------------------------------------
    // Vehicle control (SET, ns-3 → SUMO)
    // -----------------------------------------------------------------------

    // Instantly override the vehicle speed.
    // Pass -1.0 to restore autonomous SUMO control.
    bool SetVehicleSpeed(std::string_view vehicleId, double speed);

    // Gradually reduce speed to `speed` m/s over `duration` seconds (CMD_SLOWDOWN).
    // More realistic than SetVehicleSpeed: SUMO linearly interpolates the deceleration.
    bool SetVehicleSlowDown(std::string_view vehicleId, double speed, double duration);

    // Change the SUMO-GUI colour of a vehicle (useful for visualising state).
    bool SetVehicleColor(std::string_view vehicleId, const TraCIColor& color);

private:

    // -----------------------------------------------------------------------
    // Socket — RAII wrapper for a POSIX file descriptor
    //
    // Guarantees ::close() is called exactly once regardless of exceptions
    // or early returns. Move-only: ownership transfers cleanly from a local
    // Socket (created in Connect) to the member m_socket.
    // -----------------------------------------------------------------------
    class Socket {
    public:
        // Default-constructed socket is in the "invalid / not open" state.
        Socket() noexcept = default;

        // Adopt an existing file descriptor; explicit prevents accidental
        // implicit conversion from a plain int.
        explicit Socket(int fd) noexcept : m_fd{fd} {}

        // Destructor guarantees ::close() is called if the fd is valid.
        ~Socket() noexcept { reset(); }

        // Move constructor: transfer ownership. std::exchange atomically reads
        // o.m_fd and replaces it with k_invalid, so only one object will ever
        // call ::close() on the descriptor.
        Socket(Socket&& o) noexcept : m_fd{std::exchange(o.m_fd, k_invalid)} {}

        // Move assignment: close any fd we currently own, then steal from o.
        // The self-assignment guard (this != &o) prevents reset() from closing
        // the fd before we can read it from o.
        Socket& operator=(Socket&& o) noexcept {
            if (this != &o) { reset(); m_fd = std::exchange(o.m_fd, k_invalid); }
            return *this;
        }

        // Copying a file descriptor would create two owners: both destructors
        // would call ::close(), and the second call is undefined behaviour.
        Socket(const Socket&) = delete;
        Socket& operator=(const Socket&) = delete;

        // Close the fd and mark the socket invalid. Defined in the .cc file
        // to keep ::close() (from <unistd.h>) out of this header.
        void reset() noexcept;

        int fd() const noexcept { return m_fd; }
        bool valid() const noexcept { return m_fd != k_invalid; }

    private:
        static constexpr int k_invalid = -1; // POSIX sentinel for "no fd"
        int m_fd{k_invalid};
    };

    // -----------------------------------------------------------------------
    // TraCICmd — internal representation of one parsed TraCI command
    // -----------------------------------------------------------------------
    struct TraCICmd {
        uint8_t id{}; // command / response code
        std::vector<uint8_t> data; // payload bytes (excludes the id byte)
    };

    // -----------------------------------------------------------------------
    // TraCI protocol constants
    // Reference: https://sumo.dlr.de/docs/TraCI/Protocol.html
    // -----------------------------------------------------------------------

    // Commands sent by the client (ns-3 → SUMO).
    static constexpr uint8_t CMD_GETVERSION = 0x00; // version handshake
    static constexpr uint8_t CMD_SIMSTEP = 0x02; // advance simulation
    static constexpr uint8_t CMD_CLOSE = 0x7F; // end session
    static constexpr uint8_t CMD_GET_VEHICLE_VAR = 0xa4; // read vehicle variable
    static constexpr uint8_t CMD_SET_VEHICLE_VAR = 0xc4; // write vehicle variable
    static constexpr uint8_t CMD_GET_SIM_VAR = 0xab; // read simulation variable

    // Response codes: each response ID = command ID | 0x10.
    static constexpr uint8_t RESP_GET_VEHICLE_VAR = 0xb4;
    static constexpr uint8_t RESP_GET_SIM_VAR = 0xbb;

    // GET variable IDs — which property to query.
    static constexpr uint8_t VAR_ID_LIST = 0x00; // list of all active IDs
    static constexpr uint8_t VAR_SPEED = 0x40;
    static constexpr uint8_t VAR_POSITION = 0x42; // 2D Cartesian position
    static constexpr uint8_t VAR_ANGLE = 0x43;
    static constexpr uint8_t VAR_ROAD_ID = 0x50;
    static constexpr uint8_t VAR_LANE_ID = 0x51;
    static constexpr uint8_t VAR_LANE_POS = 0x56;
    static constexpr uint8_t VAR_ACCELERATION = 0x72;

    // SET variable / sub-command IDs.
    static constexpr uint8_t VAR_COLOR = 0x45; // set vehicle colour (RGBA)
    static constexpr uint8_t CMD_SLOWDOWN = 0x14; // gradual speed reduction

    // Simulation-level variable IDs (used with CMD_GET_SIM_VAR).
    static constexpr uint8_t VAR_DEPARTED_IDS = 0x74; // vehicles that entered this step
    static constexpr uint8_t VAR_ARRIVED_IDS = 0x79; // vehicles that left this step

    // Payload type tags — declare the data type of the following bytes.
    static constexpr uint8_t TYPE_DOUBLE = 0x0b;
    static constexpr uint8_t TYPE_STRINGLIST = 0x0e;
    static constexpr uint8_t TYPE_POSITION2D = 0x01;
    static constexpr uint8_t TYPE_STRING = 0x0c;
    static constexpr uint8_t TYPE_COLOR = 0x11;
    static constexpr uint8_t TYPE_COMPOUND = 0x0f; // heterogeneous container

    // Status code returned by SUMO in every acknowledgement response.
    static constexpr uint8_t RTYPE_OK = 0x00;

    // -----------------------------------------------------------------------
    // Transport layer — signatures unchanged from the original
    // -----------------------------------------------------------------------

    // Build the TraCI framing (short or extended form) and send over TCP.
    bool SendCmd(uint8_t cmdId, const std::vector<uint8_t>& payload);

    // Read one complete TraCI response message (header + body).
    // Returns an empty vector on connection error.
    std::vector<uint8_t> RecvResponse();

    // Split a raw response buffer into individual TraCI commands.
    // Static: takes no state from the object, pure transformation.
    static std::vector<TraCICmd> ParseCommands(const std::vector<uint8_t>& buf);

    // Low-level helpers: loop until all bytes are sent / received because
    // a single send()/recv() call may transfer fewer bytes than requested.
    bool SendBytes(const void* buf, size_t len);
    bool RecvBytes(void* buf, size_t len);

    // -----------------------------------------------------------------------
    // Serialisers — append C++ values to a byte buffer in big-endian order
    // -----------------------------------------------------------------------
    static void PutInt32(std::vector<uint8_t>& b, int32_t v);
    static void PutDouble(std::vector<uint8_t>& b, double v);
    // string_view accepted here: PutString only iterates, never calls .c_str().
    static void PutString(std::vector<uint8_t>& b, std::string_view s);

    // -----------------------------------------------------------------------
    // Deserialisers — read C++ values from a buffer, advancing `pos` each time.
    // Throw std::out_of_range on buffer underflow so callers catch one type.
    // -----------------------------------------------------------------------
    static int32_t GetInt32(const std::vector<uint8_t>& b, size_t& pos);
    static double GetDouble(const std::vector<uint8_t>& b, size_t& pos);
    static std::string GetString(const std::vector<uint8_t>& b, size_t& pos);

    // -----------------------------------------------------------------------
    // Shared helpers — reduce repetition across individual getters
    // -----------------------------------------------------------------------

    // Generic list-of-strings GET: used by GetVehicleIds, GetDeparted/ArrivedIds.
    std::vector<std::string> GetStringListVar(uint8_t domain, uint8_t respId, uint8_t varId);

    // Generic single-value GET helpers; string_view forwarded only to PutString.
    double GetDoubleVar(uint8_t varId, std::string_view vehicleId);
    std::string GetStringVar(uint8_t varId, std::string_view vehicleId);

    // -----------------------------------------------------------------------
    // Data members
    // -----------------------------------------------------------------------
    Socket m_socket; // RAII-managed TCP socket; default-constructed = invalid
};

} // namespace ns3