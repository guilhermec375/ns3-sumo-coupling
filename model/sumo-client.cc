#include "sumo-client.h"

// POSIX networking headers — the only C-style dependency in the module.
// They are confined to this .cc file so the header stays clean.
#include <sys/socket.h> // ::socket(), ::connect(), ::send(), ::recv()
#include <netinet/in.h> // AF_INET, SOCK_STREAM
#include <netdb.h> // ::getaddrinfo(), ::freeaddrinfo(), struct addrinfo
#include <unistd.h> // ::close()

#include <bit> // std::bit_cast — type-safe bit-pattern reinterpretation
#include <array> // std::array — fixed-size stack buffer with zero-init
#include <iostream> // std::cerr
#include <stdexcept> // std::out_of_range
#include <utility> // std::exchange — used in Socket move operations

namespace ns3 {

// ===========================================================================
// Lifetime
// ===========================================================================

SumoClient::SumoClient() {}
// m_socket default-constructs to Socket() which sets m_fd = k_invalid (-1).
// No explicit initialisation needed here.

SumoClient::~SumoClient() {
    // Send CMD_CLOSE to SUMO before destroying the object so that the
    // simulation on the SUMO side is cleanly terminated.
    Close();
}

// ===========================================================================
// Socket RAII — implementation in .cc to keep ::close() out of the header
// ===========================================================================

void SumoClient::Socket::reset() noexcept {
    if (m_fd != k_invalid) {
        // ::close() — the :: prefix calls the global POSIX function, not any
        // locally scoped overload that might exist in the ns3 namespace.
        ::close(m_fd);
        // Mark as invalid BEFORE returning so that if reset() is somehow
        // called again (e.g. from the destructor after an explicit reset()),
        // the second ::close(-1) is avoided.
        m_fd = k_invalid;
    }
}

// ===========================================================================
// Serialisers
// TraCI uses network byte order (big-endian): most-significant byte first.
// ===========================================================================

// Append a 32-bit signed integer as 4 big-endian bytes.
void SumoClient::PutInt32(std::vector<uint8_t>& b, int32_t v) {
    // Right-shift isolates each 8-bit group; the AND mask discards higher bits
    // after the implicit int promotion that happens before the cast.
    b.push_back(static_cast<uint8_t>((v >> 24) & 0xFF)); // most significant byte
    b.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    b.push_back(static_cast<uint8_t>(v & 0xFF)); // least significant byte
}

// Append a 64-bit IEEE 754 double as 8 big-endian bytes.
void SumoClient::PutDouble(std::vector<uint8_t>& b, double v) {
    // std::bit_cast reinterprets the bit pattern of `v` as uint64_t without
    // any conversion or undefined behaviour.  It replaces the old idiom:
    //   uint64_t bits; memcpy(&bits, &v, 8);
    // Both produce identical machine code; std::bit_cast expresses intent clearly.
    const auto bits = std::bit_cast<uint64_t>(v);
    for (int i = 7; i >= 0; --i) // byte 7 = MSB, byte 0 = LSB
        b.push_back(static_cast<uint8_t>((bits >> (i * 8)) & 0xFF));
}

// Append a TraCI-encoded string: 4-byte length prefix followed by raw bytes.
// Accepts string_view: only .begin(), .end(), and .size() are used, so
// there is no need for a null terminator (no .c_str() call).
void SumoClient::PutString(std::vector<uint8_t>& b, std::string_view s) {
    PutInt32(b, static_cast<int32_t>(s.size()));
    b.insert(b.end(), s.begin(), s.end());
}

// ===========================================================================
// Deserialisers
// `pos` is a cursor that advances by the number of bytes consumed.
// std::out_of_range is thrown on any buffer underflow so callers need to
// catch only one exception type.
// ===========================================================================

int32_t SumoClient::GetInt32(const std::vector<uint8_t>& b, size_t& pos) {
    if (pos + 4 > b.size())
        throw std::out_of_range("TraCI: buffer underflow reading int32");
    // Reassemble the 4 bytes into one 32-bit value.
    // Each byte is cast to int32_t before shifting to avoid undefined behaviour
    // from shifting a uint8_t (which is promoted to int, not int32_t).
    int32_t v = (int32_t(b[pos]) << 24)
              | (int32_t(b[pos + 1]) << 16)
              | (int32_t(b[pos + 2]) << 8)
              | int32_t(b[pos + 3]);
    pos += 4; // advance cursor so the next call reads the following field
    return v;
}

double SumoClient::GetDouble(const std::vector<uint8_t>& b, size_t& pos) {
    if (pos + 8 > b.size())
        throw std::out_of_range("TraCI: buffer underflow reading double");
    // Reassemble 8 bytes into a uint64_t, then reinterpret as double.
    uint64_t bits = 0;
    for (int i = 0; i < 8; ++i)
        bits = (bits << 8) | b[pos + i];
    pos += 8;
    return std::bit_cast<double>(bits); // inverse of PutDouble's bit_cast
}

std::string SumoClient::GetString(const std::vector<uint8_t>& b, size_t& pos) {
    int32_t len = GetInt32(b, pos); // reads the 4-byte length prefix
    if (len < 0)
        throw std::out_of_range("TraCI: negative string length");
    // Check BEFORE casting to size_t: a negative len would become a huge
    // positive value after the cast, bypassing the bounds check below.
    if (pos + static_cast<size_t>(len) > b.size())
        throw std::out_of_range("TraCI: buffer underflow reading string");
    std::string s(b.begin() + pos, b.begin() + pos + len);
    pos += static_cast<size_t>(len);
    return s;
}

// ===========================================================================
// Transport
// Both send and receive loop until ALL requested bytes have been transferred
// because a single send()/recv() call is allowed to transfer fewer bytes
// than requested (known as a "partial send" or "short read").
// ===========================================================================

bool SumoClient::SendBytes(const void* buf, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(buf);
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(m_socket.fd(), p + sent, len - sent, 0);
        if (n <= 0) return false; // n == 0: peer closed; n < 0: error
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool SumoClient::RecvBytes(void* buf, size_t len) {
    uint8_t* p = static_cast<uint8_t*>(buf);
    size_t received = 0;
    while (received < len) {
        ssize_t n = ::recv(m_socket.fd(), p + received, len - received, 0);
        if (n <= 0) return false;
        received += static_cast<size_t>(n);
    }
    return true;
}

// ===========================================================================
// TraCI message framing
// ===========================================================================

// Build and send one TraCI command message.
// The protocol supports two framing variants depending on payload size:
//
//  Short form  (shortLen < 256):
//    [4B total length][1B shortLen][1B cmdId][payload bytes]
//    where shortLen = 1 (own length byte) + 1 (cmdId) + N (payload)
//
//  Extended form (shortLen >= 256):
//    [4B total length][0x00 marker][4B extLen][1B cmdId][payload bytes]
//    where extLen = 4 (own length field) + 1 (cmdId) + N (payload)
bool SumoClient::SendCmd(uint8_t cmdId, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> msg;
    uint32_t shortLen = 2u + static_cast<uint32_t>(payload.size());

    if (shortLen < 256u) {
        // Short form: total wire size = 4 (header) + shortLen bytes.
        PutInt32(msg, static_cast<int32_t>(4u + shortLen));
        msg.push_back(static_cast<uint8_t>(shortLen));
    } else {
        // Extended form: 0x00 signals SUMO to read a 4-byte length next.
        uint32_t extLen = 5u + static_cast<uint32_t>(payload.size());
        PutInt32(msg, static_cast<int32_t>(4u + 1u + extLen));
        msg.push_back(0x00); // extended-length marker byte
        PutInt32(msg, static_cast<int32_t>(extLen));
    }
    msg.push_back(cmdId);
    msg.insert(msg.end(), payload.begin(), payload.end());
    return SendBytes(msg.data(), msg.size());
}

// Read one complete TraCI response message from the socket.
// Every message begins with a 4-byte total length field.
// Returns an empty vector on any receive error.
std::vector<uint8_t> SumoClient::RecvResponse() {
    // std::array instead of uint8_t[4]:
    //   - {} value-initialises all bytes to 0 (C arrays are not zero-initialised)
    //   - .data() and .size() avoid passing pointer + length separately
    std::array<uint8_t, 4> lenBuf{};
    if (!RecvBytes(lenBuf.data(), lenBuf.size())) return {};

    // Parse the 4-byte header to learn how many payload bytes follow.
    size_t pos = 0;
    int32_t totalLen = GetInt32(
        std::vector<uint8_t>(lenBuf.begin(), lenBuf.end()), pos);
    if (totalLen <= 4) return {}; // header only, no payload

    std::vector<uint8_t> buf(static_cast<size_t>(totalLen - 4));
    if (!RecvBytes(buf.data(), buf.size())) return {};
    return buf;
}

// Split a flat response buffer into individual TraCI commands.
// A single response may contain multiple commands concatenated end-to-end.
// Static: no object state is needed — pure buffer transformation.
std::vector<SumoClient::TraCICmd> SumoClient::ParseCommands(const std::vector<uint8_t>& buf) {
    std::vector<TraCICmd> result;
    size_t pos = 0;

    while (pos < buf.size()) {
        size_t cmdLen, hdrSize;

        if (buf[pos] == 0 && pos + 5 <= buf.size()) {
            // Extended form: 0x00 marker + 4-byte length.
            cmdLen = (size_t(buf[pos + 1]) << 24) | (size_t(buf[pos + 2]) << 16)
                   | (size_t(buf[pos + 3]) << 8) | size_t(buf[pos + 4]);
            hdrSize = 5; // 1 marker + 4 length bytes
        } else {
            // Short form: first byte IS the command length.
            cmdLen = buf[pos];
            hdrSize = 1;
        }

        // Guard against malformed or truncated messages.
        if (cmdLen < hdrSize + 1 || pos + cmdLen > buf.size()) break;

        TraCICmd cmd;
        cmd.id = buf[pos + hdrSize]; // byte immediately after the header
        cmd.data = std::vector<uint8_t>( // everything after the id byte
            buf.begin() + pos + hdrSize + 1,
            buf.begin() + pos + cmdLen);
        result.push_back(std::move(cmd)); // move avoids copying cmd.data
        pos += cmdLen;
    }
    return result;
}

// ===========================================================================
// Connection management
// ===========================================================================

bool SumoClient::Connect(const std::string& host, int port) {
    // Use getaddrinfo() instead of the deprecated gethostbyname():
    //   - thread-safe (gethostbyname uses a static internal buffer)
    //   - supports both IPv4 and IPv6
    //   - resolves host + port in one call
    // freeaddrinfo() is called manually before every return because the
    // function has only a few exit paths and no operation between getaddrinfo
    // and freeaddrinfo can throw a C++ exception.
    addrinfo hints{}; // zero-initialise: all fields null / 0
    hints.ai_family = AF_INET; // IPv4 only
    hints.ai_socktype = SOCK_STREAM; // TCP

    addrinfo* info = nullptr;
    const std::string portStr = std::to_string(port); // getaddrinfo needs C-string
    if (::getaddrinfo(host.c_str(), portStr.c_str(), &hints, &info) != 0) {
        std::cerr << "SumoClient: host not found: " << host << '\n';
        return false; // info is null here, no freeaddrinfo needed
    }

    // Wrap the new socket fd immediately in RAII so that ::close() is
    // guaranteed even if we return early from the checks below.
    Socket sock{::socket(info->ai_family, info->ai_socktype, info->ai_protocol)};
    if (!sock.valid()) {
        std::cerr << "SumoClient: cannot create socket\n";
        ::freeaddrinfo(info);
        return false;
    }
    if (::connect(sock.fd(), info->ai_addr, info->ai_addrlen) < 0) {
        std::cerr << "SumoClient: connection refused on " << host << ':' << port << '\n';
        ::freeaddrinfo(info);
        return false; // sock destructor closes the fd automatically
    }
    ::freeaddrinfo(info); // release before any further error paths

    // Transfer ownership of the fd from the local Socket into the member.
    // After the move, sock.valid() == false; its destructor is a no-op.
    m_socket = std::move(sock);

    // TraCI handshake: send CMD_GETVERSION and discard the reply.
    // This confirms that SUMO is listening and speaking the right protocol.
    if (!SendCmd(CMD_GETVERSION, {})) { Close(); return false; }
    if (RecvResponse().empty()) { Close(); return false; }
    return true;
}

void SumoClient::Close() {
    if (m_socket.valid()) {
        // Inform SUMO that the TraCI session is ending.
        // Ignoring the return value here is intentional: we are cleaning up.
        SendCmd(CMD_CLOSE, {});
        RecvResponse();
        m_socket.reset(); // calls ::close() on the file descriptor
    }
}

// ===========================================================================
// Simulation control
// ===========================================================================

bool SumoClient::SimStep(double targetTime) {
    std::vector<uint8_t> payload;
    PutDouble(payload, targetTime); // encode target time as 8 bytes
    if (!SendCmd(CMD_SIMSTEP, payload)) return false;
    auto resp = RecvResponse();
    if (resp.empty()) return false;
    for (const auto& cmd : ParseCommands(resp)) {
        if (cmd.id == CMD_SIMSTEP && !cmd.data.empty())
            return cmd.data[0] == RTYPE_OK; // 0x00 = success
    }
    return false;
}

// ===========================================================================
// Shared helpers
// ===========================================================================

// Generic GET that returns a list of strings.
// Used by GetVehicleIds(), GetDepartedVehicleIds(), GetArrivedVehicleIds().
std::vector<std::string> SumoClient::GetStringListVar(uint8_t domain,
                                                       uint8_t respId,
                                                       uint8_t varId) {
    // TraCI GET request format: [varId][objectId as string].
    // An empty object ID means "query all" (the global list).
    std::vector<uint8_t> payload{varId};
    PutString(payload, "");
    if (!SendCmd(domain, payload)) return {};
    auto resp = RecvResponse();
    if (resp.empty()) return {};

    for (const auto& cmd : ParseCommands(resp)) {
        if (cmd.id != respId || cmd.data.empty()) continue;
        try {
            size_t pos = 0;
            if (cmd.data[pos++] != varId) continue; // sanity-check echoed varId
            GetString(cmd.data, pos); // skip the echoed object ID
            if (cmd.data[pos++] != TYPE_STRINGLIST) continue;
            int32_t count = GetInt32(cmd.data, pos);
            if (count < 0) continue;
            std::vector<std::string> ids;
            ids.reserve(static_cast<size_t>(count)); // pre-allocate to avoid rehashing
            for (int32_t i = 0; i < count; ++i)
                ids.push_back(GetString(cmd.data, pos));
            return ids;
        } catch (const std::out_of_range& e) {
            std::cerr << "SumoClient: malformed string-list response — " << e.what() << '\n';
        }
    }
    return {};
}

// Generic GET for a single double-valued variable.
// Sends one request and reads one response.
double SumoClient::GetDoubleVar(uint8_t varId, std::string_view vehicleId) {
    std::vector<uint8_t> payload{varId};
    PutString(payload, vehicleId); // string_view works here: PutString only iterates
    if (!SendCmd(CMD_GET_VEHICLE_VAR, payload)) return 0.0;
    auto resp = RecvResponse();
    if (resp.empty()) return 0.0;

    for (const auto& cmd : ParseCommands(resp)) {
        if (cmd.id != RESP_GET_VEHICLE_VAR || cmd.data.empty()) continue;
        try {
            size_t pos = 0;
            if (cmd.data[pos++] != varId) continue;
            GetString(cmd.data, pos); // skip echoed object ID
            if (cmd.data[pos++] != TYPE_DOUBLE) continue;
            return GetDouble(cmd.data, pos);
        } catch (const std::out_of_range& e) {
            std::cerr << "SumoClient: malformed double response (var=0x"
                      << std::hex << static_cast<int>(varId) << ") — " << e.what() << '\n';
        }
    }
    return 0.0;
}

// Generic GET for a single string-valued variable.
// Sends one request and reads one response.
std::string SumoClient::GetStringVar(uint8_t varId, std::string_view vehicleId) {
    std::vector<uint8_t> payload{ varId };
    PutString(payload, vehicleId);
    if (!SendCmd(CMD_GET_VEHICLE_VAR, payload)) return {};
    auto resp = RecvResponse();
    if (resp.empty()) return {};

    for (const auto& cmd : ParseCommands(resp)) {
        if (cmd.id != RESP_GET_VEHICLE_VAR || cmd.data.empty()) continue;
        try {
            size_t pos = 0;
            if (cmd.data[pos++] != varId) continue;
            GetString(cmd.data, pos); // skip echoed object ID
            if (cmd.data[pos++] != TYPE_STRING) continue;
            return GetString(cmd.data, pos);
        } catch (const std::out_of_range& e) {
            std::cerr << "SumoClient: malformed string response (var=0x"
                      << std::hex << static_cast<int>(varId) << ") — " << e.what() << '\n';
        }
    }
    return {};
}

// ===========================================================================
// Vehicle list queries
// ===========================================================================

std::vector<std::string> SumoClient::GetVehicleIds() {
    return GetStringListVar(CMD_GET_VEHICLE_VAR, RESP_GET_VEHICLE_VAR, VAR_ID_LIST);
}
std::vector<std::string> SumoClient::GetDepartedVehicleIds() {
    return GetStringListVar(CMD_GET_SIM_VAR, RESP_GET_SIM_VAR, VAR_DEPARTED_IDS);
}
std::vector<std::string> SumoClient::GetArrivedVehicleIds() {
    return GetStringListVar(CMD_GET_SIM_VAR, RESP_GET_SIM_VAR, VAR_ARRIVED_IDS);
}

// ===========================================================================
// Vehicle state — pipelined batch request
// ===========================================================================

// Send all 7 variable requests back-to-back before reading any reply.
// This is "pipelining": SUMO queues the queries and replies in order,
// so we pay only 1 TCP round-trip instead of 7.
VehicleState SumoClient::GetVehicleState(std::string_view vid) {
    const uint8_t vars[] = {
        VAR_POSITION, VAR_SPEED, VAR_ANGLE,
        VAR_ACCELERATION, VAR_ROAD_ID, VAR_LANE_ID, VAR_LANE_POS
    };
    constexpr int N = sizeof(vars) / sizeof(vars[0]); // = 7

    // --- Send phase: dispatch all queries without waiting for replies ---
    for (int i = 0; i < N; ++i) {
        std::vector<uint8_t> p{ vars[i] };
        PutString(p, vid);
        if (!SendCmd(CMD_GET_VEHICLE_VAR, p)) return {}; // zero-initialised on error
    }

    // --- Receive phase: collect and parse the N responses ---
    VehicleState state{};
    for (int req = 0; req < N; ++req) {
        auto resp = RecvResponse();
        if (resp.empty()) continue;
        for (const auto& cmd : ParseCommands(resp)) {
            if (cmd.id != RESP_GET_VEHICLE_VAR || cmd.data.empty()) continue;
            try {
                size_t pos = 0;
                uint8_t var = cmd.data[pos++]; // which variable is this reply for?
                GetString(cmd.data, pos); // skip echoed object ID
                uint8_t tag = cmd.data[pos++]; // data type tag

                // Dispatch on (var, tag) pair — only fill the matching field.
                if (var == VAR_POSITION && tag == TYPE_POSITION2D) {
                    state.x = GetDouble(cmd.data, pos);
                    state.y = GetDouble(cmd.data, pos);
                } else if (var == VAR_SPEED && tag == TYPE_DOUBLE) {
                    state.speed = GetDouble(cmd.data, pos);
                } else if (var == VAR_ANGLE && tag == TYPE_DOUBLE) {
                    state.angle = GetDouble(cmd.data, pos);
                } else if (var == VAR_ACCELERATION && tag == TYPE_DOUBLE) {
                    state.acceleration = GetDouble(cmd.data, pos);
                } else if (var == VAR_ROAD_ID && tag == TYPE_STRING) {
                    state.roadId = GetString(cmd.data, pos);
                } else if (var == VAR_LANE_ID && tag == TYPE_STRING) {
                    state.laneId = GetString(cmd.data, pos);
                } else if (var == VAR_LANE_POS && tag == TYPE_DOUBLE) {
                    state.lanePosition = GetDouble(cmd.data, pos);
                }
            } catch (const std::out_of_range& e) {
                std::cerr << "SumoClient: malformed vehicle-state response — "
                          << e.what() << '\n';
            }
        }
    }
    return state;
}

// ===========================================================================
// Individual getters — each delegates to the appropriate shared helper
// ===========================================================================

double SumoClient::GetVehicleAngle(std::string_view vid) { return GetDoubleVar(VAR_ANGLE, vid); }
double SumoClient::GetVehicleAcceleration(std::string_view vid) { return GetDoubleVar(VAR_ACCELERATION, vid); }
std::string SumoClient::GetVehicleRoadId(std::string_view vid) { return GetStringVar(VAR_ROAD_ID, vid); }
std::string SumoClient::GetVehicleLaneId(std::string_view vid) { return GetStringVar(VAR_LANE_ID, vid); }
double SumoClient::GetVehicleLanePosition(std::string_view vid) { return GetDoubleVar(VAR_LANE_POS, vid); }

// ===========================================================================
// Vehicle SET commands
// ===========================================================================

// SET request format: [varId][objectId][TYPE_tag][value bytes].
bool SumoClient::SetVehicleSpeed(std::string_view vid, double speed) {
    std::vector<uint8_t> payload{ VAR_SPEED };
    PutString(payload, vid);
    payload.push_back(TYPE_DOUBLE);
    PutDouble(payload, speed);
    if (!SendCmd(CMD_SET_VEHICLE_VAR, payload)) return false;
    for (const auto& cmd : ParseCommands(RecvResponse())) {
        if (cmd.id == CMD_SET_VEHICLE_VAR && !cmd.data.empty())
            return cmd.data[0] == RTYPE_OK;
    }
    return false;
}

// CMD_SLOWDOWN payload uses a TYPE_COMPOUND container with 2 elements
// (target speed and duration), telling SUMO to interpolate linearly.
bool SumoClient::SetVehicleSlowDown(std::string_view vid, double speed, double duration) {
    std::vector<uint8_t> payload{ CMD_SLOWDOWN };
    PutString(payload, vid);
    payload.push_back(TYPE_COMPOUND);
    PutInt32(payload, 2); // compound holds 2 typed elements
    payload.push_back(TYPE_DOUBLE);
    PutDouble(payload, speed);
    payload.push_back(TYPE_DOUBLE);
    PutDouble(payload, duration);
    if (!SendCmd(CMD_SET_VEHICLE_VAR, payload)) return false;
    for (const auto& cmd : ParseCommands(RecvResponse())) {
        if (cmd.id == CMD_SET_VEHICLE_VAR && !cmd.data.empty())
            return cmd.data[0] == RTYPE_OK;
    }
    return false;
}

// Colour is encoded as 4 consecutive bytes: R, G, B, A.
bool SumoClient::SetVehicleColor(std::string_view vid, const TraCIColor& color) {
    std::vector<uint8_t> payload{ VAR_COLOR };
    PutString(payload, vid);
    payload.push_back(TYPE_COLOR);
    payload.push_back(color.r);
    payload.push_back(color.g);
    payload.push_back(color.b);
    payload.push_back(color.a);
    if (!SendCmd(CMD_SET_VEHICLE_VAR, payload)) return false;
    for (const auto& cmd : ParseCommands(RecvResponse())) {
        if (cmd.id == CMD_SET_VEHICLE_VAR && !cmd.data.empty())
            return cmd.data[0] == RTYPE_OK;
    }
    return false;
}

} // namespace ns3