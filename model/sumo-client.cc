#include "sumo-client.h"

#include <sys/socket.h> // socket(), connect(), send(), recv()
#include <netinet/in.h> // sockaddr_in, htons()
#include <netdb.h>      // gethostbyname()
#include <unistd.h>     // close()
#include <cstring>      // memcpy()
#include <iostream>     // std::cerr
#include <stdexcept>    // std::out_of_range

namespace ns3 {

SumoClient::SumoClient() : m_socket(-1) {}
SumoClient::~SumoClient() { Close(); }

// *******************************************************************************************************************
// Serializers: convert C++ types to big-endian byte sequences.
// TraCI uses network byte order (big-endian) for all multi-byte values.

void SumoClient::PutInt32(std::vector<uint8_t>& b, int32_t v) {
    b.push_back((v >> 24) & 0xFF);
    b.push_back((v >> 16) & 0xFF);
    b.push_back((v >>  8) & 0xFF);
    b.push_back((v      ) & 0xFF);
}

void SumoClient::PutDouble(std::vector<uint8_t>& b, double v) {
    // Copy the bit pattern into a uint64_t to allow bitwise shifting.
    uint64_t bits;
    memcpy(&bits, &v, 8);
    for (int i = 7; i >= 0; --i)
        b.push_back((bits >> (i * 8)) & 0xFF);
}

void SumoClient::PutString(std::vector<uint8_t>& b, const std::string& s) {
    // TraCI string format: 4-byte length followed by ASCII characters.
    PutInt32(b, static_cast<int32_t>(s.size()));
    b.insert(b.end(), s.begin(), s.end());
}

// *******************************************************************************************************************
// Deserializers: read bytes from a buffer and reconstruct C++ types.
// pos is a cursor passed by reference; each call advances it by the number
// of bytes consumed.  All functions throw std::out_of_range on underflow so
// callers can catch a single exception type rather than check every read.

int32_t SumoClient::GetInt32(const std::vector<uint8_t>& b, size_t& pos) {
    if (pos + 4 > b.size())
        throw std::out_of_range("TraCI: buffer underflow reading int32");
    int32_t v = (int32_t(b[pos    ]) << 24)
              | (int32_t(b[pos + 1]) << 16)
              | (int32_t(b[pos + 2]) <<  8)
              |  int32_t(b[pos + 3]);
    pos += 4;
    return v;
}

double SumoClient::GetDouble(const std::vector<uint8_t>& b, size_t& pos) {
    if (pos + 8 > b.size())
        throw std::out_of_range("TraCI: buffer underflow reading double");
    uint64_t bits = 0;
    for (int i = 0; i < 8; ++i)
        bits = (bits << 8) | b[pos + i];
    pos += 8;
    double v;
    memcpy(&v, &bits, 8);
    return v;
}

std::string SumoClient::GetString(const std::vector<uint8_t>& b, size_t& pos) {
    int32_t len = GetInt32(b, pos); // may throw
    if (len < 0)
        throw std::out_of_range("TraCI: negative string length");
    if (pos + static_cast<size_t>(len) > b.size())
        throw std::out_of_range("TraCI: buffer underflow reading string");
    std::string s(b.begin() + pos, b.begin() + pos + len);
    pos += static_cast<size_t>(len);
    return s;
}

// *******************************************************************************************************************
// TCP layer: guarantee that all requested bytes are sent or received.
// A single send()/recv() call may transfer fewer bytes than requested.

bool SumoClient::SendBytes(const void* buf, size_t len) {
    const uint8_t* p = static_cast<const uint8_t*>(buf);
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(m_socket, p + sent, len - sent, 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool SumoClient::RecvBytes(void* buf, size_t len) {
    uint8_t* p = static_cast<uint8_t*>(buf);
    size_t received = 0;
    while (received < len) {
        ssize_t n = ::recv(m_socket, p + received, len - received, 0);
        if (n <= 0) return false;
        received += static_cast<size_t>(n);
    }
    return true;
}

// *******************************************************************************************************************
// TraCI message layer.

bool SumoClient::SendCmd(uint8_t cmdId, const std::vector<uint8_t>& payload) {
    // Message layout:
    //   [4 bytes : total message length (including this header)]
    //   Short form  (cmdLen < 256): [1 byte  cmdLen][1 byte cmdId][payload]
    //   Extended form:              [0x00][4 bytes extLen][1 byte cmdId][payload]
    //
    // In the short form,  cmdLen  = 1 (itself) + 1 (cmdId) + payload.size()
    // In the extended form, extLen = 4 (itself) + 1 (cmdId) + payload.size()
    //                       and there is an extra 0x00 marker byte before extLen.

    std::vector<uint8_t> msg;

    uint32_t shortLen = 2u + static_cast<uint32_t>(payload.size()); // 1 + 1 + N

    if (shortLen < 256u) {
        // Short form: totalMsg = 4 (header) + shortLen
        PutInt32(msg, static_cast<int32_t>(4u + shortLen));
        msg.push_back(static_cast<uint8_t>(shortLen));
    } 
    else {
        // Extended form: extLen = 4 + 1 + N; wire = 0x00 + extLen bytes
        uint32_t extLen = 5u + static_cast<uint32_t>(payload.size()); // 4 + 1 + N
        // totalMsg = 4 (header) + 1 (0x00 marker) + extLen
        PutInt32(msg, static_cast<int32_t>(4u + 1u + extLen));
        msg.push_back(0x00);
        PutInt32(msg, static_cast<int32_t>(extLen));
    }

    msg.push_back(cmdId);
    msg.insert(msg.end(), payload.begin(), payload.end());
    return SendBytes(msg.data(), msg.size());
}

std::vector<uint8_t> SumoClient::RecvResponse() {
    uint8_t lenBuf[4];
    if (!RecvBytes(lenBuf, 4)) return {};
    int32_t totalLen = (int32_t(lenBuf[0]) << 24)
                     | (int32_t(lenBuf[1]) << 16)
                     | (int32_t(lenBuf[2]) <<  8)
                     |  int32_t(lenBuf[3]);
    if (totalLen <= 4) return {};
    std::vector<uint8_t> buf(static_cast<size_t>(totalLen - 4));
    if (!RecvBytes(buf.data(), buf.size())) return {};
    return buf;
}

std::vector<SumoClient::TraCICmd> SumoClient::ParseCommands(const std::vector<uint8_t>& buf) {
    // A single response message can contain multiple commands concatenated.
    std::vector<TraCICmd> result;
    size_t pos = 0;

    while (pos < buf.size()) {
        size_t cmdLen, hdrSize;

        if (buf[pos] == 0 && pos + 5 <= buf.size()) {
            // Extended length: 0x00 marker followed by a 4-byte length.
            cmdLen  = (size_t(buf[pos + 1]) << 24) | (size_t(buf[pos + 2]) << 16)
                    | (size_t(buf[pos + 3]) <<  8) |  size_t(buf[pos + 4]);
            hdrSize = 5;
        } else {
            cmdLen  = buf[pos];
            hdrSize = 1;
        }

        // Guard against malformed messages.
        if (cmdLen < hdrSize + 1 || pos + cmdLen > buf.size()) break;

        TraCICmd cmd;
        cmd.id   = buf[pos + hdrSize];
        cmd.data = std::vector<uint8_t>(buf.begin() + pos + hdrSize + 1,
                                        buf.begin() + pos + cmdLen);
        result.push_back(std::move(cmd));
        pos += cmdLen;
    }
    return result;
}

// *******************************************************************************************************************
// Public API.

bool SumoClient::Connect(const std::string& host, int port) {
    m_socket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (m_socket < 0) {
        std::cerr << "SumoClient: cannot create socket\n";
        return false;
    }

    struct hostent* he = gethostbyname(host.c_str());
    if (!he) {
        std::cerr << "SumoClient: host not found: " << host << "\n";
        Close();
        return false;
    }

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(port));
    memcpy(&addr.sin_addr, he->h_addr_list[0], static_cast<size_t>(he->h_length));

    if (::connect(m_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "SumoClient: connection refused on " << host << ":" << port << "\n";
        Close();
        return false;
    }

    // TraCI handshake: send CMD_GETVERSION and wait for any reply.
    if (!SendCmd(CMD_GETVERSION, {})) { Close(); return false; }
    if (RecvResponse().empty())       { Close(); return false; }
    return true;
}

void SumoClient::Close() {
    if (m_socket >= 0) {
        SendCmd(CMD_CLOSE, {});
        RecvResponse();
        ::close(m_socket);
        m_socket = -1;
    }
}

bool SumoClient::SimStep(double targetTime) {
    std::vector<uint8_t> payload;
    PutDouble(payload, targetTime);
    if (!SendCmd(CMD_SIMSTEP, payload)) return false;
    auto resp = RecvResponse();
    if (resp.empty()) return false;
    for (const auto& cmd : ParseCommands(resp)) {
        if (cmd.id == CMD_SIMSTEP && !cmd.data.empty())
            return cmd.data[0] == RTYPE_OK;
    }
    return false;
}

// *******************************************************************************************************************
// Private helper shared by GetVehicleIds / GetDepartedVehicleIds / GetArrivedVehicleIds.

std::vector<std::string> SumoClient::GetStringListVar(uint8_t domain, uint8_t respId, uint8_t varId) {
    std::vector<uint8_t> payload{ varId };
    PutString(payload, "");
    if (!SendCmd(domain, payload)) return {};
    auto resp = RecvResponse();
    if (resp.empty()) return {};

    for (const auto& cmd : ParseCommands(resp)) {
        if (cmd.id != respId || cmd.data.empty()) continue;
        try {
            size_t pos = 0;
            if (cmd.data[pos++] != varId) continue;
            GetString(cmd.data, pos);                        // skip echo object ID
            if (cmd.data[pos++] != TYPE_STRINGLIST) continue;
            int32_t count = GetInt32(cmd.data, pos);
            if (count < 0) continue;
            std::vector<std::string> ids;
            ids.reserve(static_cast<size_t>(count));
            for (int32_t i = 0; i < count; ++i)
                ids.push_back(GetString(cmd.data, pos));
            return ids;
        } catch (const std::out_of_range& e) {
            std::cerr << "SumoClient: malformed string-list response — " << e.what() << "\n";
        }
    }
    return {};
}

std::vector<std::string> SumoClient::GetVehicleIds() {
    return GetStringListVar(CMD_GET_VEHICLE_VAR, RESP_GET_VEHICLE_VAR, VAR_ID_LIST);
}

std::vector<std::string> SumoClient::GetDepartedVehicleIds() {
    return GetStringListVar(CMD_GET_SIM_VAR, RESP_GET_SIM_VAR, VAR_DEPARTED_IDS);
}

std::vector<std::string> SumoClient::GetArrivedVehicleIds() {
    return GetStringListVar(CMD_GET_SIM_VAR, RESP_GET_SIM_VAR, VAR_ARRIVED_IDS);
}

// *******************************************************************************************************************

VehicleState SumoClient::GetVehicleState(const std::string& vid) {
    // Send both queries back-to-back before reading any reply.
    // This halves the number of TCP round-trips compared to sequential
    // request/reply pairs.

    std::vector<uint8_t> posPayload{ VAR_POSITION };
    PutString(posPayload, vid);

    std::vector<uint8_t> spdPayload{ VAR_SPEED };
    PutString(spdPayload, vid);

    if (!SendCmd(CMD_GET_VEHICLE_VAR, posPayload)) return {};
    if (!SendCmd(CMD_GET_VEHICLE_VAR, spdPayload)) return {};

    VehicleState state{};
    for (int req = 0; req < 2; ++req) {
        auto resp = RecvResponse();
        if (resp.empty()) continue;
        for (const auto& cmd : ParseCommands(resp)) {
            if (cmd.id != RESP_GET_VEHICLE_VAR || cmd.data.empty()) continue;
            try {
                size_t pos  = 0;
                uint8_t var = cmd.data[pos++];
                GetString(cmd.data, pos);               // skip echo object ID
                uint8_t tag = cmd.data[pos++];
                if (var == VAR_POSITION && tag == TYPE_POSITION2D) {
                    state.x = GetDouble(cmd.data, pos);
                    state.y = GetDouble(cmd.data, pos);
                } else if (var == VAR_SPEED && tag == TYPE_DOUBLE) {
                    state.speed = GetDouble(cmd.data, pos);
                }
            } catch (const std::out_of_range& e) {
                std::cerr << "SumoClient: malformed vehicle-state response — " << e.what() << "\n";
            }
        }
    }
    return state;
}

bool SumoClient::SetVehicleSpeed(const std::string& vid, double speed) {
    // SET commands require: variable ID, object ID, type tag, value.
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

} // namespace ns3