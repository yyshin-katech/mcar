/**
 * pcapng/pcap에서 UDP 페이로드를 추출하고 SPaT UPER 디코딩 후 JSON 출력
 *
 * 빌드: g++ -std=c++11 -o pcap_spat_decoder pcap_spat_decoder.cpp \
 *        ../src/ffasn1-j2735-2026-KSR1600.c \
 *        -I../include -L../lib/x86_64 -lffasn1-base -lffasn1-j2735-2020
 *
 * 사용: LD_LIBRARY_PATH=../lib/x86_64 ./pcap_spat_decoder sample.pcapng [max_packets] > result.json
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>

#include "ffasn1-j2735-2026-KSR1600.h"
#include "asn1defs.h"

#define OBU_HEADER_SIZE 5
#define J2735_MSG_ID_SPAT 19

// ---------- pcapng block types ----------
#define PCAPNG_SHB  0x0A0D0D0A
#define PCAPNG_IDB  0x00000001
#define PCAPNG_EPB  0x00000006
#define PCAPNG_SPB  0x00000003
// pcap magic
#define PCAP_MAGIC      0xA1B2C3D4
#define PCAP_MAGIC_NS   0xA1B23C4D
#define PCAP_MAGIC_SWAP 0xD4C3B2A1

struct PacketInfo {
    double timestamp;
    uint16_t src_port;
    std::vector<uint8_t> udp_payload;
};

// ---------- pcapng 읽기 ----------
static bool read_u32(FILE* f, uint32_t& v) { return fread(&v, 4, 1, f) == 1; }
static bool read_u16(FILE* f, uint16_t& v) { return fread(&v, 2, 1, f) == 1; }
static bool skip(FILE* f, long n) { return fseek(f, n, SEEK_CUR) == 0; }

static bool extract_udp_payload_from_eth(const uint8_t* data, size_t len, PacketInfo& pkt)
{
    // Ethernet (14) + IP (20+) + UDP (8)
    if (len < 42) return false;

    // Ethernet type = 0x0800 (IPv4)
    uint16_t eth_type = (data[12] << 8) | data[13];
    if (eth_type != 0x0800) return false;

    const uint8_t* ip = data + 14;
    uint8_t ip_hdr_len = (ip[0] & 0x0F) * 4;
    uint8_t protocol = ip[9];
    if (protocol != 17) return false;  // UDP

    const uint8_t* udp = ip + ip_hdr_len;
    pkt.src_port = (udp[0] << 8) | udp[1];
    uint16_t dst_port = (udp[2] << 8) | udp[3];
    uint16_t udp_len = (udp[4] << 8) | udp[5];

    if (dst_port != 9999) return false;

    const uint8_t* payload = udp + 8;
    size_t payload_len = udp_len - 8;
    if (payload + payload_len > data + len) return false;

    pkt.udp_payload.assign(payload, payload + payload_len);
    return true;
}

std::vector<PacketInfo> read_pcapng(const char* path, int max_packets)
{
    std::vector<PacketInfo> packets;
    FILE* f = fopen(path, "rb");
    if (!f) { perror("fopen"); return packets; }

    // Detect format: pcap or pcapng
    uint32_t magic;
    if (!read_u32(f, magic)) { fclose(f); return packets; }
    fseek(f, 0, SEEK_SET);

    if (magic == PCAP_MAGIC || magic == PCAP_MAGIC_NS || magic == PCAP_MAGIC_SWAP)
    {
        // Classic pcap format
        bool swap = (magic == PCAP_MAGIC_SWAP);
        fseek(f, 24, SEEK_SET);  // skip global header

        while ((int)packets.size() < max_packets)
        {
            uint32_t ts_sec, ts_usec, incl_len, orig_len;
            if (!read_u32(f, ts_sec)) break;
            read_u32(f, ts_usec);
            read_u32(f, incl_len);
            read_u32(f, orig_len);
            if (swap) {
                incl_len = __builtin_bswap32(incl_len);
                ts_sec = __builtin_bswap32(ts_sec);
                ts_usec = __builtin_bswap32(ts_usec);
            }

            std::vector<uint8_t> pkt_data(incl_len);
            if (fread(pkt_data.data(), 1, incl_len, f) != incl_len) break;

            PacketInfo pkt;
            pkt.timestamp = ts_sec + ts_usec / 1e6;
            if (extract_udp_payload_from_eth(pkt_data.data(), incl_len, pkt))
                packets.push_back(pkt);
        }
    }
    else
    {
        // pcapng format
        while ((int)packets.size() < max_packets)
        {
            long block_start = ftell(f);
            uint32_t block_type, block_len;
            if (!read_u32(f, block_type)) break;
            if (!read_u32(f, block_len)) break;

            if (block_len < 12) break;

            if (block_type == PCAPNG_EPB)
            {
                // Enhanced Packet Block
                uint32_t iface_id, ts_hi, ts_lo, cap_len, orig_len;
                read_u32(f, iface_id);
                read_u32(f, ts_hi);
                read_u32(f, ts_lo);
                read_u32(f, cap_len);
                read_u32(f, orig_len);

                std::vector<uint8_t> pkt_data(cap_len);
                if (fread(pkt_data.data(), 1, cap_len, f) != cap_len) break;

                uint64_t ts = ((uint64_t)ts_hi << 32) | ts_lo;

                PacketInfo pkt;
                pkt.timestamp = ts / 1e6;  // microseconds → seconds
                if (extract_udp_payload_from_eth(pkt_data.data(), cap_len, pkt))
                    packets.push_back(pkt);
            }

            // 다음 블록으로 이동
            fseek(f, block_start + block_len, SEEK_SET);
        }
    }

    fclose(f);
    return packets;
}

// ---------- WSMP 서브헤더 스킵 ----------
static size_t skip_wsmp(const uint8_t* body, size_t len)
{
    if (len < 4 || body[0] != 0x03 || body[1] != 0x80)
        return 0;
    size_t off = 2;
    uint8_t b = body[off];
    if (b < 0x80)       off += 1;
    else if (b == 0x81) off += 2;
    else if (b == 0x82) off += 3;
    else                off += 1;
    return off;
}

// ---------- JSON 출력 헬퍼 ----------
static const char* event_state_name(int s)
{
    switch (s) {
        case 0: return "unavailable";
        case 1: return "dark";
        case 2: return "stop-Then-Proceed";
        case 3: return "stop-And-Remain";
        case 4: return "pre-Movement";
        case 5: return "permissive-Movement-Allowed";
        case 6: return "protected-Movement-Allowed";
        case 7: return "permissive-clearance";
        case 8: return "protected-clearance";
        case 9: return "caution-Conflicting-Traffic";
        default: return "unknown";
    }
}

static std::string escape_json(const char* s, size_t len)
{
    std::string r;
    for (size_t i = 0; i < len; i++) {
        char c = s[i];
        if (c == '"') r += "\\\"";
        else if (c == '\\') r += "\\\\";
        else if (c >= 0x20) r += c;
    }
    return r;
}

static std::string hex_string(const uint8_t* data, size_t len)
{
    std::string r;
    char buf[4];
    for (size_t i = 0; i < len && i < 32; i++) {
        snprintf(buf, sizeof(buf), "%02x", data[i]);
        r += buf;
        if (i + 1 < len && i + 1 < 32) r += " ";
    }
    if (len > 32) r += "...";
    return r;
}

// ---------- メイン ----------
int main(int argc, char** argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <pcapng_file> [max_packets]\n", argv[0]);
        return 1;
    }

    int max_packets = (argc >= 3) ? atoi(argv[2]) : 10;
    fprintf(stderr, "[*] Reading %s (max %d packets)...\n", argv[1], max_packets);

    // pcapng에서 UDP 9999 패킷을 많이 읽어서 SPaT 디코딩 성공한 것만 max_packets개 출력
    auto all_pkts = read_pcapng(argv[1], max_packets * 10);
    fprintf(stderr, "[*] Extracted %zu UDP packets from pcapng\n", all_pkts.size());

    int decoded_count = 0;

    printf("[\n");

    for (size_t pi = 0; pi < all_pkts.size() && decoded_count < max_packets; pi++)
    {
        auto& pkt = all_pkts[pi];
        const uint8_t* raw = pkt.udp_payload.data();
        size_t raw_len = pkt.udp_payload.size();

        if (raw_len <= OBU_HEADER_SIZE) continue;

        uint8_t frame_type = raw[0];
        uint8_t seq_no = raw[1];
        uint8_t msg_source = raw[2];
        uint8_t is_msg_frame = raw[3];

        const uint8_t* body = raw + OBU_HEADER_SIZE;
        size_t body_len = raw_len - OBU_HEADER_SIZE;

        // WSMP 서브헤더 스킵
        size_t wsmp_off = skip_wsmp(body, body_len);
        bool has_wsmp = (wsmp_off > 0);
        body += wsmp_off;
        body_len -= wsmp_off;

        j2735SPAT* spat = nullptr;
        void* frame_msg = nullptr;
        void* spat_standalone = nullptr;

        if (is_msg_frame == 0)
        {
            ASN1Error err;
            asn1_ssize_t ret = asn1_uper_decode(&frame_msg, asn1_type_j2735MessageFrame,
                                                  body, body_len, &err);
            if (ret <= 0 || !frame_msg) continue;

            j2735MessageFrame* frame = (j2735MessageFrame*)frame_msg;
            if (frame->messageId != J2735_MSG_ID_SPAT) {
                asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
                continue;
            }

            if (frame->value.type != nullptr) {
                spat = (j2735SPAT*)frame->value.u.data;
            } else {
                ASN1String* raw_bytes = &frame->value.u.octet_string;
                asn1_ssize_t spat_ret = asn1_uper_decode(&spat_standalone, asn1_type_j2735SPAT,
                                                          raw_bytes->buf, raw_bytes->len, &err);
                if (spat_ret > 0 && spat_standalone)
                    spat = (j2735SPAT*)spat_standalone;
            }
        }
        else
        {
            ASN1Error err;
            asn1_ssize_t ret = asn1_uper_decode(&spat_standalone, asn1_type_j2735SPAT,
                                                  body, body_len, &err);
            if (ret > 0 && spat_standalone)
                spat = (j2735SPAT*)spat_standalone;
        }

        if (!spat) {
            if (frame_msg) asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
            if (spat_standalone) asn1_free_value(asn1_type_j2735SPAT, spat_standalone);
            continue;
        }

        // JSON 출력
        if (decoded_count > 0) printf(",\n");

        printf("  {\n");
        printf("    \"packet_index\": %zu,\n", pi);
        printf("    \"timestamp\": %.6f,\n", pkt.timestamp);
        printf("    \"src_port\": %d,\n", pkt.src_port);
        printf("    \"udp_length\": %zu,\n", raw_len);
        printf("    \"obu_header\": {\n");
        printf("      \"frame_type\": %d,\n", frame_type);
        printf("      \"seq_no\": %d,\n", seq_no);
        printf("      \"msg_source\": %d,\n", msg_source);
        printf("      \"is_msg_frame\": %d\n", is_msg_frame);
        printf("    },\n");
        printf("    \"wsmp_header\": %s,\n", has_wsmp ? "true" : "false");
        printf("    \"raw_hex\": \"%s\",\n", hex_string(raw, raw_len).c_str());

        // SPAT 내용
        if (spat->name_option && spat->name.buf)
            printf("    \"spat_name\": \"%s\",\n",
                   escape_json((char*)spat->name.buf, spat->name.len).c_str());
        if (spat->timeStamp_option)
            printf("    \"spat_timeStamp\": %d,\n", (int)spat->timeStamp);

        printf("    \"intersections\": [\n");
        for (size_t i = 0; i < spat->intersections.count; i++)
        {
            auto& inter = spat->intersections.tab[i];
            if (i > 0) printf(",\n");
            printf("      {\n");
            if (inter.name_option && inter.name.buf)
                printf("        \"name\": \"%s\",\n",
                       escape_json((char*)inter.name.buf, inter.name.len).c_str());
            printf("        \"region\": %d,\n", inter.id.region_option ? (int)inter.id.region : 0);
            printf("        \"id\": %d,\n", (int)inter.id.id);
            printf("        \"revision\": %d,\n", (int)inter.revision);
            if (inter.moy_option)
                printf("        \"moy\": %d,\n", (int)inter.moy);
            if (inter.timeStamp_option)
                printf("        \"dsecond\": %d,\n", (int)inter.timeStamp);
            printf("        \"movements\": [\n");

            for (size_t j = 0; j < inter.states.count; j++)
            {
                auto& mv = inter.states.tab[j];
                if (j > 0) printf(",\n");
                printf("          {\n");

                if (mv.movementName_option && mv.movementName.buf)
                    printf("            \"movementName\": \"%s\",\n",
                           escape_json((char*)mv.movementName.buf, mv.movementName.len).c_str());

                printf("            \"signalGroup\": %d,\n", (int)mv.signalGroup);

                if (mv.state_time_speed.count > 0) {
                    auto& evt = mv.state_time_speed.tab[0];
                    printf("            \"eventState\": \"%s\",\n", event_state_name((int)evt.eventState));
                    printf("            \"eventStateValue\": %d,\n", (int)evt.eventState);
                    if (evt.timing_option) {
                        printf("            \"timing\": {\n");
                        if (evt.timing.startTime_option)
                            printf("              \"startTime\": %d,\n", (int)evt.timing.startTime);
                        printf("              \"minEndTime\": %d,\n", (int)evt.timing.minEndTime);
                        if (evt.timing.maxEndTime_option)
                            printf("              \"maxEndTime\": %d,\n", (int)evt.timing.maxEndTime);
                        printf("              \"minEndTime_sec\": %.1f\n", (int)evt.timing.minEndTime / 10.0);
                        printf("            }\n");
                    }
                }
                printf("          }");
            }
            printf("\n        ]\n");
            printf("      }");
        }
        printf("\n    ]\n");
        printf("  }");

        decoded_count++;

        // cleanup
        if (spat_standalone) asn1_free_value(asn1_type_j2735SPAT, spat_standalone);
        if (frame_msg) asn1_free_value(asn1_type_j2735MessageFrame, frame_msg);
    }

    printf("\n]\n");
    fprintf(stderr, "[*] Decoded %d SPaT packets\n", decoded_count);

    return 0;
}
