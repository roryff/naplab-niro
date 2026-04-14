#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

/**
 * @file ubx_protocol.hpp
 * @brief UBX binary protocol structures and helpers for u-blox receivers.
 *
 * All structs use __attribute__((packed)) to match the on-wire layout exactly.
 * Use ubx_verify_checksum() to validate incoming messages and
 * ubx_build_message() to assemble outgoing ones.
 */

// ---------------------------------------------------------------------------
// Message structures
// ---------------------------------------------------------------------------

/** UBX frame header (6 bytes, precedes every message) */
struct UBXHeader {
    uint8_t sync1;      // 0xB5
    uint8_t sync2;      // 0x62
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;    // Payload length in bytes (little-endian)
} __attribute__((packed));

/**
 * UBX NAV-PVT  –  Navigation Position Velocity Time
 * Class 0x01, ID 0x07
 */
struct UBXNAVPVT {
    uint32_t iTOW;          // GPS time of week [ms]
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;          // Validity flags
    uint32_t tAcc;          // Time accuracy estimate [ns]
    int32_t nano;           // Fraction of second [-1e9...1e9]
    uint8_t fixType;        // 0=no, 1=DR, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
    uint8_t flags;          // Fix status flags (bit7:6 = carrSoln)
    uint8_t flags2;
    uint8_t numSV;          // Number of satellites used
    int32_t lon;            // Longitude  [deg * 1e-7]
    int32_t lat;            // Latitude   [deg * 1e-7]
    int32_t height;         // Height above ellipsoid [mm]
    int32_t hMSL;           // Height above mean sea level [mm]
    uint32_t hAcc;          // Horizontal accuracy estimate [mm]
    uint32_t vAcc;          // Vertical accuracy estimate [mm]
    int32_t velN;           // NED north velocity [mm/s]
    int32_t velE;           // NED east velocity  [mm/s]
    int32_t velD;           // NED down  velocity [mm/s]
    int32_t gSpeed;         // Ground speed (2-D)  [mm/s]
    int32_t headMot;        // Heading of motion   [deg * 1e-5]
    uint32_t sAcc;          // Speed accuracy estimate    [mm/s]
    uint32_t headAcc;       // Heading accuracy estimate  [deg * 1e-5]
    uint16_t pDOP;          // Position DOP [* 0.01]
    uint8_t flags3;
    uint8_t reserved1[5];
    int32_t headVeh;        // Heading of vehicle (2-D) [deg * 1e-5]
    int16_t magDec;         // Magnetic declination     [deg * 1e-2]
    uint16_t magAcc;        // Magnetic declination accuracy [deg * 1e-2]
} __attribute__((packed));

/**
 * UBX ESF-STATUS  –  External Sensor Fusion Status
 * Class 0x10, ID 0x10
 * Followed by numSens × UBXESFSensor entries.
 */
struct UBXESFSTATUS {
    uint32_t iTOW;          // GPS time of week [ms]
    uint8_t version;
    uint8_t initStatus1;    // [1:0]=wtInitStatus  [3:2]=mntAlgStatus  [5:4]=insInitStatus
    uint8_t initStatus2;    // [1:0]=imuInitStatus
    uint8_t reserved1[5];
    uint8_t fusionMode;     // 0=init, 1=fusion, 2=suspended, 3=disabled
    uint8_t reserved2[2];
    uint8_t numSens;
} __attribute__((packed));

/** Per-sensor entry appended after UBXESFSTATUS (4 bytes each) */
struct UBXESFSensor {
    uint8_t sensStatus1;    // [5:0]=type  bit6=used  bit7=ready
    uint8_t sensStatus2;    // [1:0]=calibStatus  [3:2]=timeStatus
    uint8_t freq;           // Observation frequency [Hz]
    uint8_t faults;         // bit0=badMeas  bit1=badTTag  bit2=missingMeas  bit3=noisyMeas
} __attribute__((packed));

/**
 * UBX ESF-RAW  –  Raw / Compensated IMU Sensor Measurements
 * Class 0x10, ID 0x03
 *
 * Payload layout:
 *   Bytes 0–3: reserved
 *   Bytes 4+:  N × 8-byte blocks  { data (4 bytes), sTag (4 bytes) }
 *
 * data word encoding:
 *   bits[31:24] = dataType   (sensor type enum)
 *   bits[23:0]  = dataField  (24-bit signed value, scale depends on type)
 *
 * Relevant dataType values (ZED-F9R):
 *   14  =  z-axis gyro compensated  [0.001 deg/s per LSB]  ← yaw rate
 *   13  =  y-axis gyro compensated  [0.001 deg/s per LSB]
 *   12  =  x-axis gyro compensated  [0.001 deg/s per LSB]
 */
#define UBX_ESF_RAW_DATATYPE_GYRO_X  12
#define UBX_ESF_RAW_DATATYPE_GYRO_Y  13
#define UBX_ESF_RAW_DATATYPE_GYRO_Z  14

/** Fixed-size header at the start of ESF-RAW payload (4 reserved bytes). */
struct UBXESFRAWHEADER {
    uint8_t reserved[4];
} __attribute__((packed));

/** One sensor measurement block inside ESF-RAW. */
struct UBXESFRAWSample {
    uint32_t data;   // bits[31:24]=dataType, bits[23:0]=dataField (24-bit signed)
    uint32_t sTag;   // sensor time tag [ms]
} __attribute__((packed));

/** Extract the signed 24-bit dataField from an ESF-RAW data word. */
inline int32_t ubx_esf_raw_data_field(uint32_t data_word)
{
    int32_t val = static_cast<int32_t>(data_word & 0x00FFFFFF);
    if (val & 0x00800000) val |= static_cast<int32_t>(0xFF000000);
    return val;
}

/** Extract the dataType byte (bits[31:24]) from an ESF-RAW data word. */
inline uint8_t ubx_esf_raw_data_type(uint32_t data_word)
{
    return static_cast<uint8_t>((data_word >> 24) & 0xFF);
}

/**
 * UBX ESF-INS  –  Vehicle Dynamics (INS)
 * Class 0x10, ID 0x15
 *
 * Outputs bias-compensated angular rates and accelerations from the INS
 * fusion engine (vehicle-frame for ADR products).
 *
 * Angular rates : int32 [deg/s * 1e-3]  →  multiply by 1e-3 * π/180 for rad/s
 * Accelerations : int32 [mg]            →  multiply by 1e-3 * 9.80665 for m/s²
 *
 * Validity flags in bitfield0:
 *   bit  8 = xAngRateValid
 *   bit  9 = yAngRateValid
 *   bit 10 = zAngRateValid
 *   bit 11 = xAccelValid
 *   bit 12 = yAccelValid
 *   bit 13 = zAccelValid
 *
 * NOTE: Fields are only valid when fusionMode == 1 (FUSION).
 */
struct UBXESFINS {
    uint32_t bitfield0;     // Version (bits[7:0]) + validity flags (bits[13:8])
    uint8_t  reserved1[4];
    uint32_t iTOW;          // GPS time of week [ms]
    int32_t  xAngRate;      // Compensated x-axis angular rate [deg/s * 1e-3]
    int32_t  yAngRate;      // Compensated y-axis angular rate [deg/s * 1e-3]
    int32_t  zAngRate;      // Compensated z-axis angular rate [deg/s * 1e-3]
    int32_t  xAccel;        // Compensated x-axis acceleration (gravity-free) [mg]
    int32_t  yAccel;        // Compensated y-axis acceleration (gravity-free) [mg]
    int32_t  zAccel;        // Compensated z-axis acceleration (gravity-free) [mg]
} __attribute__((packed));

#define UBX_ESF_INS_X_ANG_RATE_VALID  (1u << 8)
#define UBX_ESF_INS_Y_ANG_RATE_VALID  (1u << 9)
#define UBX_ESF_INS_Z_ANG_RATE_VALID  (1u << 10)
#define UBX_ESF_INS_X_ACCEL_VALID     (1u << 11)
#define UBX_ESF_INS_Y_ACCEL_VALID     (1u << 12)
#define UBX_ESF_INS_Z_ACCEL_VALID     (1u << 13)

/**
 * UBX ESF-ALG  –  IMU-mount Auto-alignment Status
 * Class 0x10, ID 0x14
 */
struct UBXESFALG {
    uint32_t iTOW;
    uint8_t version;
    uint8_t flags;          // bit0=autoMntAlgOn  [2:1]=status
    uint8_t error;
    uint8_t reserved1;
    uint32_t yaw;           // Yaw   [deg * 1e-2]
    int16_t pitch;          // Pitch [deg * 1e-2]
    int16_t roll;           // Roll  [deg * 1e-2]
} __attribute__((packed));

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/**
 * Verify the Fletcher-8 checksum of a complete UBX frame.
 * @param data   Pointer to the start of the frame (sync bytes included).
 * @param length Total frame length including the 2-byte checksum at the end.
 * @return true if the checksum is valid.
 */
inline bool ubx_verify_checksum(const uint8_t * data, size_t length)
{
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < length - 2; ++i) {
        ck_a += data[i];
        ck_b += ck_a;
    }
    return data[length - 2] == ck_a && data[length - 1] == ck_b;
}

/**
 * Build a complete UBX frame (sync + header + payload + checksum).
 * @param msg_class  UBX class byte.
 * @param msg_id     UBX message ID byte.
 * @param payload    Payload bytes (may be nullptr when payload_len == 0).
 * @param payload_len Number of payload bytes.
 * @return Byte vector ready to be written to the socket.
 */
inline std::vector<uint8_t> ubx_build_message(
    uint8_t msg_class, uint8_t msg_id,
    const uint8_t * payload, uint16_t payload_len)
{
    const size_t total = 6u + payload_len + 2u;
    std::vector<uint8_t> msg(total);

    msg[0] = 0xB5;
    msg[1] = 0x62;
    msg[2] = msg_class;
    msg[3] = msg_id;
    msg[4] = static_cast<uint8_t>(payload_len & 0xFF);
    msg[5] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);
    if (payload && payload_len > 0) {
        std::memcpy(msg.data() + 6, payload, payload_len);
    }

    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < total - 2; ++i) {
        ck_a += msg[i];
        ck_b += ck_a;
    }
    msg[total - 2] = ck_a;
    msg[total - 1] = ck_b;
    return msg;
}
