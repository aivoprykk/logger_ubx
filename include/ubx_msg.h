#ifndef D15549AC_80F1_48BE_9A9D_4D34FC9D65BD
#define D15549AC_80F1_48BE_9A9D_4D34FC9D65BD

// https://github.com/iforce2d/inavFollowme/blob/master/FollowMeTag/GPS.h

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// typedef struct nav_dummy_s {
//     uint8_t cls;
//     uint8_t id;
//     uint16_t len;
//     uint8_t msg_cls;
//     uint8_t msg_id;
//     uint8_t chkA;
//     uint8_t chkB;
// } nav_dummy_t;

// #define NAV_DUMMY_DEFAULT {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

typedef struct nav_pvt_s {  // 92 bytes payload, with Beitian BN220 100 bytes total ????(0xB5,0x62,....,chkA,chkB
    uint8_t cls;
    uint8_t id;
    uint16_t len;       // 92 bytes payload
    uint32_t iTOW;      // 0(4) GPS time of week of the navigation epoch (ms)
    uint16_t year;      // 4 Year (UTC)
    uint8_t month;      // 6 Month, range 1..12 (UTC)
    uint8_t day;        // 7 Day of month, range 1..31 (UTC)
    uint8_t hour;       // 8 Hour of day, range 0..23 (UTC)
    uint8_t minute;     // 9 Minute of hour, range 0..59 (UTC)
    uint8_t second;     // 10 Seconds of minute, range 0..60 (UTC)
    uint8_t valid;      // 11 Validity Flags (see graphic below)
                        // bit0 1 = Valid UTC Date 1 = valid UTC Date (see section Time validity in Integration manual for details)
                        // bit1 1 = Valid UTC Time of Day 1 = valid UTC time of day (see section Time validity in Integration manual for details)
                        // bit2 1 = Fully resolved 1 = UTC time of day has been fully resolved (no seconds uncertainty). Cannot be used to check if time is completely solved.
                        // bit3 1 = Valid Mag 1 = valid magnetic declination
    uint32_t tAcc;      // 12 Time accuracy estimate (UTC) (ns)
    int32_t nano;       // 16 Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
    uint8_t fixType;    // 20 GNSSfix Type, range 0..5
    uint8_t flags;      // 21 Fix Status Flags
                        // bit0 gnssFixOK 1 = valid fix (i.e within DOP & accuracy masks)
                        // bit1 diffSoln 1 = differential corrections were applied
                        // bit4-2 psmState Power save mode state (see Power management section in Integration Manual for details.
                        // bit5 headVehValid Heading of vehicle is valid
                        // bit7-6 carrSoln 1 = Carrier phase range solution
    uint8_t flags2;     // 22 Additional flags // This flag is only supported in Protocol Versions 19.00, 19.10, 20.10, 20.20, 20.30, 22.00, 23.00, 23.01, 27 and 28. 
                        // bit5 confirmedAvai 1 = information about UTC Date and Time of Day validity confirmation is available (see section Time validity in Integration manual for details)
                        // bit6 confirmedDate 1 = UTC Date validity could be confirmed (see section Time validity in Integration manual for details)
                        // bit7 confirmedTime 1 = UTC Time of Day could be confirmed (see section Time validity in Integration manual for details)
    uint8_t numSV;      // 23 Number of satellites used in Nav Solution
    int32_t lon;        // 24 Longitude (deg)
    int32_t lat;        // 28 Latitude (deg)
    int32_t height;     // 32 Height above Ellipsoid (mm)
    int32_t hMSL;       // 36 Height above mean sea level (mm)
    uint32_t hAcc;      // 40 Horizontal Accuracy Estimate (mm)
    uint32_t vAcc;      // 44 Vertical Accuracy Estimate (mm)
    int32_t velN;       // 48 NED north velocity (mm/s)
    int32_t velE;       // 52 NED east velocity (mm/s)
    int32_t velD;       // 56 NED down velocity (mm/s)
    int32_t gSpeed;     // 60 Ground Speed (2-D) (mm/s)
    int32_t heading;    // 64 Heading of motion 2-D (deg)
    uint32_t sAcc;      // 68 Speed Accuracy Estimate
    uint32_t headingAcc;// 72 Heading Accuracy Estimate
    uint16_t pDOP;      // 76 Position dilution of precision
    uint8_t flags3;     // 78 Additional flags
                        // bit0 invalidLlh 1 = Invalid lon, lat, height and hMSL
    uint8_t reserved2;  // 79 Reserved
    uint8_t reserved3;  // 80 Reserved
    uint8_t reserved4;  // 81 Reserved
    uint8_t reserved5;  // 82 Reserved
    uint8_t reserved6;  // 83 Reserved
    int32_t headVeh;    // 84 Heading of vehicle (2-D), this is only valid when headVehValid is set, otherwise the output is set to the heading of motion.
                        // only valid for adr4.1, beitian bn220 !
    int16_t magDec;     // 88 Magnetic declination. only valid for adr4.1,beitian bn220 !
    uint16_t magAcc;    // 90 Magnetic declination accuracy. only valid for adr4.1,beitian bn220 !
    uint8_t chkA;
    uint8_t chkB;
} nav_pvt_t;

#define NAV_PVT_DEFAULT {\
    .cls = 0x01, \
    .id = 0x07, \
    .len = 0x5c, \
    .iTOW = 0x00, \
    .year = 0x00, \
    .month = 0x00, \
    .day = 0x00, \
    .hour = 0x00, \
    .minute = 0x00, \
    .second = 0x00, \
    .valid = 0x00, \
    .tAcc = 0x00, \
    .nano = 0x00, \
    .fixType = 0x00, \
    .flags = 0x00, \
    .flags2 = 0x00, \
    .numSV = 0x00, \
    .lon = 0x00, \
    .lat = 0x00, \
    .height = 0x00, \
    .hMSL = 0x00, \
    .hAcc = 0x00, \
    .vAcc = 0x00, \
    .velN = 0x00, \
    .velE = 0x00, \
    .velD = 0x00, \
    .gSpeed = 0x00, \
    .heading = 0x00, \
    .sAcc = 0x00, \
    .headingAcc = 0x00, \
    .pDOP = 0x00, \
    .flags3 = 0x00, \
    .reserved2 = 0x00, \
    .reserved3 = 0x00, \
    .reserved4 = 0x00, \
    .reserved5 = 0x00, \
    .reserved6 = 0x00, \
    .headVeh = 0x00, \
    .magDec = 0x00, \
    .magAcc = 0x00, \
    .chkA = 0x00, \
    .chkB = 0x00 \
}

typedef struct nav_ack_s {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint8_t msg_cls;
    uint8_t msg_id;
    uint8_t chkA;
    uint8_t chkB;
} nav_ack_t;

#define NAV_ACK_DEFAULT {\
    .cls = 0x05, \
    .id = 0x01, \
    .len = 0x02, \
    .msg_cls = 0x00, \
    .msg_id = 0x00, \
    .chkA = 0x00, \
    .chkB = 0x00 \
}

typedef struct nav_nack_s {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint8_t msg_cls;
    uint8_t msg_id;
    uint8_t chkA;
    uint8_t chkB;
} nav_nack_t;
#define NAV_NACK_DEFAULT {\
    .cls = 0x05, \
    .id = 0x00, \
    .len = 0x02, \
    .msg_cls = 0x00, \
    .msg_id = 0x00, \
    .chkA = 0x00, \
    .chkB = 0x00 \
}

typedef struct nav_id_s {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint8_t Version;   // Message version (0x02 for this version)
    uint8_t reserved1; // Reserved
    uint8_t reserved2; // Reserved
    uint8_t reserved3; // Reserved
    uint8_t ubx_id_1;  // Unique chip ID (6 bytes)
    uint8_t ubx_id_2;
    uint8_t ubx_id_3;
    uint8_t ubx_id_4;
    uint8_t ubx_id_5;  // M8 has only 5 uint8_t ID !
    uint8_t ubx_id_6;  // M10 appeared to have 6 uint8_t ID !!!
    uint8_t chkA;
    uint8_t chkB;
} nav_id_t;
#define NAV_ID_DEFAULT {\
    .cls = 0x01, \
    .id = 0x04, \
    .len = 0x00, \
    .Version = 0x00, \
    .reserved1 = 0x00, \
    .reserved2 = 0x00, \
    .reserved3 = 0x00, \
    .ubx_id_1 = 0x00, \
    .ubx_id_2 = 0x00, \
    .ubx_id_3 = 0x00, \
    .ubx_id_4 = 0x00, \
    .ubx_id_5 = 0x00, \
    .ubx_id_6 = 0x00, \
    .chkA = 0x00, \
    .chkB = 0x00 \
}

typedef struct mon_gnss_s {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint8_t Version;
    uint8_t supported_Gnss; // A bit mask showing the major GNSS that can be supported by this receiver
                            // bit0 GPSSup 1 = GPS supported
                            // bit1 GlonassSup 1 = GLONASS supported
                            // bit2 BeidouSup 1 = BeiDou supported
                            // bit3 GalileoSup 1 = Galileo supported
    uint8_t default_Gnss;   // A bit mask showing the default major GNSS selection.
                            // bit0 GPSDef 1 = GPS default enabled
                            // bit1 GlonassDef 1 = GLONASS default enabled
                            // bit2 BeidouDef 1 = BeiDou default enabled
                            // bit3 GalileoDef 1 = Galileo default enabled
    uint8_t enabled_Gnss;   // A bit mask showing the current major GNSS selection enabled for this receiver
                            // bit0 GPSEna 1 = GPS enabled
                            // bit1 GlonassEna 1 = GLONASS enabled
                            // bit2 BeidouEna 1 = BeiDou enabled
                            // bit3 GalileoEna 1 = Galileo enabled
    uint8_t simultaneous;   // Maximum number of concurrent major GNSS that can be supported by this receiver
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t chkA;
    uint8_t chkB;
} mon_gnss_t;
#define MON_GNSS_DEFAULT {\
    .cls = 0x0A, \
    .id = 0x28, \
    .len = 0x08, \
    .Version = 0x00, \
    .supported_Gnss = 0x00, \
    .default_Gnss = 0x00, \
    .enabled_Gnss = 0x00, \
    .simultaneous = 0x00, \
    .reserved1 = 0x00, \
    .reserved2 = 0x00, \
    .reserved3 = 0x00, \
    .chkA = 0x00, \
    .chkB = 0x00 \
}

typedef struct nav_dop_s {  // payload 18 bytes, total 22 bytes, without (__packed__) 24 bytes !!!
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint32_t iTOW;   // 4
    uint16_t gDOP;  // 6
    uint16_t pDOP;  // 8
    uint16_t tDOP;  // 10
    uint16_t vDOP;  // 12
    uint16_t hDOP;  // 14
    uint16_t nDOP;  // 16
    uint16_t eDOP;  // 18
    uint8_t chkA;
    uint8_t chkB;
} nav_dop_t;

#define NAV_DOP_DEFAULT {\
    .cls = 0x01, \
    .id = 0x04, \
    .len = 0x00, \
    .iTOW = 0x00, \
    .gDOP = 0x00, \
    .pDOP = 0x00, \
    .tDOP = 0x00, \
    .vDOP = 0x00, \
    .hDOP = 0x00, \
    .nDOP = 0x00, \
    .eDOP = 0x00, \
    .chkA = 0x00, \
    .chkB = 0x00 \
}

struct ver_ext_s {
    char extension[30];
};
#define VER_EXT_DEFAULT { .extension = {0} }
typedef struct mon_ver_s {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    char swVersion[30];
    char hwVersion[10];
    struct ver_ext_s ext[6];
    uint8_t chkA;
    uint8_t chkB;
} mon_ver_t;
#define MON_VER_DEFAULT {\
    .cls = 0x0A, \
    .id = 0x04, \
    .len = 0x00, \
    .swVersion = {0}, \
    .hwVersion = {0}, \
    .ext = {VER_EXT_DEFAULT, VER_EXT_DEFAULT, VER_EXT_DEFAULT, VER_EXT_DEFAULT, VER_EXT_DEFAULT, VER_EXT_DEFAULT}, \
    .chkA = 0x00, \
    .chkB = 0x00 \
}
struct svs_nav_sat_s {
    uint8_t gnssId; // 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 IMES, 6 QZSS, 7 GLONASS, 8 IRNSS
    uint8_t svId;   // 1-32 for GPS, 33-64 for SBAS, 65-96 for Galileo, 193-197 for QZSS, 211-246 for BeiDou, 1-24 for GLONASS, 1-7 for IR
    uint8_t cno;    // Carrier to Noise Ratio (Signal Strength) (dbHz)
    int8_t elev;    // Elevation (range: +/-90), unknown = 0x7FFF
    int16_t azim;   // Azimuth (range 0-359), unknown = 0x7FFF
    int16_t prRes;  // Pseudo range residual (range +/- 5000), unknown = 0x7FFF
    uint32_t flags; // flags
                    // qualityInd bit2-0 = 1  Signal quality indicator: 0 = no signal 1 = search 2 = signal acquired 3 = signal detected but unusable 4 = code lock on signal 5 = code and carrier lock 6 = code and carrier lock on signal 7 = code carrier and time lock on signal
                    // svUsed bit3 = 1  : sV is used in navigation solution
                    // health bit5-4 Signal health flag: 0 = unknown 1 = healthy 2 = unhealthy
                    // diffCorr bit6 = 1  Differential correction data is available for this SV
                    // smoothed bit7 = 1  Carrier smoothed pseudorange used
                    // orbitSource bit10-8 = 1  Orbit source: 0 = broadcast ephemeris 1 = precise ephemeris
                    // ephAvail bit11 = 1  Ephemeris is available for this SV
                    // almAvail bit12 = 1  Almanac is available for this SV
                    // anoAvail bit13  1= AssistNow Offline data is available for this SV
                    // aopAvail bit14 = 1  AssistNow Autonomous data is available for this SV
                    // sbasCorrUsed bit16 = 1  SBAS corrections have been used for this SV
                    // rtcmCorrUsed bit17 = 1  RTCM corrections have been used for this SV
                    // slasCorrUsed bit18 = 1  SLAS corrections have been used for this SV
                    // spartanCorrUsed bit19 = 1  SPARTN corrections have been used for this SV
                    // prCorrUsed bit20 = 1  Pseudorange corrections have been used for this SV
                    // crCorrUsed bit21 = 1  Carrier range corrections have been used for this SV
                    // doCorrUsed bit22 = 1  Doppler corrections have been used for this SV
};
#define SVS_NAV_SAT_DEFAULT {\
    .gnssId = 0, \
    .svId = 0, \
    .cno = 0, \
    .elev = 0, \
    .azim = 0, \
    .prRes = 0, \
    .flags = 0 \
}
#define MAX_SVS 64
typedef struct nav_sat_s {
    uint8_t cls;
    uint8_t id;
    uint16_t len;       // 8 + 12*numSV !
    uint32_t iTOW;      // 0 GPS time of week of the navigation epoch.
    uint8_t version;    // 4 Message version (0x01 for this version)
    uint8_t numSvs;     // 5 Number of satellites
    uint8_t reserved1;  // 6 Reserved
    uint8_t reserved2;  // 7 Reserved
    struct svs_nav_sat_s sat[MAX_SVS];  // for M10, how many channels ???
    uint8_t chkA;          // checksum moves with payload !!!
    uint8_t chkB;
} nav_sat_t;

#define HAVE_TEN(a) a, a, a, a, a, a, a, a, a, a

#define NAV_SAT_DEFAULT {\
    .cls = 0x01, \
    .id = 0x35, \
    .len = 0x00, \
    .iTOW = 0x00, \
    .version = 0x00, \
    .numSvs = 0x00, \
    .reserved1 = 0x00, \
    .reserved2 = 0x00, \
    .sat = {\
    HAVE_TEN(SVS_NAV_SAT_DEFAULT), \
    HAVE_TEN(SVS_NAV_SAT_DEFAULT), \
    HAVE_TEN(SVS_NAV_SAT_DEFAULT), \
    HAVE_TEN(SVS_NAV_SAT_DEFAULT), \
    HAVE_TEN(SVS_NAV_SAT_DEFAULT), \
    HAVE_TEN(SVS_NAV_SAT_DEFAULT), \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT,  \
    }, \
    .chkA = 0x00, \
    .chkB = 0x00 \
}

#define UBX_NONE_SIZE 8
typedef struct ubx_msg_s {  // was union, but messages are overwritten by next message
    uint8_t none[UBX_NONE_SIZE];
    struct nav_pvt_s navPvt;
    struct nav_dop_s navDOP;
    struct nav_ack_s navAck;
    struct nav_nack_s navNack;
    struct nav_id_s ubxId;
    struct mon_gnss_s monGNSS;
    struct mon_ver_s mon_ver;
    struct nav_sat_s nav_sat;
    uint32_t count_msg;
    uint32_t count_err;
    uint32_t count_ok;
    uint32_t count_nav_pvt;
    uint32_t count_nav_sat;
    uint32_t count_nav_pvt_prev;
    uint32_t count_nav_sat_prev;
} ubx_msg_t;

#define UBX_MSG_DEFAULT { \
    .none = {0}, \
    .navPvt = NAV_PVT_DEFAULT, \
    .navDOP = NAV_DOP_DEFAULT, \
    .navAck = NAV_ACK_DEFAULT, \
    .navNack = NAV_NACK_DEFAULT, \
    .ubxId = NAV_ID_DEFAULT, \
    .monGNSS = MON_GNSS_DEFAULT, \
    .mon_ver = MON_VER_DEFAULT, \
    .nav_sat = NAV_SAT_DEFAULT, \
    .count_msg = 0, \
    .count_err = 0, \
    .count_ok = 0, \
    .count_nav_pvt = 0, \
    .count_nav_sat = 0, \
    .count_nav_pvt_prev = 0, \
    .count_nav_sat_prev = 0 \
}

#ifdef __cplusplus
}
#endif

#endif /* D15549AC_80F1_48BE_9A9D_4D34FC9D65BD */
