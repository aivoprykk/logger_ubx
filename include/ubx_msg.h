#ifndef D15549AC_80F1_48BE_9A9D_4D34FC9D65BD
#define D15549AC_80F1_48BE_9A9D_4D34FC9D65BD

// https://github.com/iforce2d/inavFollowme/blob/master/FollowMeTag/GPS.h

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct nav_poll_s {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint8_t msg_cls;
    uint8_t msg_id;
    uint8_t chkA;
    uint8_t chkB;
} nav_poll_t;

#define NAV_POLL_DEFAULT {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

typedef struct nav_pvt_s {  // 88 bytes payload, 92 bytes total, with Beitian BN220 100 bytes total ????(0xB5,0x62,....,chkA,chkB
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint32_t iTOW;        // GPS time of week of the navigation epoch (ms)
    uint16_t year;       // Year (UTC)
    uint8_t month;       // Month, range 1..12 (UTC)
    uint8_t day;         // Day of month, range 1..31 (UTC)
    uint8_t hour;        // Hour of day, range 0..23 (UTC)
    uint8_t minute;      // Minute of hour, range 0..59 (UTC)
    uint8_t second;      // Seconds of minute, range 0..60 (UTC)
    char valid;                // Validity Flags (see graphic below)
    uint32_t tAcc;        // Time accuracy estimate (UTC) (ns)
    int32_t nano;                 // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
    uint8_t fixType;     // GNSSfix Type, range 0..5
    char flags;                // Fix Status Flags
    uint8_t reserved1;   // reserved
    uint8_t numSV;       // Number of satellites used in Nav Solution
    int32_t lon;                  // Longitude (deg)
    int32_t lat;                  // Latitude (deg)
    int32_t height;               // Height above Ellipsoid (mm)
    int32_t hMSL;                 // Height above mean sea level (mm)
    uint32_t hAcc;        // Horizontal Accuracy Estimate (mm)
    uint32_t vAcc;        // Vertical Accuracy Estimate (mm)
    int32_t velN;                 // NED north velocity (mm/s)
    int32_t velE;                 // NED east velocity (mm/s)
    int32_t velD;                 // NED down velocity (mm/s)
    int32_t gSpeed;               // Ground Speed (2-D) (mm/s)
    int32_t heading;              // Heading of motion 2-D (deg)
    uint32_t sAcc;        // Speed Accuracy Estimate
    uint32_t headingAcc;  // Heading Accuracy Estimate
    uint16_t pDOP;       // Position dilution of precision
    int16_t reserved2;           // Reserved
    uint32_t reserved3;   // Reserved
    int32_t headVeh;              // only valid for adr4.1, beitian bn220 !
    int16_t magDec;              // only valid for adr4.1,beitian bn220 !
    int16_t magAcc;              // only valid for adr4.1,beitian bn220 !
    uint8_t chkA;
    uint8_t chkB;
} nav_pvt_t;

#define NAV_PVT_DEFAULT {\
    .cls = 0x01, \
    .id = 0x07, \
    .len = 0x00, \
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
    .reserved1 = 0x00, \
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
    .reserved2 = 0x00, \
    .reserved3 = 0x00, \
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
    uint8_t Version;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t ubx_id_1;
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
    uint8_t supported_Gnss;
    uint8_t default_Gnss;
    uint8_t enabled_Gnss;
    uint8_t simultaneous;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t chkA;
    uint8_t chkB;
} mon_gnss_t;
#define MON_GNSS_DEFAULT {\
    .cls = 0x0A, \
    .id = 0x28, \
    .len = 0x00, \
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
    uint8_t gnssId;
    uint8_t svId;
    uint8_t cno;
    int8_t elev;
    int16_t azim;
    int16_t prRes;
    uint32_t X4;  // bit3 = 1  : sV is used in navigation solution
};
#define SVS_NAV_SAT_DEFAULT {\
    .gnssId = 0, \
    .svId = 0, \
    .cno = 0, \
    .elev = 0, \
    .azim = 0, \
    .prRes = 0, \
    .X4 = 0 \
}
typedef struct nav_sat_s {
    uint8_t cls;
    uint8_t id;
    uint16_t len;  // 8 + 12*numSV !
    uint32_t iTOW;  // 4
    uint8_t version;
    uint8_t numSvs;
    uint8_t reserved1;
    uint8_t reserved2;
    struct svs_nav_sat_s sat[92];  // for M10, how many channels ???
    uint8_t chkA;          // checksum moves with payload, only with 92 sats correct !!!
    uint8_t chkB;
} nav_sat_t;
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
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    SVS_NAV_SAT_DEFAULT, SVS_NAV_SAT_DEFAULT, \
    }, \
    .chkA = 0x00, \
    .chkB = 0x00 \
}

typedef struct ubx_msg_s {  // was union, but messages are overwritten by next message
    struct nav_poll_s navDummy;
    struct nav_pvt_s navPvt;
    struct nav_dop_s navDOP;
    struct nav_ack_s navAck;
    struct nav_nack_s navNack;
    struct nav_id_s ubxId;
    struct mon_gnss_s monGNSS;
    struct mon_ver_s mon_ver;
    struct nav_sat_s nav_sat;
} ubx_msg_t;

#define UBX_MSG_DEFAULT { \
    .navDummy = NAV_POLL_DEFAULT, \
    .navPvt = NAV_PVT_DEFAULT, \
    .navDOP = NAV_DOP_DEFAULT, \
    .navAck = NAV_ACK_DEFAULT, \
    .navNack = NAV_NACK_DEFAULT, \
    .ubxId = NAV_ID_DEFAULT, \
    .monGNSS = MON_GNSS_DEFAULT, \
    .mon_ver = MON_VER_DEFAULT, \
    .nav_sat = NAV_SAT_DEFAULT \
}

#ifdef __cplusplus
}
#endif

#endif /* D15549AC_80F1_48BE_9A9D_4D34FC9D65BD */
