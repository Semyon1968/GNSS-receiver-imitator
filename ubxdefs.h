#ifndef UBX_DEFS_H
#define UBX_DEFS_H

enum UbxClass {
    UBX_CLASS_NAV = 0x01,
    UBX_CLASS_INF = 0x04,
    UBX_CLASS_ACK = 0x05,
    UBX_CLASS_CFG = 0x06,
    UBX_CLASS_MON = 0x0A,
    UBX_CLASS_SEC = 0x27
};

enum UbxNavId {
    UBX_NAV_PVT = 0x07,
    UBX_NAV_STATUS = 0x03,
    UBX_NAV_SAT = 0x35,
    UBX_NAV_TIMEUTC = 0x21
};

enum UbxCfgId {
    UBX_CFG_PRT = 0x00,
    UBX_CFG_MSG = 0x01,
    UBX_CFG_RATE = 0x08,
    UBX_CFG_ANT = 0x13,
    UBX_CFG_NAV5 = 0x24,
    UBX_CFG_ITFM = 0x39,
    UBX_CFG_VALSET = 0x8A,
    UBX_CFG_VALGET = 0x8B
};

enum UbxMonId {
    UBX_MON_VER = 0x04,
    UBX_MON_HW = 0x09,
    UBX_MON_RF = 0x38
};

enum UbxInfId {
    UBX_INF_ERROR = 0x00,
    UBX_INF_WARNING = 0x01,
    UBX_INF_NOTICE = 0x02,
    UBX_INF_TEST = 0x03,
    UBX_INF_DEBUG = 0x04
};

enum UbxAckId {
    UBX_ACK_NAK = 0x00,
    UBX_ACK_ACK = 0x01
};

enum UbxSecId {
    UBX_SEC_UNIQID = 0x03
};

enum FixType {
    FIX_TYPE_NO = 0,
    FIX_TYPE_DEAD_RECKONING = 1,
    FIX_TYPE_2D = 2,
    FIX_TYPE_3D = 3,
    FIX_TYPE_GNSS_DEAD_RECKONING = 4,
    FIX_TYPE_TIME = 5
};

enum AntennaStatus {
    ANT_STATUS_INIT = 0,
    ANT_STATUS_DONTKNOW = 1,
    ANT_STATUS_OK = 2,
    ANT_STATUS_SHORT = 3,
    ANT_STATUS_OPEN = 4
};

enum AntennaPower {
    ANT_POWER_OFF = 0,
    ANT_POWER_ON = 1,
    ANT_POWER_UNKNOWN = 2
};

enum JammingState {
    JAMMING_UNKNOWN = 0,
    JAMMING_OK = 1,
    JAMMING_WARNING = 2,
    JAMMING_CRITICAL = 3
};

#endif // UBX_DEFS_H
