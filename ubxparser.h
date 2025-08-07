#ifndef UBX_PARSER_H
#define UBX_PARSER_H

#include <QObject>
#include <QByteArray>
#include <QDateTime>
#include <QtEndian>

class UbxParser : public QObject
{
    Q_OBJECT

public:
    explicit UbxParser(QObject *parent = nullptr);

    struct NavPvt {
        quint32 iTOW;        // GPS time of week (ms)
        quint16 year;        // Year (UTC)
        quint8 month;        // Month (1-12, UTC)
        quint8 day;          // Day (1-31, UTC)
        quint8 hour;         // Hour (0-23, UTC)
        quint8 min;          // Minute (0-59, UTC)
        quint8 sec;          // Second (0-59, UTC)
        quint8 valid;        // Validity flags
        quint32 tAcc;        // Time accuracy estimate (ns)
        qint32 nano;         // Fraction of second (ns)
        quint8 fixType;      // GNSS fix type
        quint8 flags;        // Fix status flags
        quint8 flags2;       // Additional flags
        quint8 numSV;        // Number of satellites used
        qint32 lon;          // Longitude (deg * 1e-7)
        qint32 lat;          // Latitude (deg * 1e-7)
        qint32 height;       // Height above ellipsoid (mm)
        qint32 hMSL;         // Height above mean sea level (mm)
        quint32 hAcc;        // Horizontal accuracy estimate (mm)
        quint32 vAcc;        // Vertical accuracy estimate (mm)
        qint32 velN;         // Velocity North (mm/s)
        qint32 velE;         // Velocity East (mm/s)
        qint32 velD;         // Velocity Down (mm/s)
        quint32 speed;       // Speed (3D, mm/s)
        quint32 gSpeed;      // Ground speed (2D, mm/s)
        quint32 headMot;     // Heading of motion (deg * 1e-5)
        quint32 sAcc;        // Speed accuracy estimate (mm/s)
        quint32 headAcc;     // Heading accuracy estimate (deg * 1e-5)
        quint16 pDOP;        // Position DOP (0.01)
        quint8 reserved[6];  // Reserved
    };

    struct NavSat {
        quint32 iTOW;
        quint8 version;
        quint8 numSvs;
        struct SatelliteInfo {
            quint8 gnssId;
            quint8 svId;
            quint8 cno;
            qint8 elev;
            qint16 azim;
            quint32 flags;
        } sats[128]; // Максимум 128 спутников
    };

    struct NavStatus {
        quint32 iTOW;
        quint8 fixType;
        quint8 flags;
        quint32 ttff;
    };

    struct CfgPrt {
        quint8 portID;
        quint32 baudRate;
        quint16 inProtoMask;
        quint16 outProtoMask;
    };

    struct MonVer {
        QString swVersion;
        QString hwVersion;
        QStringList extensions;
    };

    struct MonHw {
        quint32 pinSel;
        quint32 pinBank;
        quint32 pinDir;
        quint32 pinVal;
        quint16 noisePerMS;
        quint16 agcCnt;
        quint8 aStatus;
        quint8 aPower;
        quint8 flags;
        quint8 reserved1;
        quint32 usedMask;
        quint8 VP[17];
        quint8 jamInd;
        quint8 reserved2[2];
        quint32 pinIrq;
        quint32 pullH;
        quint32 pullL;
    };

    struct MonRf {
        quint8 version;
        quint8 nBlocks;
        quint8 reserved1[2];

        struct RfBlock {
            quint8 antId;
            quint8 flags;
            quint8 reserved2[2];
            float noisePerMS;
            float agcCnt;
            quint8 jamInd;
            quint8 ofsI;
            quint8 magI;
            quint8 reserved3;
            quint8 ofsQ;
            quint8 magQ;
            quint8 reserved4[2];
        } blocks[4]; // Maximum 4 RF blocks
    };

    struct AckPacket {
        quint8 ackClass;
        quint8 ackId;
        bool isAck; // true для ACK, false для NACK
    };

    struct CfgRate {
        quint16 measRate;
        quint16 navRate;
        quint16 timeRef;
    };

    struct CfgAnt {
        quint16 flags;
        quint16 pins;
    };

    struct CfgMsg {
        quint8 msgClass;
        quint8 msgId;
        quint8 rate;
    };

    struct SecUniqid {
        quint8 version;
        quint32 uniqueId;
    };

signals:
    void navPvtReceived(const NavPvt &data);
    void navSatReceived(const NavSat &data);
    void navStatusReceived(const NavStatus &data);
    void monVerReceived(const MonVer &data);
    void monHwReceived(const MonHw &data);
    void cfgPrtReceived(const CfgPrt &data);
    void infErrorReceived(const QString &msg);
    void secUniqidReceived(const SecUniqid &data);
    void monRfReceived(const MonRf &data);
    void cfgMsgReceived(const CfgMsg &data);

public:
    static bool isUbxMessage(const QByteArray &data) {
        return data.size() >= 6 &&
               static_cast<quint8>(data[0]) == 0xB5 &&
               static_cast<quint8>(data[1]) == 0x62;
    };

    static MonRf parseMonRf(const QByteArray &payload);
    static NavSat parseNavSat(const QByteArray &payload);
    static MonHw parseMonHw(const QByteArray &payload);
    static CfgRate parseCfgRate(const QByteArray &payload);
    static CfgAnt parseCfgAnt(const QByteArray &payload);
    static AckPacket parseAck(const QByteArray &payload);
    static bool parseUbxMessage(const QByteArray &data, quint8 &msgClass, quint8 &msgId, QByteArray &payload);
    static NavPvt parseNavPvt(const QByteArray &payload);
    static NavStatus parseNavStatus(const QByteArray &payload);
    static CfgPrt parseCfgPrt(const QByteArray &payload);
    static MonVer parseMonVer(const QByteArray &payload);
    static SecUniqid parseSecUniqid(const QByteArray &payload);
    static CfgMsg parseCfgMsg(const QByteArray &payload);
};

#endif // UBX_PARSER_H
