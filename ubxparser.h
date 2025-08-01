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
        quint32 iTOW;
        quint16 year;
        quint8 month, day, hour, min, sec;
        quint8 valid;
        quint32 tAcc;
        qint32 nano;
        quint8 fixType;
        quint8 flags;
        quint8 flags2;
        quint8 numSV;
        qint32 lon, lat;
        qint32 height, hMSL;
        quint32 hAcc, vAcc;
        qint32 velN, velE, velD;
        quint32 speed;
        quint32 gSpeed;
        quint32 headMot;
        quint32 sAcc;
        quint32 headAcc;
        quint16 pDOP;
        quint8 reserved[6];
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
        quint32 noisePerMS;
        quint32 agcCnt;
        quint8 aStatus;
        quint8 aPower;
        quint8 flags;
        quint8 reserved1;
        quint32 usedMask;
        quint8 VP[25];
        quint8 jamInd;
        quint8 reserved2[2];
        quint32 pinIrq;
        quint32 pullH;
        quint32 pullL;
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

signals:
    void navPvtReceived(const NavPvt &data);
    void navSatReceived(const NavSat &data);
    void navStatusReceived(const NavStatus &data);
    void monVerReceived(const MonVer &data);
    void monHwReceived(const MonHw &data);
    void cfgPrtReceived(const CfgPrt &data);
    void infErrorReceived(const QString &msg);

public:
    static bool isUbxMessage(const QByteArray &data) {
        return data.size() >= 6 &&
               static_cast<quint8>(data[0]) == 0xB5 &&
               static_cast<quint8>(data[1]) == 0x62;
    };

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
};

#endif // UBX_PARSER_H
