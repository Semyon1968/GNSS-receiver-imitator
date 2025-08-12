#include "ubxparser.h"
#include <QtEndian>
#include <QDebug>

UbxParser::UbxParser(QObject *parent) : QObject(parent) {
}

bool UbxParser::parseUbxMessage(const QByteArray &data,
                                quint8 &msgClass,
                                quint8 &msgId,
                                QByteArray &payload) {
    // 1. Check minimum message size
    if (data.size() < 8) {
        qDebug() << "UBX message too short. Minimum size is 8 bytes, got"
                 << data.size() << "bytes";
        return false;
    }

    // 2. Verify sync bytes
    if (static_cast<quint8>(data[0]) != 0xB5 ||
        static_cast<quint8>(data[1]) != 0x62) {
        qDebug() << "Invalid UBX sync bytes";
        return false;
    }

    // 3. Extract header fields
    msgClass = static_cast<quint8>(data[2]);
    msgId = static_cast<quint8>(data[3]);
    quint16 length = static_cast<quint8>(data[4]) |
                     (static_cast<quint8>(data[5]) << 8);

    // 4. Verify full message size
    if (data.size() != 6 + length + 2) {
        qDebug() << "UBX message length mismatch. Expected"
                 << 6 + length + 2 << "bytes, got" << data.size() << "bytes";
        return false;
    }

    // 5. Calculate and verify checksum
    quint8 ck_a = 0;
    quint8 ck_b = 0;

    for (int i = 2; i < data.size() - 2; i++) {
        ck_a += static_cast<quint8>(data[i]);
        ck_b += ck_a;
    }

    if (ck_a != static_cast<quint8>(data[data.size()-2]) ||
        ck_b != static_cast<quint8>(data[data.size()-1])) {
        qDebug() << "Checksum mismatch";
        return false;
    }

    // 6. Extract payload
    payload = data.mid(6, length);

    qDebug() << "Successfully parsed UBX message:"
             << "Class:" << QString("0x%1").arg(msgClass, 2, 16, QLatin1Char('0'))
             << "ID:" << QString("0x%1").arg(msgId, 2, 16, QLatin1Char('0'))
             << "Length:" << length;

    return true;
}

UbxParser::CfgItfm UbxParser::parseCfgItfm(const QByteArray &payload) {
    CfgItfm result = {};

    if (payload.size() >= 8) {
        result.config = qFromLittleEndian<quint32>(payload.mid(0, 4).constData());
        result.config2 = qFromLittleEndian<quint32>(payload.mid(4, 4).constData());
    }

    return result;
}

UbxParser::NavPvt UbxParser::parseNavPvt(const QByteArray &payload) {
    NavPvt result = {};

    if (payload.size() < 92) {
        qWarning() << "NAV-PVT payload too small:" << payload.size()
                   << "bytes, expected 92";
        return result;
    }

    result.iTOW = qFromLittleEndian<quint32>(payload.mid(0, 4).constData());
    result.year = qFromLittleEndian<quint16>(payload.mid(4, 2).constData());
    result.month = static_cast<quint8>(payload[6]);
    result.day = static_cast<quint8>(payload[7]);
    result.hour = static_cast<quint8>(payload[8]);
    result.min = static_cast<quint8>(payload[9]);
    result.sec = static_cast<quint8>(payload[10]);
    result.valid = static_cast<quint8>(payload[11]);
    result.tAcc = qFromLittleEndian<quint32>(payload.mid(12, 4).constData());
    result.nano = qFromLittleEndian<qint32>(payload.mid(16, 4).constData());
    result.fixType = static_cast<quint8>(payload[20]);
    result.flags = static_cast<quint8>(payload[21]);
    result.numSV = static_cast<quint8>(payload[23]);
    result.lon = qFromLittleEndian<qint32>(payload.mid(24, 4).constData());
    result.lat = qFromLittleEndian<qint32>(payload.mid(28, 4).constData());
    result.height = qFromLittleEndian<qint32>(payload.mid(32, 4).constData());
    result.hMSL = qFromLittleEndian<qint32>(payload.mid(36, 4).constData());
    result.hAcc = qFromLittleEndian<quint32>(payload.mid(40, 4).constData());
    result.vAcc = qFromLittleEndian<quint32>(payload.mid(44, 4).constData());
    result.velN = qFromLittleEndian<qint32>(payload.mid(48, 4).constData());
    result.velE = qFromLittleEndian<qint32>(payload.mid(52, 4).constData());
    result.velD = qFromLittleEndian<qint32>(payload.mid(56, 4).constData());
    result.speed = qFromLittleEndian<quint32>(payload.mid(60, 4).constData());
    result.gSpeed = qFromLittleEndian<quint32>(payload.mid(64, 4).constData());
    result.headMot = qFromLittleEndian<quint32>(payload.mid(68, 4).constData());
    result.sAcc = qFromLittleEndian<quint32>(payload.mid(72, 4).constData());
    result.headAcc = qFromLittleEndian<quint32>(payload.mid(76, 4).constData());
    result.pDOP = qFromLittleEndian<quint16>(payload.mid(80, 2).constData());

    qDebug() << "Parsed NAV-PVT:"
             << "Lat:" << result.lat/1e7 << "Lon:" << result.lon/1e7
             << "Fix:" << result.fixType << "Sats:" << result.numSV;

    return result;
}

UbxParser::NavSat UbxParser::parseNavSat(const QByteArray &payload) {
    NavSat result = {};

    if (payload.size() < 8) {
        return result;
    }

    result.iTOW = qFromLittleEndian<quint32>(payload.mid(0, 4).constData());
    result.version = static_cast<quint8>(payload[4]);
    result.numSvs = static_cast<quint8>(payload[5]);

    const int satSize = 12;
    for (int i = 0; i < result.numSvs && (8 + i * satSize + satSize) <= payload.size(); i++) {
        int offset = 8 + i * satSize;
        result.sats[i].gnssId = static_cast<quint8>(payload[offset]);
        result.sats[i].svId = static_cast<quint8>(payload[offset+1]);
        result.sats[i].cno = static_cast<quint8>(payload[offset+2]);
        result.sats[i].elev = static_cast<quint8>(payload[offset+3]);
        result.sats[i].azim = qFromLittleEndian<qint16>(payload.mid(offset+4, 2).constData());
        result.sats[i].flags = qFromLittleEndian<quint32>(payload.mid(offset+8, 4).constData());
    }

    return result;
}

UbxParser::NavStatus UbxParser::parseNavStatus(const QByteArray &payload) {
    NavStatus result = {};

    if (payload.size() < 16) {
        return result;
    }

    result.iTOW = qFromLittleEndian<quint32>(payload.mid(0, 4).constData());
    result.fixType = static_cast<quint8>(payload[4]);
    result.flags = static_cast<quint8>(payload[5]);
    result.ttff = qFromLittleEndian<quint32>(payload.mid(8, 4).constData());

    qDebug() << "Parsed NAV-STATUS:"
             << "Fix:" << result.fixType << "TTFF:" << result.ttff << "ms";

    return result;
}

UbxParser::CfgPrt UbxParser::parseCfgPrt(const QByteArray &payload) {
    CfgPrt result = {};

    if (payload.size() < 20) {
        return result;
    }

    result.portID = static_cast<quint8>(payload[0]);
    result.baudRate = qFromLittleEndian<quint32>(payload.mid(8, 4).constData());
    result.inProtoMask = qFromLittleEndian<quint16>(payload.mid(12, 2).constData());
    result.outProtoMask = qFromLittleEndian<quint16>(payload.mid(14, 2).constData());

    return result;
}

UbxParser::AckPacket UbxParser::parseAck(const QByteArray &payload) {
    AckPacket ack;
    ack.ackClass = 0;
    ack.ackId = 0;
    ack.isAck = false;

    if (payload.size() >= 2) {
        ack.ackClass = static_cast<quint8>(payload[0]);
        ack.ackId = static_cast<quint8>(payload[1]);
        ack.isAck = true;
    }

    return ack;
}

UbxParser::SecUniqid UbxParser::parseSecUniqid(const QByteArray &payload) {
    SecUniqid result = {};

    if (payload.size() < 5) {
        return result;
    }

    result.version = static_cast<quint8>(payload[0]);
    result.uniqueId = qFromLittleEndian<quint32>(payload.mid(1, 4).constData());

    return result;
}

UbxParser::CfgMsg UbxParser::parseCfgMsg(const QByteArray &payload) {
    CfgMsg result = {};

    if (payload.size() < 3) {
        return result;
    }

    result.msgClass = static_cast<quint8>(payload[0]);
    result.msgId = static_cast<quint8>(payload[1]);
    result.rate = static_cast<quint8>(payload[2]);

    return result;
}

UbxParser::MonRf UbxParser::parseMonRf(const QByteArray &payload) {
    MonRf result = {};
    const int headerSize = 4;
    const int blockSize = 24;

    if (payload.size() < headerSize) {
        qWarning() << "MON-RF payload too small:" << payload.size()
                   << "bytes, expected at least" << headerSize;
        return result;
    }

    result.version = static_cast<quint8>(payload[0]);
    result.nBlocks = static_cast<quint8>(payload[1]);
    result.reserved1[0] = static_cast<quint8>(payload[2]);
    result.reserved1[1] = static_cast<quint8>(payload[3]);

    int expectedSize = headerSize + result.nBlocks * blockSize;
    if (payload.size() < expectedSize) {
        qWarning() << "MON-RF payload too small for blocks:" << payload.size()
                   << "bytes, expected" << expectedSize;
        return result;
    }

    for (int i = 0; i < result.nBlocks && i < 4; i++) {
        int offset = headerSize + i * blockSize;
        auto& block = result.blocks[i];

        block.antId = static_cast<quint8>(payload[offset]);
        block.flags = static_cast<quint8>(payload[offset+1]);
        block.antStatus = static_cast<quint8>(payload[offset+2]);
        block.antPower = static_cast<quint8>(payload[offset+3]);
        block.postStatus = qFromLittleEndian<quint32>(payload.mid(offset+4, 4).constData());
        block.noisePerMS = qFromLittleEndian<quint16>(payload.mid(offset+12, 2).constData());
        block.agcCnt = qFromLittleEndian<quint16>(payload.mid(offset+14, 2).constData());
        block.cwSuppression = static_cast<quint8>(payload[offset+16]);
        block.ofsI = static_cast<qint8>(payload[offset+17]);
        block.magI = static_cast<quint8>(payload[offset+18]);
        block.ofsQ = static_cast<qint8>(payload[offset+19]);
        block.magQ = static_cast<quint8>(payload[offset+20]);
    }

    return result;
}

UbxParser::MonVer UbxParser::parseMonVer(const QByteArray &payload) {
    MonVer result;
    int pos = 0;

    // Parse SW version
    result.swVersion = QString::fromLatin1(payload.constData() + pos);
    pos += result.swVersion.length() + 1;

    // Parse HW version
    result.hwVersion = QString::fromLatin1(payload.constData() + pos);
    pos += result.hwVersion.length() + 1;

    // Parse extensions
    while (pos < payload.size()) {
        QString ext = QString::fromLatin1(payload.constData() + pos);
        if (ext.isEmpty()) {
            break;
        }
        result.extensions.append(ext);
        pos += ext.length() + 1;
    }

    return result;
}

UbxParser::MonHw UbxParser::parseMonHw(const QByteArray &payload) {
    MonHw result = {};
    const int minSize = 28 + 4 + 17 + 1 + 2;

    if (payload.size() < minSize) {
        qWarning() << "MON-HW payload too small:" << payload.size()
                   << "bytes, expected at least" << minSize;
        return result;
    }

    // Read 32-bit values
    result.pinSel = qFromLittleEndian<quint32>(payload.mid(0, 4).constData());
    result.pinBank = qFromLittleEndian<quint32>(payload.mid(4, 4).constData());
    result.pinDir = qFromLittleEndian<quint32>(payload.mid(8, 4).constData());
    result.pinVal = qFromLittleEndian<quint32>(payload.mid(12, 4).constData());

    // Read 16-bit values
    result.noisePerMS = qFromLittleEndian<quint16>(payload.mid(16, 2).constData());
    result.agcCnt = qFromLittleEndian<quint16>(payload.mid(18, 2).constData());

    // Read 8-bit values
    result.aStatus = static_cast<quint8>(payload[20]);
    result.aPower = static_cast<quint8>(payload[21]);
    result.flags = static_cast<quint8>(payload[22]);
    result.reserved1 = static_cast<quint8>(payload[23]);

    // Read usedMask
    result.usedMask = qFromLittleEndian<quint32>(payload.mid(24, 4).constData());

    // Read VP array (17 bytes)
    const int vpStart = 28;
    for (int i = 0; i < 17 && (vpStart + i) < payload.size(); i++) {
        result.VP[i] = static_cast<quint8>(payload[vpStart + i]);
    }

    // Read remaining fields
    const int jamIndPos = vpStart + 17;
    if (payload.size() > jamIndPos) {
        result.jamInd = static_cast<quint8>(payload[jamIndPos]);
    }

    if (payload.size() > jamIndPos + 1) {
        result.reserved2[0] = static_cast<quint8>(payload[jamIndPos + 1]);
    }

    if (payload.size() > jamIndPos + 2) {
        result.reserved2[1] = static_cast<quint8>(payload[jamIndPos + 2]);
    }

    // Read additional 32-bit fields if available
    const int pinIrqPos = jamIndPos + 3;
    if (payload.size() >= pinIrqPos + 12) {
        result.pinIrq = qFromLittleEndian<quint32>(payload.mid(pinIrqPos, 4).constData());
        result.pullH = qFromLittleEndian<quint32>(payload.mid(pinIrqPos + 4, 4).constData());
        result.pullL = qFromLittleEndian<quint32>(payload.mid(pinIrqPos + 8, 4).constData());
    }

    return result;
}
