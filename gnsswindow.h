#ifndef GNSSWINDOW_H
#define GNSSWINDOW_H

#include <QTimer>
#include <QMainWindow>
#include <QTcpSocket>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class GNSSWindow;
}
QT_END_NAMESPACE

enum UbxClass
{
    UBX_CLASS_NAV = 0x01,
    UBX_CLASS_RXM = 0x02,
    UBX_CLASS_INF = 0x04,
    UBX_CLASS_ACK = 0x05,
    UBX_CLASS_CFG = 0x06,
    UBX_CLASS_MON = 0x0A,
    UBX_CLASS_AID = 0x0B,
    UBX_CLASS_TIM = 0x0D,
    UBX_CLASS_ESF = 0x10,
    UBX_CLASS_MGA = 0x13,
    UBX_CLASS_LOG = 0x21,
    UBX_CLASS_SEC = 0x27,
    UBX_CLASS_HNR = 0x28,
    UBX_CLASS_UPD = 0x09
};

// NAV messages
enum UbxNavMsg
{
    UBX_NAV_POSECEF = 0x01,
    UBX_NAV_POSLLH = 0x02,
    UBX_NAV_STATUS = 0x03,
    UBX_NAV_DOP = 0x04,
    UBX_NAV_PVT = 0x07,
    UBX_NAV_VELECEF = 0x11,
    UBX_NAV_VELNED = 0x12,
    UBX_NAV_TIMEGPS = 0x20,
    UBX_NAV_TIMEUTC = 0x21,
    UBX_NAV_CLOCK = 0x22,
    UBX_NAV_SAT = 0x35,
    UBX_NAV_SIG = 0x43
};

// RXM messages
enum UbxRxmMsg
{
    UBX_RXM_SFRBX = 0x13,
    UBX_RXM_MEASX = 0x14,
    UBX_RXM_RAWX = 0x15,
    UBX_RXM_RLM = 0x59
};

// INF messages
enum UbxInfMsg
{
    UBX_INF_ERROR = 0x00,
    UBX_INF_WARNING = 0x01,
    UBX_INF_NOTICE = 0x02,
    UBX_INF_TEST = 0x03,
    UBX_INF_DEBUG = 0x04
};

// ACK messages
enum UbxAckMsg
{
    UBX_ACK_NAK = 0x00,
    UBX_ACK_ACK = 0x01
};

// CFG messages
enum UbxCfgMsg
{
    UBX_CFG_PRT = 0x00,
    UBX_CFG_MSG = 0x01,
    UBX_CFG_RATE = 0x08,
    UBX_CFG_GNSS = 0x3E,
    UBX_CFG_VALSET = 0x8A,
    UBX_CFG_VALGET = 0x8B,
    UBX_CFG_VALDEL = 0x8C
};

// MON messages
enum UbxMonMsg
{
    UBX_MON_VER = 0x04,
    UBX_MON_HW = 0x09,
    UBX_MON_HW2 = 0x0B,
    UBX_MON_IO = 0x02,
    UBX_MON_MSGPP = 0x06
};

// TIM messages
enum UbxTimMsg
{
    UBX_TIM_TP = 0x01,
    UBX_TIM_TM2 = 0x03,
    UBX_TIM_VRFY = 0x06
};

// MGA messages
enum UbxMgaMsg
{
    UBX_MGA_GPS = 0x00,
    UBX_MGA_GAL = 0x02,
    UBX_MGA_BDS = 0x03,
    UBX_MGA_QZSS = 0x05,
    UBX_MGA_GLO = 0x06,
    UBX_MGA_ACK = 0x60
};

// LOG messages
enum UbxLogMsg
{
    UBX_LOG_ERASE = 0x03,
    UBX_LOG_STRING = 0x04,
    UBX_LOG_CREATE = 0x07,
    UBX_LOG_INFO = 0x08
};

class GNSSWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GNSSWindow(QWidget *parent = nullptr);
    ~GNSSWindow();
    void setModel(const QString &model);
    void setSerialNumber(const QString &serial);
    void setRate(const QString &rate);

private slots:
    void onSendBtnClicked();
    void clearGuiFields();
    void onUTCTimeModeChanged(const QString &mode);
    void chkGetStatus();
    void onRateCheckboxChanged(int state);
    void onSendRateClicked();

private:
    Ui::GNSSWindow *ui;
    QTcpSocket *socket;
    QByteArray receiveBuffer;
    QString currentModel;
    QString currentRate;
    QString currentSerialNumber;
    QTimer* statusTimeoutTimer = nullptr;
    QTimer *rateTimer = nullptr;
    int currentRateInt = 0;
    int printedCount = 0;
    void setupSocket();
    void connectToServer();
    void sendUBXPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload = {});
};

#endif // GNSSWINDOW_H
