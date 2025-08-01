#ifndef GNSSWINDOW_H
#define GNSSWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QTimer>
#include <QStandardItemModel>
#include <QLabel>
#include <QMessageBox>
#include <QMap>
#include "ubxparser.h"
#include "qcustomplot.h"

namespace Ui {
class GNSSWindow;
}

class GNSSWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GNSSWindow(QWidget *parent = nullptr);
    ~GNSSWindow();

    void setSocket(QTcpSocket *socket);
    void onConnectionStatusChanged(bool connected);

private slots:
    void updateAvailableIds();
    void on_btnClearLog_clicked();
    void appendToLog(const QString &message, const QString &type = "info");
    void onReadyRead();
    void onSendButtonClicked();
    void onAutoSendToggled(bool checked);
    void sendUbxNavPvt();
    void sendUbxCfgPrt();
    void sendUbxMonVer();
    void sendUbxSecUniqid();
    void sendUbxNavStatus();
    void sendUbxCfgItfm();
    void sendUbxCfgRate(quint16 measRate, quint16 navRate);
    void sendUbxCfgAnt(bool enablePower);
    void saveLogToFile();
    void clearLog();
    void pauseLog(bool paused);

private:
    Ui::GNSSWindow *ui;
    QTcpSocket *m_socket;
    QTimer *m_timer;
    UbxParser m_ubxParser;
    QStandardItemModel *m_satModel;
    QLabel *m_antennaStatusLabel;
    QCustomPlot *m_signalPlot;
    QMap<quint8, QMap<int, QString>> m_classIdMap;

    void setupConnections();
    void setupSatelliteView();
    void setupSignalPlot();
    void registerHandlers();
    void initClassIdMapping();
    void processUbxMessage(quint8 msgClass, quint8 msgId, const QByteArray& payload);
    void processNavSat(const UbxParser::NavSat &sat);
    void processMonHw(const UbxParser::MonHw &hw);
    void updateSignalPlot();
    void displayNavPvt(const UbxParser::NavPvt &data);
    void displayNavStatus(const UbxParser::NavStatus &data);
    void displayCfgPrt(const UbxParser::CfgPrt &data);
    void displayMonVer(const UbxParser::MonVer &data);
    void createUbxPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload);
    QString getMessageName(quint8 msgClass, quint8 msgId);
};

#endif // GNSSWINDOW_H
