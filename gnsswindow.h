#ifndef GNSSWINDOW_H
#define GNSSWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QTimer>
#include <QMessageBox>

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

private slots:
    void onReadyRead();
    void onSendButtonClicked();
    void onAutoSendToggled(bool checked);
    void sendUbxNavPvt();
    void sendUbxCfgPrt();
    void sendUbxMonVer();
    void sendUbxSecUniqid();
    void sendUbxNavStatus();
    void sendUbxCfgItfm();

private:
    Ui::GNSSWindow *ui;
    QTcpSocket *m_socket;
    QTimer *m_timer;

    void setupConnections();
    void createUbxPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload);
};

#endif // GNSSWINDOW_H
