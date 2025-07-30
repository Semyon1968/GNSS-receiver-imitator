#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QTcpSocket>

class GNSSWindow;

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = nullptr);
    ~Dialog();  // Убедитесь, что объявлен

    QTcpSocket* getSocket() const { return m_socket; }

private slots:
    void on_connectButton_clicked();
    void onConnected();
    void onDisconnected();
    void onError(QAbstractSocket::SocketError error);  // Убедитесь, что объявлен

private:
    Ui::Dialog *ui;
    QTcpSocket *m_socket;
    GNSSWindow *m_gnssWindow = nullptr;
};

#endif // DIALOG_H
