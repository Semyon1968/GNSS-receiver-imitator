#include "dialog.h"
#include "gnsswindow.h"
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QRandomGenerator>
#include <QString>
#include "dialog.h"
#include "gnsswindow.h"
#include "ui_dialog.h"

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
    , mSocket(new QTcpSocket(this))
{
    ui->setupUi(this);

    // Обработка ошибок TCP-соединения
    connect(mSocket, &QAbstractSocket::errorOccurred, this, [=](QAbstractSocket::SocketError socketError) {
        Q_UNUSED(socketError);
        QMessageBox::critical(this, "Ошибка подключения", mSocket->errorString());
    });

    // Обработка успешного подключения
    connect(mSocket, &QTcpSocket::connected, this, &Dialog::onConnected);
}

Dialog::~Dialog()
{
    delete ui;
}

// Нажатие на кнопку "Enter"
void Dialog::on_ButtonEnter_clicked()
{
    QString ip = ui->leIpAddress->text().trimmed();
    QString portStr = ui->lePort->text().trimmed();

    if (ip.isEmpty() || portStr.isEmpty()) {
        QMessageBox::warning(this, "Ошибка", "Введите IP-адрес и порт.");
        return;
    }

    bool ok;
    quint16 port = portStr.toUShort(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Ошибка", "Некорректный порт.");
        return;
    }

    mSocket->connectToHost(ip, port);
    qDebug() << "Соединение установлено";
}

// Обработка успешного подключения
void Dialog::onConnected()
{
    GNSSWindow *win = new GNSSWindow();

    win->setSocket(mSocket);
    win->setupSocket();
    win->show();
    this->close();
}
