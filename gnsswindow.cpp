#include "gnsswindow.h"
#include "./ui_gnsswindow.h"
#include <QMessageBox>
#include <QDebug>

GNSSWindow::GNSSWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::GNSSWindow)
{
    ui->setupUi(this);

    // Инициализация сокета
    socket = new QTcpSocket(this);
    socket->connectToHost("127.0.0.1", 5000); // IP и порт сервера

    connect(socket, &QTcpSocket::connected, this, [](){
        qDebug() << "Соединение установлено";
    });

    connect(socket, &QTcpSocket::readyRead, this, [this]() {
        QByteArray data = socket->readAll();
        qDebug() << "Приняты данные:" << data.toHex(' ');
        // Здесь можно вызвать функцию парсинга UBX-ответа
    });

    // Меню
    connect(ui->action_3, &QAction::triggered, this, []() {
        QMessageBox::information(nullptr, "О разработчике", "Разработчик: Семён Тихонов");
    });
    connect(ui->action_4, &QAction::triggered, this, []() {
        QMessageBox::information(nullptr, "Версия программы", "Версия: 1.0.0\nДата релиза: 15.06.2025");
    });
    connect(ui->action_2, &QAction::triggered, qApp, &QApplication::quit);

    connect(ui->pushBtnTrancfer, &QPushButton::clicked, this, [this]() {
        sendUBXPacket(0x01, 0x07);
    });
}

GNSSWindow::~GNSSWindow()
{
    delete ui;
}

// Отдельно — реализация sendUBXPacket вне конструктора и деструктора
void GNSSWindow::sendUBXPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload)
{
    QByteArray packet;
    packet.append(0xB5); // sync char 1
    packet.append(0x62); // sync char 2
    packet.append(msgClass);
    packet.append(msgId);

    quint16 len = payload.size();
    packet.append(len & 0xFF);         // length LSB
    packet.append((len >> 8) & 0xFF);  // length MSB
    packet.append(payload);

    // Checksum calculation
    quint8 ck_a = 0, ck_b = 0;
    for (int i = 2; i < packet.size(); ++i) {
        ck_a += static_cast<quint8>(packet[i]);
        ck_b += ck_a;
    }
    packet.append(ck_a);
    packet.append(ck_b);

    if (socket && socket->state() == QAbstractSocket::ConnectedState) {
        socket->write(packet);
        socket->flush();
        statusBar()->showMessage("UBX пакет отправлен", 2000);
    } else {
        QMessageBox::warning(this, "Ошибка", "Нет TCP-соединения");
    }
}
