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
    setupSocket();
    connectToServer();

    connect(ui->action_3, &QAction::triggered, this, []() {
        QMessageBox::information(nullptr, "О разработчике", "Разработчик: Семён Тихонов");
    });
    connect(ui->action_4, &QAction::triggered, this, []() {
        QMessageBox::information(nullptr, "Версия программы", "Версия: 1.0.0\nДата релиза: 15.06.2025");
    });
    connect(ui->action_2, &QAction::triggered, qApp, &QApplication::quit);

    connect(ui->pushBtnTrancfer, &QPushButton::clicked, this, &GNSSWindow::onSendBtnClicked);
}

GNSSWindow::~GNSSWindow()
{
    delete ui;
}

void GNSSWindow::setupSocket()
{
    connect(socket, &QTcpSocket::connected, this, [](){
        qDebug() << "Соединение установлено";
    });

    connect(socket, &QTcpSocket::readyRead, this, [this]() {
        QByteArray data = socket->readAll();
        qDebug() << "Приняты данные:" << data.toHex(' ');

        if (data.size() < 8) {
            return; // слишком короткий пакет
        }

        if (static_cast<quint8>(data[0]) != 0xB5 || static_cast<quint8>(data[1]) != 0x62) {
            qDebug() << "Некорректный пакет (нет UBX sync)";
            return;
        }

        quint8 msgClass = static_cast<quint8>(data[2]);
        quint8 msgId = static_cast<quint8>(data[3]);
        quint16 length = static_cast<quint8>(data[4]) | (static_cast<quint8>(data[5]) << 8);

        if (data.size() < 8 + length) {
            qDebug() << "Пакет неполный";
            return;
        }

        QByteArray payload = data.mid(6, length);

        ui->leClassReceiver->setText(QString("%1").arg(msgClass, 2, 16, QChar('0')).toUpper());
        ui->leIDReceiver->setText(QString("%1").arg(msgId, 2, 16, QChar('0')).toUpper());
        ui->tePayloadReceiver->setPlainText(payload.toHex(' ').toUpper());
    });

    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
            this, [this](QAbstractSocket::SocketError){
                qDebug() << "Socket error:" << socket->errorString();
                QMessageBox::warning(this, "Socket Error", socket->errorString());
            });
}

void GNSSWindow::connectToServer()
{
    socket->connectToHost("127.0.0.1", 5000);
}

void GNSSWindow::onSendBtnClicked()
{
    // Считываем Class и ID из lineEdit как числа в hex
    bool okClass = false, okId = false;
    quint8 msgClass = ui->leClass->text().toUInt(&okClass, 16);
    quint8 msgId = ui->leID->text().toUInt(&okId, 16);

    if (!okClass || !okId) {
        QMessageBox::warning(this, "Ошибка", "Неверный формат Class или ID (ожидается hex)");
        return;
    }

    // Считываем payload из QTextEdit, ожидая hex-строку (без пробелов или с пробелами)
    QString payloadText = ui->tePayload->toPlainText().trimmed();

    // Уберём пробелы, переведём в QByteArray
    QByteArray payload = QByteArray::fromHex(payloadText.toUtf8());

    if (payload.isEmpty() && !payloadText.isEmpty()) {
        QMessageBox::warning(this, "Ошибка", "Payload должен содержать корректные hex-данные");
        return;
    }

    sendUBXPacket(msgClass, msgId, payload);
}

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

     qDebug() << "Отправляем пакет:" << packet.toHex(' ');

    if (socket && socket->state() == QAbstractSocket::ConnectedState) {
        socket->write(packet);
        socket->flush();
        statusBar()->showMessage("UBX пакет отправлен", 2000);
    } else {
        QMessageBox::warning(this, "Ошибка", "Нет TCP-соединения");
    }
}

