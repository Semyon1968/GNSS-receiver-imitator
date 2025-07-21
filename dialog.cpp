#include "dialog.h"
#include "gnsswindow.h"
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QRandomGenerator>
#include <QString>

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
{
    ui->setupUi(this);
    loadModels();

    // Генерируем серийный номер длиной 12 символов
    QString serial = generateSerialNumber(12);
    ui->leSerialNum->setText(serial);
    // Генерация частоты обновления
    int randomRate = (QRandomGenerator::global()->bounded(6) + 1) * 10;
    ui->leRate->setText(QString::number(randomRate));
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::loadModels()
{
    QFile file("Models.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Ошибка", "Не удалось открыть файл Models.txt");
        return;
    }

    QStringList models;
    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (!line.isEmpty()) {
            models.append(line);
        }
    }
    file.close();

    if (!models.isEmpty()) {
        int randomIndex = QRandomGenerator::global()->bounded(models.size());
        ui->BoxModel->setText(models[randomIndex]);
    } else {
        ui->BoxModel->clear();  // Если файл пустой — очистить поле
    }
}

// Метод для генерации серийного номера
QString Dialog::generateSerialNumber(int length)
{
    const QString chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    QString result;
    for (int i = 0; i < length; ++i) {
        int index = QRandomGenerator::global()->bounded(chars.length());
        result.append(chars.at(index));
    }
    return result;
}

void Dialog::on_ButtonEnter_clicked()
{
    GNSSWindow *win = new GNSSWindow();

    // Получаем данные из полей
    QString model = ui->BoxModel->text();
    QString serial = ui->leSerialNum->text();
    QString rate = ui->leRate->text();

    // Передаем в GNSSWindow
    win->setModel(model);
    win->setSerialNumber(serial);
    win->setRate(rate);
    win->show();
    this->close();  // Закрываем текущее окно
}
