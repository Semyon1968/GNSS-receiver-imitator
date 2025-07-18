#include "dialog.h"
#include "gnsswindow.h"
#include <QFile>
#include <QTextStream>
#include <QMessageBox>

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
{
    ui->setupUi(this);
    loadModels();
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

    ui->BoxModel->clear();  // очистить текущие элементы

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (!line.isEmpty()) {
            ui->BoxModel->addItem(line);
        }
    }

    file.close();
}

void Dialog::on_ButtonEnter_clicked()
{
    GNSSWindow *win = new GNSSWindow();
    win->show();

    this->close();  // Закрываем текущее окно
}
