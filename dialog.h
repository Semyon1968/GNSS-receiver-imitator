#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "ui_dialog.h"  // Подключаем сгенерированный UI класс

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = nullptr);
    ~Dialog();

private slots:
    void on_ButtonEnter_clicked();
    void loadModels();

private:
    Ui::Dialog *ui;  // Указатель на UI класс
};

#endif // DIALOG_H
