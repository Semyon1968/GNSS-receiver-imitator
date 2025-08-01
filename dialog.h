#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "ui_dialog.h"

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = nullptr);
    ~Dialog();

private slots:
    void on_ButtonEnter_clicked();
    void loadModels();
    QString generateSerialNumber(int length);

private:
    Ui::Dialog *ui;
};

#endif // DIALOG_H
