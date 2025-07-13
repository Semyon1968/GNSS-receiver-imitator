#ifndef GNSSWINDOW_H
#define GNSSWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class GNSSWindow;
}
QT_END_NAMESPACE

class GNSSWindow : public QMainWindow
{
    Q_OBJECT

public:
    GNSSWindow(QWidget *parent = nullptr);
    ~GNSSWindow();

private:
    Ui::GNSSWindow *ui;
};
#endif // GNSSWINDOW_H
