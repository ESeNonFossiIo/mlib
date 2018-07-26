#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "mlib/math/matrix/rotation.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_reset_clicked();

    void on_deg_clicked(bool checked);
    void on_rad_clicked(bool checked);
    void on_RPH_clicked(bool checked);
    void on_HPR_clicked(bool checked);

    void on_compute_matrix_clicked();

    void on_compute_angles_clicked();

private:
    void reset();
    Ui::MainWindow *ui;
    _mlib::RotationType rotation_type;
    _mlib::AngleType    angle_type;
};

#endif // MAINWINDOW_H
