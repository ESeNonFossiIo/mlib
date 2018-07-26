#include "lib_RPY.h"
#include "ui_RPY.h"
#include <QMessageBox>
#include <cmath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    rotation_type = _mlib::RotationType::XYZ;
    angle_type = _mlib::AngleType::deg;

    reset();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::reset()
{
    QString null_val("0.0");
    ui->roll->setText(null_val);
    ui->pitch->setText(null_val);
    ui->heading->setText(null_val);
    ui->m00->setText(null_val);
    ui->m01->setText(null_val);
    ui->m02->setText(null_val);
    ui->m10->setText(null_val);
    ui->m11->setText(null_val);
    ui->m12->setText(null_val);
    ui->m20->setText(null_val);
    ui->m21->setText(null_val);
    ui->m22->setText(null_val);

    ui->roll->setAlignment(Qt::AlignCenter);
    ui->pitch->setAlignment(Qt::AlignCenter);
    ui->heading->setAlignment(Qt::AlignCenter);
    ui->m00->setAlignment(Qt::AlignCenter);
    ui->m01->setAlignment(Qt::AlignCenter);
    ui->m02->setAlignment(Qt::AlignCenter);
    ui->m10->setAlignment(Qt::AlignCenter);
    ui->m11->setAlignment(Qt::AlignCenter);
    ui->m12->setAlignment(Qt::AlignCenter);
    ui->m20->setAlignment(Qt::AlignCenter);
    ui->m21->setAlignment(Qt::AlignCenter);
    ui->m22->setAlignment(Qt::AlignCenter);
}

void MainWindow::on_reset_clicked()
{
    reset();
}

void MainWindow::on_compute_matrix_clicked()
{
    _mlib::Angle angle_roll(ui->roll->toPlainText().toDouble(), angle_type);
    _mlib::Angle angle_pitch(ui->pitch->toPlainText().toDouble(), angle_type);
    _mlib::Angle angle_yaw(ui->heading->toPlainText().toDouble(), angle_type);
    _mlib::TaitBryanAngles angles(angle_roll, angle_pitch, angle_yaw);
    _mlib::RotationMatrix RM(angles, rotation_type);
    ui->m00->setText(QString::number(RM(0,0),'f',5));
    ui->m01->setText(QString::number(RM(0,1),'f',5));
    ui->m02->setText(QString::number(RM(0,2),'f',5));
    ui->m10->setText(QString::number(RM(1,0),'f',5));
    ui->m11->setText(QString::number(RM(1,1),'f',5));
    ui->m12->setText(QString::number(RM(1,2),'f',5));
    ui->m20->setText(QString::number(RM(2,0),'f',5));
    ui->m21->setText(QString::number(RM(2,1),'f',5));
    ui->m22->setText(QString::number(RM(2,2),'f',5));
}

void MainWindow::on_compute_angles_clicked()
{
    double default_yaw = ui->heading->toPlainText().toDouble();
    _mlib::Matrixd m(3,3);
    m(0,0) = ui->m00->toPlainText().toDouble();
    m(0,1) = ui->m01->toPlainText().toDouble();
    m(0,2) = ui->m02->toPlainText().toDouble();
    m(1,0) = ui->m10->toPlainText().toDouble();
    m(1,1) = ui->m11->toPlainText().toDouble();
    m(1,2) = ui->m12->toPlainText().toDouble();
    m(2,0) = ui->m20->toPlainText().toDouble();
    m(2,1) = ui->m21->toPlainText().toDouble();
    m(2,2) = ui->m22->toPlainText().toDouble();

    double det = m.det();
    if( std::abs(det - 1) < 1e-10)
{
    _mlib::TaitBryanAngles angles = get_roll_pitch_yaw(m, rotation_type, default_yaw);
    ui->roll->setText(QString::number(angles.roll(angle_type),'f',5));
    ui->pitch->setText(QString::number(angles.pitch(angle_type),'f',5));
    ui->heading->setText(QString::number(angles.yaw(angle_type),'f',5));
}
else
{
    QMessageBox::warning(
        this,
        tr("Error: det != 1"),
        tr("The matrix is not a rotation matrix!"));
}
}

void MainWindow::on_RPH_clicked(bool checked)
{
    rotation_type = checked ? _mlib::RotationType::ZYX : _mlib::RotationType::XYZ;
    ui->HPR->setChecked(!checked);
}

void MainWindow::on_HPR_clicked(bool checked)
{
    rotation_type = checked ? _mlib::RotationType::XYZ : _mlib::RotationType::ZYX;
    ui->RPH->setChecked(!checked);
}

void MainWindow::on_deg_clicked(bool checked)
{
    angle_type = checked ? _mlib::AngleType::deg : _mlib::AngleType::rad;
    ui->rad->setChecked(!checked);
}

void MainWindow::on_rad_clicked(bool checked)
{
        angle_type = checked ? _mlib::AngleType::rad : _mlib::AngleType::deg;
    ui->deg->setChecked(!checked);
}
