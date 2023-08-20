#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QFormLayout>
#include <cmath>

#include "geometry.h"
#include "robot.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void calculateForwardKinematics();

private:
    DH_Robot *robot;

    QLineEdit *angleInputs[DH_Robot::JOINT_COUNT];
    QLabel *resultLabel;
};

#endif // MAINWINDOW_H
