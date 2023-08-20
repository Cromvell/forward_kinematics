#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QFormLayout>
#include <cmath>

#include "matrix.h"

struct DH_Parameters {
    float cos_theta;
    float sin_theta;
    float cos_alpha;
    float sin_alpha;
    float a;
    float d;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void calculateForwardKinematics();

private:
    DH_Parameters dh_parameters[6];
    QLineEdit *angleInputs[6];
    QLabel *resultLabel;

    Vector3f ForwardKinematics(const DH_Parameters dh_parameters[]);
    Matrix4f make_dh_matrix(DH_Parameters p);
};

#endif // MAINWINDOW_H
