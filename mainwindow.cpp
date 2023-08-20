#include "mainwindow.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QFormLayout *formLayout = new QFormLayout;

    for (int i = 0; i < 6; ++i) {
        angleInputs[i] = new QLineEdit(this);
        formLayout->addRow(tr("Joint %1 angle:").arg(i), angleInputs[i]);
    }

    // Set default values for DH parameters
    angleInputs[0]->setText(QString::number(10.0));
    angleInputs[1]->setText(QString::number(-50.0));
    angleInputs[2]->setText(QString::number(-60.0));
    angleInputs[3]->setText(QString::number(90.0));
    angleInputs[4]->setText(QString::number(50.0));
    angleInputs[5]->setText(QString::number(0.0));

    QPushButton *calculateButton = new QPushButton(tr("Calculate"), this);
    connect(calculateButton, &QPushButton::clicked, this, &MainWindow::calculateForwardKinematics);

    resultLabel = new QLabel(this);

    QVBoxLayout *layout = new QVBoxLayout(centralWidget);
    layout->addLayout(formLayout);
    layout->addWidget(calculateButton);
    layout->addWidget(resultLabel);

    #define RAD(d) ((d) / 180. * M_PI)
    // DH Parameters
    auto &dh = dh_parameters;
    dh[0] = {cosf(RAD( 10.0)), sinf(RAD( 10.0)), cosf( M_PI_2), sinf( M_PI_2),    0.0,  0.21};
    dh[1] = {cosf(RAD(-50.0)), sinf(RAD(-50.0)),           1.0,           0.0,   -0.8, 0.193};
    dh[2] = {cosf(RAD(-60.0)), sinf(RAD(-60.0)),           1.0,           0.0, -0.598, -0.16};
    dh[3] = {cosf(RAD( 90.0)), sinf(RAD( 90.0)), cosf( M_PI_2), sinf( M_PI_2),    0.0,  0.25};
    dh[4] = {cosf(RAD( 50.0)), sinf(RAD( 50.0)), cosf(-M_PI_2), sinf(-M_PI_2),    0.0,  0.25};
    dh[5] = {cosf(RAD(  0.0)), sinf(RAD(  0.0)),           1.0,           0.0,    0.0,  0.25};
}
MainWindow::~MainWindow()
{
}

Matrix4f MainWindow::make_dh_matrix(DH_Parameters p) {
    return Matrix4f{{p.cos_theta, -p.sin_theta * p.cos_alpha,  p.sin_theta * p.sin_alpha, p.a * p.cos_theta},
                    {p.sin_theta,  p.cos_theta * p.cos_alpha, -p.cos_theta * p.sin_alpha, p.a * p.sin_theta},
                    {          0,                p.sin_alpha,                p.cos_alpha,               p.d},
                    {          0,                          0,                          0,                 1},};

}

void MainWindow::calculateForwardKinematics()
{
    float angles[6];
    for (int i = 0; i < 6; ++i) {
        angles[i] = angleInputs[i]->text().toFloat();
    }

    DH_Parameters custom_parameters[6];
    for (int i = 0; i < 6; ++i) {
        custom_parameters[i] = {cosf(RAD(angles[i])), sinf(RAD(angles[i])),
                                dh_parameters[i].cos_alpha, dh_parameters[i].sin_alpha,
                                dh_parameters[i].a, dh_parameters[i].d};
    }

    Vector3f end_effector_position = ForwardKinematics(custom_parameters);

    resultLabel->setText(tr("End Effector Position (x, y, z): %1, %2, %3")
                         .arg(end_effector_position.x).arg(end_effector_position.y).arg(end_effector_position.z));
}

Vector3f MainWindow::ForwardKinematics(const DH_Parameters dh_parameters[])
{
    auto transformation_matrix = Matrix4f::eye();
    auto end_effector_hom = Vector4f::zeros_homogenious();

    for (int i = 0; i < 6; i++) {
        auto dh_m = make_dh_matrix(dh_parameters[i]);
        // qDebug() << "TF: \n" << dh_m;
        // qDebug() << "BEFORE: " << end_effector_o;
        // end_effector_o = dh_m * end_effector_o;
        // qDebug() << "AFTER: " << end_effector_o;
        transformation_matrix = matmul(dh_m, transformation_matrix);
    }

    qDebug() << "MATRIX: " << transformation_matrix;
    end_effector_hom = transformation_matrix * end_effector_hom;
    qDebug() << "EE: " << end_effector_hom;

    return Vector3f::from_homogenious(end_effector_hom);
}
