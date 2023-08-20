#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // Initialize robot
    DH_Parameters defaultDHParameters[DH_Robot::JOINT_COUNT] = {
        { 10.0,  M_PI_2,    0.0,  0.21},
        {-50.0,     0.0,   -0.8, 0.193},
        {-60.0,     0.0, -0.598, -0.16},
        { 90.0,  M_PI_2,    0.0,  0.25},
        { 50.0, -M_PI_2,    0.0,  0.25},
        {  0.0,     0.0,    0.0,  0.25},
    };

    robot = new DH_Robot(defaultDHParameters);

    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QFormLayout *formLayout = new QFormLayout;

    for (size_t i = 0; i < DH_Robot::JOINT_COUNT; ++i) {
        angleInputs[i] = new QLineEdit(this);
        angleInputs[i]->setText(QString::number(defaultDHParameters[i].theta_deg));
        formLayout->addRow(tr("Joint %1 angle:").arg(i), angleInputs[i]);
    }

    QPushButton *calculateButton = new QPushButton(tr("Calculate FK"), this);
    connect(calculateButton, &QPushButton::clicked, this, &MainWindow::calculateForwardKinematics);

    resultLabel = new QLabel(this);

    QVBoxLayout *layout = new QVBoxLayout(centralWidget);
    layout->addLayout(formLayout);
    layout->addWidget(calculateButton);
    layout->addWidget(resultLabel);
}

MainWindow::~MainWindow() {
    delete robot;
}

void MainWindow::calculateForwardKinematics() {
    float angles[DH_Robot::JOINT_COUNT];
    for (size_t i = 0; i < DH_Robot::JOINT_COUNT; ++i) {
        angles[i] = angleInputs[i]->text().toFloat();
    }

    // Renew robot position
    robot->update_joint_positions(angles);

    auto end_effector_position = robot->calculate_forward_kinematics();

    resultLabel->setText(tr("End Effector Position (x, y, z): %1, %2, %3")
                         .arg(end_effector_position.x).arg(end_effector_position.y).arg(end_effector_position.z));
}
