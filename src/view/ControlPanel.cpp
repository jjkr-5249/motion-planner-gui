#include "ControlPanel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <cmath>

ControlPanel::ControlPanel(int numJoints, QWidget* parent)
    : QWidget(parent) {
    auto* mainLayout = new QVBoxLayout(this);
    setFixedWidth(280);
    setStyleSheet(
        "QGroupBox { font-weight: bold; border: 1px solid #555; "
        "border-radius: 5px; margin-top: 10px; padding-top: 15px; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; }"
        "QPushButton { padding: 8px; font-weight: bold; "
        "background-color: #2d8cf0; color: white; border-radius: 4px; }"
        "QPushButton:hover { background-color: #5ba3f5; }"
    );

    // DOF selector
    auto* dofGroup = new QGroupBox("Robot Configuration");
    auto* dofLayout = new QVBoxLayout(dofGroup);
    dofCombo_ = new QComboBox();
    dofCombo_->addItem("2-DOF Arm");
    dofCombo_->addItem("3-DOF Arm");
    dofCombo_->addItem("4-DOF Arm");
    dofCombo_->addItem("5-DOF Arm");
    dofCombo_->addItem("6-DOF Arm");
    dofCombo_->setCurrentIndex(1);
    dofLayout->addWidget(dofCombo_);
    mainLayout->addWidget(dofGroup);
    connect(dofCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ControlPanel::onDOFChanged);

    // Motion type selector
    auto* motionGroup = new QGroupBox("Motion Type");
    auto* motionLayout = new QVBoxLayout(motionGroup);
    motionCombo_ = new QComboBox();
    motionCombo_->addItem("Joint Space");
    motionCombo_->addItem("Linear (Cartesian)");
    motionCombo_->addItem("Circular (Cartesian)");
    motionLayout->addWidget(motionCombo_);
    mainLayout->addWidget(motionGroup);
    connect(motionCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ControlPanel::onMotionChanged);

    // Input mode selector
    auto* modeGroup = new QGroupBox("Input Mode");
    auto* modeLayout = new QVBoxLayout(modeGroup);
    modeCombo_ = new QComboBox();
    modeCombo_->addItem("Joint Angles");
    modeCombo_->addItem("Cartesian (X, Y)");
    modeLayout->addWidget(modeCombo_);
    mainLayout->addWidget(modeGroup);

    // Planner selector (only for Joint Space)
    plannerGroup_ = new QGroupBox("Planner");
    auto* plannerLayout = new QVBoxLayout(plannerGroup_);
    plannerCombo_ = new QComboBox();
    plannerCombo_->addItem("RRT-Connect");
    plannerCombo_->addItem("RRT");
    plannerCombo_->addItem("RRT*");
    plannerCombo_->addItem("PRM");
    plannerLayout->addWidget(plannerCombo_);
    mainLayout->addWidget(plannerGroup_);

    // Start: Joint mode
    startJointWidget_ = new QWidget();
    startJointLayout_ = new QFormLayout(startJointWidget_);
    startJointLayout_->setContentsMargins(0, 0, 0, 0);

    // Start: Cartesian mode
    auto* startCartWidget = new QWidget();
    auto* startCartLayout = new QFormLayout(startCartWidget);
    startCartLayout->setContentsMargins(0, 0, 0, 0);
    startX_ = new QDoubleSpinBox();
    startX_->setRange(-300.0, 300.0);
    startX_->setValue(240.0);
    startX_->setSuffix(" px");
    startY_ = new QDoubleSpinBox();
    startY_->setRange(-300.0, 300.0);
    startY_->setValue(0.0);
    startY_->setSuffix(" px");
    startCartLayout->addRow("X:", startX_);
    startCartLayout->addRow("Y:", startY_);

    auto* startGroup = new QGroupBox("Start Configuration");
    auto* startGroupLayout = new QVBoxLayout(startGroup);
    startStack_ = new QStackedWidget();
    startStack_->addWidget(startJointWidget_);
    startStack_->addWidget(startCartWidget);
    startGroupLayout->addWidget(startStack_);
    mainLayout->addWidget(startGroup);

    // Via point (only for Circular)
    viaGroup_ = new QGroupBox("Via Point");
    auto* viaLayout = new QFormLayout(viaGroup_);
    viaX_ = new QDoubleSpinBox();
    viaX_->setRange(-300.0, 300.0);
    viaX_->setValue(150.0);
    viaX_->setSuffix(" px");
    viaY_ = new QDoubleSpinBox();
    viaY_->setRange(-300.0, 300.0);
    viaY_->setValue(150.0);
    viaY_->setSuffix(" px");
    viaLayout->addRow("X:", viaX_);
    viaLayout->addRow("Y:", viaY_);
    mainLayout->addWidget(viaGroup_);
    viaGroup_->hide();

    // Goal: Joint mode
    goalJointWidget_ = new QWidget();
    goalJointLayout_ = new QFormLayout(goalJointWidget_);
    goalJointLayout_->setContentsMargins(0, 0, 0, 0);

    // Goal: Cartesian mode
    auto* goalCartWidget = new QWidget();
    auto* goalCartLayout = new QFormLayout(goalCartWidget);
    goalCartLayout->setContentsMargins(0, 0, 0, 0);
    goalX_ = new QDoubleSpinBox();
    goalX_->setRange(-300.0, 300.0);
    goalX_->setValue(100.0);
    goalX_->setSuffix(" px");
    goalY_ = new QDoubleSpinBox();
    goalY_->setRange(-300.0, 300.0);
    goalY_->setValue(150.0);
    goalY_->setSuffix(" px");
    goalCartLayout->addRow("X:", goalX_);
    goalCartLayout->addRow("Y:", goalY_);

    auto* goalGroup = new QGroupBox("Goal Configuration");
    auto* goalGroupLayout = new QVBoxLayout(goalGroup);
    goalStack_ = new QStackedWidget();
    goalStack_->addWidget(goalJointWidget_);
    goalStack_->addWidget(goalCartWidget);
    goalGroupLayout->addWidget(goalStack_);
    mainLayout->addWidget(goalGroup);

    connect(modeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this](int index) {
                startStack_->setCurrentIndex(index);
                goalStack_->setCurrentIndex(index);
            });

    rebuildJointSpins(numJoints);

    // Plan button
    planButton_ = new QPushButton("Plan Trajectory");
    mainLayout->addWidget(planButton_);
    connect(planButton_, &QPushButton::clicked, this, &ControlPanel::planRequested);

    // Playback controls
    auto* playbackGroup = new QGroupBox("Playback");
    auto* playbackLayout = new QVBoxLayout(playbackGroup);
    auto* buttonLayout = new QHBoxLayout();
    playButton_ = new QPushButton(QString::fromUtf8("▶ Play"));
    resetButton_ = new QPushButton(QString::fromUtf8("↺ Reset"));
    resetButton_->setStyleSheet(
        "QPushButton { background-color: #555; }"
        "QPushButton:hover { background-color: #777; }");
    buttonLayout->addWidget(playButton_);
    buttonLayout->addWidget(resetButton_);
    playbackLayout->addLayout(buttonLayout);

    frameSlider_ = new QSlider(Qt::Horizontal);
    frameSlider_->setRange(0, 0);
    playbackLayout->addWidget(frameSlider_);
    mainLayout->addWidget(playbackGroup);

    connect(frameSlider_, &QSlider::valueChanged, this, &ControlPanel::frameChanged);
    connect(playButton_, &QPushButton::clicked, this, &ControlPanel::onPlayPause);
    connect(resetButton_, &QPushButton::clicked, this, &ControlPanel::onReset);

    playTimer_ = new QTimer(this);
    playTimer_->setInterval(40);
    connect(playTimer_, &QTimer::timeout, this, &ControlPanel::onTimerTick);

    // Status
    statusLabel_ = new QLabel("Ready");
    statusLabel_->setStyleSheet("color: #888; padding: 5px;");
    mainLayout->addWidget(statusLabel_);

    mainLayout->addStretch();
}

void ControlPanel::onMotionChanged(int index) {
    MotionType type = static_cast<MotionType>(index);

    if (type == MotionType::Joint) {
        plannerGroup_->show();
        modeCombo_->setEnabled(true);
        viaGroup_->hide();
    } else if (type == MotionType::Linear) {
        plannerGroup_->hide();
        modeCombo_->setCurrentIndex(1); // Force Cartesian
        modeCombo_->setEnabled(false);
        viaGroup_->hide();
    } else if (type == MotionType::Circular) {
        plannerGroup_->hide();
        modeCombo_->setCurrentIndex(1); // Force Cartesian
        modeCombo_->setEnabled(false);
        viaGroup_->show();
    }
}

void ControlPanel::rebuildJointSpins(int numJoints) {
    for (auto* spin : startSpins_) {
        startJointLayout_->removeRow(spin);
    }
    startSpins_.clear();

    for (auto* spin : goalSpins_) {
        goalJointLayout_->removeRow(spin);
    }
    goalSpins_.clear();

    for (int i = 0; i < numJoints; i++) {
        auto* spin = new QDoubleSpinBox();
        spin->setRange(-180.0, 180.0);
        spin->setValue(0.0);
        spin->setSuffix(QString::fromUtf8("°"));
        startSpins_.push_back(spin);
        startJointLayout_->addRow(QString("Joint %1:").arg(i + 1), spin);
    }

    double defaultGoals[] = {45.0, -30.0, 60.0, -20.0, 40.0, -15.0};
    for (int i = 0; i < numJoints; i++) {
        auto* spin = new QDoubleSpinBox();
        spin->setRange(-180.0, 180.0);
        spin->setValue(i < 6 ? defaultGoals[i] : 0.0);
        spin->setSuffix(QString::fromUtf8("°"));
        goalSpins_.push_back(spin);
        goalJointLayout_->addRow(QString("Joint %1:").arg(i + 1), spin);
    }
}

void ControlPanel::onDOFChanged(int index) {
    int dof = index + 2;
    rebuildJointSpins(dof);
    emit dofChanged(dof);
}

void ControlPanel::onPlayPause() {
    if (playing_) {
        playTimer_->stop();
        playButton_->setText(QString::fromUtf8("▶ Play"));
        playing_ = false;
    } else {
        if (frameSlider_->value() >= frameSlider_->maximum()) {
            frameSlider_->setValue(0);
        }
        playTimer_->start();
        playButton_->setText(QString::fromUtf8("⏸ Pause"));
        playing_ = true;
    }
}

void ControlPanel::onReset() {
    playTimer_->stop();
    playButton_->setText(QString::fromUtf8("▶ Play"));
    playing_ = false;
    frameSlider_->setValue(0);
}

void ControlPanel::onTimerTick() {
    int next = frameSlider_->value() + 1;
    if (next > frameSlider_->maximum()) {
        playTimer_->stop();
        playButton_->setText(QString::fromUtf8("▶ Play"));
        playing_ = false;
        return;
    }
    frameSlider_->setValue(next);
}

std::vector<double> ControlPanel::getStartAngles() const {
    std::vector<double> angles;
    for (auto* spin : startSpins_)
        angles.push_back(spin->value() * M_PI / 180.0);
    return angles;
}

std::vector<double> ControlPanel::getGoalAngles() const {
    std::vector<double> angles;
    for (auto* spin : goalSpins_)
        angles.push_back(spin->value() * M_PI / 180.0);
    return angles;
}

std::vector<double> ControlPanel::getStartXY() const {
    return {startX_->value(), startY_->value()};
}

std::vector<double> ControlPanel::getGoalXY() const {
    return {goalX_->value(), goalY_->value()};
}

std::vector<double> ControlPanel::getViaXY() const {
    return {viaX_->value(), viaY_->value()};
}

bool ControlPanel::isCartesianMode() const {
    return modeCombo_->currentIndex() == 1;
}

PlannerType ControlPanel::getSelectedPlanner() const {
    switch (plannerCombo_->currentIndex()) {
        case 0: return PlannerType::RRTConnect;
        case 1: return PlannerType::RRT;
        case 2: return PlannerType::RRTStar;
        case 3: return PlannerType::PRM;
        default: return PlannerType::RRTConnect;
    }
}

MotionType ControlPanel::getSelectedMotion() const {
    return static_cast<MotionType>(motionCombo_->currentIndex());
}

int ControlPanel::getSelectedDOF() const {
    return dofCombo_->currentIndex() + 2;
}
