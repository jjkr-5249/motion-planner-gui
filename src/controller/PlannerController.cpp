#include "PlannerController.h"

PlannerController::PlannerController(RobotModel& robot, PlannerWrapper& planner,
                                     TrajectoryGenerator& trajGen,
                                     ControlPanel& panel, VisualizationWidget& viz,
                                     JointPlotWidget& plot)
    : robot_(robot), planner_(planner), trajGen_(trajGen),
      panel_(panel), viz_(viz), plot_(plot) {

    connect(&panel_, &ControlPanel::planRequested, this, &PlannerController::onPlanRequested);
    connect(&panel_, &ControlPanel::frameChanged, this, &PlannerController::onFrameChanged);
    connect(&panel_, &ControlPanel::dofChanged, this, &PlannerController::onDOFChanged);

    std::vector<double> initAngles(robot_.numJoints(), 0.0);
    viz_.setArmPositions(robot_.forwardKinematics(initAngles));
}

void PlannerController::onDOFChanged(int dof) {
    robot_.setDOF(dof);

    std::vector<double> initAngles(dof, 0.0);
    viz_.setTrajectoryPositions({});
    viz_.setArmPositions(robot_.forwardKinematics(initAngles));
    plot_.setTrajectoryData({});

    panel_.statusLabel_->setText(QString("%1-DOF arm loaded").arg(dof));
    panel_.statusLabel_->setStyleSheet("color: #888; padding: 5px;");
    panel_.frameSlider_->setRange(0, 0);
}

void PlannerController::displayTrajectory(const Trajectory& traj, const QString& label) {
    std::vector<std::vector<Eigen::Vector2d>> trajPositions;
    for (const auto& wp : traj.waypoints) {
        trajPositions.push_back(robot_.forwardKinematics(wp));
    }

    viz_.setTrajectoryPositions(trajPositions);
    plot_.setTrajectoryData(traj.waypoints);

    panel_.frameSlider_->setRange(0, static_cast<int>(trajPositions.size()) - 1);
    panel_.frameSlider_->setValue(0);
    viz_.setCurrentFrame(0);
    plot_.setCurrentFrame(0);

    panel_.statusLabel_->setText(
        QString("%1 | %2-DOF | %3 waypoints")
            .arg(label)
            .arg(robot_.numJoints())
            .arg(traj.waypoints.size()));
    panel_.statusLabel_->setStyleSheet("color: #5cb85c; padding: 5px;");
}

void PlannerController::onPlanRequested() {
    MotionType motionType = panel_.getSelectedMotion();

    panel_.statusLabel_->setText("Planning...");
    panel_.statusLabel_->setStyleSheet("color: #f0ad4e; padding: 5px;");

    if (motionType == MotionType::Joint) {
        // Joint space planning with OMPL
        std::vector<double> startAngles, goalAngles;

        if (panel_.isCartesianMode()) {
            auto startXY = panel_.getStartXY();
            auto goalXY = panel_.getGoalXY();

            Eigen::Vector2d startTarget(startXY[0], startXY[1]);
            Eigen::Vector2d goalTarget(goalXY[0], goalXY[1]);

            bool startOk = robot_.inverseKinematics(startTarget, startAngles);
            bool goalOk = robot_.inverseKinematics(goalTarget, goalAngles);

            if (!startOk) {
                panel_.statusLabel_->setText("Start position unreachable!");
                panel_.statusLabel_->setStyleSheet("color: #d9534f; padding: 5px;");
                return;
            }
            if (!goalOk) {
                panel_.statusLabel_->setText("Goal position unreachable!");
                panel_.statusLabel_->setStyleSheet("color: #d9534f; padding: 5px;");
                return;
            }
        } else {
            startAngles = panel_.getStartAngles();
            goalAngles = panel_.getGoalAngles();
        }

        PlannerType selectedPlanner = panel_.getSelectedPlanner();
        currentTrajectory_ = planner_.plan(startAngles, goalAngles, selectedPlanner);

        if (currentTrajectory_.valid) {
            displayTrajectory(currentTrajectory_,
                QString::fromStdString(PlannerWrapper::plannerName(selectedPlanner)));
        } else {
            panel_.statusLabel_->setText("No solution found!");
            panel_.statusLabel_->setStyleSheet("color: #d9534f; padding: 5px;");
        }

    } else if (motionType == MotionType::Linear) {
        auto startXY = panel_.getStartXY();
        auto goalXY = panel_.getGoalXY();

        Eigen::Vector2d start(startXY[0], startXY[1]);
        Eigen::Vector2d goal(goalXY[0], goalXY[1]);

        TrajectoryResult result = trajGen_.generateLinear(start, goal);

        if (result.valid) {
            currentTrajectory_ = result.trajectory;
            displayTrajectory(currentTrajectory_, "Linear");
        } else {
            panel_.statusLabel_->setText(QString::fromStdString(result.errorMessage));
            panel_.statusLabel_->setStyleSheet("color: #d9534f; padding: 5px;");
        }

    } else if (motionType == MotionType::Circular) {
        auto startXY = panel_.getStartXY();
        auto viaXY = panel_.getViaXY();
        auto goalXY = panel_.getGoalXY();

        Eigen::Vector2d start(startXY[0], startXY[1]);
        Eigen::Vector2d via(viaXY[0], viaXY[1]);
        Eigen::Vector2d goal(goalXY[0], goalXY[1]);

        TrajectoryResult result = trajGen_.generateCircular(start, via, goal);

        if (result.valid) {
            currentTrajectory_ = result.trajectory;
            displayTrajectory(currentTrajectory_, "Circular");
        } else {
            panel_.statusLabel_->setText(QString::fromStdString(result.errorMessage));
            panel_.statusLabel_->setStyleSheet("color: #d9534f; padding: 5px;");
        }
    }
}

void PlannerController::onFrameChanged(int frame) {
    viz_.setCurrentFrame(frame);
    plot_.setCurrentFrame(frame);
}
