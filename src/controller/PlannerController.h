#ifndef PLANNER_CONTROLLER_H
#define PLANNER_CONTROLLER_H

#include <QObject>
#include "model/RobotModel.h"
#include "model/PlannerWrapper.h"
#include "model/TrajectoryGenerator.h"
#include "view/ControlPanel.h"
#include "view/VisualizationWidget.h"
#include "view/JointPlotWidget.h"

class PlannerController : public QObject {
    Q_OBJECT

public:
    PlannerController(RobotModel& robot, PlannerWrapper& planner,
                      TrajectoryGenerator& trajGen,
                      ControlPanel& panel, VisualizationWidget& viz,
                      JointPlotWidget& plot);

private slots:
    void onPlanRequested();
    void onFrameChanged(int frame);
    void onDOFChanged(int dof);

private:
    void displayTrajectory(const Trajectory& traj, const QString& label);

    RobotModel& robot_;
    PlannerWrapper& planner_;
    TrajectoryGenerator& trajGen_;
    ControlPanel& panel_;
    VisualizationWidget& viz_;
    JointPlotWidget& plot_;
    Trajectory currentTrajectory_;
};

#endif
