#ifndef PLANNER_WRAPPER_H
#define PLANNER_WRAPPER_H

#include "RobotModel.h"
#include <vector>
#include <string>

struct Trajectory {
    std::vector<std::vector<double>> waypoints;
    bool valid = false;
};

enum class PlannerType {
    RRT,
    RRTConnect,
    RRTStar,
    PRM
};

class PlannerWrapper {
public:
    PlannerWrapper(const RobotModel& robot);

    Trajectory plan(const std::vector<double>& start,
                    const std::vector<double>& goal,
                    PlannerType type = PlannerType::RRTConnect,
                    double solveTime = 1.0);

    static std::string plannerName(PlannerType type);

private:
    const RobotModel& robot_;
};

#endif
