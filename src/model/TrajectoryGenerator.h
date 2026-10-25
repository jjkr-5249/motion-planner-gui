#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "RobotModel.h"
#include "PlannerWrapper.h"
#include <Eigen/Core>
#include <vector>
#include <string>

enum class MotionType {
    Joint,
    Linear,
    Circular
};

struct TrajectoryResult {
    Trajectory trajectory;
    bool valid = false;
    int failedPoints = 0;
    int totalPoints = 0;
    std::string errorMessage;
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator(const RobotModel& robot);

    TrajectoryResult generateLinear(const Eigen::Vector2d& start,
                                     const Eigen::Vector2d& goal,
                                     int numPoints = 50);

    TrajectoryResult generateCircular(const Eigen::Vector2d& start,
                                       const Eigen::Vector2d& via,
                                       const Eigen::Vector2d& goal,
                                       int numPoints = 50);

    static std::string motionName(MotionType type);

private:
    bool fitCircle(const Eigen::Vector2d& p1,
                   const Eigen::Vector2d& p2,
                   const Eigen::Vector2d& p3,
                   Eigen::Vector2d& center,
                   double& radius) const;

    const RobotModel& robot_;
};

#endif
