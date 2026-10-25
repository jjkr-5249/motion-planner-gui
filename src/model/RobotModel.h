#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <Eigen/Core>
#include <vector>
#include <utility>

class RobotModel {
public:
    RobotModel(int dof = 3);

    void setDOF(int dof);
    int numJoints() const;
    const std::vector<double>& linkLengths() const;
    const std::vector<std::pair<double, double>>& jointLimits() const;

    std::vector<Eigen::Vector2d> forwardKinematics(const std::vector<double>& jointAngles) const;
    bool isValid(const std::vector<double>& jointAngles) const;
    Eigen::Vector2d endEffectorPosition(const std::vector<double>& jointAngles) const;
    bool inverseKinematics(const Eigen::Vector2d& target,
                           std::vector<double>& result,
                           const std::vector<double>& initialGuess = {},
                           int maxIterations = 1000,
                           double tolerance = 0.5) const;

private:
    void initialize(int dof);
    std::vector<double> linkLengths_;
    std::vector<std::pair<double, double>> jointLimits_;
};

#endif
