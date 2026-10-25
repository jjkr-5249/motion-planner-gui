#include "RobotModel.h"
#include <Eigen/Dense>
#include <cmath>

RobotModel::RobotModel(int dof) {
    initialize(dof);
}

void RobotModel::setDOF(int dof) {
    initialize(dof);
}

void RobotModel::initialize(int dof) {
    linkLengths_.clear();
    jointLimits_.clear();

    // Total arm reach stays ~240px regardless of DOF
    // Distribute link lengths to look natural
    double totalLength = 240.0;
    for (int i = 0; i < dof; i++) {
        // Each successive link gets slightly shorter
        double ratio = 1.0 - (i * 0.15);
        if (ratio < 0.4) ratio = 0.4;
        linkLengths_.push_back(totalLength * ratio / dof);
    }

    // Normalize so total length is 240
    double sum = 0;
    for (double l : linkLengths_) sum += l;
    for (double& l : linkLengths_) l *= totalLength / sum;

    double limit = M_PI;
    for (int i = 0; i < dof; i++) {
        jointLimits_.push_back({-limit, limit});
    }
}

int RobotModel::numJoints() const {
    return static_cast<int>(linkLengths_.size());
}

const std::vector<double>& RobotModel::linkLengths() const {
    return linkLengths_;
}

const std::vector<std::pair<double, double>>& RobotModel::jointLimits() const {
    return jointLimits_;
}

std::vector<Eigen::Vector2d> RobotModel::forwardKinematics(const std::vector<double>& jointAngles) const {
    std::vector<Eigen::Vector2d> positions;
    double x = 0.0;
    double y = 0.0;
    double cumulativeAngle = 0.0;

    positions.push_back(Eigen::Vector2d(x, y));

    for (int i = 0; i < numJoints(); i++) {
        cumulativeAngle += jointAngles[i];
        x += linkLengths_[i] * std::cos(cumulativeAngle);
        y += linkLengths_[i] * std::sin(cumulativeAngle);
        positions.push_back(Eigen::Vector2d(x, y));
    }

    return positions;
}

bool RobotModel::isValid(const std::vector<double>& jointAngles) const {
    if (static_cast<int>(jointAngles.size()) != numJoints()) return false;
    for (int i = 0; i < numJoints(); i++) {
        if (jointAngles[i] < jointLimits_[i].first ||
            jointAngles[i] > jointLimits_[i].second) {
            return false;
        }
    }
    return true;
}

Eigen::Vector2d RobotModel::endEffectorPosition(const std::vector<double>& jointAngles) const {
    auto positions = forwardKinematics(jointAngles);
    return positions.back();
}

bool RobotModel::inverseKinematics(const Eigen::Vector2d& target,
                                    std::vector<double>& result,
                                    const std::vector<double>& initialGuess,
                                    int maxIterations,
                                    double tolerance) const {
    int n = numJoints();
    result.resize(n);
    if (static_cast<int>(initialGuess.size()) == n) {
        result = initialGuess;
    } else {
        for (int i = 0; i < n; i++) result[i] = 0.0;
    }

    double step = 0.01;

    for (int iter = 0; iter < maxIterations; iter++) {
        Eigen::Vector2d current = endEffectorPosition(result);
        Eigen::Vector2d error = target - current;

        if (error.norm() < tolerance) return true;

        Eigen::MatrixXd J(2, n);
        for (int j = 0; j < n; j++) {
            std::vector<double> perturbed = result;
            perturbed[j] += step;
            Eigen::Vector2d pPos = endEffectorPosition(perturbed);
            J(0, j) = (pPos.x() - current.x()) / step;
            J(1, j) = (pPos.y() - current.y()) / step;
        }

        Eigen::MatrixXd JT = J.transpose();
        Eigen::MatrixXd JJT = J * JT;
        JJT += 0.001 * Eigen::MatrixXd::Identity(2, 2);
        Eigen::Vector2d deltaTheta_small = JJT.ldlt().solve(error);
        Eigen::VectorXd deltaTheta = JT * deltaTheta_small;

        for (int j = 0; j < n; j++) {
            result[j] += deltaTheta(j);
            result[j] = std::max(jointLimits_[j].first,
                        std::min(jointLimits_[j].second, result[j]));
        }
    }

    return false;
}
