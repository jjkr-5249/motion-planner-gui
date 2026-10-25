#include "TrajectoryGenerator.h"
#include <cmath>

TrajectoryGenerator::TrajectoryGenerator(const RobotModel& robot)
    : robot_(robot) {}

std::string TrajectoryGenerator::motionName(MotionType type) {
    switch (type) {
        case MotionType::Joint: return "Joint Space";
        case MotionType::Linear: return "Linear";
        case MotionType::Circular: return "Circular";
        default: return "Unknown";
    }
}

TrajectoryResult TrajectoryGenerator::generateLinear(const Eigen::Vector2d& start,
                                                       const Eigen::Vector2d& goal,
                                                       int numPoints) {
    TrajectoryResult result;
    result.totalPoints = numPoints;
    result.failedPoints = 0;

    std::vector<double> prevAngles;
    
    for (int i = 0; i < numPoints; i++) {
        double t = static_cast<double>(i) / (numPoints - 1);
        Eigen::Vector2d point = start + t * (goal - start);

        std::vector<double> angles;
        bool ok = robot_.inverseKinematics(point, angles, prevAngles);

        if (!ok) {
            result.failedPoints++;
        } else {
            result.trajectory.waypoints.push_back(angles);
            prevAngles = angles;
        }
    }

    if (result.failedPoints > 0) {
        result.valid = false;
        result.errorMessage = "Linear path invalid: " +
            std::to_string(result.failedPoints) + "/" +
            std::to_string(result.totalPoints) + " points unreachable";
    } else {
        result.valid = true;
        result.trajectory.valid = true;
    }

    return result;
}

bool TrajectoryGenerator::fitCircle(const Eigen::Vector2d& p1,
                                     const Eigen::Vector2d& p2,
                                     const Eigen::Vector2d& p3,
                                     Eigen::Vector2d& center,
                                     double& radius) const {
    double ax = p1.x(), ay = p1.y();
    double bx = p2.x(), by = p2.y();
    double cx = p3.x(), cy = p3.y();

    double d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));

    if (std::abs(d) < 1e-10) {
        return false; // Points are collinear
    }

    double ux = ((ax * ax + ay * ay) * (by - cy) +
                 (bx * bx + by * by) * (cy - ay) +
                 (cx * cx + cy * cy) * (ay - by)) / d;

    double uy = ((ax * ax + ay * ay) * (cx - bx) +
                 (bx * bx + by * by) * (ax - cx) +
                 (cx * cx + cy * cy) * (bx - ax)) / d;

    center = Eigen::Vector2d(ux, uy);
    radius = (p1 - center).norm();

    return true;
}

TrajectoryResult TrajectoryGenerator::generateCircular(const Eigen::Vector2d& start,
                                                         const Eigen::Vector2d& via,
                                                         const Eigen::Vector2d& goal,
                                                         int numPoints) {
    TrajectoryResult result;
    result.totalPoints = numPoints;
    result.failedPoints = 0;

    Eigen::Vector2d center;
    double radius;

    if (!fitCircle(start, via, goal, center, radius)) {
        result.valid = false;
        result.errorMessage = "Points are collinear - cannot fit a circle";
        return result;
    }

    // Calculate angles from center to each point
    double startAngle = std::atan2(start.y() - center.y(), start.x() - center.x());
    double viaAngle = std::atan2(via.y() - center.y(), via.x() - center.x());
    double goalAngle = std::atan2(goal.y() - center.y(), goal.x() - center.x());

    // Determine direction: check if via is between start and goal going CCW or CW
    auto normalizeAngle = [](double a) -> double {
        while (a < -M_PI) a += 2.0 * M_PI;
        while (a > M_PI) a -= 2.0 * M_PI;
        return a;
    };

    double diffToVia = normalizeAngle(viaAngle - startAngle);
    double diffToGoal = normalizeAngle(goalAngle - startAngle);

    // Check if going through via requires CCW or CW
    bool ccw;
    if (diffToVia > 0) {
        ccw = (diffToGoal > diffToVia) || (diffToGoal < 0);
        if (ccw && diffToGoal > 0 && diffToGoal < diffToVia) ccw = false;
    } else {
        ccw = (diffToGoal < diffToVia);
        if (!ccw && diffToGoal < 0 && diffToGoal > diffToVia) ccw = true;
    }

    // Calculate total arc angle
    double totalArc;
    if (ccw) {
        totalArc = normalizeAngle(goalAngle - startAngle);
        if (totalArc >= 0) totalArc -= 2.0 * M_PI;
    } else {
        totalArc = normalizeAngle(goalAngle - startAngle);
        if (totalArc <= 0) totalArc += 2.0 * M_PI;
    }

    // Check via point is on the correct arc
    double viaCheck = normalizeAngle(viaAngle - startAngle);
    if (ccw) {
        if (viaCheck > 0) viaCheck -= 2.0 * M_PI;
        if (viaCheck < totalArc) {
            totalArc = normalizeAngle(goalAngle - startAngle);
            if (totalArc <= 0) totalArc += 2.0 * M_PI;
        }
    } else {
        if (viaCheck < 0) viaCheck += 2.0 * M_PI;
        if (viaCheck > totalArc) {
            totalArc = normalizeAngle(goalAngle - startAngle);
            if (totalArc >= 0) totalArc -= 2.0 * M_PI;
        }
    }

    std::vector<double> prevAngles;

    for (int i = 0; i < numPoints; i++) {
        double t = static_cast<double>(i) / (numPoints - 1);
        double angle = startAngle + t * totalArc;
        
        Eigen::Vector2d point(
            center.x() + radius * std::cos(angle),
            center.y() + radius * std::sin(angle)
        );

        std::vector<double> angles;
        bool ok = robot_.inverseKinematics(point, angles, prevAngles);

        if (!ok) {
            result.failedPoints++;
        } else {
            result.trajectory.waypoints.push_back(angles);
            prevAngles = angles;
        }
    }

    if (result.failedPoints > 0) {
        result.valid = false;
        result.errorMessage = "Circular path invalid: " +
            std::to_string(result.failedPoints) + "/" +
            std::to_string(result.totalPoints) + " points unreachable";
    } else {
        result.valid = true;
        result.trajectory.valid = true;
    }

    return result;
}
