#include "PlannerWrapper.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

PlannerWrapper::PlannerWrapper(const RobotModel& robot)
    : robot_(robot) {}

std::string PlannerWrapper::plannerName(PlannerType type) {
    switch (type) {
        case PlannerType::RRT: return "RRT";
        case PlannerType::RRTConnect: return "RRT-Connect";
        case PlannerType::RRTStar: return "RRT*";
        case PlannerType::PRM: return "PRM";
        default: return "Unknown";
    }
}

Trajectory PlannerWrapper::plan(const std::vector<double>& start,
                                 const std::vector<double>& goal,
                                 PlannerType type,
                                 double solveTime) {
    Trajectory result;
    int n = robot_.numJoints();

    auto space = std::make_shared<ob::RealVectorStateSpace>(n);
    ob::RealVectorBounds bounds(n);

    const auto& limits = robot_.jointLimits();
    for (int i = 0; i < n; i++) {
        bounds.setLow(i, limits[i].first);
        bounds.setHigh(i, limits[i].second);
    }
    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    ss.setStateValidityChecker([](const ob::State*) { return true; });

    // Select planner
    ob::PlannerPtr planner;
    switch (type) {
        case PlannerType::RRT:
            planner = std::make_shared<og::RRT>(ss.getSpaceInformation());
            break;
        case PlannerType::RRTConnect:
            planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
            break;
        case PlannerType::RRTStar:
            planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
            break;
        case PlannerType::PRM:
            planner = std::make_shared<og::PRM>(ss.getSpaceInformation());
            break;
    }
    ss.setPlanner(planner);

    ob::ScopedState<> startState(space);
    for (int i = 0; i < n; i++) startState[i] = start[i];

    ob::ScopedState<> goalState(space);
    for (int i = 0; i < n; i++) goalState[i] = goal[i];

    ss.setStartAndGoalStates(startState, goalState);

    ob::PlannerStatus solved = ss.solve(solveTime);

    if (solved) {
        ss.simplifySolution();
        og::PathGeometric& path = ss.getSolutionPath();
        path.interpolate(50);

        for (size_t i = 0; i < path.getStateCount(); i++) {
            const auto* state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
            std::vector<double> config(n);
            for (int j = 0; j < n; j++) {
                config[j] = state->values[j];
            }
            result.waypoints.push_back(config);
        }
        result.valid = true;
    }

    return result;
}
