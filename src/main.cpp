#include <QApplication>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include "model/RobotModel.h"
#include "model/PlannerWrapper.h"
#include "model/TrajectoryGenerator.h"
#include "view/ControlPanel.h"
#include "view/VisualizationWidget.h"
#include "view/JointPlotWidget.h"
#include "controller/PlannerController.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    RobotModel robot;
    PlannerWrapper planner(robot);
    TrajectoryGenerator trajGen(robot);

    QMainWindow window;
    window.setWindowTitle("Motion Planner GUI");
    window.resize(1000, 700);

    auto* central = new QWidget();
    auto* layout = new QHBoxLayout(central);

    auto* panel = new ControlPanel(robot.numJoints());

    auto* rightSplitter = new QSplitter(Qt::Vertical);
    auto* viz = new VisualizationWidget();
    auto* plot = new JointPlotWidget();
    rightSplitter->addWidget(viz);
    rightSplitter->addWidget(plot);
    rightSplitter->setStretchFactor(0, 3);
    rightSplitter->setStretchFactor(1, 2);

    layout->addWidget(panel);
    layout->addWidget(rightSplitter, 1);

    window.setCentralWidget(central);

    PlannerController controller(robot, planner, trajGen, *panel, *viz, *plot);

    window.show();
    return app.exec();
}
