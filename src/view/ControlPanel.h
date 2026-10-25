#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

#include <QWidget>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QComboBox>
#include <QStackedWidget>
#include <QTimer>
#include <QFormLayout>
#include <QGroupBox>
#include <vector>
#include "model/PlannerWrapper.h"
#include "model/TrajectoryGenerator.h"

class ControlPanel : public QWidget {
    Q_OBJECT

public:
    explicit ControlPanel(int numJoints, QWidget* parent = nullptr);

    std::vector<double> getStartAngles() const;
    std::vector<double> getGoalAngles() const;
    std::vector<double> getStartXY() const;
    std::vector<double> getGoalXY() const;
    std::vector<double> getViaXY() const;
    bool isCartesianMode() const;
    PlannerType getSelectedPlanner() const;
    MotionType getSelectedMotion() const;
    int getSelectedDOF() const;

signals:
    void planRequested();
    void frameChanged(int frame);
    void dofChanged(int dof);

private slots:
    void onPlayPause();
    void onReset();
    void onTimerTick();
    void onDOFChanged(int index);
    void onMotionChanged(int index);

private:
    void rebuildJointSpins(int numJoints);

    std::vector<QDoubleSpinBox*> startSpins_;
    std::vector<QDoubleSpinBox*> goalSpins_;
    QWidget* startJointWidget_;
    QWidget* goalJointWidget_;
    QFormLayout* startJointLayout_;
    QFormLayout* goalJointLayout_;

    QDoubleSpinBox* startX_;
    QDoubleSpinBox* startY_;
    QDoubleSpinBox* goalX_;
    QDoubleSpinBox* goalY_;
    QDoubleSpinBox* viaX_;
    QDoubleSpinBox* viaY_;

    QComboBox* dofCombo_;
    QComboBox* modeCombo_;
    QComboBox* plannerCombo_;
    QComboBox* motionCombo_;
    QStackedWidget* startStack_;
    QStackedWidget* goalStack_;
    QGroupBox* viaGroup_;
    QGroupBox* plannerGroup_;
    QPushButton* planButton_;
    QPushButton* playButton_;
    QPushButton* resetButton_;
    QSlider* frameSlider_;
    QLabel* statusLabel_;
    QTimer* playTimer_;
    bool playing_ = false;

    friend class PlannerController;
};

#endif
