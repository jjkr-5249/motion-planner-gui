#ifndef JOINT_PLOT_WIDGET_H
#define JOINT_PLOT_WIDGET_H

#include <QWidget>
#include <QPainter>
#include <vector>

class JointPlotWidget : public QWidget {
    Q_OBJECT

public:
    explicit JointPlotWidget(QWidget* parent = nullptr);

    void setTrajectoryData(const std::vector<std::vector<double>>& waypoints);
    void setCurrentFrame(int frame);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawAxes(QPainter& painter, int plotX, int plotY, int plotW, int plotH);
    void drawCurves(QPainter& painter, int plotX, int plotY, int plotW, int plotH);
    void drawFrameLine(QPainter& painter, int plotX, int plotY, int plotW, int plotH);
    void drawLegend(QPainter& painter, int plotX, int plotY, int plotW);

    QColor jointColor(int index) const;

    std::vector<std::vector<double>> waypoints_;
    int currentFrame_ = 0;
    int numJoints_ = 0;
};

#endif
