#ifndef VISUALIZATION_WIDGET_H
#define VISUALIZATION_WIDGET_H

#include <QWidget>
#include <QPainter>
#include <vector>
#include <Eigen/Core>

class VisualizationWidget : public QWidget {
    Q_OBJECT

public:
    explicit VisualizationWidget(QWidget* parent = nullptr);

    void setArmPositions(const std::vector<Eigen::Vector2d>& positions);
    void setTrajectoryPositions(const std::vector<std::vector<Eigen::Vector2d>>& trajectory);
    void setCurrentFrame(int frame);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawGrid(QPainter& painter);
    void drawArm(QPainter& painter, const std::vector<Eigen::Vector2d>& positions,
                 const QColor& color, int thickness);
    void drawTrajectoryTrace(QPainter& painter);
    QPointF toScreen(const Eigen::Vector2d& point) const;

    std::vector<Eigen::Vector2d> currentPositions_;
    std::vector<std::vector<Eigen::Vector2d>> trajectoryPositions_;
    int currentFrame_ = 0;
};

#endif
