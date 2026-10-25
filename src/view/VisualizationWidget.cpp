#include "VisualizationWidget.h"
#include <QPaintEvent>

VisualizationWidget::VisualizationWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(500, 500);
    setStyleSheet("background-color: #1e1e2e;");
}

void VisualizationWidget::setArmPositions(const std::vector<Eigen::Vector2d>& positions) {
    currentPositions_ = positions;
    update();
}

void VisualizationWidget::setTrajectoryPositions(
    const std::vector<std::vector<Eigen::Vector2d>>& trajectory) {
    trajectoryPositions_ = trajectory;
    update();
}

void VisualizationWidget::setCurrentFrame(int frame) {
    currentFrame_ = frame;
    if (currentFrame_ < static_cast<int>(trajectoryPositions_.size())) {
        currentPositions_ = trajectoryPositions_[currentFrame_];
    }
    update();
}

QPointF VisualizationWidget::toScreen(const Eigen::Vector2d& point) const {
    double centerX = width() / 2.0;
    double centerY = height() / 2.0;
    return QPointF(centerX + point.x(), centerY - point.y());
}

void VisualizationWidget::drawGrid(QPainter& painter) {
    painter.setPen(QPen(QColor(60, 60, 80), 1));
    int step = 50;

    for (int x = 0; x < width(); x += step) {
        painter.drawLine(x, 0, x, height());
    }
    for (int y = 0; y < height(); y += step) {
        painter.drawLine(0, y, width(), y);
    }

    // Draw axes
    painter.setPen(QPen(QColor(100, 100, 130), 2));
    painter.drawLine(width() / 2, 0, width() / 2, height());
    painter.drawLine(0, height() / 2, width(), height() / 2);
}

void VisualizationWidget::drawArm(QPainter& painter,
                                   const std::vector<Eigen::Vector2d>& positions,
                                   const QColor& color, int thickness) {
    if (positions.size() < 2) return;

    // Draw links
    painter.setPen(QPen(color, thickness, Qt::SolidLine, Qt::RoundCap));
    for (size_t i = 0; i < positions.size() - 1; i++) {
        QPointF from = toScreen(positions[i]);
        QPointF to = toScreen(positions[i + 1]);
        painter.drawLine(from, to);
    }

    // Draw joints
    for (size_t i = 0; i < positions.size(); i++) {
        QPointF pos = toScreen(positions[i]);
        if (i == 0) {
            // Base joint — larger, different color
            painter.setBrush(QColor(255, 200, 50));
            painter.setPen(QPen(QColor(200, 150, 0), 2));
            painter.drawEllipse(pos, 8, 8);
        } else {
            painter.setBrush(QColor(100, 200, 255));
            painter.setPen(QPen(QColor(50, 150, 220), 2));
            painter.drawEllipse(pos, 6, 6);
        }
    }

    // Draw end effector
    QPointF endPos = toScreen(positions.back());
    painter.setBrush(QColor(255, 100, 100));
    painter.setPen(QPen(QColor(200, 50, 50), 2));
    painter.drawEllipse(endPos, 5, 5);
}

void VisualizationWidget::drawTrajectoryTrace(QPainter& painter) {
    if (trajectoryPositions_.empty()) return;

    // Draw end-effector path as dotted trail
    painter.setPen(QPen(QColor(255, 255, 100, 120), 2, Qt::DashLine));
    for (size_t i = 1; i < trajectoryPositions_.size(); i++) {
        QPointF from = toScreen(trajectoryPositions_[i - 1].back());
        QPointF to = toScreen(trajectoryPositions_[i].back());
        painter.drawLine(from, to);
    }
}

void VisualizationWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    drawGrid(painter);
    drawTrajectoryTrace(painter);

    // Draw ghost arms along trajectory
    if (!trajectoryPositions_.empty()) {
        for (size_t i = 0; i < trajectoryPositions_.size(); i += 10) {
            drawArm(painter, trajectoryPositions_[i], QColor(100, 100, 255, 40), 2);
        }
    }

    // Draw current arm
    if (!currentPositions_.empty()) {
        drawArm(painter, currentPositions_, QColor(100, 200, 255), 4);
    }
}
