#include "JointPlotWidget.h"
#include <QPaintEvent>
#include <cmath>
#include <algorithm>

JointPlotWidget::JointPlotWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(500, 200);
    setStyleSheet("background-color: #1e1e2e;");
}

void JointPlotWidget::setTrajectoryData(const std::vector<std::vector<double>>& waypoints) {
    waypoints_ = waypoints;
    if (!waypoints_.empty()) {
        numJoints_ = static_cast<int>(waypoints_[0].size());
    } else {
        numJoints_ = 0;
    }
    update();
}

void JointPlotWidget::setCurrentFrame(int frame) {
    currentFrame_ = frame;
    update();
}

QColor JointPlotWidget::jointColor(int index) const {
    const QColor colors[] = {
       QColor(180, 30, 30),    // Dark Red
        QColor(30, 80, 180),    // Dark Blue
        QColor(30, 150, 30),    // Dark Green
        QColor(180, 150, 0),    // Dark Yellow
        QColor(130, 30, 180),   // Dark Purple
        QColor(200, 100, 0)     // Dark Orange
    };
    return colors[index % 6];
}

void JointPlotWidget::drawAxes(QPainter& painter, int plotX, int plotY, int plotW, int plotH) {
    // Axes
    painter.setPen(QPen(QColor(150, 150, 170), 2));
    painter.drawLine(plotX, plotY + plotH, plotX + plotW, plotY + plotH); // X axis
    painter.drawLine(plotX, plotY, plotX, plotY + plotH);                 // Y axis

    // Grid lines
    painter.setPen(QPen(QColor(200, 200, 200), 1, Qt::DashLine));
    int numGridY = 4;
    for (int i = 1; i <= numGridY; i++) {
        int y = plotY + (plotH * i) / (numGridY + 1);
        painter.drawLine(plotX, y, plotX + plotW, y);
    }

    // Y-axis labels (degrees)
    painter.setPen(QColor(0,0,0));
    QFont font("Arial", 8);
    painter.setFont(font);
    painter.drawText(plotX - 35, plotY + 5, "180°");
    painter.drawText(plotX - 25, plotY + plotH / 2 + 5, "0°");
    painter.drawText(plotX - 40, plotY + plotH + 5, "-180°");

    // Title
    QFont titleFont("Arial", 10, QFont::Bold);
    painter.setFont(titleFont);
    painter.setPen(QColor(0, 0, 0));
    painter.drawText(plotX, plotY - 10, "Joint Angles Over Trajectory");

    // X-axis label
    painter.setFont(font);
    painter.setPen(QColor(0,0,0));
    painter.drawText(plotX + plotW / 2 - 20, plotY + plotH + 20, "Waypoint");
}

void JointPlotWidget::drawCurves(QPainter& painter, int plotX, int plotY, int plotW, int plotH) {
    if (waypoints_.empty() || numJoints_ == 0) return;

    int numPoints = static_cast<int>(waypoints_.size());
    double maxAngle = M_PI;

    for (int j = 0; j < numJoints_; j++) {
        painter.setPen(QPen(jointColor(j), 2));

        for (int i = 1; i < numPoints; i++) {
            double x1 = plotX + (plotW * (i - 1.0)) / (numPoints - 1);
            double x2 = plotX + (plotW * (double)i) / (numPoints - 1);

            // Map angle from [-pi, pi] to [plotY + plotH, plotY]
            double y1 = plotY + plotH - ((waypoints_[i - 1][j] + maxAngle) / (2 * maxAngle)) * plotH;
            double y2 = plotY + plotH - ((waypoints_[i][j] + maxAngle) / (2 * maxAngle)) * plotH;

            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2));
        }
    }
}

void JointPlotWidget::drawFrameLine(QPainter& painter, int plotX, int plotY, int plotW, int plotH) {
    if (waypoints_.empty()) return;

    int numPoints = static_cast<int>(waypoints_.size());
    double x = plotX + (plotW * (double)currentFrame_) / (numPoints - 1);

    painter.setPen(QPen(QColor(0, 0, 0, 180), 2, Qt::DashLine));
    painter.drawLine(QPointF(x, plotY), QPointF(x, plotY + plotH));

    // Draw dots at current joint values
    double maxAngle = M_PI;
    if (currentFrame_ < numPoints) {
        for (int j = 0; j < numJoints_; j++) {
            double y = plotY + plotH - ((waypoints_[currentFrame_][j] + maxAngle) / (2 * maxAngle)) * plotH;
            painter.setBrush(jointColor(j));
            painter.setPen(QPen(Qt::white, 1));
            painter.drawEllipse(QPointF(x, y), 4, 4);
        }
    }
}

void JointPlotWidget::drawLegend(QPainter& painter, int plotX, int plotY, int plotW) {
    if (numJoints_ == 0) return;

    QFont font("Arial", 8);
    painter.setFont(font);

    int legendX = plotX + plotW - (numJoints_ * 70);
    int legendY = plotY - 10;

    for (int j = 0; j < numJoints_; j++) {
        int x = legendX + j * 70;
        painter.setPen(Qt::NoPen);
        painter.setBrush(jointColor(j));
        painter.drawRect(x, legendY - 4, 12, 4);

        painter.setPen(QColor(0, 0, 0));
        painter.drawText(x + 16, legendY, QString("Joint %1").arg(j + 1));
    }
}

void JointPlotWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int margin = 50;
    int plotX = margin;
    int plotY = margin;
    int plotW = width() - 2 * margin;
    int plotH = height() - 2 * margin;

    if (plotW < 50 || plotH < 50) return;

    drawAxes(painter, plotX, plotY, plotW, plotH);
    drawCurves(painter, plotX, plotY, plotW, plotH);
    drawFrameLine(painter, plotX, plotY, plotW, plotH);
    drawLegend(painter, plotX, plotY, plotW);
}
