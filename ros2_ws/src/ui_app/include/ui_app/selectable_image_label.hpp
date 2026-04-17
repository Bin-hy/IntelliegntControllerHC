#ifndef SELECTABLE_IMAGE_LABEL_HPP
#define SELECTABLE_IMAGE_LABEL_HPP

#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QSize>
#include <QPointF>
#include <algorithm>

class SelectableImageLabel : public QLabel {
    Q_OBJECT
public:
    explicit SelectableImageLabel(QWidget* parent = nullptr)
        : QLabel(parent) {
        setCursor(Qt::CrossCursor);
    }

    QPointF selectedImagePoint() const { return selected_point_; }
    bool hasSelection() const { return has_selection_; }
    void setImageSize(const QSize& size) { image_size_ = size; }
    void clearSelection() { has_selection_ = false; update(); }

signals:
    void pointSelected(double u, double v);

protected:
    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            QPointF img_pt = widgetToImage(event->pos());
            if (img_pt.x() >= 0 && img_pt.y() >= 0) {
                selected_point_ = img_pt;
                has_selection_ = true;
                emit pointSelected(img_pt.x(), img_pt.y());
                update();
            }
        }
        QLabel::mousePressEvent(event);
    }

    void paintEvent(QPaintEvent* event) override {
        QLabel::paintEvent(event);
        if (!has_selection_ || image_size_.isEmpty()) return;

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        QPointF wp = imageToWidget(selected_point_);

        // Crosshair
        painter.setPen(QPen(QColor(0, 255, 80), 2));
        painter.drawLine(QPointF(wp.x() - 18, wp.y()), QPointF(wp.x() + 18, wp.y()));
        painter.drawLine(QPointF(wp.x(), wp.y() - 18), QPointF(wp.x(), wp.y() + 18));
        painter.drawEllipse(wp, 12, 12);

        // Coordinate text
        painter.setPen(QPen(Qt::white, 1));
        painter.setFont(QFont("monospace", 9));
        painter.drawText(QPointF(wp.x() + 16, wp.y() - 6),
            QString("(%1, %2)").arg((int)selected_point_.x()).arg((int)selected_point_.y()));
    }

private:
    QPointF widgetToImage(const QPoint& wp) const {
        if (image_size_.isEmpty()) return QPointF(-1, -1);
        double scale = std::min(
            (double)width() / image_size_.width(),
            (double)height() / image_size_.height());
        double dw = image_size_.width() * scale;
        double dh = image_size_.height() * scale;
        double ox = (width() - dw) / 2.0;
        double oy = (height() - dh) / 2.0;
        double u = (wp.x() - ox) / scale;
        double v = (wp.y() - oy) / scale;
        if (u < 0 || v < 0 || u >= image_size_.width() || v >= image_size_.height())
            return QPointF(-1, -1);
        return QPointF(u, v);
    }

    QPointF imageToWidget(const QPointF& ip) const {
        double scale = std::min(
            (double)width() / image_size_.width(),
            (double)height() / image_size_.height());
        double ox = (width() - image_size_.width() * scale) / 2.0;
        double oy = (height() - image_size_.height() * scale) / 2.0;
        return QPointF(ox + ip.x() * scale, oy + ip.y() * scale);
    }

    QSize image_size_;
    QPointF selected_point_;
    bool has_selection_ = false;
};

#endif // SELECTABLE_IMAGE_LABEL_HPP
