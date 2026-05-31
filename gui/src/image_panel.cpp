#include "image_panel.hpp"
#include <QPainter>
#include <QResizeEvent>
#include <QSizePolicy>

ImagePanel::ImagePanel(const QString &title, QWidget *parent)
    : QLabel(parent), m_title(title) {
    setMinimumSize(80, 60);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    setAlignment(Qt::AlignCenter);
    setStyleSheet("background:#222; color:#888;");
    setText(title);
    setFrameShape(QFrame::Box);
}

void ImagePanel::setImage(const QImage &img) {
    m_source = img;
    redraw();
}

void ImagePanel::resizeEvent(QResizeEvent *e) {
    QLabel::resizeEvent(e);
    redraw();
}

void ImagePanel::paintEvent(QPaintEvent *e) {
    QLabel::paintEvent(e);
    if (!m_source.isNull()) {
        QPainter p(this);
        p.setPen(Qt::white);
        p.drawText(rect().adjusted(4, 2, -4, -4),
                   Qt::AlignTop | Qt::AlignLeft, m_title);
    }
}

void ImagePanel::redraw() {
    if (m_source.isNull()) return;
    if (width() <= 0 || height() <= 0) return;
    QPixmap px = QPixmap::fromImage(m_source).scaled(
        size(), Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation);
    if (px.width() > width() || px.height() > height()) {
        const int x = qMax(0, (px.width() - width()) / 2);
        const int y = qMax(0, (px.height() - height()) / 2);
        px = px.copy(x, y, width(), height());
    }
    setPixmap(px);
}
