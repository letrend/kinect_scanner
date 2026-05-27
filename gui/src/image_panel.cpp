#include "image_panel.hpp"
#include <QPainter>
#include <QResizeEvent>

ImagePanel::ImagePanel(const QString &title, QWidget *parent)
    : QLabel(parent), m_title(title) {
    setMinimumSize(160, 120);
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
    QPixmap px = QPixmap::fromImage(m_source).scaled(
        size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    setPixmap(px);
}
