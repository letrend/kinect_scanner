#pragma once

#include <QLabel>
#include <QImage>

/// A QLabel that displays a QImage scaled to fit while preserving aspect ratio.
/// Caches the source image so resize events re-scale without losing fidelity.
class ImagePanel : public QLabel {
    Q_OBJECT
public:
    explicit ImagePanel(const QString &title, QWidget *parent = nullptr);

public slots:
    void setImage(const QImage &img);

protected:
    void resizeEvent(QResizeEvent *e) override;
    void paintEvent(QPaintEvent *e) override;

private:
    void redraw();
    QImage m_source;
    QString m_title;
};
