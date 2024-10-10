#ifndef CCPUSHBUTTON_H
#define CCPUSHBUTTON_H
#include <QPushButton>

class CCPushButton : public QPushButton
{
    Q_OBJECT
public:
    explicit CCPushButton(QWidget *parent = nullptr);
    explicit CCPushButton(const QString &text, QWidget *parent = nullptr);
    void mouseReleaseEvent(QMouseEvent* ) override;
signals:
    void leftMouseClicked();
    void rightMouseClicked();
};

#endif // CCPUSHBUTTON_H
