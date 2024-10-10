#include "CCPushButton.h"
#include <QMouseEvent>

CCPushButton::CCPushButton(QWidget *parent)
    : QPushButton{parent}
{

}

CCPushButton::CCPushButton(const QString &text, QWidget *parent)
    :QPushButton{text, parent}
{

}

void CCPushButton::mouseReleaseEvent(QMouseEvent *e)
{
    if (Qt::LeftButton == e->button())
    {
        emit leftMouseClicked();
    }
    else if (Qt::RightButton == e->button())
    {
        emit rightMouseClicked();
    }
}
