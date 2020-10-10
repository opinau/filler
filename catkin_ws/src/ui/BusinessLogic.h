#ifndef BUSINESSLOGIC_H
#define BUSINESSLOGIC_H

#include <QObject>

class RosThread;
class QQuickWindow;

class BusinessLogic : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool labelPresent READ getLabelPresent WRITE setLabelPresent NOTIFY labelPresentChanged)
    Q_PROPERTY(int conveyorSpeed READ getConveyorSpeed WRITE setConveyorSpeed NOTIFY conveyorSpeedChanged)

public:
    explicit BusinessLogic(QObject *parent, RosThread *rosThread, QQuickWindow *qml);

    void setLabelPresent(bool value);

    bool getLabelPresent() const;

    int getConveyorSpeed() const;
    void setConveyorSpeed(int conveyorSpeed);

private slots:
    void onInkStatusChanged(bool labelPresent);

private:
    RosThread *m_ros;
    QQuickWindow *m_qml;

    bool m_labelPresent;
    int m_conveyorSpeed;

signals:
    void labelPresentChanged();
    void conveyorSpeedChanged();

};

#endif // BUSINESSLOGIC_H
