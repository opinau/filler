#ifndef BUSINESSLOGIC_H
#define BUSINESSLOGIC_H

#include <QObject>
#include <BottleModel.h>

class RosThread;
class QQmlApplicationEngine;

class BusinessLogic : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool labelPresent READ getLabelPresent WRITE setLabelPresent NOTIFY labelPresentChanged)
    Q_PROPERTY(int conveyorSpeed READ getConveyorSpeed WRITE setConveyorSpeed NOTIFY conveyorSpeedChanged)
    Q_PROPERTY(BottleModel* bottleModel READ getBottleModel NOTIFY bottleModelChanged)

public:
    explicit BusinessLogic(QObject *parent, RosThread *rosThread, QQmlApplicationEngine *qml);

    void setLabelPresent(bool value);

    bool getLabelPresent() const;

    int getConveyorSpeed() const;
    void setConveyorSpeed(int conveyorSpeed);

    BottleModel* getBottleModel();

private slots:
    void onInkStatusChanged(bool labelPresent);

private:
    RosThread *m_ros;
    QQmlApplicationEngine *m_qml;

    bool m_labelPresent;
    int m_conveyorSpeed;

    BottleModel *m_bottleModel;

signals:
    void labelPresentChanged();
    void conveyorSpeedChanged();
    void bottleModelChanged();
};

#endif // BUSINESSLOGIC_H
