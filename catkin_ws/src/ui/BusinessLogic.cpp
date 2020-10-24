#include "BusinessLogic.h"
#include <RosThread.h>
#include <QQmlApplicationEngine>
#include <BottleModel.h>

BusinessLogic::BusinessLogic(QObject *parent, RosThread *rosThread, QQmlApplicationEngine *qml)
    : QObject(parent), m_ros(rosThread), m_qml(qml)
{
    connect(m_ros, &RosThread::inkStatusChanged, this, &BusinessLogic::onInkStatusChanged);

    m_bottleModel = new BottleModel;

    float progress = 0.0;

    for (int i = 0; i < 100; i++) {
        m_bottleModel->addBottle(Bottle(progress, "Wolf", "Medium"));
        progress += 0.01;
    }
}

void BusinessLogic::onInkStatusChanged(bool labelPresent)
{
    setLabelPresent(labelPresent);
}

BottleModel *BusinessLogic::getBottleModel()
{
    return m_bottleModel;
}

int BusinessLogic::getConveyorSpeed() const
{
    return m_conveyorSpeed;
}

void BusinessLogic::setConveyorSpeed(int conveyorSpeed)
{
    m_conveyorSpeed = conveyorSpeed;
    emit conveyorSpeedChanged();
}

bool BusinessLogic::getLabelPresent() const
{
    return m_labelPresent;
}

void BusinessLogic::setLabelPresent(bool value)
{
    if (m_labelPresent != value) {
        m_labelPresent = value;
        emit labelPresentChanged();
    }

    // Update UI --> is this the right approach
    // Perhaps use Q_PROPRETIES accesses from QML?

    // Would it work with the more dynamic bottles?
    //    QObject *labelSensor = m_qml->findChild<QObject*>("labelSensor");
    //    if (labelSensor)
    //        labelSensor->setProperty("labelPresent", QVariant(value));
}
