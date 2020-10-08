#include "RosThread.h"
#include "std_msgs/String.h"
#include "opinau_msgs/motor.h"
#include "opinau_msgs/ink.h"
#include "opinau_msgs/relay.h"

RosThread::RosThread(int argc, char** pArgv, const char* topic) : m_Init_argc(argc), m_pInit_argv(pArgv), m_topic(topic)
{ /** Constructor for the robot thread **/
}

RosThread::~RosThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }

    m_pThread->wait();
}

bool RosThread::init()
{
    m_pThread = new QThread();
    this->moveToThread(m_pThread);

    connect(m_pThread, &QThread::started, this, &RosThread::run);
    ros::init(m_Init_argc, m_pInit_argv, "ui");

    if (!ros::master::check())
        return false;  // do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;

    m_pubLabellerMotors = nh.advertise<opinau_msgs::motor>("labeller_motors", 5);
    m_pubInk = nh.advertise<opinau_msgs::motor>("ink", 5);
    m_pubLabellerRelays = nh.advertise<opinau_msgs::relay>("labeller_relays", 5);


    m_subInkStatus = nh.subscribe("ink_status", 1, &RosThread::inkStatusCallback, this);
    m_pThread->start();

    return true;
}

 void RosThread::inkStatusCallback(const opinau_msgs::ink_status &msg)
{
    //qDebug() << "ink status changed";
    QMutex * pMutex = new QMutex();

    pMutex->lock();

    emit inkStatusChanged(msg.label_present);

    pMutex->unlock();

    delete pMutex;
}

void RosThread::run()
{
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RosThread::test()
{
    qDebug() << "in slot";
}

void RosThread::messageLabellerMotor(int index, bool enabled, int speed)
{
    qDebug() << "messaging motor" << index << enabled << speed;
    // Do we need this lock
    QMutex* pMutex = new QMutex();
    pMutex->lock();

    opinau_msgs::motor motorMessage;
    motorMessage.index = index;
    motorMessage.enabled = enabled;
    motorMessage.speed = speed;

    m_pubLabellerMotors.publish(motorMessage);

    pMutex->unlock();

    delete pMutex;
}

void RosThread::messageLabellerRelay(int index, bool enabled)
{
    qDebug() << "messaging relay" << index << enabled;
    // Do we need this lock
    QMutex* pMutex = new QMutex();
    pMutex->lock();

    opinau_msgs::relay relayMessage;
    relayMessage.index = index;
    relayMessage.enabled = enabled;

    m_pubLabellerRelays.publish(relayMessage);

    pMutex->unlock();

    delete pMutex;
}

void RosThread::messageInk(bool enabled, QString bbd)
{
    qDebug() << "messaging ink" << enabled << bbd;
    QMutex* pMutex = new QMutex();
    pMutex->lock();

    opinau_msgs::ink inkMessage;
    inkMessage.enabled = enabled;
    inkMessage.bbd_lot = bbd.toStdString();

    m_pubInk.publish(inkMessage);

    pMutex->unlock();

    delete pMutex;
}
