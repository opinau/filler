#include "RosThread.h"
#include "std_msgs/String.h"
#include "opinau_msgs/motor.h"

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
    m_pThread->start();

    return true;
}

// void RosThread::poseCallback(const nav_msgs::Odometry & msg)
//{
//    QMutex * pMutex = new QMutex();

//    pMutex->lock();
//    m_xPos = msg.pose.pose.position.x;
//    m_yPos = msg.pose.pose.position.y;
//    m_aPos = msg.pose.pose.orientation.w;
//    pMutex->unlock();

//    delete pMutex;
//    Q_EMIT newPose(m_xPos, m_yPos, m_aPos);
//}//callback method to update the robot's position.

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
    // Do we need this mutex
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

double RosThread::getXSpeed()
{
    return m_speed;
}
double RosThread::getASpeed()
{
    return m_angle;
}

double RosThread::getXPos()
{
    return m_xPos;
}
double RosThread::getYPos()
{
    return m_yPos;
}
double RosThread::getAPos()
{
    return m_aPos;
}
