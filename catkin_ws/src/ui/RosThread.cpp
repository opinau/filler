#include "RosThread.h"
#include "std_msgs/String.h"

RosThread::RosThread(int argc, char** pArgv, const char* topic) : m_Init_argc(argc), m_pInit_argv(pArgv), m_topic(topic)
{ /** Constructor for the robot thread **/
}

RosThread::~RosThread()
{
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }  // end if

  m_pThread->wait();
}  // end destructor

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

  pub = nh.advertise<std_msgs::String>("topic_name", 5);
  std_msgs::String str;
  str.data = "hello world";
  pub.publish(str);

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

void RosThread::SetSpeed(double speed, double angle)
{
  QMutex* pMutex = new QMutex();
  pMutex->lock();
  m_speed = speed;
  m_angle = angle;
  pMutex->unlock();

  delete pMutex;
}  // set the speed of the robot.

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
