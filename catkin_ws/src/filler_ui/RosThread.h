#ifndef ___RosThread_H___
#define ___RosThread_H___

#include <QtCore>
#include <QThread>
#include <QStringList>
#include <stdlib.h>
#include <QMutex>
#include <iostream>
#include "assert.h"

#include <ros/ros.h>
#include <ros/network.h>

class RosThread : public QObject
{
  Q_OBJECT
public:
  RosThread(int argc, char** pArgv, const char* topic = "/odom");
  virtual ~RosThread();

  double getXPos();
  double getXSpeed();
  double getASpeed();
  double getYPos();
  double getAPos();

  bool init();

  // void poseCallback(const nav_msgs::Odometry & msg);

  void SetSpeed(double speed, double angle);
  void setPose(QList<double> to_set);

  Q_SLOT void run();

  Q_SIGNAL void newPose(double, double, double);

private:
  int m_Init_argc;
  char** m_pInit_argv;
  const char* m_topic;

  double m_speed;
  double m_angle;

  double m_xPos;
  double m_yPos;
  double m_aPos;

  double m_maxRange;
  double m_minRange;

  QThread* m_pThread;

  ros::Subscriber pose_listener;
  ros::Publisher sim_velocity;
};
#endif
