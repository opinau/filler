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

#include "opinau_msgs/ink_status.h"

class RosThread : public QObject
{
  Q_OBJECT
public:
  RosThread(int argc, char** pArgv, const char* topic = "/odom");
  virtual ~RosThread();

  bool init();

  void inkStatusCallback(const opinau_msgs::ink_status&);

  void setPose(QList<double> to_set);

  public slots:
    void run();
    void test();
    void messageLabellerMotor(int, bool, int);
    void messageLabellerRelay(int, bool);
    void messageInk(bool, QString);

  signals:
    void newPose(double, double, double);
    void inkStatusChanged(QVariant labelPresent);

private:
  int m_Init_argc;
  char** m_pInit_argv;
  const char* m_topic;

  QThread* m_pThread;

  ros::Subscriber m_subInkStatus;
  ros::Publisher m_pubLabellerMotors;
  ros::Publisher m_pubLabellerRelays;
  ros::Publisher m_pubInk;

};
#endif
