#include <ros.h>
#include <opinau_msgs/motor.h>

const int STEPS_PER_REVOLUTION = 200;
const int TOP_SPEED_DELAY = 3;
const int TOP_SPEED = 128;

struct Motor {
  byte step_pin;
  byte direction_pin;
  byte enable_pin;

  int step_delay;
  bool running;
  bool enabled;
  bool forwards;
};

Motor motor_0 = {2, 3, 4, TOP_SPEED_DELAY, false, false, true};
Motor motor_1 = {5, 6, 7, TOP_SPEED_DELAY, false, false, true};

ros::NodeHandle nh;

void adjust_motor_parameters(const opinau_msgs::motor &msg, Motor *motor)
{
  if (msg.enabled != motor->enabled)
  {
    if (msg.enabled)
    {
      digitalWrite(motor->enable_pin, LOW);
    }
    else
    {
      digitalWrite(motor->enable_pin, HIGH);
    }
    motor->enabled = msg.enabled;
  }

  if (msg.speed < 0 && motor->forwards)
  {
    digitalWrite(motor->direction_pin, HIGH);
    motor->forwards = false;
  }
  else if (msg.speed > 0 && !motor->forwards)
  {
    digitalWrite(motor->direction_pin, LOW);
    motor->forwards = true;
  }

  if (msg.speed == 0)
  {
    motor->running = false;
  }
  else
  {
    motor->running = true;
    motor->step_delay = TOP_SPEED / abs(msg.speed) * TOP_SPEED_DELAY;
  }
}

void motor_message_cb(const opinau_msgs::motor &msg)
{
  if (msg.index == 0)
  {
    adjust_motor_parameters(msg, &motor_0);
  }
  else {
    adjust_motor_parameters(msg, &motor_1);
  }
}

ros::Subscriber<opinau_msgs::motor> motor_sub("labeller_motors", &motor_message_cb);

void run_motor(Motor motor) {
  if (motor.running)
  {
    digitalWrite(motor.step_pin, HIGH);
    delay(motor.step_delay);
    nh.spinOnce();
    digitalWrite(motor.step_pin, LOW);
  }
}

void setup_motor(Motor motor) {
  pinMode(motor.step_pin, OUTPUT);
  pinMode(motor.enable_pin, OUTPUT);
  pinMode(motor.direction_pin, OUTPUT);

  digitalWrite(motor.direction_pin, LOW);
  digitalWrite(motor.enable_pin, HIGH); // HIGH is enabled on for DM332T, LOW is on for HY & DM860T driver
}

void setup()
{
  nh.getHardware()->setBaud(115200); // needs to be synchronised with value in launch file
  nh.initNode();

  nh.subscribe(motor_sub);

  setup_motor(motor_0);
  setup_motor(motor_1);
}

void loop()
{
  nh.spinOnce();

  run_motor(motor_0);
  run_motor(motor_1);
}
