#include <ros.h>
#include <opinau_msgs/motor.h>
#include <opinau_msgs/relay.h>

const int STEPS_PER_REVOLUTION = 200;
const int TOP_SPEED_DELAY = 3;
const int TOP_SPEED = 128;

const int RELAY_PIN = 12;

struct Motor
{
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
Motor motor_2 = {8, 9, 10, TOP_SPEED_DELAY, false, false, true};

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

  if (msg.speed == 0 || !msg.enabled)
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
  else if (msg.index == 1)
  {
    adjust_motor_parameters(msg, &motor_1);
  }
  else
  {
    adjust_motor_parameters(msg, &motor_2);
  }
}

void relay_message_cb(const opinau_msgs::relay &msg)
{
  if (msg.index == 0)
  {
    if (msg.enabled)
    {
      digitalWrite(RELAY_PIN, HIGH);
    }
    else
    {
      digitalWrite(RELAY_PIN, LOW);
    }
  }
}

ros::Subscriber<opinau_msgs::motor> motor_sub("labeller_motors", &motor_message_cb);
ros::Subscriber<opinau_msgs::relay> relay_sub("labeller_relays", &relay_message_cb);

void run_motor(Motor motor)
{
  if (motor.running)
  {
    digitalWrite(motor.step_pin, HIGH);
    delay(motor.step_delay);
    nh.spinOnce();
    digitalWrite(motor.step_pin, LOW);
  }
}

void setup_motor(Motor motor)
{
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
  nh.subscribe(relay_sub);

  // Setup relay
  pinMode(RELAY_PIN, OUTPUT);

  setup_motor(motor_0);
  setup_motor(motor_1);
  setup_motor(motor_2);
}

void loop()
{
  nh.spinOnce();

  run_motor(motor_0);
  run_motor(motor_1);
  run_motor(motor_2);
}
