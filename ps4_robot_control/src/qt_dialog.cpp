#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include "qt_dialog.h"

MainDialog::MainDialog(QWidget* parent)
  : QDialog(parent),
    nh_(), 
    pnh_("~"), 
    last_msg(sensor_msgs::Joy()), 
    default_max(0.04),
    is_right(true), 
    current_pressure(0.0), 
    release_pressure(0.0), 
    max_pressure_high_limit(1.5), 
    max_pressure_low_limit(-0.3),
    pitch(0), 
    roll(0)
{
  // Qt
  arm_label = new QLabel(this);
  arm = new QLabel(this);
  pressure_label = new QLabel(this);
  pressure = new QLabel(this);

  arm_label->setText("Arm : ");
  arm->setText("Right");
  pressure_label->setText("Pressure : ");
  pressure->setText("no signal");

  QHBoxLayout* horizontal_layout1 = new QHBoxLayout;
  horizontal_layout1->addWidget(arm_label);
  horizontal_layout1->addWidget(arm);

  QHBoxLayout* horizontal_layout2 = new QHBoxLayout;
  horizontal_layout2->addWidget(pressure_label);
  horizontal_layout2->addWidget(pressure);
  
  QVBoxLayout* vertical_layout = new QVBoxLayout;
  vertical_layout->addLayout(horizontal_layout1);
  vertical_layout->addLayout(horizontal_layout2);
  setLayout(vertical_layout);


  // ROS
  cmd_pub_ = {
    nh_.advertise<geometry_msgs::Twist>("/cmd_vel/right", 1), 
    nh_.advertise<geometry_msgs::Twist>("/cmd_vel/left", 1)
  };
  now_cmd_pub_ = &cmd_pub_[0];
  hand_pressure_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/hand_ref_pressure", 1);
  joy_sub_ = nh_.subscribe("joy", 10, &MainDialog::joyCallback, this);

  // string_pub_ = nh_.advertise<std_msgs::String>("chatter", 10);
}

void MainDialog::setMaxPressure(float set_pressure) {
  set_pressure = min(set_pressure, max_pressure_high_limit);
  set_pressure = max(set_pressure, max_pressure_low_limit);
  current_pressure = set_pressure;
  pressure->setText(QString("%1").arg(set_pressure));
}

void MainDialog::joyCallback(const sensor_msgs::Joy& joy_msg) {
    int assign_x = 1; //axes, 左joyの左右
    int assign_y = 0; //axes, 左joyの上下
    int assign_z = 5; //axes, 右joyの上下
    int assign_yaw = 4; //buttons, L1
    int assign_yaw_r = 5; //buttons, R1

    int assign_grab = 2; //buttons, 丸
    int assign_release = 0; //buttons, 四角

    int max_increment = 3; // buttons, 三角
    int max_decrement = 1; // buttons, バツ

    int switch_left = 6; // buttons, L2
    int switch_right = 7; // buttons, R2


    float max_x = default_max;
    float max_y = default_max;
    float max_z = default_max;
    float max_yaw = default_max * 10;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = max_x * joy_msg.axes[assign_x];
    cmd_vel.linear.y = max_y * joy_msg.axes[assign_y];
    cmd_vel.linear.z = max_z * joy_msg.axes[assign_z];

    float kaiten = joy_msg.buttons[assign_yaw] - joy_msg.buttons[assign_yaw_r];
    cmd_vel.angular.z = max_yaw * kaiten;

    now_cmd_pub_->publish(cmd_vel);

    if(joy_msg.buttons[assign_grab] > 0) {
      // current_pressure = 0.8;
      setMaxPressure(0.8);
      std_msgs::Float64MultiArray hand_ref_pressure;
      hand_ref_pressure.data.assign(2, release_pressure);
      if(is_right) {
        hand_ref_pressure.data[1] = current_pressure;
      } else {
        hand_ref_pressure.data[0] = current_pressure;
      }
      hand_pressure_pub_.publish(hand_ref_pressure);
    }
    if(joy_msg.buttons[assign_release] > 0) {
      // current_pressure = 0.0;
      setMaxPressure(0.0);
      std_msgs::Float64MultiArray hand_ref_pressure;
      hand_ref_pressure.data.assign(2, release_pressure);
      hand_pressure_pub_.publish(hand_ref_pressure);
    }
    if(joy_msg.buttons[max_increment] > 0 and last_msg.buttons[max_increment] == 0) {
      // current_pressure += 0.1;
      // current_pressure = min(current_pressure, max_pressure_high_limit);
      setMaxPressure(current_pressure + 0.1);
      std_msgs::Float64MultiArray hand_ref_pressure;
      hand_ref_pressure.data.assign(2, release_pressure);
      if(is_right) {
        hand_ref_pressure.data[1] = current_pressure;
      } else {
        hand_ref_pressure.data[0] = current_pressure;
      }
      hand_pressure_pub_.publish(hand_ref_pressure);
    }
    if(joy_msg.buttons[max_decrement] > 0 and last_msg.buttons[max_decrement] == 0) {
      // current_pressure -= 0.1;
      // current_pressure = max(current_pressure, max_pressure_low_limit);
      setMaxPressure(current_pressure - 0.1);
      std_msgs::Float64MultiArray hand_ref_pressure;
      hand_ref_pressure.data.assign(2, release_pressure);
      if(is_right) {
        hand_ref_pressure.data[1] = current_pressure;
      } else {
        hand_ref_pressure.data[0] = current_pressure;
      }
      hand_pressure_pub_.publish(hand_ref_pressure);
    }
    if(joy_msg.buttons[switch_right] > 0 and last_msg.buttons[switch_right] == 0) {
      // current_pressure = 0.0;
      setMaxPressure(0.0);
      now_cmd_pub_ = &cmd_pub_[0];
      is_right = true;
      arm->setText("Right");
    }
    if(joy_msg.buttons[switch_left] > 0 and last_msg.buttons[switch_left] == 0) {
      // current_pressure = 0.0;
      setMaxPressure(current_pressure + 0.0);
      now_cmd_pub_ = &cmd_pub_[1];
      is_right = false;
      arm->setText("Left");
    }

    last_msg = joy_msg;
}