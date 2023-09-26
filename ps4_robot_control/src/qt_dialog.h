#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <sensor_msgs/Joy.h>
#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <vector>

using namespace std;

class MainDialog : public QDialog
{
  Q_OBJECT
public:
  MainDialog(QWidget* parent);

private:
  void joyCallback(const sensor_msgs::Joy& joy_msg);
  void setMaxPressure(float pressure);

  QLabel* arm_label;
  QLabel* arm;
  QLabel* pressure_label;
  QLabel* pressure;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  vector<ros::Publisher> cmd_pub_;
  ros::Publisher* now_cmd_pub_;
  ros::Publisher hand_pressure_pub_;
  ros::Subscriber joy_sub_;

  sensor_msgs::Joy last_msg;

  float default_max;
  bool is_right;
  float current_pressure;
  float release_pressure;
  float max_pressure_high_limit;
  float max_pressure_low_limit;
  int pitch;
  int roll;

};
