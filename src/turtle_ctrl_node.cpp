#include <chrono>
#include <memory>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "turtle_ctrl_ros2/msg/goal.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// ros2 run turtle_ctrl_ros2 turtle_ctrl_ros2_node --ros-args -p lambda:=0.0 -p gamma:=0.0

class TurtleControl : public rclcpp::Node
{
  double goalX_ = 2.0;
  double goalY_ = 6.0;

  double lambda_; // = 2.0;
  double gamma_;  // = 0.5;

  bool controlling = false;

  rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr poseSub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_;
  rclcpp::Subscription<turtle_ctrl_ros2::msg::Goal>::SharedPtr goalSub_;
  rclcpp::TimerBase::SharedPtr timer_;

  turtlesim_msgs::msg::Pose turtlePose_;
  geometry_msgs::msg::Twist turtleCmdVel_;

  turtle_ctrl_ros2::msg::Goal goalMsg;

public:
  TurtleControl()
  : Node("turtle_ctrl_node")
  {
    // A questi parametri viene date un valore di default di 1.0,
    // nel caso non fossero specificati dall'utente.
    this->declare_parameter("gamma", 1.0);
    this->declare_parameter("lambda", 1.0);

    // Se utilizziamo il launch file, usando:
    // "ros2 launch turtle_ctrl launchCtrl.py"
    // andiamo a lanciare il nodo con i parametri specificati nel launch file.
    gamma_ = this->get_parameter("gamma").as_double();
    lambda_ = this->get_parameter("lambda").as_double();

    // Stampa a schermo dei valori usati
    std::cout << "gamma: " <<  gamma_ << ", lambda: " << lambda_ << std::endl;


    poseSub_ = this->create_subscription<turtlesim_msgs::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&TurtleControl::pose_callback, this, _1));
      
    goalSub_ = this->create_subscription<turtle_ctrl_ros2::msg::Goal>(
      "/turtle1/goal", 10, std::bind(&TurtleControl::goal_callback, this, _1));


    cmdVelPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&TurtleControl::ctrl_callback, this));
  }

private:
  void pose_callback(const turtlesim_msgs::msg::Pose::SharedPtr msg)
  {
    turtlePose_ = *msg;
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    //std::cout << "Pose: " << msg->x << ", " << msg->y << ", " << msg->theta << std::endl; 
  }

  void goal_callback(const turtle_ctrl_ros2::msg::Goal::SharedPtr msg)
  {
    goalMsg = *msg;

    goalX_ = msg->pos_x;
    goalY_ = msg->pos_y;
    controlling = true;
    RCLCPP_INFO(this->get_logger(), "New Goal received: (%.2f, %.2f)", goalX_, goalY_);
  }

  void ctrl_callback()
  {
    if (controlling == false){
      return;
    } else {
      auto cmd = geometry_msgs::msg::Twist();

      double goalHeading = std::atan2(goalY_ - turtlePose_.y, goalX_ - turtlePose_.x);
      double thetaErr = wrap_around_pi(turtlePose_.theta - goalHeading);
      auto yawRateCmd = -lambda_ * thetaErr;

      double posErr = std::sqrt(std::pow(goalX_ - turtlePose_.x, 2)
                                + std::pow(goalY_ - turtlePose_.y, 2));
      auto surgeCmd = gamma_ * posErr;
      surgeCmd = std::clamp(surgeCmd, -1.0, 1.0);
      
      cmd.linear.x = surgeCmd;    // u
      cmd.angular.z = yawRateCmd; // r
      cmdVelPub_->publish(cmd);


      std::cout << "Error: " << posErr << std::endl;
      if(std::abs(posErr) < 0.05){
        RCLCPP_INFO(this->get_logger(), "Goal Reached!");
        //rclcpp::shutdown();
        controlling = false;
      }
    }
  }

  //WRAP_AROUND_PI Bounds angle between -pi and pi
  double wrap_around_pi(double angle){ 

    if (angle > M_PI){
        angle = angle - 2*M_PI;
    } else if( angle < -M_PI){
      angle = angle + 2*M_PI;
    }
    return angle;
  }



};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());



  rclcpp::shutdown();
  return 0;
}
