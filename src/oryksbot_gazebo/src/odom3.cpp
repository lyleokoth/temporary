#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <iostream>

using namespace std;

double goal_linear_velocity = 0.0;
double goal_angular_velocity = 0.0;
double left_wheel_speed = 0.0;
double right_wheel_speed = 0.0;
ros::Time last_time;
ros::Time current_time;
double wheel_separation = 0.160;
double wheel_radius = 0.033;
double vx = 0.0;
double vy = 0.0;
double x = 0.0;
double y = 0.0;
double th = 0.0;
nav_msgs::Odometry odom;
sensor_msgs::JointState joint_states_;
ros::Publisher joint_states_pub_;

double last_position_left = 0.0;
double omega_left = 0.0;

double last_position_right = 0.0;
double omega_right = 0.0;

ros::Publisher left_wheel_vel_pub;
ros::Publisher right_wheel_vel_pub;

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
    current_time = ros::Time::now();
    goal_linear_velocity  = cmd_vel_msg->linear.x; // linear velocity in m/s
    goal_angular_velocity = cmd_vel_msg->angular.z;// angular velocity in rad/s

    //Both velocities in m/s
    left_wheel_speed  = goal_linear_velocity - (goal_angular_velocity * wheel_separation / 2 * wheel_radius);
    right_wheel_speed = goal_linear_velocity + (goal_angular_velocity * wheel_separation / 2 * wheel_radius);
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool updateOdometry(double dt)
{
    double delta_th = goal_angular_velocity * dt;
    double delta_x = vx * dt;
    double delta_y = vy * dt;

    vx = ((left_wheel_speed + right_wheel_speed)*cos(th)/2);
    vy = ((left_wheel_speed + right_wheel_speed)*sin(th)/2);

    omega_left = left_wheel_speed/wheel_radius;
    double d_th_left = omega_left * dt;
    last_position_left += d_th_left;

    omega_right = right_wheel_speed/wheel_radius;
    double d_th_right = omega_right * dt;
    last_position_right += d_th_right;    

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //Filling the odometry
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th);

    //Position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0; 
    odom.pose.pose.orientation = odom_quat;

    //velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = goal_angular_velocity; 

    return true;
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void updateJoint(void)
{
  joint_states_.position[0]  = last_position_left;
  joint_states_.position[1] = last_position_right;
  joint_states_.velocity[0]  = omega_left;
  joint_states_.velocity[1] = omega_right;
}

void update_gazebo()
{
    std_msgs::Float64 left_speed;
    std_msgs::Float64 right_speed;

    if(goal_angular_velocity == 0)
    {
      left_speed.data = left_wheel_speed*5;
      right_speed.data = right_wheel_speed*5;      
      left_wheel_vel_pub.publish(left_speed);
      right_wheel_vel_pub.publish(right_speed);
    }
    else if(goal_angular_velocity > 0)
    {
      left_speed.data = left_wheel_speed*1000;
      right_speed.data = right_wheel_speed*1000;      
      right_wheel_vel_pub.publish(right_speed);
      //left_speed.data = 0.0;
      left_wheel_vel_pub.publish(left_speed);
    }
    else{
      left_speed.data = left_wheel_speed*1000;
      right_speed.data = right_wheel_speed*1000;      
      left_wheel_vel_pub.publish(left_speed);
      //right_speed.data = 0.0;
      right_wheel_vel_pub.publish(right_speed);      
    }
}

/*******************************************************************************
* Update function
*******************************************************************************/
bool update(ros::Publisher odom_pub, tf::TransformBroadcaster tf_broadcaster)
{
  ros::Time time_now = ros::Time::now();
  double step_time = (time_now - last_time).toSec();
  last_time = time_now;

  // odom
  updateOdometry(step_time);
  odom.header.stamp = time_now;
  odom_pub.publish(odom);

  // joint_states
  updateJoint();
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  // tf
  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  tf_broadcaster.sendTransform(odom_tf);

  //update gazebo
  update_gazebo();

  return true;
}

int main(int argc, char** argv)
{
    ROS_INFO("---Initializing the robot_state_publisher------");
    ros::init(argc, argv, "robot_state_publisher_2");
    ROS_INFO("Initialized the robot_state_publisher...");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    ROS_INFO("Created the odom publisher...");
    tf::TransformBroadcaster tf_broadcaster;
    ROS_INFO("Created the transform publisher...");
    ros::Subscriber cmdVel_sub = nh.subscribe("cmd_vel", 10, commandVelocityCallback);
    ROS_INFO("Created the cmd_vel subscriber...");
    joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    ROS_INFO("Created the joint state publisher...");
    left_wheel_vel_pub = nh.advertise<std_msgs::Float64>("left_wheel_velocity_controller/command", 10);
    right_wheel_vel_pub = nh.advertise<std_msgs::Float64>("right_wheel_velocity_controller/command", 10);

    joint_states_.header.frame_id = "base_footprint";
    joint_states_.name.push_back("left_joint");
    joint_states_.name.push_back("right_joint");    
    joint_states_.position.resize(2,0.0);
    joint_states_.velocity.resize(2,0.0);
    joint_states_.effort.resize(2,0.0);    

    ros::Rate loop_rate(30);

    while (ros::ok()) 
    {
        update(odom_pub, tf_broadcaster);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}