// Pure Pursuit Boilerplate Exercise
//
// Copyright (c) 2018 Karl D. Hansen
// MIT License - see LICENSE file.

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

int points_initialized = 0;
double from_x, from_y, to_x, to_y;

// Callback function to handle clicked points in RViz
// When more than two points have been clicked, the "to" point changes to the "from" point.
void clickCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  ROS_INFO("Clicked: %f, %f, %f", msg->point.x, msg->point.y, msg->point.z);
  switch (points_initialized)
  {
  case 0:
    from_x = msg->point.x;
    from_y = msg->point.y;
    break;
  case 1:
    to_x = msg->point.x;
    to_y = msg->point.y;
    break;
  default:
    from_x = to_x;
    from_y = to_y;
    to_x = msg->point.x;
    to_y = msg->point.y;
  }
  points_initialized += 1;
}

int main(int argc, char **argv)
{

  // Initialize the ROS node
  ros::init(argc, argv, "turtlebot_pure_pursuit");
  ros::NodeHandle node;

  // Publish our output; the wanted speed of the robot.
  ros::Publisher turtle_vel =
      node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Initialize the transform listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Subscribe to the clicked point from RViz
  ros::Subscriber click_sub = node.subscribe("clicked_point", 1000, clickCallback);

  // The business code
  // - Lookup the position of the robot in the map frame.
  // - Compute the lookahead point.
  // - Find the coresponding linear and angular speeds.
  // - Publish them.
  ros::Rate rate(10.0);
  while (node.ok())
  {
    // Execute any callbacks
    ros::spinOnce();

    // Check that two points has been clicked
    if (points_initialized < 2)
    {
      rate.sleep();
      continue;
    }

    // Lookup position and orientation using the tf2 system.
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform("base_footprint", "map",
                                                  ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    double robot_pos_x = transformStamped.transform.translation.x;
    double robot_pos_y = transformStamped.transform.translation.y;
    double robot_rool, robot_pitch, robot_yaw;
    tf2::getEulerYPR(transformStamped.transform.rotation,
                     robot_yaw, robot_pitch, robot_rool);

    // Lookahead point
    // TODO: Compute the right values for the lookahead.
    // Use the values of: from_x, from_y, to_x, to_y, robot_pos_x, robot_pos_y
    double lookahead_x;
    double lookahead_y;

    // Compute coresponding speeds and publish them
    geometry_msgs::Twist vel_msg;

    // Angular speed
    // TODO: Find angle between heading (yaw) and direction to the lookahead
    //       point. Use the atan2 function and the points lookahead_x/y and
    //       robot_pos_x/y.
    //       Consider using a scalar on the angle difference before sending the
    //       speed reference.
    vel_msg.angular.z;

    // Linear speed
    // TODO: Find the Euclidean distance between the lookahead point and the
    //       robot. Use the sqrt and pow functions, and the the points
    //       lookahead_x/y and robot_pos_x/y.
    //       Again, consider a scalar.
    vel_msg.linear.x;

    // Publish
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
