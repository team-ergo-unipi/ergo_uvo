#include <thread>
#include <string>
#include <iostream>
#include <visual_odometry.h>


string VO_NODE;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "UVO_node");
  ros::NodeHandle main_node_obj;

  ros::Rate loop_rate(20);
  
  visual_odometry_node visual_odometry_node_object;

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    ros::param::get("/visual_odometry_node", VO_NODE);

    visual_odometry_node_object.visual_odometry_workflow(VO_NODE);
  }

  return 0;
}