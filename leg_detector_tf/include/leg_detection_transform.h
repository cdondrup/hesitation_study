#ifndef LEG_DETECTOR_TRANSFORM_H
#define LEG_DETECTOR_TRANSFORM_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>

#include <string.h>
#include <vector>
#include <math.h>

#include "leg_detector_tf_msgs/Distance.h"

// Custom message includes. Auto-generated from msg/ directory.
// #include "node_example/node_example_data.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
// #include <node_example/node_example_paramsConfig.h>

#define MAX_LEG_DISTANCE 5

using std::string;

class LegDetectorTf
{
private:
    ros::Publisher *pub_msg, *pub_vis, *pub_detect;
    tf::TransformListener listener;
    long unsigned int dist_seq, marker_seq, detect_seq;

    geometry_msgs::PointStamped polarToCartesian(float dist, float angle);
    geometry_msgs::PointStamped transformLaser(geometry_msgs::PointStamped, std::string);
    
    void createVisualisation(std::vector<geometry_msgs::PointStamped>);

    //! Publish the message.
    void publishMessage(leg_detector_tf_msgs::Distance distance);
    void publishVisualisation(visualization_msgs::MarkerArray marker_array);
    void publishDetections(std::vector<geometry_msgs::PointStamped> centers);
    std::vector<geometry_msgs::PointStamped> getCenterPoints(std::vector<std::vector<geometry_msgs::PointStamped> >);
	
public:
  //! Constructor.
  LegDetectorTf(ros::Publisher *pub_msg, ros::Publisher *pub_vis, ros::Publisher *pub_detect);

  //! Destructor.
  ~LegDetectorTf();

  //! Callback function for dynamic reconfigure server.
  //void configCallback(leg_detector_transform::leg_detector_transform_paramsConfig &config, uint32_t level);

  //! Callback function for subscriber.
  void messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg);


};

#endif // LEG_DETECTOR_TRANSFORM_h
