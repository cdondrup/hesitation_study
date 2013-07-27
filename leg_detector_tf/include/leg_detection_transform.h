#ifndef LEG_DETECTOR_TRANSFORM_H
#define LEG_DETECTOR_TRANSFORM_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"

#include <string.h>
#include <vector>

#include "leg_detector_tf_msgs/Distance.h"

// Custom message includes. Auto-generated from msg/ directory.
// #include "node_example/node_example_data.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
// #include <node_example/node_example_paramsConfig.h>

using std::string;

class LegDetectorTf
{
private:
	leg_detector_tf_msgs::Distance distance;
	ros::Publisher *pub;
public:
  //! Constructor.
  LegDetectorTf(ros::Publisher *pub);

  //! Destructor.
  ~LegDetectorTf();

  //! Callback function for dynamic reconfigure server.
  //void configCallback(leg_detector_transform::leg_detector_transform_paramsConfig &config, uint32_t level);

  //! Publish the message.
  void publishMessage();

  //! Callback function for subscriber.
  void messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg);


};

#endif // LEG_DETECTOR_TRANSFORM_h
