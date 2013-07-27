#include "leg_detection_transform.h"

/*--------------------------------------------------------------------
 * NodeExample()
 * Constructor.
 *------------------------------------------------------------------*/

LegDetectorTf::LegDetectorTf(ros::Publisher *pub)
{
	distance.header.seq = 0;
	distance.header.frame_id = "leg_detector_tf";
	this->pub = pub;
} // end NodeExample()

/*--------------------------------------------------------------------
 * ~NodeExample()
 * Destructor.
 *------------------------------------------------------------------*/

LegDetectorTf::~LegDetectorTf()
{
} // end ~NodeExample()

/*--------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *------------------------------------------------------------------*/

void LegDetectorTf::publishMessage()
{
  pub->publish(distance);
} // end publishMessage()

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void  LegDetectorTf::messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	++distance.header.seq;
	distance.max = 0.0;
	distance.min = 1000.0;
	distance.avg = 0.0;
//	ROS_INFO("----------------------------------------------");
	std::vector<float> legs;
	for(int i = 0; i < msg->intensities.size(); i++) {
		if(msg->intensities[i] == 1.0) {
			legs.push_back(msg->ranges[i]);
//			ROS_INFO("Range: %f", msg->ranges[i]);
			distance.max = msg->ranges[i] > distance.max ? msg->ranges[i] : distance.max;
			distance.min = msg->ranges[i] < distance.min ? msg->ranges[i] : distance.min;
			distance.avg += msg->ranges[i];
		}
	}
	distance.avg /= legs.size();
	distance.header.stamp = ros::Time::now();
	publishMessage();
} // end publishCallback()

/*--------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *------------------------------------------------------------------*/

//void NodeExample::configCallback(node_example::node_example_paramsConfig &config, uint32_t level)
//{
//  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
//  message = config.message.c_str();
//  a = config.a;
//  b = config.b;
//} // end configCallback()
