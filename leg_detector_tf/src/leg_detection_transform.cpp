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

void LegDetectorTf::messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	++distance.header.seq;
	distance.max = 0.0;
	distance.min = 1000.0;
	distance.avg = 0.0;
	std::vector<float> legs;
	for(int i = 0; i < msg->intensities.size(); i++) {
		if(msg->intensities[i] == 1.0) {
		    float angle = msg->angle_min + ((i+1) * msg->angle_increment);
		    ROS_DEBUG("LegDetectorTf::messageCallback: Angel calculation: a_min:%f + ((i:%i+1) * a_inc:%f) = %f", msg->angle_min, i, msg->angle_increment, angle);
		    transformLaserToBase(polarToCartesian(msg->ranges[i], angle)); //TODO: calculate angle.
			legs.push_back(msg->ranges[i]);
			ROS_DEBUG("LegDetectorTf::messageCallback: Range: %f", msg->ranges[i]);
			distance.max = msg->ranges[i] > distance.max ? msg->ranges[i] : distance.max;
			distance.min = msg->ranges[i] < distance.min ? msg->ranges[i] : distance.min;
			distance.avg += msg->ranges[i];
		}
	}
	distance.avg /= legs.size();
	ROS_DEBUG("LegDetectorTf::messageCallback: Range: Min: %f, Max: %f, Avg: %f", distance.min, distance.max, distance.avg);
	distance.header.stamp = ros::Time::now();
	publishMessage();
} // end publishCallback()

geometry_msgs::PointStamped LegDetectorTf::polarToCartesian(float dist, float angle) {
    ROS_DEBUG("LegDetectorTf::polarToCartesian: Received: distance: %f, angle: %f", dist, angle);
    geometry_msgs::PointStamped output;
    output.header.frame_id = "/base_laser_link";
    output.header.stamp = ros::Time();
    output.point.x = dist * cos(angle);
    output.point.y = dist * sin(angle);
    output.point.z = 0.365;
    ROS_DEBUG("LegDetectorTf::polarToCartesian: Cartesian point: x: %f, y: %f, z %f", output.point.x, output.point.y, output.point.z);
    return output;
}

geometry_msgs::PointStamped LegDetectorTf::transformLaserToBase(geometry_msgs::PointStamped input) {
    geometry_msgs::PointStamped output;
    try {
        listener.transformPoint("/base_link", input, output);

        ROS_DEBUG("LegDetectorTf::transformLaserToBase: base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
            input.point.x, input.point.y, input.point.z,
            output.point.x, output.point.y, output.point.z, output.header.stamp.toSec());
    } catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from \"base_laser_link\" to \"base_link\": %s", ex.what());
    }
    return output;
}

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
