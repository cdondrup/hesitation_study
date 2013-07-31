#include "leg_detection_transform.h"

/*--------------------------------------------------------------------
 * NodeExample()
 * Constructor.
 *------------------------------------------------------------------*/

LegDetectorTf::LegDetectorTf(ros::Publisher *pub_msg, ros::Publisher *pub_vis) :
    pub_msg(pub_msg),
    pub_vis(pub_vis)
{
	distance.header.seq = 0;
	distance.header.frame_id = "leg_detector_tf";
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

void LegDetectorTf::publishMessage(leg_detector_tf_msgs::Distance distance)
{
  pub_msg->publish(distance);
} // end publishMessage()

void LegDetectorTf::publishVisualisation(visualization_msgs::MarkerArray marker_array) {
    pub_vis->publish(marker_array);
}

void LegDetectorTf::createVisualisation(std::vector<geometry_msgs::PointStamped> leg_points) {
    visualization_msgs::MarkerArray marker_array;
    for(int i = 0; i < leg_points.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "leg_detector_tf";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = leg_points[i].point.x;
        marker.pose.position.y = leg_points[i].point.y;
        marker.pose.position.z = leg_points[i].point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    publishVisualisation(marker_array);
}

/*--------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *------------------------------------------------------------------*/

void LegDetectorTf::messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::vector<geometry_msgs::PointStamped> leg_points;
	++distance.header.seq;
	distance.max = 0.0;
	distance.min = 1000.0;
	distance.avg = 0.0;
	std::vector<float> legs;
	for(int i = 0; i < msg->intensities.size(); i++) {
		if(msg->intensities[i] == 1.0) {
		    float angle = msg->angle_min + ((i+1) * msg->angle_increment);
		    ROS_DEBUG("LegDetectorTf::messageCallback: Angel calculation: a_min:%f + ((i:%i+1) * a_inc:%f) = %f", 
		        msg->angle_min, 
		        i, 
		        msg->angle_increment, 
		        angle);
		    geometry_msgs::PointStamped coords = polarToCartesian(msg->ranges[i], angle);
		    transformLaser(coords, "/base_link");
		    leg_points.push_back(transformLaser(coords, "/map"));
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
	publishMessage(distance);
	createVisualisation(leg_points);
} // end publishCallback()

geometry_msgs::PointStamped LegDetectorTf::polarToCartesian(float dist, float angle) {
    ROS_DEBUG("LegDetectorTf::polarToCartesian: Received: distance: %f, angle: %f", dist, angle);
    geometry_msgs::PointStamped output;
    output.header.frame_id = "/base_laser_link";
    output.header.stamp = ros::Time();
    output.point.x = dist * cos(angle);
    output.point.y = dist * sin(angle);
    output.point.z = 0.0;
    ROS_DEBUG("LegDetectorTf::polarToCartesian: Cartesian point: x: %f, y: %f, z %f", output.point.x, output.point.y, output.point.z);
    return output;
}

geometry_msgs::PointStamped LegDetectorTf::transformLaser(geometry_msgs::PointStamped input, std::string target) {
    ROS_DEBUG("LegDetectorTf::transformLaser: Transforming %s to %s", input.header.frame_id.c_str(), target.c_str());
    geometry_msgs::PointStamped output;
    try {
        listener.transformPoint(target, input, output);

        ROS_DEBUG("LegDetectorTf::transformLaser: %s: (%.2f, %.2f. %.2f) ---> %s: (%.2f, %.2f, %.2f) at time %.2f",
            input.header.frame_id.c_str(), input.point.x, input.point.y, input.point.z,
            output.header.frame_id.c_str(), output.point.x, output.point.y, output.point.z, output.header.stamp.toSec());
    } catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s",
            input.header.frame_id.c_str(),
            output.header.frame_id.c_str(),
            ex.what());
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
