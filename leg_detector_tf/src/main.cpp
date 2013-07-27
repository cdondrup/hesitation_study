#include "leg_detection_transform.h"

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "leg_detector_tf");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  string sup_topic, pub_topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("sup_topic", sup_topic, string("/people_detector_node/people_detected"));
  private_node_handle_.param("pub_topic", pub_topic, string("/leg_detector_tf/distance"));

  // Create a new NodeExample object.
  ros::Publisher pub_message = n.advertise<leg_detector_tf_msgs::Distance>(pub_topic.c_str(), 10);
  LegDetectorTf *ldt = new LegDetectorTf(&pub_message);

  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  ros::Subscriber sub_message = n.subscribe(sup_topic.c_str(), 1000, &LegDetectorTf::messageCallback, ldt);

  // Main loop.
  ros::spin();

  return 0;
} // end main()
