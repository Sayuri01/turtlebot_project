#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <fstream>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include "find_object_2d/Float64MultiArrayStamped.h"

using namespace std;

class MarkerPublisher
{
public: 
	MarkerPublisher();
	void spin();
	
private:
	ros::NodeHandle nh;
	ros::Publisher pubMarkers_;
	ros::Subscriber subObjectPosition_;
	void objectsDetectedCallback(const find_object_2d::Float64MultiArrayStamped msg);
	
};

MarkerPublisher::MarkerPublisher(): nh("~")
{
	pubMarkers_ = nh.advertise<visualization_msgs::MarkerArray>("/markersPublisher", 1);
	subObjectPosition_ = nh.subscribe("/objectStorage", 1, &MarkerPublisher::objectsDetectedCallback, this);;
	
	ROS_INFO("marker_publisher ready to receive object positions !");
}

void MarkerPublisher::spin() {
    
     ros::Rate loop(10);
    
     while(ros::ok()) {
         
        // call all waiting callbacks
        ros::spinOnce();
        // enforce a max publish rate
        loop.sleep();
        //ros::spin();
      }
}

void MarkerPublisher::objectsDetectedCallback(const find_object_2d::Float64MultiArrayStamped msg)
{
	// Marker array creation
	visualization_msgs::MarkerArray markers;
	
	// Retrieve some information from the message
	int array_width = msg.positions.layout.dim[1].size;
	int array_height = msg.positions.layout.dim[0].size;
	
	int id;
	float x_position;
	float y_position;
	float z_position;
	
	// For each object in the message, create a marker
	for(int i=0; i<array_height; i++)
	{
		id = msg.positions.data[i*array_width ];
		x_position = msg.positions.data[i*array_width + 1];
		y_position = msg.positions.data[i*array_width + 2];
		z_position = msg.positions.data[i*array_width + 3];
		
		printf("\nObject %d: [%f %f %f]\n", id, x_position, y_position, z_position);
		
		// Define marker properties
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = msg.header.stamp;
		marker.ns = "objects";
		// if in rviz, we want 1 marker to recognize a box with 4 images
		//marker.id = id;
		// if in rviz, we want 4 markers to recognize a box with 4 images
		marker.id = i*array_width;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x_position;
		marker.pose.position.y = y_position;
		marker.pose.position.z = z_position;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.3;
		marker.scale.y = 0.3;
		marker.scale.z = 0.3;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		
		// Define the marker shape -> Here is a cube
		marker.type = visualization_msgs::Marker::CUBE;
		// Put the marker in the marker array
		//markers.markers.push_back(marker);
		
		// Creation of marker text
		std::string object_title;
		switch(id){
			case 0:
				object_title = "nosmoking";
				break;
				
			case 1:
				object_title = "radioactive";
				break;
				
			case 2:
				object_title = "toxic";
				break;
				
			case 3:
				object_title = "dead";
				break;
				
			case 4:
				object_title = "biohazard";
				break;
				
			case 5:
				object_title = "danger";
				break;
				
			case 6:
				object_title = "flamable";
				break;
				
			case 7:
				object_title = "alive";
				break;
			
			default:
				break;
		}
		
		// text
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.text = object_title;
		// if in rviz, we want 1 marker to recognize a box with 4 images
		//marker.id = id;
		// if in rviz, we want 4 markers to recognize a box with 4 images
		marker.id = -i*array_width;
		marker.color.a = 1.0;
		marker.pose.position.z += 0.3; // text over the cube
		// Put the marker in the marker array
		markers.markers.push_back(marker);
		
	}
			
	pubMarkers_.publish(markers);
}


int main(int argc, char** argv){
	
	//Init the ROS node
	ros::init(argc, argv, "marker_publisher");

	MarkerPublisher marker_publisher;
	
	// Keep the node running
	ros::spin();
	
	return 0;
}

