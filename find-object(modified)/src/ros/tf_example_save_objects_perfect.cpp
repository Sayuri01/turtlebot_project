/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <QtCore/QString>
#include "std_msgs/Float64MultiArray.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include </usr/local/include/eigen3/Eigen/Geometry>

#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include "std_msgs/MultiArrayDimension.h"
#include "find_object_2d/float64_array_of_array.h"
#include <algorithm>

using namespace std;

class TfExample
{
public:
	TfExample() :
		mapFrameId_("/map"),
		objFramePrefix_("object")
	{
		ros::NodeHandle pnh("~");
		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

		ros::NodeHandle nh;
		subs_ = nh.subscribe("/objectsStamped", 1, &TfExample::objectsDetectedCallback, this);
		
		pubMarkers_ = nh.advertise<visualization_msgs::MarkerArray>("objectsMarkers", 1);
		pubObjectMapPosition_ = nh.advertise<std_msgs::Float64MultiArray>("objectStorage", 1);

	}

		
	vector<vector<float> > removeDuplicate(vector<vector<float> > vect){

		vector<vector<float> > temp;

		for(int i=0; i<vect.size(); i++){
			
			vector<float> duplicate = vect[i];
			int difference = 0;
			
			if(!temp.empty()){
				
				// Count duplicates of vect[i] in temp
				int dups = std::count(temp.begin(), temp.end(), vect[i]);
				
				// if there is none duplicate of vect[i]
				// then add it in temp
				if (dups == 0){
					temp.push_back(vect[i]);
				}

			} else {
				temp.push_back(vect[i]);
			}
		}
		
		return temp;
		
	}
	
	float round(float var) 
	{ 
		// we use array of chars to store number 
		// as a string. 
		char str[40];  
	  
		// Print in string the value of var  
		// with two decimal point 
		sprintf(str, "%.2f", var); 
	  
		// scan string value in var  
		sscanf(str, "%f", &var);  
	  
		return var;  
	} 

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
	{

		vector<float> objectDataVector(4);
		visualization_msgs::MarkerArray markers;
		
		if(msg->objects.data.size())
		{
			//char multiSubId = 'b';
			int previousId = -1;
			for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
			{
				// get data
				int id = (int)msg->objects.data[i];

				previousId = id;

				// "object_1", "object_1_b", "object_1_c", "object_2"
				std::string objectFrameId = QString("%1_%2").arg(objFramePrefix_.c_str()).arg(id).toStdString();

				tf::StampedTransform objectToCamera;
				try
				{
					// Get transformation from "object_#" frame to target frame "map"
					// The timestamp matches the one sent over TF
					tfListener_.lookupTransform(mapFrameId_, objectFrameId, msg->header.stamp, objectToCamera);
				}
				 
				catch(tf::TransformException & ex)
				{
					ROS_WARN("%s",ex.what());
					continue;
				}
				
				
				
				ROS_INFO("%s [x,y,z] in \"%s\" frame: [%f,%f,%f]",
						objectFrameId.c_str(), "/map",
						 roundf(objectToCamera.getOrigin().x() * 100) / 100 ,
						 roundf(objectToCamera.getOrigin().y() * 100) / 100 , 
						 roundf(objectToCamera.getOrigin().z() * 100) / 100 );
						 

				// if at least one object has been detected
				if (!this->objectVector.empty()){
					
					ROS_INFO("Object Number: %d", objectVector.size());
					
					objectDataVector[0] = msg->objects.data[0];
					/*objectDataVector[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
					objectDataVector[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
					objectDataVector[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;*/
					
					objectDataVector[1] = round(objectToCamera.getOrigin().x());
					objectDataVector[2] = round(objectToCamera.getOrigin().y());
					objectDataVector[3] = round(objectToCamera.getOrigin().z());
					
					objectVector.push_back(objectDataVector);
					
					vector<vector<float> > temp = removeDuplicate(objectVector);
					objectVector.erase(objectVector.begin(), objectVector.end());
					objectVector = temp;
					
					ROS_INFO("Object Number: %d", objectVector.size());

					
					
				} 
				// otherwise (which means if none object has been detected)
				else {
					
					ROS_INFO("First fill !!!");
					objectDataVector[0] = msg->objects.data[0];
					/*objectDataVector[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
					objectDataVector[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
					objectDataVector[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;*/
					
					objectDataVector[1] = round(objectToCamera.getOrigin().x());
					objectDataVector[2] = round(objectToCamera.getOrigin().y());
					objectDataVector[3] = round(objectToCamera.getOrigin().z());
					
					objectVector.push_back(objectDataVector);
					
				}
				
				// Prepare to publish object detected array
				std_msgs::Float64MultiArray objects;
				
				int height = objectVector.size();
				int width = objectVector[0].size();
				
				objects.data.resize(width * height);
				
				objects.layout.dim.push_back(std_msgs::MultiArrayDimension());
				objects.layout.dim.push_back(std_msgs::MultiArrayDimension());
				objects.layout.dim[0].label = "rows";
				objects.layout.dim[1].label = "columns";
				objects.layout.dim[0].size = height;
				objects.layout.dim[1].size = width;
				objects.layout.dim[0].stride = height * width;
				objects.layout.dim[1].stride = width;
				objects.layout.data_offset = 0;
				
				
				vector<float> vect(width * height);
				ROS_INFO("Size %d\n", width * height);
				
				for (int i=0; i<height; i++)
				{
					for (int j=0; j<width; j++)
					{
						vect[i*width + j] = objectVector[i][j];
					}
				}
				
				for(int i=0; i<vect.size(); i++){
					objects.data[i] = vect[i];
				}

				ROS_INFO("Publish Objects Message done ! ");
				pubObjectMapPosition_.publish(objects);
				
				
				// Prepare a marker
				visualization_msgs::Marker marker;
				marker.header.frame_id = "/map";
				marker.header.stamp = msg->header.stamp;
				marker.ns = "objects";
				marker.id = id;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = objectToCamera.getOrigin().x();
				marker.pose.position.y = objectToCamera.getOrigin().y();
				marker.pose.position.z = objectToCamera.getOrigin().z();
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 0.3;
				marker.scale.y = 0.3;
				marker.scale.z = 0.3;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 0.0;
				marker.color.b = 1.0;
				
				// cube
				marker.type = visualization_msgs::Marker::CUBE;
				markers.markers.push_back(marker);
				
				// Creation of marker text
				std::ostringstream stream;
			    stream << objectFrameId.c_str() << "_" << id;
			    std::string object_title = stream.str();
				
				// text
				marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
				marker.text = object_title;
				marker.id = -id;
				marker.color.a = 1.0;
				marker.pose.position.z += 0.3; // text over the cube
				markers.markers.push_back(marker);
				
				pubMarkers_.publish(markers);
				
			}
		}
	}
	


private:
	std::string mapFrameId_;
	std::string objFramePrefix_;
    ros::Subscriber subs_;
    tf::TransformListener tfListener_;
    
    ros::Publisher pubMarkers_;
    ros::Publisher pubObjectMapPosition_;
    vector<vector<float> > objectVector;
    
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tf_example_node_publisher");

    TfExample sync;
    ros::spin();
}
