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
		subs_ = nh.subscribe("/objectsStamped", 10, &TfExample::objectsDetectedCallback, this);
		
		pubMarkers_ = nh.advertise<visualization_msgs::MarkerArray>("objectsMarkers", 1);
		pubObjectMapPosition_ = nh.advertise<find_object_2d::float64_array_of_array>("objectStorage", 1, true);
		
		
	}
	
	
	
	std_msgs::Float64MultiArray convertVectorToFloatMultiArray(vector<float> objectDataVector){
		
		std_msgs::Float64MultiArray objectData;
		objectData.data.resize(4);
		
		for(int i=0; i<objectVector.size(); i++){
			objectData.data[i] = objectDataVector[i];
		}
		
		return objectData;
	}
	
	static bool sortFunc( vector<float> p1, vector<float> p2 ) {
		return p1[1] < p2[1];
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
	

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
	{

		vector<float> objectDataVector(4);
		
		/*std_msgs::Float64MultiArray objectData;
		objectData.data.resize(4);*/
		
		visualization_msgs::MarkerArray markers;
		
		//find_object_2d::float64_array_of_array objects;
		std_msgs::Float64MultiArray objects;

		if(msg->objects.data.size())
		{
			//char multiSubId = 'b';
			int previousId = -1;
			for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
			{
				// get data
				int id = (int)msg->objects.data[i];

				/*if(id == previousId)
				{
					multiSuffix = QString("_") + multiSubId++;
				}
				else
				{
					multiSubId = 'b';
				}*/
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
				
				
				/*if (!this->objectVector.empty()){
					
					ROS_INFO("Object Number: %d", objectVector.size());
					
					//find_object_2d::float64_array_of_array same_id_objects;
					vector<vector<float> > same_id_objects_vector;
					
					// Get all objects.array[i] with the same id as msg->objects.data[0]
					for(int i=0; i < objectVector.size(); i++){
						
						//ROS_INFO("Object Number id: %f", objectVector[i][0]);
						if( objectVector[i][0] == msg->objects.data[0] ){
							same_id_objects_vector.push_back(objectVector[i]);
						}
					}
					
					std::sort(same_id_objects_vector.begin(), same_id_objects_vector.end(), TfExample::sortFunc);
					
					bool duplicate = false;
					
					if(same_id_objects_vector.size() >= 2){
						
						for(int i=0; i < same_id_objects_vector.size(); i++){
							
							if( (fabs(same_id_objects_vector[i][1] - roundf(objectToCamera.getOrigin().x() * 100) / 100) > 0.5) 
							 || (fabs(same_id_objects_vector[i][2] - roundf(objectToCamera.getOrigin().y() * 100) / 100) > 0.5) )
							{
								
							}
							
						}
					
					}
					
					
					
					
				} else {
					
					ROS_INFO("First fill !!!");
					objectDataVector[0] = msg->objects.data[0];
					objectDataVector[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
					objectDataVector[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
					objectDataVector[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;
					
					objectVector.push_back(objectDataVector);
					//objects.array.push_back( convertVectorToFloatMultiArray(objectDataVector) );
					
				}*/
				
				
				
				if (!this->objectVector.empty()){
					
					ROS_INFO("Object Number: %d", objectVector.size());
					
					//find_object_2d::float64_array_of_array same_id_objects;
					vector<vector<float> > same_id_objects_vector;
					
					// Get all objects.array[i] with the same id as msg->objects.data[0]
					for(int i=0; i < objectVector.size(); i++){
						
						//ROS_INFO("Object Number id: %f", objectVector[i][0]);
						if( objectVector[i][0] == msg->objects.data[0] ){
							same_id_objects_vector.push_back(objectVector[i]);
						}
					}
					
					std::sort(same_id_objects_vector.begin(), same_id_objects_vector.end(), TfExample::sortFunc);
					//ROS_INFO("Before Size: %d" ,same_id_objects_vector.size());
					//same_id_objects_vector.erase(std::unique(same_id_objects_vector.begin(), same_id_objects_vector.end()), same_id_objects_vector.end());
					//ROS_INFO("After Size: %d" ,same_id_objects_vector.size());
					
					// If same_id_objects.array is not empty
					if(!same_id_objects_vector.empty()){
						
						ROS_INFO("If Same ID vector !!! Same_id_array size: %d", same_id_objects_vector.size());
						
						/*int adding = 0;
						
						for(int i=0; i < same_id_objects_vector.size(); i++){
							
							ROS_INFO("I am in the loop !!!");
							
							if(((same_id_objects_vector[i][1] == roundf(objectToCamera.getOrigin().x() * 100) / 100) 
							 && (same_id_objects_vector[i][2] == roundf(objectToCamera.getOrigin().y() * 100) / 100) )
							 && adding == 0 )
							{
								ROS_INFO("Object %f:\nDiff x: %f\nDiff y: %f\nDiff z: %f", 
								same_id_objects_vector[i][0],
								fabs(same_id_objects_vector[i][1] - roundf(objectToCamera.getOrigin().x() * 100) / 100 ),
								fabs(same_id_objects_vector[i][2] - roundf(objectToCamera.getOrigin().y() * 100) / 100 ) );
								
								objectDataVector[0] = msg->objects.data[0];
								objectDataVector[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
								objectDataVector[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
								objectDataVector[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;
						
								ROS_INFO("Adding !!!");
								objectVector.push_back(objectDataVector);
								
								//realloc(objects.array, sizeof( std_msgs::Float64MultiArray) * (objectVector.size() + 1) );
								//objects.array.push_back( convertVectorToFloatMultiArray(objectDataVector) );
								//break;
								adding = 1;
							}
						}*/
						
						
						
						objectDataVector[0] = msg->objects.data[0];
						objectDataVector[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
						objectDataVector[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
						objectDataVector[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;
						
						ROS_INFO("Adding !!!");
						same_id_objects_vector.push_back(objectDataVector);
						same_id_objects_vector.erase(std::unique(same_id_objects_vector.begin(), same_id_objects_vector.end()), same_id_objects_vector.end());
					
						for(int i=0; i<same_id_objects_vector.size(); i++){
							objectVector.push_back(same_id_objects_vector[i]);
						}
						
						std::sort(objectVector.begin(), objectVector.end(), TfExample::sortFunc);
						
						
						for(int i=0; i<objectVector.size(); i++){
							
							if( objectVector[i][0] == msg->objects.data[0] 
							 && objectVector[i][1] == roundf(objectToCamera.getOrigin().x() * 100) / 100
							 && objectVector[i][2] == roundf(objectToCamera.getOrigin().y() * 100) / 100
							 && objectVector[i][3] == roundf(objectToCamera.getOrigin().z() * 100) / 100)
							{
								 objectVector.erase();
								 
							}
						}
						
						//ROS_INFO("Before Size: %d", objectVector.size());
						ROS_INFO("Before Size: %d", objectVector.size());
						objectVector.erase(std::unique(objectVector.begin(), objectVector.end()), objectVector.end() );
						ROS_INFO("After Size: %d", objectVector.size());
						
						/*objectVector.erase( objectVector.begin(), objectVector.end() );
						ROS_INFO("After Size: %d", objectVector.size());*/
					
						ROS_INFO("Adding after !!!");
					
						//for(int i=0; i<objectVector.size(); i++){
							
							/*objects.array[i].data[0] = objectVector[i][0];
							objects.array[i].data[1] = objectVector[i][1];
							objects.array[i].data[2] = objectVector[i][2];
							objects.array[i].data[3] = objectVector[i][3];*/
							
							//objects.array.push_back( convertVectorToFloatMultiArray(objectVector[i]) );
							
						//}
						
						
					} else {
						
						ROS_INFO("If Different ID !!!");
						objectDataVector[0] = msg->objects.data[0];
						objectDataVector[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
						objectDataVector[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
						objectDataVector[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;
						
						objectVector.push_back(objectDataVector);
						
						//objects.array.push_back( convertVectorToFloatMultiArray(objectDataVector) );
						
						//objectVector.erase( objectVector.begin(), objectVector.end() );
						
						//objectVector.erase(std::unique(objectVector.begin(), objectVector.end()), objectVector.end() );
					
						//for(int i=0; i<objectVector.size(); i++){
							
							/*objects.array[i].data[0] = objectVector[i][0];
							objects.array[i].data[1] = objectVector[i][1];
							objects.array[i].data[2] = objectVector[i][2];
							objects.array[i].data[3] = objectVector[i][3];*/
							
							//objects.array.push_back( convertVectorToFloatMultiArray(objectVector[i]) );
							
						//}
						
						//realloc(objects.array, sizeof( std_msgs::Float64MultiArray) * (objectVector.size() + 1) );
						//objects.array.push_back( convertVectorToFloatMultiArray(objectDataVector) );
						
					}
					
				} else {
					
					ROS_INFO("First fill !!!");
					objectDataVector[0] = msg->objects.data[0];
					objectDataVector[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
					objectDataVector[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
					objectDataVector[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;
					
					objectVector.push_back(objectDataVector);
					
					//objectVector.erase( objectVector.begin(), objectVector.end() );
					//objectVector.erase(std::unique(objectVector.begin(), objectVector.end()), objectVector.end() );
					
					//ROS_INFO("First fill loop !!!");
					
					/*objects.array[0].data[0] = objectVector[0][0];
					objects.array[0].data[1] = objectVector[0][1];
					objects.array[0].data[2] = objectVector[0][2];
					objects.array[0].data[3] = objectVector[0][3];*/
					
					//objects.array.push_back( convertVectorToFloatMultiArray(objectDataVector) );
					
					//ROS_INFO("First fill after loop !!!");
					//realloc(objects.array, sizeof( std_msgs::Float64MultiArray) * (objectVector.size() + 1) );
					//objects.array.push_back( convertVectorToFloatMultiArray(objectDataVector) );
					
				}
				
				//ROS_INFO("First fill: Publish Objects!!!");
				//pubObjectMapPosition_.publish(objects);
				
				// Get all objects.array[i] with the same id as msg->objects.data[0]
					/*for(int i=0; i < objects.array.size(); i++){
						
						if( objects.array[i].data[0] == msg->objects.data[0] ){
							same_id_objects.array.push_back(objects.array[i]);
						}
					}
					
					
					// If same_id_objects.array is not empty
					if(!same_id_objects.array.empty()){
						
						
						ROS_INFO("If Same ID !!! Same_id_array size: %d", same_id_objects.array.size());
						
						for(int i=0; i < same_id_objects.array.size(); i++){
							
							if( (fabs(same_id_objects.array[i].data[1] - roundf(objectToCamera.getOrigin().x() * 100) / 100) > 0.2) 
							 || (fabs(same_id_objects.array[i].data[2] - roundf(objectToCamera.getOrigin().y() * 100) / 100) > 0.2) )
							{
								ROS_INFO("Object %f:\nDiff x: %f\nDiff y: %f\nDiff z: %f", 
								same_id_objects.array[i].data[0],
								fabs(objects.array[i].data[1] - roundf(objectToCamera.getOrigin().x() * 100) / 100 ),
								fabs(objects.array[i].data[2] - roundf(objectToCamera.getOrigin().y() * 100) / 100 ) );
								
								objectData.data[0] = msg->objects.data[0];
								objectData.data[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
								objectData.data[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
								objectData.data[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;
						
								ROS_INFO("Adding !!!");
								objects.array.push_back(objectData);
							}
						}
						
					} else {
						
						ROS_INFO("If Different ID !!!");
						objectData.data[0] = msg->objects.data[0];
						
						objectData.data[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
						objectData.data[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
						objectData.data[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;
						
						objects.array.push_back(objectData);
					}
					
				} else {
					
					ROS_INFO("First fill !!!");
					objectData.data[0] = msg->objects.data[0];
					objectData.data[1] = roundf(objectToCamera.getOrigin().x() * 100) / 100;
					objectData.data[2] = roundf(objectToCamera.getOrigin().y() * 100) / 100;
					objectData.data[3] = roundf(objectToCamera.getOrigin().z() * 100) / 100;
				
					objects.array.push_back(objectData);
				}*/
				
				
				
				/*objects_storage.layout.dim.push_back(std_msgs::MultiArrayDimension());
				objects_storage.layout.dim.push_back(std_msgs::MultiArrayDimension());
				
				objects_storage.layout.dim[0].label = "rows";
				objects_storage.layout.dim[1].label = "columns";
				
				objects_storage.layout.dim[0].size = objects.size();
				objects_storage.layout.dim[1].size = 4;
				
				objects_storage.layout.dim[0].stride = objects.size()*4;
				objects_storage.layout.dim[1].stride = 4;
				
				objects_storage.layout.data_offset = 0;
				
				vector<float> vec(objects.size()*4, 0);
				for (int i=0; i<objects.size(); i++){
					for (int j=0; j<4; j++){
						vec[i*4 + j] = objects[i][j];
					}
				}
				
				for(int i=0; i < vec.size(); i++){
					objects_storage.data[i] = vec[i];
				}*/
				
				
				
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
				//ROS_INFO("First fill: Publish Objects!!!");
				//pubObjectMapPosition_.publish(objects);
				
				//objects_file.close();
				
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
    //find_object_2d::float64_array_of_array objects;
    
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tf_example_node_publisher");

    TfExample sync;
    ros::spin();
}
