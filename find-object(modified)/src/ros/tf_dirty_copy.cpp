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
		pubObjectMapPosition_ = nh.advertise<find_object_2d::float64_array_of_array>("objectStorage", 1, true);
		
		
	}

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
	{

		//vector<float> objectData(4);
		std_msgs::Float64MultiArray objectData;
		objectData.data.resize(4);
		
		visualization_msgs::MarkerArray markers;
		
		//fstream objects_file;
		//objects_file.open("/home/linda/catkin_ws/src/turtlebot_object_detection/object_map_positions.txt",  std::ios::app);
				
		
		if(msg->objects.data.size())
		{
			//char multiSubId = 'b';
			int previousId = -1;
			for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
			{
				// get data
				int id = (int)msg->objects.data[i];

				/*QString multiSuffix;
				if(id == previousId)
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
				
				/*objects_file << msg->header.stamp << '_' 
						 << objectFrameId.c_str() << '_' 
						 << objectToCamera.getOrigin().x() << '_'
						 << objectToCamera.getOrigin().y() << endl;*/
					
					 
				/*if (!this->objects.array.empty()){
					
					for(int i=0; i < objects.array.size(); i++){
						
						ROS_WARN("Array ID: %f | Object ID: %f", objects.array[i].data[0], msg->objects.data[0]);
						
						if(objects.array[i].data[0] == msg->objects.data[0]){
							
							//ROS_INFO("If Same ID !!!");
							
							/*ROS_INFO("Object %f:\nDiff x: %f\nDiff y: %f\nDiff z: %f", id,
							fabs(objects.array[i].data[1] - objectToCamera.getOrigin().x()),
							fabs(objects.array[i].data[2] - objectToCamera.getOrigin().y()),
							fabs(objects.array[i].data[3] - objectToCamera.getOrigin().z()) );*/
							
							/*if( (fabs(objects[i][1] - objectToCamera.getOrigin().x()) > 0.1) 
							 || (fabs(objects[i][2] - objectToCamera.getOrigin().y()) > 0.1)
							 || (fabs(objects[i][3] - objectToCamera.getOrigin().z()) > 0.1) )
							{*/
							//if( (fabs(objects.array[i].data[1] - objectToCamera.getOrigin().x()) > 1.0) 
							 //|| (fabs(objects.array[i].data[2] - objectToCamera.getOrigin().y()) > 1.0) )
							//{
								 /*objectData[0] = id;
								 objectData[1] = objectToCamera.getOrigin().x();
								 objectData[2] = objectToCamera.getOrigin().y();
								 objectData[3] = objectToCamera.getOrigin().z();*/
								 //objects.push_back(objectData);
								 
								 //ROS_INFO("If big difference !!!");
								 
								 /*objectData.data[0] = id;
								 objectData.data[1] = objectToCamera.getOrigin().x();
								 objectData.data[2] = objectToCamera.getOrigin().y();
								 objectData.data[3] = objectToCamera.getOrigin().z();
											
								 objects.array.push_back(objectData);
							}
							
						} else {
							//ROS_INFO("If Different ID !!!");
							
							objectData.data[0] = id;
							objectData.data[1] = objectToCamera.getOrigin().x();
							objectData.data[2] = objectToCamera.getOrigin().y();
							objectData.data[3] = objectToCamera.getOrigin().z();
							//objects.push_back(objectData);
							
							objects.array.push_back(objectData);
						}
						
					}
					
				} else {
					objectData.data[0] = id;
					objectData.data[1] = objectToCamera.getOrigin().x();
					objectData.data[2] = objectToCamera.getOrigin().y();
					objectData.data[3] = objectToCamera.getOrigin().z();
					//objects.push_back(objectData);
					
					objects.array.push_back(objectData);
				}*/
					
				
				if (!this->objects.array.empty()){
					
					find_object_2d::float64_array_of_array same_id_objects;
					
					// Get all objects.array[i] with the same id as msg->objects.data[0]
					for(int i=0; i < objects.array.size(); i++){
						
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
				}
				
				
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
				pubObjectMapPosition_.publish(objects);
				
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
    //vector<vector<float> > objects;
    find_object_2d::float64_array_of_array objects;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tf_example_node_publisher");

    TfExample sync;
    ros::spin();
}
