/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR 
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2022-05-27
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved. Licensed under the MIT License (the "License"); you may not use
 * this file except in compliance with the License. You may obtain a copy of the
 * License in the file LICENSE Unless required by applicable law or agreed to in
 * writing, software distributed under the License is distributed on an "AS IS"
 * BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied. See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "ros_api.h"
#include "rtrnet.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"

void ToLaserscanMessagePublish(ldlidar::Points2D& src, LaserScanSetting& setting, 
  ldlidar::RTRNet* lidar_comm, ros::Publisher& lidar_pub);

int main(int argc, char **argv) {
	ros::init(argc, argv, "ldlidar_data_publish");
	ros::NodeHandle nh; 
	ros::NodeHandle nh_private("~");
	LaserScanSetting setting;
	
	ldlidar::RTRNet* lidar_comm = new ldlidar::RTRNet();
	ldlidar::CmdInterfaceLinux* cmd_port = new ldlidar::CmdInterfaceLinux();

	nh_private.param("product_type", setting.product_type, std::string("SSL"));
	nh_private.param("port_name", setting.port_name, std::string("/dev/ttyUSB0"));
	nh_private.param("frame_id", setting.frame_id, std::string("base_laser"));
	nh_private.param("topic_name", setting.topic_name, std::string("scan"));
	nh_private.param("product_type", setting.product_type, std::string("SSL20N"));

	ROS_INFO_STREAM("[ldrobot] SDK Pack Version is " << lidar_comm->GetSdkPackVersionNumber());

    ldlidar::LDType type;
	int sleep_rate = 25; // 20L 25Hz, 20N 30Hz, 20P 60Hz
	if ("SSL20L" == setting.product_type) {
		type = ldlidar::LDType::LD_SSL20_L;
		sleep_rate = 25;
	}else if ("SSL20N" == setting.product_type) {
		type = ldlidar::LDType::LD_SSL20_N;
		sleep_rate = 30;
	}else if ("SSL20P" == setting.product_type) {
		type = ldlidar::LDType::LD_SSL20_P;
		sleep_rate = 60;
	}else if ("LD07N" == setting.product_type) {
		type = ldlidar::LDType::LD_07N;
		sleep_rate = 28;
	}else {
		type = ldlidar::LDType::LD_NO_TYPE;
	}

	if (!lidar_comm->SetProductType(type)) {
		ROS_ERROR("[ldrobot] set product type is fail");
		exit(EXIT_FAILURE);
	}

	if(!cmd_port->Open(setting.port_name, ldlidar::STDBaudRateType::BAUD_921600)) {
		ROS_ERROR("[ldrobot] open %s device is fail", setting.port_name.c_str());
		exit(EXIT_FAILURE);
	} else {
		ROS_INFO("[ldrobot] open %s device is success", setting.port_name.c_str());
	}
	
	cmd_port->SetReadCallback(std::bind(&ldlidar::RTRNet::CommReadCallback, 
                            lidar_comm, std::placeholders::_1, std::placeholders::_2));

	int error = 0;
	while (!lidar_comm->IsParametersReady()) {
		if (!lidar_comm->SendCmd(cmd_port, 0, ldlidar::PACK_CONFIG_ADDRESS)) {
			ROS_INFO_STREAM("[ldrobot] send to PACK_CONFIG_ADDRESS cmd fail");
			exit(EXIT_FAILURE);
		}
		sleep(1);
		if (!lidar_comm->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, ldlidar::PACK_GET_COE)) {
			ROS_INFO_STREAM("[ldrobot] send to PACK_GET_COE cmd fail");
			exit(EXIT_FAILURE);
		}
		sleep(1);
		if (!lidar_comm->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, ldlidar::PACK_VIDEO_SIZE)) {
			ROS_INFO_STREAM("[ldrobot] send to PACK_VIDEO_SIZE cmd fail");
			exit(EXIT_FAILURE);
		}
		error++;
		if (error > 2) { /* Exit if the number of errors is more than 2*/
			ROS_INFO_STREAM("[ldrobot] Error: get ssl20 lidar parameters fail");
			exit(EXIT_FAILURE);
		}
	}
	lidar_comm->ResetParametersReady();

	ROS_INFO_STREAM("[ldrobot] get ssl20 lidar parameters success");

	if (!lidar_comm->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, ldlidar::PACK_GET_DISTANCE)) {
		ROS_INFO_STREAM("[ldrobot] send to PACK_GET_DISTANCE cmd fail");
	} else {
		ROS_INFO_STREAM("[ldrobot] send to PACK_GET_DISTANCE cmd success");
	}

    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(setting.topic_name, 10); /*create a ROS topic */
    ros::Publisher lid = nh.advertise<geometry_msgs::Polygon>("polyGon", 10); /*create a ROS topic */

	ros::Rate r(sleep_rate);  
	auto last_time = std::chrono::steady_clock::now();

	while (ros::ok()) {
		if (lidar_comm->IsFrameReady()) {
			lidar_comm->ResetFrameReady();
			last_time = std::chrono::steady_clock::now();
			ldlidar::Points2D laserscan = lidar_comm->GetLaserScanData();
			for (auto n: laserscan) {
				std::cout << "angle: " << n.angle << " " << "distance: " << n.distance << " "
									<< "intensity: " << (int)n.intensity << std::endl;
			}
			std::cout << "----" << std::endl;
			ToLaserscanMessagePublish(laserscan, setting, lidar_comm, lidar_pub);
		}

		if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-last_time).count() > 1000) { 
			// 数据发布超时或者串口设备拔出处理
			ROS_ERROR("[ldrobot] lidar pub data is time out, please check lidar device");
			exit(EXIT_FAILURE);
		}

		r.sleep();
	}

	if (!lidar_comm->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, ldlidar::PACK_STOP)) {
		ROS_INFO_STREAM("[ldrobot] send to PACK_STOP_DISTANCE_TRANS cmd fail");
	} else {
		ROS_INFO_STREAM("[ldrobot] send to PACK_STOP_DISTANCE_TRANS cmd success");
	}
	cmd_port->Close();

	delete lidar_comm;
	lidar_comm = nullptr;
	delete cmd_port;
	cmd_port = nullptr;

	return 0;
}

void ToLaserscanMessagePublish(ldlidar::Points2D& src, LaserScanSetting& setting, ldlidar::RTRNet* lidar_comm, ros::Publisher& lidar_pub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;

  ros::Time start_scan_time;
  static ros::Time end_scan_time;
  static bool first_pub = false;
  float scan_time = 0;

  start_scan_time = ros::Time::now();
  scan_time = (start_scan_time - end_scan_time).toSec();

  if (first_pub == false) {
    scan_time = (float)(1 / 30.f); // 1/30Hz 按理论设计估计
    first_pub = true;
  }
  
  /*Adjust the parameters according to the demand*/
  angle_min = ANGLE_TO_RADIAN(0);
  angle_max = ANGLE_TO_RADIAN(360);
  range_min = 0.015;
  range_max = 1.5;
  /*Angle resolution, the smaller the resolution, the smaller the error after conversion*/
	double fov = lidar_comm->GetFovAngleVal();
	double total_point_number = lidar_comm->GetTotalPointNumberVal();
  angle_increment = ANGLE_TO_RADIAN(fov / total_point_number); 
  /*Calculate the number of scanning points*/
  unsigned int beam_size = ceil((angle_max - angle_min) / angle_increment);
	sensor_msgs::LaserScan output;
  output.header.stamp = start_scan_time;
  output.header.frame_id = setting.frame_id;
  output.angle_min = angle_min;
  output.angle_max = angle_max;
  output.range_min = range_min;
  output.range_max = range_max;
  output.angle_increment = angle_increment;

  output.time_increment = scan_time / (beam_size - 1);
  output.scan_time = scan_time;
  
  /*First fill all the data with Nan*/
  output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
  output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

  for (auto point : src) {
	  float range = point.distance / 1000.f; // distance uint (mm) transform to (m)
		float dir_angle = point.angle;
		dir_angle = 360.f - dir_angle; // clockwise transform to counterclockwise
    float angle = dir_angle / 180 * M_PI; // degree transform to radian
    int index = (int)((angle - output.angle_min) / output.angle_increment);
    if (index >= 0 && index < (int)beam_size) {
      /*If the current content is Nan, it is assigned directly*/
      if (std::isnan(output.ranges[index])) {
        output.ranges[index] = range;
      } else {/*Otherwise, only when the distance is less than the current value, it can be re assigned*/
        if (range < output.ranges[index]) {
          output.ranges[index] = range;
        }
      }
      output.intensities[index] = point.intensity;
    }
  }
	lidar_pub.publish(output);
  end_scan_time = start_scan_time;
	ROS_INFO("[ldrobot] pub lidar data");
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/