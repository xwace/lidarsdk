/**
 * @file rtrnet.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR data protocol processing App
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
#ifndef _R_TR_NET_H
#define _R_TR_NET_H

#include <chrono>

#include "cmd_interface_linux.h"
#include "sslbf.h"
#include "trnet.h"
#include "pointdata.h"

namespace ldlidar {

typedef enum {

	PACK_GET_DISTANCE = 0x02,               /*Frame ID of distance data*/
	PACK_RESET_SYSTEM = 0x0D,								/*Frame ID of distance reset system*/
	PACK_STOP = 0x0F,		                		/*Frame ID of stop distance data transmission*/
	PACK_ACK = 0x10,												/*Frame ID of ACK*/
	PACK_GET_COE = 0x12,	                	/*Frame ID of Get parameters*/
	PACK_VERSION = 0x14,	                	/*Frame ID of Get Machine version*/
	PACK_VIDEO_SIZE = 0x15,	                /*Frame ID of Get camera resolution*/
	PACK_CONFIG_ADDRESS = 0x16,             /*Frame ID of Configure address*/
	PACK_GET_DISTANCE_TRIG_MODE = 0x26,     /*Frame ID of trig distance data*/
	PACK_SET_GROUND_DISTANCE = 0x28,        /*Frame ID of set ground distance*/

}PackageIDTypeDef;

#define PACK_STRING_LEN 16
#define THIS_DEVICE_ADDREESS 0x01 /*Device address*/

struct TRData {
	uint8_t device_address;
	uint8_t pack_id;
	uint16_t chunk_offset;
	std::vector<uint8_t> data;
};

class RTRNet {
public:
	RTRNet();
	~RTRNet();
	
	std::string GetSdkPackVersionNumber(void);
	bool SetProductType(LDType type_number);
	LDType GetProductType(void);
	double GetFovAngleVal(void);
	double GetTotalPointNumberVal(void);

	bool SendCmd(CmdInterfaceLinux* port, uint8_t address, uint8_t id);
	bool sendWords(CmdInterfaceLinux* port, uint8_t address, uint8_t id, uint32_t *data, uint32_t len, uint16_t offset);
	bool SendStopDistanceTransmitCmd(CmdInterfaceLinux* port);
	void ResetFrameReady(void);
	bool IsFrameReady(void); 
	void ResetParametersReady(void);
	bool IsParametersReady(void);
	bool Pack(const TRData &in, std::vector<uint8_t> &out);
	bool FindLeadingCode(const uint8_t *buff); //make sure buffer size bigger than 4
	uint32_t GetParseDataLen() { return parse_data_len_; }
	bool UnpackData(const uint8_t *data, uint32_t len);
	/**
   * @brief 串口通讯接收回调处理， serial communication read callback
  */
  void CommReadCallback(const char *byte, size_t len);
	/**
	 * @brief 获取激光雷达扫描一帧的点云数据
	*/
	Points2D GetLaserScanData(void);
protected:
	TRData tr_unpack_data_;
private:
	std::string sdk_pack_version_number_;
	LDType product_type_number_;
	bool frame_ready_;
	bool parameters_ready_;
	uint16_t coe_u_;
	uint16_t coe_v_;
  double fov_angle_;
	double total_point_number_;
	const TRData* tr_data_;
	uint32_t coe_[35];
	std::vector<uint8_t> data_tmp_;
	Points2D  laser_scan_data_;
	std::mutex mutex_lock1_;
	std::mutex mutex_lock2_;
	std::mutex mutex_lock3_;

	const uint32_t LEADING_CODE = 0xAAAAAAAA;
	const uint32_t HEADER_LEN = 4; //device_address , pack_id , chunk_offset len
	const uint32_t EXTRA_LEN = 11;

	uint32_t parse_data_len_;

	uint8_t CalCheckSum(const uint8_t *data, uint16_t len);
	bool Transform(const TRData *tr_data);			   /*transform raw data to stantard data */
	void SetFrameReady(void);
	void SetParametersReady(void);
	void SetLaserScanData(Points2D& src);
	
	const TRData *Unpack(const uint8_t *data, uint32_t len);
	bool AnalysisTRNetByte(uint8_t byte);
};

}

#endif // _TR_NET_H
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/