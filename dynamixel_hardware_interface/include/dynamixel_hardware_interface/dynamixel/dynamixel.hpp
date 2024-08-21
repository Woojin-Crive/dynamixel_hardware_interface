/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
/* Authors: Hye-Jong KIM, Yong-Ho Na*/

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_HPP_

#include "dynamixel_hardware_interface/dynamixel/dynamixel_info.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <map>
#include <queue>
#include <string>
#include <vector>
#include <iostream>
#include <cstdarg>
#include <memory>

namespace dynamixel_hardware_interface
{

// dxl control mode
#define DXL_CURRENT_CTRL_MODE   0
#define DXL_POSITION_CTRL_MODE  3
#define DXL_VELOCITY_CTRL_MODE  1

// dxl torque states
#define TORQUE_ON  1
#define TORQUE_OFF 0

// communication type
#define SYNC 0
#define BULK 1

// dxl error states
enum DxlError
{
  OK = 0,
  CANNOT_FIND_CONTROL_ITEM = -1,
  OPEN_PORT_FAIL = -2,
  INDIRECT_ADDR_FAIL = -3,
  ITEM_WRITE_FAIL = -4,
  ITEM_READ_FAIL = -5,

  SYNC_WRITE_FAIL = -6,
  SYNC_READ_FAIL  = -7,
  SET_SYNC_WRITE_FAIL = -8,
  SET_SYNC_READ_FAIL = -9,

  BULK_WRITE_FAIL = -10,
  BULK_READ_FAIL  = -11,
  SET_BULK_WRITE_FAIL = -12,
  SET_BULK_READ_FAIL = -13,

  SET_READ_ITEM_FAIL = -14,
  SET_WRITE_ITEM_FAIL = -15,

  DLX_HARDWARE_ERROR = -16,
  DXL_REBOOT_FAIL = -17,
};

typedef struct
{
  uint16_t indirect_data_addr;
  uint16_t cnt;   // number of item
  uint8_t size;  // total byte size
  std::vector<std::string> item_name;
  std::vector<uint8_t> item_size;
} IndirectInfo;

typedef struct
{
  uint8_t id;
  ControlItem control_item;
  uint32_t data;
  bool read_flag;
} RWItemBufInfo;

typedef struct
{
  uint8_t id;
  std::vector<std::string> item_name;
  std::vector<uint8_t> item_size;
  std::vector<uint16_t> item_addr;
  std::vector<std::shared_ptr<double>> item_data_ptr_vec;
} RWItemList;

class Dynamixel
{
private:
  ///////////////////////////////////////////////////////////////////////
  // dxl communication variable
  dynamixel::PortHandler * port_handler_;
  dynamixel::PacketHandler * packet_handler_;

  ///////////////////////////////////////////////////////////////////////
  // dxl info variable from dxl_model file
  DynamixelInfo dxl_info_;

  ///////////////////////////////////////////////////////////////////////
  // item write variable
  std::vector<RWItemBufInfo> write_item_buf_;
  std::vector<RWItemBufInfo> read_item_buf_;
  std::map<uint8_t /*id*/, bool> torque_state_;

  ///////////////////////////////////////////////////////////////////////
  // read item (sync or bulk) variable
  bool read_type_;
  std::vector<RWItemList> read_data_list_;

  // sync read
  dynamixel::GroupSyncRead * group_sync_read_;
  // indirect inform for sync read
  std::map<uint8_t /*id*/, IndirectInfo> indirect_info_read_;

  // bulk read
  dynamixel::GroupBulkRead * group_bulk_read_;

  ///////////////////////////////////////////////////////////////////////
  // write item (sync or bulk) variable
  bool write_type_;
  std::vector<RWItemList> write_data_list_;

  // sync write
  dynamixel::GroupSyncWrite * group_sync_write_;
  // indirect inform for sync write
  std::map<uint8_t /*id*/, IndirectInfo> indirict_info_write_;

  // bulk write
  dynamixel::GroupBulkWrite * group_bulk_write_;

public:
  explicit Dynamixel(const char * path);
  ~Dynamixel();

  ///////////////////////////////////////////////////////////////////////
  // DXL Communication Setting
  DxlError InitDxlComm(std::vector<uint8_t> id_arr, std::string port_name, std::string baudrate);
  DxlError Reboot(uint8_t id);
  void RWDataReset();

  // DXL Read Setting
  DxlError SetDxlReadItems(
    uint8_t id, std::vector<std::string> item_names,
    std::vector<std::shared_ptr<double>> data_vec_ptr);
  DxlError SetMultiDxlRead();

  // DXL Write Setting
  DxlError SetDxlWriteItems(
    uint8_t id, std::vector<std::string> item_names,
    std::vector<std::shared_ptr<double>> data_vec_ptr);
  DxlError SetMultiDxlWrite();

  ///////////////////////////////////////////////////////////////////////
  // Read Item (sync or bulk)
  DxlError ReadMultiDxlData();
  // Write Item (sync or bulk)
  DxlError WriteMultiDxlData();

  ///////////////////////////////////////////////////////////////////////
  // Set Dxl Option
  DxlError SetOperatingMode(uint8_t id, uint8_t dynamixel_mode);
  DxlError DynamixelEnable(std::vector<uint8_t> id_arr);
  DxlError DynamixelDisable(std::vector<uint8_t> id_arr);

  ///////////////////////////////////////////////////////////////////////
  // DXL Item Write
  DxlError WriteItem(uint8_t id, std::string item_name, uint32_t data);
  DxlError WriteItem(uint8_t id, uint16_t addr, uint8_t size, uint32_t data);
  DxlError InsertWriteItemBuf(uint8_t id, std::string item_name, uint32_t data);
  DxlError WriteItemBuf();

  ///////////////////////////////////////////////////////////////////////
  // DXL Item Read
  DxlError ReadItem(uint8_t id, std::string item_name, uint32_t & data);
  DxlError InsertReadItemBuf(uint8_t id, std::string item_name);
  DxlError ReadItemBuf();
  bool CheckReadItemBuf(uint8_t id, std::string item_name);
  uint32_t GetReadItemDataBuf(uint8_t id, std::string item_name);

  ///////////////////////////////////////////////////////////////////////
  // etc.
  DynamixelInfo GetDxlInfo() {return dxl_info_;}
  std::map<uint8_t, bool> GetDxlTorqueState() {return torque_state_;}

  static std::string DxlErrorToString(DxlError error_num);

private:
  bool checkReadType();
  bool checkWriteType();

  // SyncRead
  DxlError SetSyncReadItemAndHandler();
  DxlError SetSyncReadHandler(std::vector<uint8_t> id_arr);
  DxlError GetDxlValueFromSyncRead();

  // BulkRead
  DxlError SetBulkReadItemAndHandler();
  DxlError SetBulkReadHandler(std::vector<uint8_t> id_arr);
  DxlError GetDxlValueFromBulkRead();

  // Read - Indirect Address
  void ResetIndirectRead(std::vector<uint8_t> id_arr);
  DxlError AddIndirectRead(
    uint8_t id,
    std::string item_name,
    uint16_t item_addr,
    uint8_t item_size);

  // SyncWrite
  DxlError SetSyncWriteItemAndHandler();
  DxlError SetSyncWriteHandler(std::vector<uint8_t> id_arr);
  DxlError SetDxlValueToSyncWrite();

  // BulkWrite
  DxlError SetBulkWriteItemAndHandler();
  DxlError SetBulkWriteHandler(std::vector<uint8_t> id_arr);
  DxlError SetDxlValueToBulkWrite();

  // Write - Indirect Address
  void ResetIndirectWrite(std::vector<uint8_t> id_arr);
  DxlError AddIndirectWrite(
    uint8_t id,
    std::string item_name,
    uint16_t item_addr,
    uint8_t item_size);
};

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__DYNAMIXEL__DYNAMIXEL_HPP_
