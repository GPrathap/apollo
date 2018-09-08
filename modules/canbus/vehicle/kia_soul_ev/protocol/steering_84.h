/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file steering_64.h
 * @brief the class of Steering84 (for kia_soul_ev vehicle)
 */

#ifndef MODULES_CANBUS_VEHICL_KIASOULEV_PROTOCOL_STEERING_84_H_
#define MODULES_CANBUS_VEHICL_KIASOULEV_PROTOCOL_STEERING_84_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/oscc_magic.h"

/**
 * @namespace apollo::canbus::kia_soul_ev
 * @brief apollo::canbus::kia_soul_ev
 */
namespace apollo {
namespace canbus {
namespace kia_soul_ev {

/**
 * @class Steering84
 *
 * @brief one of the protocol data of kia_soul_ev vehicle
 */
class Steering84 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  /**
   * @brief get the data period
   * @return the value of data period
   */
  virtual uint32_t GetPeriod() const;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  virtual void UpdateData(uint8_t *data);

  /**
   * @brief reset the private variables
   */
  virtual void Reset();

  /**
   * @brief set steering angle
   * @return a this pointer to the instance itself
   */
  Steering84 *set_steering_angle(double angle);

 private:
  /**
   * config detail: {'name': 'scmd', 'offset': 0.0, 'precision': 0.1, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[-470|470]',
   * 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': '"degrees"'}
   */
  void set_steering_angle_p(uint8_t *data, double angle);

 private:
  double steering_angle_ = 0.0;
};

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_KIASOULEV_PROTOCOL_STEERING_84_H_
