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

#include "modules/canbus/vehicle/kia_soul_ev/kia_soul_ev_message_manager.h"

#include "modules/canbus/vehicle/kia_soul_ev/protocol/brake_72.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/brake_73.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/steering_83.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/steering_84.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/throttle_92.h"
#include "modules/canbus/vehicle/kia_soul_ev/protocol/throttle_93.h"

namespace apollo {
namespace canbus {
namespace kia_soul_ev {

KiaSoulEvMessageManager::KiaSoulEvMessageManager() {
  // TODO(Authors): verify which one is recv/sent
  AddSendProtocolData<Brake72, true>();
  AddSendProtocolData<Throttle92, true>();
  AddSendProtocolData<Steering84, true>();

  AddRecvProtocolData<Brake73, true>();
  AddRecvProtocolData<Throttle93, true>();
  AddRecvProtocolData<Steering83, true>();
}

KiaSoulEvMessageManager::~KiaSoulEvMessageManager() {}

}  // namespace kia_soul_ev
}  // namespace canbus
}  // namespace apollo
