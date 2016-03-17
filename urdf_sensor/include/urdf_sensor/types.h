/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Robert Haschke */

#ifndef URDF_SENSOR_TYPES_H
#define URDF_SENSOR_TYPES_H

#include "urdf_model/types.h"
#include "urdf_model/pose.h"

namespace urdf {

URDF_TYPEDEF_CLASS_POINTER(Sensor);
URDF_TYPEDEF_CLASS_POINTER(SensorBase);
URDF_TYPEDEF_CLASS_POINTER(Camera);
URDF_TYPEDEF_CLASS_POINTER(Ray);

class SensorBase
{
public:
  virtual void clear() = 0;
  virtual ~SensorBase(void) {}
};

/** Sensor class is wrapper comprising generic sensor information
 *  like name, update rate, sensor frame (parent link + origin transform)
 *  TODO: think about mergin with SensorBase
 */
class Sensor
{
public:
  Sensor() { this->clear(); };

  /// sensor name must be unique
  std::string name_;

  /// optional group name
  std::string group_;

  /// update rate in Hz
  double update_rate_;

  /// name of parent link this sensor is attached to
  std::string parent_link_;

  /// transform from parent frame sensor frame
  Pose origin_;

  /// sensor
  SensorBaseSharedPtr sensor_;

  void clear()
  {
    name_.clear();
    group_.clear();
    parent_link_.clear();
    origin_.clear();
    sensor_.reset();
  }
};

}

#endif
