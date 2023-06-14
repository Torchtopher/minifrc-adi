// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <string>
#include "adi_driver/adis16470.h"
// cout
#include <iostream>

class ImuNode {
public:
  Adis16470 imu;
  std::string device_ = "/dev/spidev0.0";
  bool burst_mode_ = false;
  bool publish_temperature_ = true;
  double rate_ = 100.0;
  int bias_conf_ = 0x0000;
  int dec_rate_ = static_cast<int>(2000 / rate_);
  int filt_ = 7;

  ImuNode(){
  }

  ~ImuNode() { }

  /**
   * @brief Open IMU device file
   */
  bool open(void) {
    // Open device file
    if (imu.open_port(device_) < 0) {
      //ROS_ERROR("Failed to open device %s", device_.c_str());
      // use std::cout instead of ROS_ERROR
      std::cout << "Failed to open device " << device_.c_str() << std::endl;
      return false;
    }
    // Wait 10ms for SPI ready
    usleep(10000);
    int16_t pid = 0;
    imu.get_product_id(pid);
    //ROS_INFO("Product ID: %x\n", pid);
    std::cout << "Product ID: " << pid << std::endl;
    imu.set_bias_estimation_time(bias_conf_);
    imu.set_filt_ctrl(filt_);
    imu.set_dec_rate(dec_rate_);
    return true;
  }

  void publish_imu_data() {
    /* 
    sensor_msgs::Imu data;
    data.header.frame_id = frame_id_;
    data.header.stamp = ros::Time::now();

    // Linear acceleration
    data.linear_acceleration.x = imu.accl[0];
    data.linear_acceleration.y = imu.accl[1];
    data.linear_acceleration.z = imu.accl[2];

    // Angular velocity
    data.angular_velocity.x = imu.gyro[0];
    data.angular_velocity.y = imu.gyro[1];
    data.angular_velocity.z = imu.gyro[2];

    // Orientation (not provided)
    data.orientation.x = 0;
    data.orientation.y = 0;
    data.orientation.z = 0;
    data.orientation.w = 1;

    imu_data_pub_.publish(data);
    */
    // print out angular velocity with std::cout
    std::cout << "Angular Velocity: " << imu.gyro[0] << ", " << imu.gyro[1] << ", " << imu.gyro[2] << std::endl;
  }


  bool spin() {

    while (true) {
      if (burst_mode_) {
        if (imu.update_burst() == 0) {
          publish_imu_data();
          //publish_temp_data();
        } else {
          //ROS_ERROR("Cannot update burst");
          std::cout << "Cannot update burst" << std::endl;
        }
      } else if (imu.update() == 0) {
        publish_imu_data();
        //publish_temp_data();
      } else {
        //ROS_ERROR("Cannot update");
        std::cout << "Cannot update" << std::endl;
      }
      //ros::spinOnce();
      // sleep for 1ms
      usleep(1000);
    }
    return true;
  }

};

int main(int argc, char **argv) {
  ImuNode node;

  if (!node.open()) {
    //ROS_ERROR("Failed to Open an IMU");
    std::cout << "Failed to Open an IMU" << std::endl;
    return 1;
  }
  node.spin();
  return (0);
}
