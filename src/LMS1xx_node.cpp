/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <csignal>
#include <cstdio>
#include <LMS1xx/LMS1xx.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <limits>
#include <string>

#define DEG2RAD M_PI/180.0

int main(int argc, char **argv)
{
  // laser data
  LMS1xx laser;
  scanCfg cfg;
  scanOutputRange outputRange;
  scanDataCfg dataCfg;
  sensor_msgs::msg::LaserScan scan_msg;

  // parameters
  std::string host;
  std::string frame_id;
  bool inf_range;
  int port;

  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("lms1xx");
  rclcpp::Node n("~");
  auto scan_pub = nh.create_publisher<sensor_msgs::msg::LaserScan>("scan");

  n.param<std::string>("host", host, "192.168.1.2");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<bool>("publish_min_range_as_inf", inf_range, false);
  n.param<int>("port", port, 2111);

  while (rclcpp::ok())
  {
    ROS_INFO_STREAM("Connecting to laser at " << host);
    laser.connect(host, port);
    if (!laser.isConnected())
    {
      RCLCPP_WARN(rclcpp::get_logger("Lms1Xx"), "Unable to connect, retrying.");
      rclcpp::Duration(1).sleep();
      continue;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Logging in to laser.");
    laser.login();
    cfg = laser.getScanCfg();
    outputRange = laser.getScanOutputRange();

    if (cfg.scaningFrequency != 5000)
    {
      laser.disconnect();
      RCLCPP_WARN(rclcpp::get_logger("Lms1Xx"), "Unable to get laser output range. Retrying.");
      rclcpp::Duration(1).sleep();
      continue;
    }

    RCLCPP_INFO(rclcpp::get_logger("Lms1Xx"), "Connected to laser.");

    RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Laser configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
              cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle, cfg.stopAngle);
    RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Laser output range:angleResolution %d, startAngle %d, stopAngle %d",
              outputRange.angleResolution, outputRange.startAngle, outputRange.stopAngle);

    scan_msg.header.frame_id = frame_id;
    scan_msg.range_min = 0.01;
    scan_msg.range_max = 20.0;
    scan_msg.scan_time = 100.0 / cfg.scaningFrequency;
    scan_msg.angle_increment = static_cast<double>(outputRange.angleResolution / 10000.0 * DEG2RAD);
    scan_msg.angle_min = static_cast<double>(outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2);
    scan_msg.angle_max = static_cast<double>(outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2);

    ROS_DEBUG_STREAM("Device resolution is " << (double)outputRange.angleResolution / 10000.0 << " degrees.");
    ROS_DEBUG_STREAM("Device frequency is " << (double)cfg.scaningFrequency / 100.0 << " Hz");

    int angle_range = outputRange.stopAngle - outputRange.startAngle;
    int num_values = angle_range / outputRange.angleResolution;
    if (angle_range % outputRange.angleResolution == 0)
    {
      // Include endpoint
      ++num_values;
    }
    scan_msg.ranges.resize(num_values);
    scan_msg.intensities.resize(num_values);

    scan_msg.time_increment =
      (outputRange.angleResolution / 10000.0)
      / 360.0
      / (cfg.scaningFrequency / 100.0);

    ROS_DEBUG_STREAM("Time increment is " << static_cast<int>(scan_msg.time_increment * 1000000) << " microseconds");

    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Setting scan data configuration.");
    laser.setScanDataCfg(dataCfg);

    RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Starting measurements.");
    laser.startMeas();

    RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Waiting for ready status.");
    rclcpp::Time ready_status_timeout = rclcpp::Time::now() + rclcpp::Duration(5);

    // while(1)
    // {
    status_t stat = laser.queryStatus();
    rclcpp::Duration(1.0).sleep();
    if (stat != ready_for_measurement)
    {
      RCLCPP_WARN(rclcpp::get_logger("Lms1Xx"), "Laser not ready. Retrying initialization.");
      laser.disconnect();
      rclcpp::Duration(1).sleep();
      continue;
    }
    /*if (stat == ready_for_measurement)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Ready status achieved.");
      break;
    }

      if (rclcpp::Time::now() > ready_status_timeout)
      {
        RCLCPP_WARN(rclcpp::get_logger("Lms1Xx"), "Timed out waiting for ready status. Trying again.");
        laser.disconnect();
        continue;
      }

      if (!rclcpp::ok())
      {
        laser.disconnect();
        return 1;
      }
    }*/

    RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Starting device.");
    laser.startDevice();  // Log out to properly re-enable system after config

    RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Commanding continuous measurements.");
    laser.scanContinous(1);

    while (rclcpp::ok())
    {
      rclcpp::Time start = rclcpp::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      scanData data;
      RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Reading scan data.");
      if (laser.getScanData(&data))
      {
        for (int i = 0; i < data.dist_len1; i++)
        {
          float range_data = data.dist1[i] * 0.001;

          if (inf_range && range_data < scan_msg.range_min)
          {
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
          }
          else
          {
            scan_msg.ranges[i] = range_data;
          }
        }

        for (int i = 0; i < data.rssi_len1; i++)
        {
          scan_msg.intensities[i] = data.rssi1[i];
        }

        RCLCPP_DEBUG(rclcpp::get_logger("Lms1Xx"), "Publishing scan data.");
        scan_pub.publish(scan_msg);
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("Lms1Xx"), "Laser timed out on delivering scan, attempting to reinitialize.");
        break;
      }

      rclcpp::spin_some(node);
    }

    laser.scanContinous(0);
    laser.stopMeas();
    laser.disconnect();
  }

  return 0;
}
