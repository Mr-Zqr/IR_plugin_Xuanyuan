/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "include/DTrackSDK.hpp"
#include <mc_control/GlobalPlugin.h>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>
#include <fstream>

namespace mc_plugin
{

struct MyPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~MyPlugin() override;
  
  void assign(mc_control::MCGlobalController & controller);
  
  bool data_error_to_console();

private:
  Eigen::Vector4d loc_tar_0, loc_tar_1, Quatern_temp, rot_bias_temp, gripper_offset;
  Eigen::Vector3d trans;
  Eigen::Quaterniond rot_tar_0, rot_tar_1, rot_T, rot_bias;
  Eigen::Matrix4d T0;
  Eigen::Matrix3d R0;
  DTrackSDK * dt;
  char line[256];
  int linenum = 1;
  double intarr[4];
  bool tracked = false;

  // For data transfer: 
  Eigen::Quaterniond rot;
};

} // namespace mc_plugin
