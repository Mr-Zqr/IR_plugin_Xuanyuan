/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include "DTrackSDK.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>

namespace mc_plugin
{

struct IRPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~IRPlugin() override;

  void assign(mc_control::MCGlobalController & controller);

  bool data_error_to_console();

private:
  Eigen::Vector4d loc_tar_0, loc_tar_1, Quatern_temp;
  Eigen::Vector3d trans0, trans_body;
  Eigen::Quaterniond rot_tar_0, rot_tar_1, rot_body, rot_T;
  Eigen::Matrix4d T0, T_m2b, T_marker, T_body;
  Eigen::Matrix3d R0, R_body;
  DTrackSDK * dt;
  char line[256];
  int linenum = 1;
  double intarr[4];
  bool tracked = false;

  // For data transfer:
  Eigen::Quaterniond rot;
};

} // namespace mc_plugin
