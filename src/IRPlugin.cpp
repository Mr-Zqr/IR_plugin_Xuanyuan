#include "IRPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

// global DTrackSDK
static DTrackSDK * dt = NULL;

// prototypes
namespace mc_plugin
{

void IRPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  // Reading transform matrix from file.
  std::ifstream fi("/home/zhenyuanfu/devel/src/transform_matrix/build/cali.txt");
  if(!fi)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[IRPlugin] No calibrate transform matrix file, aborting.");
  }
  while(fi.good() && linenum < 5)
  {
    fi.getline(line, 256);
    // puts(line);
    std::istringstream iss(line);
    iss >> intarr[0] >> intarr[1] >> intarr[2] >> intarr[3];

    for(int i = 0; i < 4; i++)
    {
      T0(linenum - 1, i) = intarr[i];
    }
    linenum++;
  }
  fi.close();

  // Read calibration marker to body transform matrix from file.
  linenum = 1;
  std::ifstream ma2bo("/home/zhenyuanfu/devel/src/transform_body/build/marker2body.txt");
  if(!ma2bo)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[IRPlugin] No calibrate transform matrix from marker to body file, aborting.");
  }
  while(ma2bo.good() && linenum < 5)
  {
    ma2bo.getline(line, 256);
    // puts(line);
    std::istringstream iss(line);
    iss >> intarr[0] >> intarr[1] >> intarr[2] >> intarr[3];

    for(int i = 0; i < 4; i++)
    {
      T_m2b(linenum - 1, i) = intarr[i];
    }
    linenum++;
  }
  ma2bo.close();

  mc_rtc::log::info("[IRPlugin] Transform matrix from camera frame to car frame:\n {}", T0);
  mc_rtc::log::info("\n[IRPlugin] Transform matrix marker to robot body:\n {}", T_m2b);

  std::istringstream portstream("6324");
  unsigned short port;
  portstream >> port; // data port
  if(portstream.fail())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[IRPlugin] invalid port {}", port);
  }
  // initialization:
  dt = new DTrackSDK(port);
  if(!dt->isDataInterfaceValid())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[IRPlugin] DTrackSDK init error");
  }
  dt->setDataTimeoutUS(10000); // NOTE: change here timeout for receiving tracking data, if necessary
  dt->setDataBufferSize(20000); // NOTE: change here buffer size for receiving tracking data, if necessary

  // initialize vectors
  loc_tar_0 = Eigen::Vector4d::Ones();
  loc_tar_1 << 0.92, 0, 0.31, 1;

  // Initialize Rotation quatrenions.
  R0 = T0.block(0, 0, 3, 3);
  rot_T = R0;
  rot_T = rot_T.normalized();

  reset(controller);
}

void IRPlugin::reset(mc_control::MCGlobalController & controller)
{
  if(update_thread_.joinable())
  {
    running_ = false;
    update_thread_.join();
  }
  controller.controller().datastore().make<Eigen::Quaterniond>("rotation_trunk", rot_body.conjugate());
  controller.controller().datastore().make<Eigen::Vector3d>("translation_trunk", trans_body);
  controller.controller().gui()->addElement({"IRMarker"}, mc_rtc::gui::Transform("Marker", [this]() {
                                              return sva::PTransformd{rot_body.conjugate(), trans_body};
                                            }));
  running_ = true;
  update_thread_ = std::thread([this]() {
    while(running_)
    {
      // measurement:
      if(dt->receive())
      {
        update_data();
      }
      else
      {
        data_error_to_console();
      }
      {
        std::unique_lock<std::mutex> lock(update_mutex_);
        datastore_rotation_ = rot_body.conjugate();
        datastore_translation_ = trans_body;
      }
      // FIXME Make this configurable
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
  mc_rtc::log::info("[IRPlugin] IRPlugin::reset called");
}

void IRPlugin::before(mc_control::MCGlobalController & controller)
{
  std::unique_lock<std::mutex> lock(update_mutex_);
  controller.controller().datastore().assign("rotation_trunk", rot_body.conjugate());
  controller.controller().datastore().assign("translation_trunk", trans_body);
}

mc_control::GlobalPlugin::GlobalPluginConfiguration IRPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

/**
 * \brief Prints current tracking data to console.
 */
void IRPlugin::update_data()
{
  for(int i = 0; i < dt->getNumBody(); i++)
  {

    const DTrackBody * body = dt->getBody(i);
    if(body == NULL)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[IRPlugin] DTrackSDK fatal error: invalid body id ");
      break;
    }

    if(!body->isTracked())
    {
      mc_rtc::log::warning("[IRPlugin] bod {} not tracked", body->id);
    }
    else
    {
      DTrackQuaternion quat = body->getQuaternion();

      for(int i = 0; i < 3; i++)
      {
        loc_tar_0[i] = body->loc[i];
      }
      loc_tar_1 = (T0 * loc_tar_0);

      // assign quaternion from IR data pack to my eigen variables.
      Quatern_temp[0] = quat.x;
      Quatern_temp[1] = quat.y;
      Quatern_temp[2] = quat.z;
      Quatern_temp[3] = quat.w;
      rot_tar_0 = Quatern_temp;

      rot_tar_1 = rot_T * rot_tar_0;
    }

    T_marker.block(0, 0, 3, 3) = rot_tar_1.toRotationMatrix();
    T_marker.block(0, 3, 4, 1) = loc_tar_1;

    T_body = T_marker * T_m2b;

    R_body = T_body.block(0, 0, 3, 3);
    rot_body = R_body;
    rot_body = rot_body.normalized();
    trans_body = (T_body.block(0, 3, 3, 1)) / 1000;
  }
}

/**
 * \brief Prints error messages to console.
 *
 * @return No error occured?
 */
bool IRPlugin::data_error_to_console()
{
  // if ( dt->getLastDataError() == DTrackSDK::ERR_TIMEOUT )
  // {
  // 	mc_rtc::log::error("[IRPlugin]--- timeout while waiting for tracking data");
  // 	return false;
  // }

  if(dt->getLastDataError() == DTrackSDK::ERR_NET)
  {
    mc_rtc::log::error("[IRPlugin]--- error while receiving tracking data");
    return false;
  }

  if(dt->getLastDataError() == DTrackSDK::ERR_PARSE)
  {
    mc_rtc::log::error("[IRPlugin]--- error while parsing tracking data");
    return false;
  }

  mc_rtc::log::error("[IRPlugin]--- Unknown error {}", dt->getLastDataError());
  return true;
}

IRPlugin::~IRPlugin()
{
  delete dt;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("IRPlugin", mc_plugin::IRPlugin)
