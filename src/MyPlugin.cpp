#include "MyPlugin.h"
#include "include/DTrackSDK.hpp"

#include <mc_control/GlobalPluginMacros.h>

#include <iostream>
#include <sstream>
#include <Eigen/Dense>
// global DTrackSDK
static DTrackSDK* dt = NULL;
Eigen::Quaterniond Q, Q_c, P_loc_quat, P_rot_quat, P_loc_new, P_rot_new;
Eigen::Vector4d P_loc_vec, Q_temp, P_rot_vec;

// prototypes
static void output_to_console(mc_control::MCGlobalController & controller);
static bool data_error_to_console();
int portN;
namespace mc_plugin
{

MyPlugin::~MyPlugin() = default;

void MyPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  portN=config("port");
  	Eigen::Quaterniond rot;
	Eigen::Vector3d trans;
	controller.controller().datastore().make<Eigen::Quaterniond>("rotation",rot);
	controller.controller().datastore().make<Eigen::Vector3d>("translation",trans);
  mc_rtc::log::success("port number is {}", portN);
  mc_rtc::log::info("MyPlugin::init called with configuration:\n{}", config.dump(true, true));
}

void MyPlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("MyPlugin::reset called");
}

void MyPlugin::before(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("MyPlugin::before");  
  std::istringstream portstream( "6324" );
	unsigned short port;
	portstream >> port;  // data port

	if ( portstream.fail() )
	{
		std::cout << "invalid port '" << portN << "'" << std::endl;
		//return -2;
	}

	// initialization:

	dt = new DTrackSDK( port );

	if ( ! dt->isDataInterfaceValid() )
	{
		std::cout << "DTrackSDK init error" << std::endl;
		//return -3;
	}
	std::cout << "listening at local data port " << dt->getDataPort() << std::endl;

	dt->setDataTimeoutUS( 1000 );  // NOTE: change here timeout for receiving tracking data, if necessary
	dt->setDataBufferSize( 20000 );  // NOTE: change here buffer size for receiving tracking data, if necessary

	// measurement:

	int count = 0;
	//while ( 1 )  // collect 1000 frames
	//{
		if ( dt->receive() )
		{
			output_to_console(controller);
		}
		else
		{
			data_error_to_console();
		}
	//}


	delete dt;  // clean up
	//return 0;
}

void MyPlugin::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("MyPlugin::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration MyPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("MyPlugin", mc_plugin::MyPlugin)
/**
 * \brief Prints current tracking data to console.
 */
static void output_to_console(mc_control::MCGlobalController & controller)
{
	// std::cout.precision( 3 );
	// std::cout.setf( std::ios::fixed, std::ios::floatfield );

	// std::cout << std::endl << "frame " << dt->getFrameCounter() << " ts " << dt->getTimeStamp()
	//           << " nbod " << dt->getNumBody() << " nfly " << dt->getNumFlyStick()
	//           << " nmea " << dt->getNumMeaTool() << " nmearef " << dt->getNumMeaRef() 
	//           << " nhand " << dt->getNumHand() << " nmar " << dt->getNumMarker() 
	//           << " nhuman " << dt->getNumHuman() << " ninertial " << dt->getNumInertial()
	//           << std::endl;

	// Standard bodies:
	for ( int i = 0; i < dt->getNumBody(); i++ )
	{
		
		const DTrackBody* body = dt->getBody( i );
		if ( body == NULL )
		{
			std::cout << "DTrackSDK fatal error: invalid body id " << i << std::endl;
			break;
		}

		if ( ! body->isTracked() )
		{
			std::cout << "bod " << body->id << " not tracked" << std::endl;
		}
		else
		{

			std::cout << "bod " << body->id << " qu " << body->quality;
			//           << " loc " << body->loc[ 0 ] << " " << body->loc[ 1 ] << " " << body->loc[ 2 ]
			//           << " rot " << body->rot[ 0 ] << " " << body->rot[ 1 ] << " " << body->rot[ 2 ]
			//           << " "     << body->rot[ 3 ] << " " << body->rot[ 4 ] << " " << body->rot[ 5 ]
			//           << " "     << body->rot[ 6 ] << " " << body->rot[ 7 ] << " " << body->rot[ 8 ]
			//           << std::endl;

			DTrackQuaternion quat = body->getQuaternion();
			// std::cout << "bod " << body->id << " quatw " << quat.w
			//           << " quatxyz " << quat.x << " " << quat.y << " " << quat.z << std::endl;
			// Assign loc into quaternion P
			for(int i = 0; i < 3; i++)
			{
				P_loc_vec[i] = body->loc[i]/1000;
			}
			P_loc_vec[3] = 0;
			P_loc_quat = P_loc_vec;
			// Assign Q
			Q_temp << 0.5, 0.5, 0.5, 0.5;
			Q = Q_temp;
			// calc. P_n
			Q_c = Q.conjugate();
			P_loc_new = Q*P_loc_quat*Q_c;

			P_rot_vec << quat.x, quat.y, quat.z, quat.w;
			P_rot_quat = P_rot_vec;
			P_rot_new = Q*P_rot_quat*Q_c;

			Eigen::Quaterniond rot(P_rot_new.w(),P_rot_new.x(),P_rot_new.y(),P_rot_new.z());
			Eigen::Vector3d trans(P_loc_new.x(), P_loc_new.y(), P_loc_new.z());
			controller.controller().datastore().assign("rotation",rot);
			controller.controller().datastore().assign("translation",trans);
		}

	}
}


/**
 * \brief Prints error messages to console.
 *
 * @return No error occured?
 */
static bool data_error_to_console()
{
	if ( dt->getLastDataError() == DTrackSDK::ERR_TIMEOUT )
	{
		std::cout << "--- timeout while waiting for tracking data" << std::endl;
		return false;
	}

	if ( dt->getLastDataError() == DTrackSDK::ERR_NET )
	{
		std::cout << "--- error while receiving tracking data" << std::endl;
		return false;
	}

	if ( dt->getLastDataError() == DTrackSDK::ERR_PARSE )
	{
		std::cout << "--- error while parsing tracking data" << std::endl;
		return false;
	}

	return true;
}

