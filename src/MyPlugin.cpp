#include "MyPlugin.h"

#include <mc_control/GlobalPluginMacros.h>


// global DTrackSDK
static DTrackSDK* dt = NULL;

// prototypes
namespace mc_plugin
{

void MyPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{

	controller.controller().datastore().make<Eigen::Quaterniond>("rotation",rot);
	controller.controller().datastore().make<Eigen::Vector3d>("translation",trans);

	// Reading transform matrix from file. 
	std::ifstream fi("/home/moro/mcProjects/transform_matrix/build/cali.txt");
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
            T0(linenum-1, i) = intarr[i];
        }
        linenum++;
    }
	fi.close();

	mc_rtc::log::info("[IRPlugin] Transform matrix:\n {}", T0);

	std::istringstream portstream( "6324" );
	unsigned short port;
	portstream >> port;  // data port
	if ( portstream.fail() )
	{
		mc_rtc::log::error_and_throw<std::runtime_error>("[IRPlugin] invalid port {}", port);
	}
	// initialization:
	dt = new DTrackSDK( port );
	if ( ! dt->isDataInterfaceValid() )
	{
		mc_rtc::log::error_and_throw<std::runtime_error>("[IRPlugin] DTrackSDK init error");
	}
	dt->setDataTimeoutUS( 10000 );  // NOTE: change here timeout for receiving tracking data, if necessary
	dt->setDataBufferSize( 20000 );  // NOTE: change here buffer size for receiving tracking data, if necessary
	
	// initialize vectors
	loc_tar_0 = Eigen::Vector4d::Ones();
	loc_tar_1 << 0.82, 0, 0.31, 1;
	gipper_offset << -10, 0, 0, 1;

	// Initialize Rotation quatrenions. 
	R0 = T0.block(0,0,3,3);
	rot_T = R0;
	rot_T = rot_T.normalized();

	// rot_bias compensates for difference between the IR and arm frame. 
	rot_bias_temp << 0.5, 0.5, 0.5, 0.5;
	rot_bias = rot_bias_temp;
}

void MyPlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("[IRPlugin] MyPlugin::reset called");
}

void MyPlugin::before(mc_control::MCGlobalController & controller)
{
	// measurement:
	int count = 0;
	if( dt->receive() )
	{
		assign(controller);
	}
	else
	{
		data_error_to_console();
	}
}

void MyPlugin::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("[IRPlugin] MyPlugin::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration MyPlugin::configuration()
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
void MyPlugin::assign(mc_control::MCGlobalController & controller)
{
	for ( int i = 0; i < dt->getNumBody(); i++ )
	{
		
		const DTrackBody* body = dt->getBody( i );
		if ( body == NULL )
		{
			mc_rtc::log::error_and_throw<std::runtime_error>("[IRPlugin] DTrackSDK fatal error: invalid body id ");
			break;
		}

		if ( ! body->isTracked() )
		{
			mc_rtc::log::warning("[IRPlugin] bod {} not tracked", body->id);

			trans[0] = loc_tar_1[0];
			trans[1] = loc_tar_1[1];
			trans[2] = loc_tar_1[2];
			controller.controller().datastore().assign("rotation",rot);
			controller.controller().datastore().assign("translation",trans);
		}
		else
		{
			DTrackQuaternion quat = body->getQuaternion();

			for(int i = 0; i < 3; i++)
			{
				loc_tar_0[i] = body->loc[i];
			}
			loc_tar_1 = (T0*loc_tar_0)/1000;

			// assign quaternion from IR data pack to my eigen variables. 
			Quatern_temp[0] = quat.x;
			Quatern_temp[1] = quat.y;
			Quatern_temp[2] = quat.z;
			Quatern_temp[3] = quat.w;
			rot_tar_0 = Quatern_temp;

			rot_tar_1 = rot_tar_0.conjugate();

			trans[0] = loc_tar_1[0];
			trans[1] = loc_tar_1[1];
			trans[2] = loc_tar_1[2];
			controller.controller().datastore().assign("rotation",rot_tar_1);
			controller.controller().datastore().assign("translation",trans);
		}

	}
}


/**
 * \brief Prints error messages to console.
 *
 * @return No error occured?
 */
bool MyPlugin::data_error_to_console()
{
	// if ( dt->getLastDataError() == DTrackSDK::ERR_TIMEOUT )
	// {
	// 	mc_rtc::log::error("[IRPlugin]--- timeout while waiting for tracking data");
	// 	return false;
	// }

	if ( dt->getLastDataError() == DTrackSDK::ERR_NET )
	{
		mc_rtc::log::error("[IRPlugin]--- error while receiving tracking data");
		return false;
	}

	if ( dt->getLastDataError() == DTrackSDK::ERR_PARSE )
	{
		mc_rtc::log::error("[IRPlugin]--- error while parsing tracking data");
		return false;
	}

	return true;
}

MyPlugin::~MyPlugin()
{
	delete dt;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("MyPlugin", mc_plugin::MyPlugin)