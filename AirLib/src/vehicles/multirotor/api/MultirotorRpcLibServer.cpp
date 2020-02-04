// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include pre-compiled header file first

#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"


#include "common/Common.hpp"
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/server.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"
#include "common/common_utils/ctpl_stl.h"

#include "vehicles/multirotor/api/MultirotorRpcLibAdapators.hpp"

STRICT_MODE_ON


namespace msr { namespace airlib {

// VICTECH
// To implement lockstep robustly, we have to ensure the async command (like moveToPositionAsync) 
// from external process (like python) is actually IGNITED before returning rpc handler. 
// Since 'lockstep' simulates physical/control updates very fast as strand, if previous async command
// (which is called before 'lockstep' command) did not create Waiter object yet, this command execution
// has very much time-lag noise for training. To avoid this case, we create functor for worker thread and
// make the rpc handler wait for the ignition signal of control command, 
// so that next 'lockstep' is guaranteed to execute this control command.
// original async command: [from python] client.async_call("moveToPosition", params) # never mind after rpc call
// lockstep post command: [from python] client.call("postMoveToPosition", params) # wait until the command is registered
thread_local std::promise<void>* tlsPostedControlCommandIgnition = nullptr;
class PostedControlCommandHandler
{
public:
	template<typename F>
	void Run(F func, std::shared_ptr<std::promise<void>> ignition)
	{
		threads_.push([=](int i) {
			unused(i);
			tlsPostedControlCommandIgnition = ignition.get();
			try { 
				if (!ClockFactory::get()->isLockstepMode())
					throw std::runtime_error("PostCommand only works for lockstep mode !");
				func(); 
			} 
			catch (std::exception & e) { 
				Utils::log(e.what(), Utils::kLogLevelError);
			}
			if (tlsPostedControlCommandIgnition != nullptr)
			{
				tlsPostedControlCommandIgnition->set_value();
				tlsPostedControlCommandIgnition = nullptr;
			}
		});
	}
private:
	ctpl::thread_pool threads_{ 4 };
};
static PostedControlCommandHandler sPostedControlCommandHandler;
// VICTECH

typedef msr::airlib_rpclib::MultirotorRpcLibAdapators MultirotorRpcLibAdapators;

MultirotorRpcLibServer::MultirotorRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port)
        : RpcLibServerBase(api_provider, server_address, port)
{
    (static_cast<rpc::server*>(getServer()))->
        bind("takeoff", [&](float timeout_sec, const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->takeoff(timeout_sec); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("land", [&](float timeout_sec, const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->land(timeout_sec); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("goHome", [&](float timeout_sec, const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->goHome(timeout_sec); 
    });

    (static_cast<rpc::server*>(getServer()))->
        bind("moveByAngleZ", [&](float pitch, float roll, float z, float yaw, float duration, const std::string& vehicle_name) ->
        bool { return getVehicleApi(vehicle_name)->moveByAngleZ(pitch, roll, z, yaw, duration); });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByAngleThrottle", [&](float pitch, float roll, float throttle, float yaw_rate, float duration, 
            const std::string& vehicle_name) -> bool { 
                return getVehicleApi(vehicle_name)->moveByAngleThrottle(pitch, roll, throttle, yaw_rate, duration); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByVelocity", [&](float vx, float vy, float vz, float duration, DrivetrainType drivetrain, 
            const MultirotorRpcLibAdapators::YawMode& yaw_mode, const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->moveByVelocity(vx, vy, vz, duration, drivetrain, yaw_mode.to()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByVelocityZ", [&](float vx, float vy, float z, float duration, DrivetrainType drivetrain, 
            const MultirotorRpcLibAdapators::YawMode& yaw_mode, const std::string& vehicle_name) -> bool {
            return getVehicleApi(vehicle_name)->moveByVelocityZ(vx, vy, z, duration, drivetrain, yaw_mode.to()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveOnPath", [&](const vector<MultirotorRpcLibAdapators::Vector3r>& path, float velocity, float timeout_sec, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead, const std::string& vehicle_name) -> bool {
            vector<Vector3r> conv_path;
            MultirotorRpcLibAdapators::to(path, conv_path);
            return getVehicleApi(vehicle_name)->moveOnPath(conv_path, velocity, timeout_sec, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead);
        });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveToPosition", [&](float x, float y, float z, float velocity, float timeout_sec, DrivetrainType drivetrain,
        const MultirotorRpcLibAdapators::YawMode& yaw_mode, float lookahead, float adaptive_lookahead, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->moveToPosition(x, y, z, velocity, timeout_sec, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveToZ", [&](float z, float velocity, float timeout_sec, const MultirotorRpcLibAdapators::YawMode& yaw_mode, 
            float lookahead, float adaptive_lookahead, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->moveToZ(z, velocity, timeout_sec, yaw_mode.to(), lookahead, adaptive_lookahead); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByManual", [&](float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, 
            const MultirotorRpcLibAdapators::YawMode& yaw_mode, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->moveByManual(vx_max, vy_max, z_min, duration, drivetrain, yaw_mode.to()); 
    });

    (static_cast<rpc::server*>(getServer()))->
        bind("rotateToYaw", [&](float yaw, float timeout_sec, float margin, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->rotateToYaw(yaw, timeout_sec, margin); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("rotateByYawRate", [&](float yaw_rate, float duration, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->rotateByYawRate(yaw_rate, duration); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("hover", [&](const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->hover(); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByRC", [&](const MultirotorRpcLibAdapators::RCData& data, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->moveByRC(data.to()); 
    });

    (static_cast<rpc::server*>(getServer()))->
        bind("setSafety", [&](uint enable_reasons, float obs_clearance, const SafetyEval::ObsAvoidanceStrategy& obs_startegy,
        float obs_avoidance_vel, const MultirotorRpcLibAdapators::Vector3r& origin, float xy_length, 
            float max_z, float min_z, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->setSafety(SafetyEval::SafetyViolationType(enable_reasons), obs_clearance, obs_startegy,
            obs_avoidance_vel, origin.to(), xy_length, max_z, min_z); 
    });

    //getters
    (static_cast<rpc::server*>(getServer()))->
        bind("getMultirotorState", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdapators::MultirotorState {
        return MultirotorRpcLibAdapators::MultirotorState(getVehicleApi(vehicle_name)->getMultirotorState()); 
    });

	// VICTECH - in case of post command of lockstep mode (analogue to async command of normal mode)
	(static_cast<rpc::server*>(getServer()))->bind("postTakeoff", 
		[&](float timeout_sec, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::takeoff, getVehicleApi(vehicle_name), timeout_sec), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postLand", 
		[&](float timeout_sec, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::land, getVehicleApi(vehicle_name), timeout_sec), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postGoHome", 
		[&](float timeout_sec, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::goHome, getVehicleApi(vehicle_name), timeout_sec), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))-> bind("postMoveByAngleZ", 
		[&](float pitch, float roll, float z, float yaw, float duration, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::moveByAngleZ, getVehicleApi(vehicle_name), pitch, roll, z, yaw, duration), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postMoveByAngleThrottle", 
		[&](float pitch, float roll, float throttle, float yaw_rate, float duration, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::moveByAngleThrottle,
			getVehicleApi(vehicle_name), pitch, roll, throttle, yaw_rate, duration), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postMoveByVelocity", 
		[&](float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::moveByVelocity,
			getVehicleApi(vehicle_name), vx, vy, vz, duration, drivetrain, yaw_mode.to()), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postMoveByVelocityZ", 
		[&](float vx, float vy, float z, float duration, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::moveByVelocityZ,
			getVehicleApi(vehicle_name), vx, vy, z, duration, drivetrain, yaw_mode.to()), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postMoveOnPath", 
		[&](const vector<MultirotorRpcLibAdapators::Vector3r>& path, float velocity, float timeout_sec, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode, float lookahead, float adaptive_lookahead, const std::string& vehicle_name) {
		vector<Vector3r> conv_path;
		MultirotorRpcLibAdapators::to(path, conv_path);
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::moveOnPath,
			getVehicleApi(vehicle_name), std::move(conv_path), velocity, timeout_sec, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postMoveToPosition", 
		[&](float x, float y, float z, float velocity, float timeout_sec, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode, float lookahead, float adaptive_lookahead, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::moveToPosition,
			getVehicleApi(vehicle_name), x, y, z, velocity, timeout_sec, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postMoveToZ", 
		[&](float z, float velocity, float timeout_sec, const MultirotorRpcLibAdapators::YawMode& yaw_mode, float lookahead, float adaptive_lookahead, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::moveToZ, 
			getVehicleApi(vehicle_name), z, velocity, timeout_sec, yaw_mode.to(), lookahead, adaptive_lookahead), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postMoveByManual", 
		[&](float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const MultirotorRpcLibAdapators::YawMode& yaw_mode, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::moveByManual, 
			getVehicleApi(vehicle_name), vx_max, vy_max, z_min, duration, drivetrain, yaw_mode.to()), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postRotateToYaw", 
		[&](float yaw, float timeout_sec, float margin, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::rotateToYaw, getVehicleApi(vehicle_name), yaw, timeout_sec, margin), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postRotateByYawRate", 
		[&](float yaw_rate, float duration, const std::string& vehicle_name) {
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::rotateByYawRate, getVehicleApi(vehicle_name), yaw_rate, duration), ignition);
		ignition_signal.wait();
	});
	(static_cast<rpc::server*>(getServer()))->bind("postHover", 
		[&](const std::string& vehicle_name) {
		return getVehicleApi(vehicle_name)->hover();
		auto ignition = std::make_shared<std::promise<void>>();
		std::future<void> ignition_signal = ignition->get_future();
		sPostedControlCommandHandler.Run(std::bind(&MultirotorApiBase::hover, getVehicleApi(vehicle_name)), ignition);
		ignition_signal.wait();
	});
	// VICTECH - in case of async command with lockstep
}

//required for pimpl
MultirotorRpcLibServer::~MultirotorRpcLibServer()
{
}


}} //namespace


#endif
#endif
