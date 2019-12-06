#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <fstream>
#include <math.h>
#include "Waypoints.h"
#include "LongitudinalControl.h"
#include "LateralControl.h"

using namespace msr::airlib;

bool EndOfRace(const Vector3r &pose)
{
	if ((pose[0] > -10.0f) && (pose[0] < 10.0f) &&
        (pose[1] > -1.0f) && (pose[1] < -0.1f))
			return true;
	return false;
		
}

bool EuclideanDistance(const Vector3r &pose1, const Vector3r &pose2, float offset)
{
	float pose2x = pose2[0];
	float pose2y = pose2[1];
	float pose1x = pose1[0];
	float pose1y = pose1[1];
	
    float dist = sqrt(pow(pose2x - pose1x, 2.0f) + pow(pose2y - pose1y, 2.0f));
	
    if (dist > offset)
		return true;
    return false;
}

void GetCurrentState(msr::airlib::CarRpcLibClient &simulator, Vector3r &current_pose, float &current_velocity) {
    auto car_state = simulator.getCarState();
    auto car_speed = car_state.speed;
    auto car_pose = car_state.kinematics_estimated.pose;
    
    current_pose = {car_pose.position[0],car_pose.position[1], VectorMath::yawFromQuaternion(car_pose.orientation)};
    current_velocity = car_speed;
}

CarApiBase::CarControls GetCarControls(float steering, float throttle) {
    CarApiBase::CarControls controls;
    if (throttle > 0.0f)
        controls.throttle = throttle;
    else
        controls.brake = -throttle;
    controls.steering = steering;
    return controls;
}

int main()
{
	int autonomous;
    
	std::cout << "Escolha 0 para manual e 1 para automatico: \n";
	std::cin >> autonomous;

    Waypoints checkpoints, trajectory;
	    
    LateralControl lateral_control(0.1f, 0.0f, 0.18f);
    
    LongitudinalControl longitudinal_control(0.2f, 0.0f, 0.08f);

    float current_speed(0.0f);
    Vector3r current_pose, last_pose;

    msr::airlib::CarRpcLibClient simulator("192.168.0.162");
	try {
		simulator.confirmConnection();
		simulator.reset();

        if (autonomous) {
            simulator.enableApiControl(true);
            checkpoints.LoadWaypoints("auto.csv");
        }
        
		do {

            GetCurrentState(simulator, current_pose, current_speed);

            if (autonomous) {
                float desired_speed = checkpoints.GetWaypointVelocity(current_pose);
                
                float throttle = longitudinal_control.Update(current_speed, desired_speed);

                float steering = lateral_control.Update(checkpoints, current_pose, current_speed);

                auto controls = GetCarControls(steering, throttle);
                
                simulator.setCarControls(controls);
            }
            
			if (EuclideanDistance(current_pose, last_pose, 0.0))
				trajectory.AddWaypoints(current_pose[0], current_pose[1], current_speed);
			
            last_pose = current_pose;

		} while (!EndOfRace(current_pose));
        
		trajectory.SaveWaypoints("manual.csv");
	}
	catch (rpc::rpc_error&  e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Ocorreu um erro!" << std::endl << msg << std::endl; std::cin.get();
	}
	return 0;
}
