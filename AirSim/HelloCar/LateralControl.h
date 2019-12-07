#pragma once

#include "Waypoints.h"
#include "Interp.h"
#include "common/PidController.hpp"

class LateralControl: msr::airlib::PidController
{
public:

    LateralControl(float p, float i, float d) {
        setPoint(0.0f, p, i, d);
    }
    
    float DistanceOfPointToLine(const Vector3r &pose, const Vector3r &line_point_0, const Vector3r &line_point_1){
        // Get distance of the car position to the trajectory, calculating steering error
        float distance(0.0);
        // Car
        float x0 = pose[0];
        float y0 = pose[1];
        // Line Segment
        float x1 = line_point_0[0];
        float y1 = line_point_0[1];
        float x2 = line_point_1[0];
        float y2 = line_point_1[1];
        // equations and checks
        float denominator = sqrt(pow(y2-y1,2.0f) + pow(x2-x1,2.0f));
        if (denominator<0.0000001) // Avoid equal waypoints
            denominator = 0.001;
        distance = abs( ((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1) / denominator );
        return distance;
    }
    /*
    float Update(Waypoints &checkpoints, const Vector3r &pose, float velocity){
        auto nearest_waypoint_index = checkpoints.GetCurrentWaypointIndex();
        std::cout << nearest_waypoint_index << std::endl;
        auto point_A = checkpoints.GetWaypoint(nearest_waypoint_index);
        auto point_B = checkpoints.GetWaypoint(nearest_waypoint_index+1);
        float cte = DistanceOfPointToLine(pose, point_A, point_B);
        
        // Get direction of steering (+ or -)
        Vector2f point_B_line(point_B[0]-point_A[0], point_B[1]-point_A[1]);
        Vector2f point_C_line(pose[0]-point_A[0], pose[1]-point_A[1]);

        // BC cross product
        float cross_product = point_B_line[0]*point_C_line[1]-point_C_line[0]*point_B_line[1];
        // Check direction
        if (cross_product < 0.0f)
            cte *= -1.0f;
            
        float steering = control(cte);
        steering = std::max(-0.5f, std::min(steering, 0.5f));
        
        return steering;
    }
    */
    ///*
    float Update(Waypoints &checkpoints, const Vector3r &pose, float velocity){
        float steering(0.0f);
        float baseline(2.5f);
        float lookahead(1.0f);
        if (velocity > 0.001f)
        {
            auto nearest_waypoint_index = checkpoints.GetCurrentWaypointIndex();
            auto waypoints = checkpoints.TransformWaypointsWithRespectToCar(pose[2]);
            auto x = waypoints.col(0).segment(nearest_waypoint_index,2).eval();
            auto y = waypoints.col(1).segment(nearest_waypoint_index,2).eval();
            auto coeffs = polyfit(x, y, 1);
            float cte = polyval(coeffs, lookahead);
            
            steering = atan(2.0f * baseline * cte / (lookahead * lookahead));
        }
        steering = std::max(-0.5f, std::min(steering, 0.5f));
        return steering;
    }
    //*/
    ~LateralControl(){};
};

