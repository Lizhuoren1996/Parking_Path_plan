// #ifndef _MPC_COMMON_H_
// #define _MPC_COMMON_H_

#pragma once


#include <vector>
#include <string>
#include <iostream>


// struct Point2d {
// 	Point2d() : x(0), y(0) {}
// 	Point2d(double x_, double y_) : x(x_), y(y_) {}
// 	Point2d(const Point2d& other) { *this = other; }
// 	Point2d& operator= (const Point2d& other) {
// 		if (this == &other) {
// 			return *this;
// 		}
// 		this->x = other.x;
// 		this->y = other.y;
// 	}
// 	double x;
// 	double y;
// };

struct Pose2d {
	Pose2d() : x(0), y(0), theta(0) {}
	Pose2d(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
	Pose2d(const Pose2d& other) { *this = other; }
	Pose2d& operator= (const Pose2d& other) {
		if (this == &other){
			return *this;
		}
		this->x = other.x;
		this->y = other.y;
		this->theta = other.theta;
		// this->kappa = other.kappa;
		return *this;
	}
	double x;
	double y;
	double theta;// rad
	// double kappa;
};

struct TrajectoryPose {
	TrajectoryPose() : x(0), y(0), theta(0), kappa(0), v(0), relative_time(0), a(0), s(0) {}
	TrajectoryPose(double x_, double y_, double theta_, double kappa_, double v_, double relative_time_, double a_, double s_) :
		x(x_), y(y_), theta(theta_), kappa(kappa_), v(v_), relative_time(relative_time_), a(a_), s(s_) {}
	TrajectoryPose(const TrajectoryPose& other) { *this = other; }
	TrajectoryPose& operator= (const TrajectoryPose& other) {
		if (this == &other){
			return *this;
		}
		this->x = other.x;
		this->y = other.y;
		this->theta = other.theta;
		this->kappa = other.kappa;
		this->v = other.v;
		this->relative_time = other.relative_time;
        this->a = other.a;
        this->s = other.s;
	}

    // std::string DebugString() {
    //     return "x: " + std::to_string(x) + " y: " + std::to_string(y) + " theta: " + std::to_string(theta)
    //         +  " kappa: " + std::to_string(kappa) + " v: " + std::to_string(v) 
    //         + " a: " + std::to_string(a) + " relative_time: " +  std::to_string(relative_time) + " s: " + std::to_string(s); 
    // }
	double x;
	double y;
	double theta;// rad
	double kappa;
	double v;// m/s
	double relative_time;//s
    double a;// m2/s
    double s; //m
};

using Trajectory = std::vector<TrajectoryPose>;
// typedef std::vector<TrajectoryPose> Trajectory;
// vehicle cordinate: ox is forward and oy is left, right hand
struct VehicleState {
    VehicleState() : vx(0), vy(0), w(0), ax(0), ay(0), steer_angle(0) {}
	double vx;//km/h
	double vy;//km/h
	double w;//radps
	double ax;
	double ay;
    double steer_angle;
};


struct ControlCommand {
    /* data */
};

struct State {
    State() : x(0.0),y(0.0),yaw(0.0),v(0.0) {}
    double x;
    double y;
    double yaw;
    double v;

};

struct Obstacle {
	double x;
	double y;

	double width;
	double length;
    
	double min_s;
	double max_s;
	double min_l;
	double max_l;
};



// #endif