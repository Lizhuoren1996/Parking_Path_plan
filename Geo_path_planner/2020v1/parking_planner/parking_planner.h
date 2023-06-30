#ifndef _PLANNING_PARKING_PLANNER_H_
#define _PLANNING_PARKING_PLANNER_H_
// #include "common.h"
#include <vector>
#include <cmath>
// #include "common/planning_common.h"
#include "common.h"
#include "log.h"
// #include "common/vehicle_parameters.h"

namespace planning {
/////  xs
///    ^
///    |
///    |
///y----
///0-------1
///|       |
///|       |
///|       |
///|       |
///3-------2

// enum ParkingType {
//     PARALOT0 = 0,
//     PARALOT1 = 1,
//     VERALOT = 2  
// };
enum ParkingType {
    PARALOT0_right = 0,//泊入+前进/后退
    PARALOT0_left = 1,
    PARALOT1_right = 2,//驶入+后退
    PARALOT1_left = 3,
    VERLOT_right = 4,
    VERLOT_left = 5,  
};

struct ParkingPose {
	ParkingPose() : x(0), y(0), theta(0), v(0), kappa(0) {}
	ParkingPose(double x_, double y_, double theta_, double v_, double kappa_) : x(x_), y(y_), theta(theta_), v(v_), kappa(kappa_)  {}
	ParkingPose(const ParkingPose& other) { *this = other; }
	ParkingPose& operator= (const ParkingPose& other) {
		if (this == &other){
			return *this;
		}
		this->x = other.x;
		this->y = other.y;
		this->theta = other.theta;
		this->kappa = other.kappa;
        this->v = other.v;
		return *this;
	}
	double x;
	double y;
	double theta;// rad
	double kappa;
    double v;
    double a;// m2/s
    double s; //m
    ParkingType parking_type;
};
// typedef std::vector<Pose2d> ParkingLot;
// typedef std::vector<ParkingPose> ParkingPath; 
using ParkingLot = std::vector<Pose2d>;
using ParkingPath = std::vector<ParkingPose>;


// #define LENGTHOFPLAN 1000

class ParkingGeoPlanner {

public:
    ParkingGeoPlanner();
    ~ParkingGeoPlanner();
    void Init();
    bool MakePlan(const ParkingPose& start_pose, const ParkingPose& target_pose, const ParkingType& parking_type, Trajectory& trajectory);
    bool MakePlan(const ParkingPose& start_pose, const ParkingPose& target_pose, const ParkingLot& parking_lot, const ParkingType& parking_type, Trajectory& trajectory);
private:
    void LocalToGlobal(const ParkingPose& current_pose, const Trajectory& local_trajectory, 
				   Trajectory& global_trajectory);
    void BuildParallelParkingLot(const ParkingPose& target_pose, const ParkingType& parking_type);
    void BuildVerticalParkingLot(const ParkingPose& target_pose, const ParkingType& parking_type);

    bool MakeParallelParking(Trajectory& trajectory);
    bool MakeVerticalParking(Trajectory& trajectory);

    void CalcuForwardPath(ParkingPath& path);
    void CalcuParkingPath(const ParkingPose& start_pose, ParkingPath& path);
    void CalcuReplanPath(const ParkingPose& start_pose, ParkingPath& path);
    void AfterReplanPath(const ParkingPose& start_pose, ParkingPath& path);
    void CalcuFinalPath(const ParkingPose& start_pose, ParkingPath& path);

    bool IsDoneOnceStep();
    void ForwardBezierPath(float x, float y, float theta, ParkingPath& path); 

    void GenerateCircle(const ParkingPose& start_pose, double length, double radius, int flag, ParkingPath& path);
    void GenerateLine(const ParkingPose& start_pose, double length, ParkingPath& path);

private:
    ParkingPose start_pose_;
    ParkingPose target_pose_;
    ParkingPose pose_A_;
    ParkingPose forward_pose_A;
    ParkingPose pose_o1_;

    ParkingLot parking_lot_;
    ParkingType parking_type_;

    bool Isdoneoncestep;

    double dis_o1_23_;
    double beta1_, beta2_, beta3_;
    float length2A_; 

    double half_lot_width_ = 1.25;
    double lot_length_f_ = 3.7;
    double lot_length_r_ = 1.6;

// veh param
    double min_radius_ = 5.5; // 6.0 
    double r_rmin_ = 3.5;
    double r_rout_ = 0.5;
    // float t[LENGTHOFPLAN];

    double vehicle_width_ = 1.8;
    double vehicle_len_ = 5.0;
    double vehicle_l_r_ = 1.0; //后轴到车尾的距离
    double vehicle_l_f_ = 4.0; //后轴到车头的距离
    double vehicle_l_rf = 2.5;  //zhouju 2.78

    double forward_speed_ = 0.5;
    double backward_speed_ = -0.5;

    double L_safe_1 = 0.1;
    double L_safe_2 = 0.1;
    float lateral_disA_allowed_ver;
    float lateral_disA_allowed_para;
    float adjust_angle_para; 
    float adjust_angle_ver;

    int is_right = 0;
};



}


#endif