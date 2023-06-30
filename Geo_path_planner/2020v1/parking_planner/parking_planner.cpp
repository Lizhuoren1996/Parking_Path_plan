
#include "parking_planner.h"
// #include <math.h>
#include <iostream>
#include <fstream>
// #include "common/log.h"
// 2021.06.04 for nqyavp

using namespace std;
namespace planning {
ParkingGeoPlanner::ParkingGeoPlanner(){

}
ParkingGeoPlanner::~ParkingGeoPlanner(){

}
void ParkingGeoPlanner::Init() {

    r_rout_ = std::sqrt(vehicle_l_r_ * vehicle_l_r_ + std::pow(min_radius_ + vehicle_width_/2.0, 2));
    adjust_angle_ver = 5* M_PI / 180.0;
    adjust_angle_para = 0 * M_PI / 180.0;
    lateral_disA_allowed_ver = 5;
    lateral_disA_allowed_para =  3.5/2.0 + half_lot_width_ ;
}

bool ParkingGeoPlanner::MakePlan(const ParkingPose& start_pose, const ParkingPose& target_pose, 
    const ParkingType& parking_type, Trajectory& trajectory) {
    parking_type_ = parking_type;
    target_pose_ = target_pose;
    parking_lot_.clear();
    //////trans r2l
    AERROR <<"PARKING TARGET POSE XY ==" << target_pose.x << "  "<<target_pose.y;
    ParkingPose target_pose_r;
    target_pose_r = target_pose;
    ///////////////////////
    // target_pose_r.x = target_pose.x - vehicle_l_rf * std::cos(target_pose.theta);
    // target_pose_r.y = target_pose.y - vehicle_l_rf * std::sin(target_pose.theta);
    //////////////////////
    if (parking_type == ParkingType::VERLOT_right || parking_type == ParkingType::VERLOT_left) {
        
        BuildVerticalParkingLot(target_pose_r,parking_type);
    } else {
        BuildParallelParkingLot(target_pose_r,parking_type);
    }
    MakePlan(start_pose, target_pose_r, parking_lot_, parking_type, trajectory);
    
}

bool ParkingGeoPlanner::MakePlan(const ParkingPose& start_pose, const ParkingPose& target_pose, const ParkingLot& parking_lot, 
    const ParkingType& parking_type, Trajectory& trajectory) {
    start_pose_ = start_pose;
    parking_lot_.clear();
    parking_lot_ = parking_lot;
    parking_type_ = parking_type;
    target_pose_ = target_pose;
    AERROR << "--------IN MakePlan--------target_pose_xy === " << target_pose_.x <<"  , "<<target_pose_.y;
    AERROR << "parking_type === " << (int)parking_type << "  parking_type_ === " << (int)parking_type_;
    if (parking_type == ParkingType::VERLOT_right || parking_type == ParkingType::VERLOT_left) {
        if(!MakeVerticalParking(trajectory)){
            return false;
        }
    } else {
        if (!MakeParallelParking(trajectory)) {
            return false;
        }
    }
     
}

void ParkingGeoPlanner::BuildParallelParkingLot(const ParkingPose& target_pose, const ParkingType& parking_type) {
    std::string filename = "../parking_result/result0.csv";
    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);
    writing_file << "x" << "," << "y" << std::endl; 
    
    Pose2d p0, p1, p2, p3;
    ParkingPose target_pose_tmp = target_pose;
    if (parking_type == PARALOT0_left || parking_type ==PARALOT1_left)
        // target_pose_tmp.theta += M_PI;
    std::vector<double> llx = {lot_length_f_,-1.0 * lot_length_r_};
    std::vector<double> lly = {1.0 * half_lot_width_, -1.0 * half_lot_width_};
    std::vector<double> lly2 = {-1.0 * half_lot_width_, 1.0 * half_lot_width_};

    double lx = lot_length_f_;
    for (const auto& ly : lly) {
        p0.x = target_pose_tmp.x + lx * std::cos(target_pose_tmp.theta) - ly * std::sin(target_pose_tmp.theta);
        p0.y = target_pose_tmp.y + lx * std::sin(target_pose_tmp.theta) + ly * std::cos(target_pose_tmp.theta);
        parking_lot_.emplace_back(p0);
        std::cout << "p0.x: "<< p0.x << " p0.y: "<< p0.y << std::endl;
        writing_file << p0.x << "," << p0.y << std::endl;
    }
    
    lx = -1.0 * lot_length_r_;
    for (const auto& ly : lly2) {
        p0.x = target_pose_tmp.x + lx * std::cos(target_pose_tmp.theta) - ly * std::sin(target_pose_tmp.theta);
        p0.y = target_pose_tmp.y + lx * std::sin(target_pose_tmp.theta) + ly * std::cos(target_pose_tmp.theta);
        parking_lot_.emplace_back(p0);
        std::cout << "p0.x: "<< p0.x << " p0.y: "<< p0.y << std::endl;
        writing_file << p0.x << "," << p0.y << std::endl;
    }

}

void ParkingGeoPlanner::BuildVerticalParkingLot(const ParkingPose& target_pose, const ParkingType& parking_type) {
    Pose2d p0, p1, p2, p3;
    std::vector<double> llx = {lot_length_f_,-1.0 * lot_length_r_};
    std::vector<double> llx2 = {-1.0 * lot_length_r_, lot_length_f_};

    // std::vector<double> lly = {1.0 * half_lot_width_, -1.0 * half_lot_width_};
    // std::vector<double> lly2 = {-1.0 * half_lot_width_, 1.0 * half_lot_width_};
    std::string filename = "../parking_result/result0.csv";
    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);
    writing_file << "x" << "," << "y" << std::endl;

    double ly = -1.0 * half_lot_width_;
    for (const auto& lx : llx) {
        p0.x = target_pose.x + lx * std::cos(target_pose.theta) - ly * std::sin(target_pose.theta);
        p0.y = target_pose.y + lx * std::sin(target_pose.theta) + ly * std::cos(target_pose.theta);
        std::cout << "p0.x: "<< p0.x << " p0.y: "<< p0.y << std::endl;
        writing_file << p0.x << "," << p0.y << std::endl;
        parking_lot_.emplace_back(p0);
    }
    
    ly = half_lot_width_;
    for (const auto& lx : llx2) {
        p0.x = target_pose.x + lx * std::cos(target_pose.theta) - ly * std::sin(target_pose.theta);
        p0.y = target_pose.y + lx * std::sin(target_pose.theta) + ly * std::cos(target_pose.theta);
        std::cout << "p0.x: "<< p0.x << " p0.y: "<< p0.y << std::endl;
        writing_file << p0.x << "," << p0.y << std::endl;

        parking_lot_.emplace_back(p0);
    }
    if (parking_type == VERLOT_left)
    {
        Pose2d tmppose2 = parking_lot_[2];
        Pose2d tmppose3 = parking_lot_[3];
        parking_lot_[2].x = parking_lot_[0].x;
        parking_lot_[2].y = parking_lot_[0].y;
        parking_lot_[3].x = parking_lot_[1].x;
        parking_lot_[3].y = parking_lot_[1].y;
        parking_lot_[0].x = tmppose2.x;
        parking_lot_[0].y = tmppose2.y;
        parking_lot_[1].x = tmppose3.x;
        parking_lot_[1].y = tmppose3.y;
        std::cout<<"build  vertical_left Lot success"<<std::endl;
    }

    for(int i = 0; i < parking_lot_.size();i++)
    {
        std::cout << "p0.x: "<< parking_lot_[i].x << " p0.y: "<< parking_lot_[i].y << std::endl;
    }
    std::cout<<"build  vertical_ Lot success"<<std::endl;
}

bool ParkingGeoPlanner::MakeParallelParking(Trajectory& trajectory) {
    // std::string filename = "../para_result2.csv";
    // std::ofstream writing_file;
    // writing_file.open(filename, std::ios::out);
    // writing_file << "x" << "," << "y" << std::endl;
//  step 1 to forward    
    ParkingPath forward_path;
    ParkingPath tmp_path;
    CalcuForwardPath(forward_path);
//  step 2 circle one  //  step 3 circle two 
    ParkingPose parking_start_pose = forward_path.back();
    tmp_path.insert(tmp_path.end(),forward_path.begin(),forward_path.end());
    ParkingPath parking_path;
    CalcuParkingPath(parking_start_pose, parking_path);
    tmp_path.insert(tmp_path.end(),parking_path.begin(),parking_path.end());
//  step 4 final path
    ParkingPose final_start_pose = parking_path.back();
    // ParkingPose final_start_pose = start_pose_;
    ParkingPath final_path;
    CalcuFinalPath(final_start_pose, final_path);
    tmp_path.insert(tmp_path.end(),final_path.begin(),final_path.end());

    //do not trans traj from r to f
    for(int i =0;i<tmp_path.size();i++)
    {
        TrajectoryPose tmp;
        tmp.x = tmp_path[i].x;
        tmp.y = tmp_path[i].y;
        float tmp_theta = tmp_path[i].theta;
        if(tmp_path[i].v<0)
            tmp_theta = tmp_theta + M_PI;
        if(tmp_theta > M_PI){
            tmp_theta -= 2. * M_PI;
        }
        else if(tmp_theta < -M_PI){
            tmp_theta += 2. * M_PI;
        }
        /////////////////////
        // tmp.x = tmp_path[i].x + vehicle_l_rf * std::cos(tmp_theta);
        // tmp.y = tmp_path[i].y + vehicle_l_rf * std::sin(tmp_theta);
        ///////////////////////
        tmp.theta = tmp_theta;
        tmp.v = tmp_path[i].v;
        trajectory.push_back(tmp);
    }


}

// 1：找前进的目标点 不需要
// 2: 判断能否一次泊入，是否发生碰撞
// 3: forward 
// 4: parkingpath 计算第1,2段圆弧
// 7: finalpath 最终直线段 
bool ParkingGeoPlanner::MakeVerticalParking(Trajectory& trajectory) {
    Isdoneoncestep = false;
    trajectory.clear();
    ParkingPath tmp_path;
    ParkingPath forward_path;
    AERROR << "IN MakeVerticalParking--------start " ;
    CalcuForwardPath(forward_path);
    AERROR << "IN MakeVerticalParking--------CalcuForwardPath suc " ;
    tmp_path.insert(tmp_path.end(),forward_path.begin(),forward_path.end());
   
    ParkingPose parking_start_pose = forward_path.back();
    ParkingPath parking_path;
    CalcuParkingPath(parking_start_pose, parking_path);
    AERROR << "IN MakeVerticalParking--------CalcuParkingPath suc " ;
    tmp_path.insert(tmp_path.end(),parking_path.begin(),parking_path.end());
    ParkingPose final_start_pose;

    if (!IsDoneOnceStep()){
        AERROR << "IN MakeVerticalParking--------Muti Step " ;
        ParkingPose replan_start_pose = parking_path.back();
        ParkingPath replan_path;
        CalcuReplanPath(replan_start_pose, replan_path);
        tmp_path.insert(tmp_path.end(),replan_path.begin(),replan_path.end());
        AERROR << "IN MakeVerticalParking--------CalcuReplanPath Step " ;
        ParkingPose after_start_pose = replan_path.back();
        ParkingPath after_replan_path;
        AfterReplanPath(after_start_pose, after_replan_path);
        tmp_path.insert(tmp_path.end(),after_replan_path.begin(),after_replan_path.end());
        AERROR << "IN MakeVerticalParking--------AfterReplanPath Step " ;
        final_start_pose = after_replan_path.back();    
    }
    else{
        std::cout<<"------parkingend size----"<<parking_path.size()<<std::endl;
        final_start_pose = parking_path.back();    
    }
    ParkingPath final_path;
    std::cout<<"------------------start final---------"<<std::endl;
    CalcuFinalPath(final_start_pose, final_path);
    AERROR << "IN MakeVerticalParkingCalcuFinalPath--------CalcuFinalPath" ;
    std::cout<<"-------finalpath---------"<<final_path.size()<<std::endl;
    tmp_path.insert(tmp_path.end(),final_path.begin(),final_path.end());
    for(int i =0;i<tmp_path.size();i++)
    {
        TrajectoryPose tmp;
        tmp.x = tmp_path[i].x;
        tmp.y = tmp_path[i].y;
        float tmp_theta = tmp_path[i].theta;
        if(tmp_path[i].v<0)
            tmp_theta = tmp_theta + M_PI;
        if(tmp_theta > M_PI){
            tmp_theta -= 2. * M_PI;
        }
        else if(tmp_theta < -M_PI){
            tmp_theta += 2. * M_PI;
        }
        tmp.x = tmp_path[i].x + vehicle_l_rf * std::cos(tmp_theta);
        tmp.y = tmp_path[i].y + vehicle_l_rf * std::sin(tmp_theta);
        tmp.theta = tmp_theta;
        tmp.v = tmp_path[i].v;
        // trajectory[0] = tmp;
        trajectory.push_back(tmp);
    }
}

void ParkingGeoPlanner::CalcuForwardPath(ParkingPath& path) {
    std::string filename = "../parking_result/result2.csv";
    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);
    writing_file << "x" << "," << "y" << std::endl;
    path.clear();
    switch (parking_type_) {
    case PARALOT0_right:
    {
        AERROR << "IN MakeVerticalParking--------PARALOT0_right " ;
        float dx = parking_lot_[0].x - parking_lot_[3].x;
        float dy = parking_lot_[0].y - parking_lot_[3].y;
        float yaw = std::atan2(dy, dx);
        //to adjust the theta in final path
        double adjust_dis = -2.6;// -1.2 front for adjust  1.0 back for adjust
        /////////////////////////////////////////////////////////////////////
        double beta1 = std::acos(((1 + std::cos(adjust_angle_para)) * min_radius_ - lateral_disA_allowed_para)/(2.0 * min_radius_));
        beta1_ = beta1;
        double long_disC =  min_radius_ * (2.0 * std::sin(beta1) - std::sin(adjust_angle_para));
        double l1_para = long_disC + adjust_dis;
        // insure not collision at Lot_0
        double l0_o1 = min_radius_ - vehicle_width_/2.0 - L_safe_1;
        double alpha = std::asin((min_radius_ - lateral_disA_allowed_para + half_lot_width_)/l0_o1);
        double l1_max = lot_length_f_ + l0_o1 * std::cos(alpha);
        // if(l1_para > l1_max)
        //     l1_para = l1_max;
        double l2_para = lateral_disA_allowed_para + 0.5;
        AERROR << "l2_para ======  : " <<l2_para;
        ////////////////////////////////////////////////////
        std::cout << "para beta1 ----------------- :"<<beta1 <<std::endl;
        std::cout <<"l1_prar ===========:"<<l1_para <<"   l2_para ==========+:"<< l2_para <<std::endl;
        pose_A_.x = target_pose_.x + l1_para * std::cos(yaw) - l2_para * std::sin(yaw);
        pose_A_.y = target_pose_.y + l1_para * std::sin(yaw) + l2_para * std::cos(yaw);

        float target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta);
        float target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta);
        float target_theta = yaw + adjust_angle_para;
        // target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta) + vehicle_l_rf * std::cos(target_theta);
        // target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta) + vehicle_l_rf * std::sin(target_theta);

        std::cout<<"pose_A_.x  & y ======="<<pose_A_.x<<"\t"<<pose_A_.y<<std::endl;
        ForwardBezierPath(target_x, target_y, target_theta, path);
        ///////////////////////trans bezier path to r
        // for(int i=0;i<path.size();i++){
        //     path[i].x = path[i].x - vehicle_l_rf * std::cos(target_theta);
        //     path[i].x = path[i].x - vehicle_l_rf * std::sin(target_theta);
        // }
        // std::cout<<"forward-----------path.size================================="<<path.size()<<std::endl;
        std::cout<<"forwardpath-----------end==============="<<path.back().x<<"\t"<<path.back().y<<std::endl;
        break;
    }
    case PARALOT0_left:
    {   
        AERROR << "IN MakeVerticalParking--------PARALOT0_left " ;
        float dx = parking_lot_[1].x - parking_lot_[2].x;
        float dy = parking_lot_[1].y - parking_lot_[2].y;
        float yaw = std::atan2(dy, dx);
        
        //to adjust the theta in final path
        double adjust_dis = 2.2;// -1.2 front for adjust  1.0 back for adjust
        double beta1 = std::acos(((1 + std::cos(adjust_angle_para)) * min_radius_ - lateral_disA_allowed_para)/(2.0 * min_radius_));
        beta1_ = beta1;
        std::cout << "para beta1 ----------------- :"<<beta1 <<std::endl;
        double long_disC =  min_radius_ * (2.0 * std::sin(beta1) - std::sin(adjust_angle_para));
        double l1_para = long_disC + adjust_dis;
        // insure not collision at Lot_0
        double l0_o1 = min_radius_ - vehicle_width_/2.0 - L_safe_1;
        double alpha = std::asin((min_radius_ - lateral_disA_allowed_para + half_lot_width_)/l0_o1);
        double l1_max = lot_length_f_ + l0_o1 * std::cos(alpha);
        if(l1_para > l1_max)
            l1_para = l1_max;
        double l2_para = lateral_disA_allowed_para;
        std::cout <<"l1_prar ===========:"<<l1_para <<"   l2_para ==========+:"<< l2_para <<std::endl;
        pose_A_.x = target_pose_.x + l1_para * std::cos(yaw) + l2_para * std::sin(yaw);
        pose_A_.y = target_pose_.y + l1_para * std::sin(yaw) - l2_para * std::cos(yaw);

        float target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta);
        float target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta);
        float target_theta = yaw - adjust_angle_para;
        std::cout<<"pose_A_==========================="<<pose_A_.x<<"\t"<<pose_A_.y<<std::endl;
        ForwardBezierPath(target_x, target_y, target_theta, path);
        // std::cout<<"forward-----------path.size================================="<<path.size()<<std::endl;
        std::cout<<"forwardpath-----------end==============="<<path.back().x<<"\t"<<path.back().y<<std::endl;
   
        break;
    }
    case PARALOT1_right:
    {
        float dx = parking_lot_[0].x - parking_lot_[3].x;
        float dy = parking_lot_[0].y - parking_lot_[3].y;
        float yaw = std::atan2(dy, dx);
        double adjust_dis = -2.5; //abs<lot_length
        double beta1 = std::acos(((1 + std::cos(adjust_angle_para)) * (min_radius_) - lateral_disA_allowed_para)/(2.0 * (min_radius_)));
        beta1_ = beta1;
        double long_disC =  (min_radius_) * (2.0 * std::sin(beta1) - std::sin(adjust_angle_para));

        double l1_para = -long_disC + adjust_dis;
        double l2_para = lateral_disA_allowed_para + 0.2;
        pose_A_.x = target_pose_.x + l1_para * std::cos(yaw) - l2_para * std::sin(yaw);
        pose_A_.y = target_pose_.y + l1_para * std::sin(yaw) + l2_para * std::cos(yaw);

        float target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta);
        float target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta);
        float target_theta = yaw + adjust_angle_para;
        /////////////////////////////////////
        // target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta) + vehicle_l_rf * std::cos(target_theta);
        // target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta) + vehicle_l_rf * std::sin(target_theta);

        ForwardBezierPath(target_x, target_y, target_theta, path);
        // for(int i=0;i<path.size();i++){
        //     path[i].x = path[i].x - vehicle_l_rf * std::cos(target_theta);
        //     path[i].x = path[i].x - vehicle_l_rf * std::sin(target_theta);
        // }
        std::cout<<"forward-----------path.size================================="<<path.size()<<std::endl;
        std::cout<<"forwardpath-----------end==============="<<path.back().x<<"\t"<<path.back().y<<std::endl;
    
        break;
    }
    case PARALOT1_left:
    {
        float dx = parking_lot_[1].x - parking_lot_[2].x;
        float dy = parking_lot_[1].y - parking_lot_[2].y;
        float yaw = std::atan2(dy, dx);
        double adjust_dis = -3.0; //abs<lot_length
        double beta1 = std::acos(((1 + std::cos(adjust_angle_para)) * (min_radius_) - lateral_disA_allowed_para)/(2.0 * (min_radius_)));
        beta1_ = beta1;
        double long_disC =  (min_radius_) * (2.0 * std::sin(beta1) - std::sin(adjust_angle_para));

        double l1_para = -long_disC + adjust_dis;
        double l2_para = lateral_disA_allowed_para;
        pose_A_.x = target_pose_.x + l1_para * std::cos(yaw) + l2_para * std::sin(yaw);
        pose_A_.y = target_pose_.y + l1_para * std::sin(yaw) - l2_para * std::cos(yaw);

        float target_x = pose_A_.x ;
        float target_y = pose_A_.y ;
        float target_theta = yaw + adjust_angle_para;
        target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta) + vehicle_l_rf * std::cos(target_theta);
        target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta) + vehicle_l_rf * std::sin(target_theta);
        ForwardBezierPath(target_x, target_y, target_theta, path);

        for(int i=0;i<path.size();i++){
            path[i].x = path[i].x - vehicle_l_rf * std::cos(target_theta);
            path[i].x = path[i].x - vehicle_l_rf * std::sin(target_theta);
        }

        std::cout<<"forward-----------path.size================================="<<path.size()<<std::endl;
        std::cout<<"forwardpath-----------end==============="<<path.back().x<<"\t"<<path.back().y<<std::endl;
         //trans bezier path to r
        
        break;
        
    }
    case VERLOT_right:
    {
        AERROR << "IN MakeVerticalParking--------VERLOT_right " ;
        float adjust_dis_1 = 0.3;
        
        float dx = parking_lot_[0].x - parking_lot_[3].x;
        float dy = parking_lot_[0].y - parking_lot_[3].y;
        float yaw = std::atan2(dy, dx);
        std::cout<<"VERLOT_right yaw============="<<yaw<<std::endl;
        std::cout<<"yaw-PI/2============="<<yaw - M_PI/2<<std::endl;
        if (IsDoneOnceStep()) {
            float target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta);
            float target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta);
            float target_theta = pose_A_.theta ;
            target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta) + vehicle_l_rf * std::cos(target_theta);
            target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta) + vehicle_l_rf * std::sin(target_theta);
        
            std::cout << "IsDoneOnceStep------target_x: " << target_x << " target_y: " << target_y << std::endl;
            ForwardBezierPath(target_x, target_y, target_theta, path);
                // trans bezier path to r
            for(int i=0;i<path.size();i++){
                path[i].x = path[i].x - vehicle_l_rf * std::cos(target_theta);
                path[i].x = path[i].x - vehicle_l_rf * std::sin(target_theta);
            }
            std::cout <<"forwardend========" <<path.back().x <<"\t"<<path.back().y <<"\t"<< path.back().theta<<std::endl;
            
        } 
        else {
            float l0_o1 = min_radius_-vehicle_width_/2.0- L_safe_1;
            float lateral_disA = lateral_disA_allowed_ver;
            // double alpha = std::asin((min_radius_ * std::cos(adjust_angle_ver) - lateral_disA)/l0_o1);
            
            double alpha = std::asin((min_radius_ * std::cos(adjust_angle_ver) + vehicle_l_f_ * std::sin(adjust_angle_ver) - lateral_disA)/l0_o1);
            std::cout<<"alpha==========="<<alpha<<std::endl;
            double xo1 = parking_lot_[0].x + l0_o1 * std::cos(alpha + yaw);
            double yo1 = parking_lot_[0].y - l0_o1 * std::sin(alpha + yaw);
            std::cout<<"xo1=====:"<<xo1<<"yo1=====:"<<yo1<<std::endl;

            pose_A_.x = xo1 - min_radius_ * std::sin(adjust_angle_ver + yaw);
            pose_A_.y = yo1 + min_radius_ * std::cos(adjust_angle_ver + yaw); 
            pose_A_.theta = yaw + adjust_angle_ver;
            std::cout<<"pose A----x:"<<pose_A_.x<<"    pose A---y:"<<pose_A_.y<<"    pose A----theta:"<<pose_A_.theta<<std::endl;
            pose_o1_.x = xo1;
            pose_o1_.y = yo1;// for calparking
            float target_x = pose_A_.x;
            float target_y = pose_A_.y;
            float target_theta = pose_A_.theta ;
        
            ForwardBezierPath(target_x, target_y, target_theta, path);
            for(int i=0;i<path.size();i++){
                path[i].x = path[i].x - vehicle_l_rf * std::cos(target_theta);
                path[i].x = path[i].x - vehicle_l_rf * std::sin(target_theta);
            }
            std::cout<<"forwardend.theta========"<<path.back().theta<<"+PI/2===="<<path.back().theta+ M_PI/2.<<std::endl;
        }
        break;    
    }
    case VERLOT_left:
    {
        AERROR << "IN MakeVerticalParking--------VERLOT_left " ;
        float adjust_dis_1 = 0.3;
        
        float dx = parking_lot_[1].x - parking_lot_[2].x;
        float dy = parking_lot_[1].y - parking_lot_[2].y;
        float yaw = std::atan2(dy, dx);
        std::cout<<"VERLOT_left yaw============="<<yaw<<std::endl;
        // std::cout<<"yaw-PI/2============="<<yaw - M_PI/2<<std::endl;
        if (IsDoneOnceStep()) {
            float target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta);
            float target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta);
            float target_theta = pose_A_.theta ;
            // float target_theta = yaw + adjust_angle_ver;
            // target_x = pose_A_.x + start_pose_.x * std::cos(start_pose_.theta) + vehicle_l_rf * std::cos(target_theta);
            // target_y = pose_A_.y + start_pose_.y * std::sin(start_pose_.theta) + vehicle_l_rf * std::sin(target_theta);
        
            std::cout << "IsDoneOnceStep------target_x: " << target_x << " target_y: " << target_y << std::endl;
            ForwardBezierPath(target_x, target_y, target_theta, path);
    
            std::cout <<"forwardend========" <<path.back().x <<"\t"<<path.back().y <<"\t"<< path.back().theta<<std::endl;
            
        } 
        else {
            float l1_o1 = min_radius_-vehicle_width_/2.0- L_safe_1;
            float lateral_disA = lateral_disA_allowed_ver;
            // double alpha = std::asin((min_radius_ * std::cos(adjust_angle_ver) - lateral_disA)/l0_o1);
            
            double alpha = std::asin((min_radius_ * std::cos(adjust_angle_ver) + vehicle_l_f_ * std::sin(adjust_angle_ver) - lateral_disA)/l1_o1);
            std::cout<<"alpha==========="<<alpha<<std::endl;
            double xo1 = parking_lot_[1].x + l1_o1 * std::cos(alpha + yaw);
            double yo1 = parking_lot_[1].y + l1_o1 * std::sin(alpha + yaw);
            std::cout<<"xo1=====:"<<xo1<<"yo1=====:"<<yo1<<std::endl;

            pose_A_.x = xo1 - min_radius_ * std::sin(adjust_angle_ver - yaw);
            pose_A_.y = yo1 - min_radius_ * std::cos(adjust_angle_ver - yaw); 
            pose_A_.theta = yaw - adjust_angle_ver;
            std::cout<<"pose A----x:"<<pose_A_.x<<"    pose A---y:"<<pose_A_.y<<"    pose A----theta:"<<pose_A_.theta<<std::endl;
            pose_o1_.x = xo1;
            pose_o1_.y = yo1;// for calparking
            float target_x = pose_A_.x;
            float target_y = pose_A_.y;
            float target_theta = pose_A_.theta ;
     
            ForwardBezierPath(target_x, target_y, target_theta, path);
            // std::cout<<"forwardend.theta========"<<path.back().theta<<"+PI/2===="<<path.back().theta+ M_PI/2.<<std::endl;
        }
        break;   
    }  
    }

    for (auto& pose : path) {
        pose.v = forward_speed_;
    }
    for (const auto& pose : path) {
        writing_file << pose.x << "," << pose.y << std::endl;
    }
    //trans path  TO start
    // for (int i=0;i<path.size();i++){
    //     path[i].x = path[i].x - start_pose_.x * std::cos(start_pose_.theta);
    //     path[i].y = path[i].y - start_pose_.y * std::sin(start_pose_.theta);
    // }
    
}

void ParkingGeoPlanner::CalcuParkingPath(const ParkingPose& start_pose, ParkingPath& path) {

    std::string filename = "../parking_result/result3.csv";
    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);
    writing_file << "x" << "," << "y" << std::endl;
    path.clear();
    switch (parking_type_) {
    case PARALOT0_right:
    {   
        double length1 = min_radius_ * (beta1_ - adjust_angle_para);
        int flag1 = 2;
        ParkingPose start_pose1 = start_pose;
        GenerateCircle(start_pose1, length1, min_radius_, flag1, path);
        ParkingPose start_pose2 = path.back();
        int flag2 = 0;
        double length2 = min_radius_ * beta1_;
        GenerateCircle(start_pose2, length2, -min_radius_, flag2, path);

        for (auto& pose : path) {
        // pose.theta += M_PI;
        pose.v = backward_speed_;
        }
        break;
    }
    case PARALOT0_left:
    {
        double length1 = min_radius_ * (beta1_ - adjust_angle_para);
        int flag1 = 2;
        ParkingPose start_pose1 = start_pose;
        GenerateCircle(start_pose1, length1, -min_radius_, flag1, path);
        ParkingPose start_pose2 = path.back();
        int flag2 = 0;
        double length2 = min_radius_ * beta1_;
        GenerateCircle(start_pose2, length2, min_radius_, flag2, path);

        for (auto& pose : path) {
        // pose.theta += M_PI;
        pose.v = backward_speed_;
        }
        break;
    }
    case PARALOT1_right:
    {
        double length1 = (min_radius_) * (beta1_ - adjust_angle_para);
        int flag1 = 0;
        ParkingPose start_pose1 = start_pose;
        GenerateCircle(start_pose1, length1, -(min_radius_), flag1, path);
        ParkingPose start_pose2 = path.back();
        int flag2 = 0;
        double length2 = (min_radius_) * beta1_;
        GenerateCircle(start_pose2, length2, (min_radius_), flag2, path);

        for (auto& pose : path) {
        // pose.theta += M_PI;
        pose.v = forward_speed_;
        }
        break;
    }
    case PARALOT1_left:
    {
        double length1 = (min_radius_) * (beta1_ - adjust_angle_para);
        int flag1 = 0;
        ParkingPose start_pose1 = start_pose;
        GenerateCircle(start_pose1, length1, (min_radius_), flag1, path);
        ParkingPose start_pose2 = path.back();
        int flag2 = 0;
        double length2 = (min_radius_) * beta1_;
        GenerateCircle(start_pose2, length2, -(min_radius_), flag2, path);

        for (auto& pose : path) {
        // pose.theta += M_PI;
        pose.v = forward_speed_;
        }
        break;
    }   
    case VERLOT_right:
    {
        float dx = parking_lot_[0].x - parking_lot_[3].x;
        float dy = parking_lot_[0].y - parking_lot_[3].y;
        float yaw = std::atan2(dy, dx);
        float lotw = std::sqrt(dx * dx + dy * dy);
        if (IsDoneOnceStep()) {
            double beta1 = M_PI/2.0;
            beta1 = M_PI_2 - adjust_angle_ver;
            // beta1 = M_PI_2 - yaw ;
            // if(beta1> M_PI/2.0){
            //     beta1 = M_PI_2;
            // }
        
            double length = min_radius_ * beta1;
            int flag = 2;
            std::cout<<"IsDoneOnceStep-----start_pose======"<<start_pose.x<<"\t"<<start_pose.y<<std::endl;
            // std::cout<<"IsDoneOnceStep-----length=="<<length<<std::endl;
            GenerateCircle(start_pose, length, min_radius_, flag, path);
            std::cout<<"IsDoneOnceStep----path.size()========="<<path.size()<<std::endl;
            for (auto& pose : path) {
                // pose.theta += M_PI;
                pose.v = backward_speed_;
            }
        } 
        else {
            // path.clear();
            dis_o1_23_ = 0;
            double l_o1_2 = std::sqrt(std::pow(pose_o1_.x - parking_lot_[2].x,2)+std::pow(pose_o1_.y - parking_lot_[2].y,2));
            double l_2_3 = std::sqrt(std::pow(parking_lot_[2].x - parking_lot_[3].x,2) + std::pow(parking_lot_[2].y - parking_lot_[3].y,2));
            double l_o1_3 = std::sqrt(std::pow(pose_o1_.x - parking_lot_[3].x,2)+std::pow(pose_o1_.y - parking_lot_[3].y,2));
            std::cout<<"l_o1_2------:"<<l_o1_2<<" l_2_3-----:"<<l_2_3<<"l_o1_3-----:"<<l_o1_3<<std::endl;
            double angle_23o1 = std::acos((l_o1_3*l_o1_3 + l_2_3*l_2_3 - l_o1_2*l_o1_2)/(2.0*l_o1_3*l_2_3));
            
            double angle_Bo1J = std::asin(vehicle_l_r_/(min_radius_ + vehicle_width_/2.0));
            double diso1_23 = l_o1_3 * sin(angle_23o1);
            dis_o1_23_ = diso1_23;
            // std::cout<<"diso1_23============"<<diso1_23<<std::endl;
            double gamma = std::asin((diso1_23 - L_safe_2)/r_rout_); 
            std::cout<<"gamma============="<<gamma<<std::endl;
            double test = ((l_o1_3 * sin(angle_23o1))/r_rout_);
            // double beta1 = gamma - angle_Bo1J - adjust_angle_ver;
            
            double beta1 = gamma - angle_Bo1J - adjust_angle_ver ;
            std::cout<< "beta1============="<<beta1 <<std::endl;
            std::cout<<"l_o1_3====="<<l_o1_3<<"\t"<<"angle_23o1===="<<angle_23o1<<"\t"<<"test==="<<test<<std::endl;
            beta1_ = beta1;
            double length = min_radius_ * beta1;
            int flag = 2;//2
            GenerateCircle(start_pose, length, min_radius_, flag, path);
            std::cout<<"parkingpath end.theta+PI =========="<<path.back().theta + M_PI<<std::endl;
            // std::cout<<"yaw + beta1 =========="<<yaw + beta1_<<std::endl;
        } 
    
        for (auto& pose : path) {
            // pose.theta += M_PI;
            pose.v = backward_speed_;
        }
        break;
    }
    case VERLOT_left:
    {
        float dx = parking_lot_[1].x - parking_lot_[2].x;
        float dy = parking_lot_[1].y - parking_lot_[2].y;
        float yaw = std::atan2(dy, dx);
        float lotw = std::sqrt(dx * dx + dy * dy);

        if (IsDoneOnceStep()) {
            double beta1 = M_PI/2.0;
            beta1 = M_PI_2 - adjust_angle_ver;
            // beta1 = M_PI_2 - yaw ;
            // if(beta1> M_PI/2.0){
            //     beta1 = M_PI_2;
            // }
            double length = min_radius_ * beta1;
            int flag = 2;
            std::cout<<"IsDoneOnceStep-----start_pose======"<<start_pose.x<<"\t"<<start_pose.y<<std::endl;
            // std::cout<<"IsDoneOnceStep-----length=="<<length<<std::endl;
            GenerateCircle(start_pose, length, -min_radius_, flag, path);
            std::cout<<"IsDoneOnceStep----path.size()========="<<path.size()<<std::endl;
            for (auto& pose : path) {
                // pose.theta += M_PI;
                pose.v = backward_speed_;
                // pose.kappa = 
            }
        } 
        else {
            // path.clear();
            dis_o1_23_ = 0;
            double l_o1_2 = std::sqrt(std::pow(pose_o1_.x - parking_lot_[2].x,2)+std::pow(pose_o1_.y - parking_lot_[2].y,2));
            double l_2_3 = std::sqrt(std::pow(parking_lot_[2].x - parking_lot_[3].x,2) + std::pow(parking_lot_[2].y - parking_lot_[3].y,2));
            double l_o1_3 = std::sqrt(std::pow(pose_o1_.x - parking_lot_[3].x,2)+std::pow(pose_o1_.y - parking_lot_[3].y,2));
            std::cout<<"l_o1_2------:"<<l_o1_2<<" l_2_3-----:"<<l_2_3<<"l_o1_3-----:"<<l_o1_3<<std::endl;
        
            double angle_32o1 = std::acos((l_o1_2*l_o1_2 + l_2_3*l_2_3 - l_o1_3*l_o1_3)/(2.0*l_o1_2*l_2_3));
            // std::cout<<"l_o1_2 * sin(angle_32o1)====="<<l_o1_2 * std::sin(angle_32o1)<<std::endl;
            double angle_Bo1J = std::asin(vehicle_l_r_/(min_radius_ + vehicle_width_/2.0));
            double diso1_23 = l_o1_2 * std::sin(angle_32o1);
            dis_o1_23_ = diso1_23;
            std::cout<<"diso1_23============"<<diso1_23<<std::endl;
            double gamma = std::asin((diso1_23 - L_safe_2)/r_rout_); 
            std::cout<<"gamma============="<<gamma<<std::endl;
            double test = ((l_o1_2 * sin(angle_32o1))/r_rout_);
            // double beta1 = gamma - angle_Bo1J - adjust_angle_ver;
            
            double beta1 = gamma - angle_Bo1J - adjust_angle_ver ;
            std::cout<< "beta1============="<<beta1 <<std::endl;
            // std::cout<<"r_rout_===="<<r_rout_<<"\t"<<"angle_Bo1J===="<<angle_Bo1J<<"\t"<<"test==="<<test<<std::endl;
            beta1_ = beta1;
            double length = min_radius_ * beta1;
            int flag = 2;//2
            GenerateCircle(start_pose, length, -min_radius_, flag, path);
            std::cout<<"parkingpath end.theta======="<<path.back().theta<<std::endl;
            // std::cout<<"yaw + beta1 =========="<<yaw + beta1_<<std::endl;
        } 
    
        for (auto& pose : path) {
            // pose.theta += M_PI;
            pose.v = backward_speed_;
        }
        break;
    }
    }
    for (const auto& pose : path) {
        writing_file << pose.x << "," << pose.y << std::endl;
    }
    std::cout<<"-------------------CalcuParkingPath success--------------------"<<std::endl;
    
    
}

void ParkingGeoPlanner::CalcuReplanPath(const ParkingPose& start_pose, ParkingPath& path) {
    std::string filename = "../parking_result/result4.csv";
    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);
    writing_file << "x" << "," << "y" << std::endl;
    path.clear();
    switch (parking_type_)
    {
    case VERLOT_right:
    {   
        float dx = parking_lot_[0].x - parking_lot_[3].x;
        float dy = parking_lot_[0].y - parking_lot_[3].y;
        float yaw = std::atan2(dy, dx);
        double diso_o3_23 = min_radius_ + half_lot_width_;
        std::cout<<"dis_o3_23============"<<diso_o3_23<<std::endl;
        double beta2 = std::asin(0.5*(diso_o3_23 - dis_o1_23_)/min_radius_ + std::sin(beta1_ + adjust_angle_ver)) - beta1_ - adjust_angle_ver; 
        // double beta2 = std::asin(0.5*(diso_o3_23 - dis_o1_23_)/min_radius_ + std::sin(beta1_ + adjust_angle_ver)) - beta1_ - adjust_angle_ver; 
        // beta2 = std::fabs(beta2);
        beta2_ = beta2;
        double length = std::fabs(min_radius_ * beta2);
        // std::cout << "start_pose.x : " << start_pose.x << " y: " << start_pose.y << " theta: " << start_pose.theta << std::endl;
        int flag = 2;//2
        GenerateCircle(start_pose, length, min_radius_, flag, path);
        std::cout<<"ReplanPathend.theta =========="<<path.back().theta<<std::endl;
        // std::cout<<"yaw + beta1 + beta2 =========="<<yaw + beta1_ + beta2_<<std::endl;
        break;
    }
    case VERLOT_left:
    {   
        float dx = parking_lot_[1].x - parking_lot_[2].x;
        float dy = parking_lot_[1].y - parking_lot_[2].y;
        float yaw = std::atan2(dy, dx);
        double diso_o3_23 = min_radius_ + half_lot_width_;
        std::cout<<"dis_o3_23============"<<diso_o3_23<<std::endl;
        double beta2 = std::asin(0.5*(diso_o3_23 - dis_o1_23_)/min_radius_ + std::sin(beta1_ + adjust_angle_ver)) - (beta1_ + adjust_angle_ver); 
        // beta2 = std::fabs(beta2);
        beta2_ = beta2;
        double length = std::fabs(min_radius_ * beta2);
        // std::cout << "start_pose.x : " << start_pose.x << " y: " << start_pose.y << " theta: " << start_pose.theta << std::endl;
        int flag = 2;//2
        GenerateCircle(start_pose, length, -min_radius_, flag, path);
        std::cout<<"ReplanPathend.theta =========="<<path.back().theta<<std::endl;
        // std::cout<<"yaw + beta1 + beta2 =========="<<yaw + beta1_ + beta2_<<std::endl;
        break;
    }
    }
    
    for (auto& pose : path) {
        pose.v = forward_speed_;
    }
    for (const auto& pose : path) {
        writing_file << pose.x << "," << pose.y << std::endl;
    }
    std::cout<<"-------------------CalcuReplanPath success--------------------"<<std::endl;
   
}

void ParkingGeoPlanner::AfterReplanPath(const ParkingPose& start_pose, ParkingPath& path) {;
    std::string filename = "../parking_result/result5.csv";
    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);
    writing_file << "x" << "," << "y" << std::endl;
    path.clear();
    switch (parking_type_)
    {
    case VERLOT_right:
    {
        float dx = parking_lot_[0].x - parking_lot_[3].x;
        float dy = parking_lot_[0].y - parking_lot_[3].y;
        float yaw = std::atan2(dy, dx);
        double beta3 =  M_PI/2.0 - beta2_ -beta1_ - adjust_angle_ver;
        // double beta3 =  M_PI/2.0 - beta2_ -beta1_ - adjust_angle_ver;
        beta3_ = beta3;
        std::cout <<"beta1_======"<<beta1_<<"beta2_========"<<beta2_<<"beta3_=========="<<beta3_<<std::endl;
        double length = std::fabs(min_radius_ * beta3_);
        int flag = 2;//2
        GenerateCircle(start_pose, length, min_radius_, flag, path);
        std::cout<< "AfterReplanPath-end_pose.theta==" << path.back().theta<<std::endl;
        // std::cout<<"yaw + beta1 + beta2 + beta3 =========="<<yaw + beta1_ + beta2_ + beta3_<<std::endl;
        std::cout <<"target point.t==="<<target_pose_.theta<<std::endl;
        std::cout <<"delta======="<<path.back().theta - target_pose_.theta + M_PI<<endl;
        break;
    }
    case VERLOT_left:
    {
        float dx = parking_lot_[1].x - parking_lot_[2].x;
        float dy = parking_lot_[1].y - parking_lot_[2].y;
        float yaw = std::atan2(dy, dx);
        double beta3 =  M_PI/2.0 - beta2_ -beta1_ - adjust_angle_ver;
        // double beta3 =  M_PI/2.0 - beta2_ -beta1_ - adjust_angle_ver;
        beta3_ = beta3;
        std::cout <<"beta1_======"<<beta1_<<"beta2_========"<<beta2_<<"beta3_=========="<<beta3_<<std::endl;
        double length = std::fabs(min_radius_ * beta3_);
        int flag = 2;//2
        GenerateCircle(start_pose, length, -min_radius_, flag, path);
        std::cout<< "AfterReplanPath-end_pose.theta==" << path.back().theta<<std::endl;
        // std::cout<<"yaw + beta1 + beta2 + beta3 =========="<<yaw + beta1_ + beta2_ + beta3_<<std::endl;
        std::cout <<"target point.t==="<<target_pose_.theta<<std::endl;
        std::cout <<"delta======="<<path.back().theta - target_pose_.theta + M_PI<<endl;

        break;
    }
    }

    
    for (auto& pose : path) {
        // pose.theta += M_PI;
        pose.v = backward_speed_;
    }
    for (const auto& pose : path) {
        writing_file << pose.x << "," << pose.y << std::endl;
    }
    std::cout<<"-------------------AfterReplanPath success--------------------"<<std::endl;
    
}

void ParkingGeoPlanner::CalcuFinalPath(const ParkingPose& start_pose, ParkingPath& path) {
    std::string filename = "../parking_result/result6.csv";
    std::ofstream writing_file;
    writing_file.open(filename, std::ios::out);
    writing_file << "x" << "," << "y" << std::endl;
    
    AERROR<<"start_pose.x =" << start_pose.x <<"    start_pose.y =" << start_pose.y<<"    start_pose.theta =" << start_pose.theta;
    double dx = target_pose_.x - start_pose.x;
    double dy = target_pose_.y - start_pose.y;
    double dtheta ;
    double target_theta_final;
    double dlength_back = dx * std::cos(target_pose_.theta) + dy * std::sin(target_pose_.theta);
    
    double length = std::sqrt(dx*dx+dy*dy);
    ParkingPose start_pose_final;
    start_pose_final.x = start_pose.x;
    start_pose_final.y = start_pose.y;
    if( std::abs(start_pose.theta)>0.5*M_PI){
        start_pose_final.theta = start_pose.theta;
        std::cout <<"back----------start_pose_final.theta=:"<<start_pose_final.theta <<std::endl;
    }else{
        start_pose_final.theta = start_pose.theta + M_PI;
    }

    if(dlength_back>0){// s behind g  
       start_pose_final.theta += M_PI;
       target_theta_final = target_pose_.theta;
       std::cout <<"-----------------startp behind goalp--------------"<<std::endl;
    }else{//s forward g 
        // dx = -dx;
        // dy = -dy;
        target_theta_final = target_pose_.theta + M_PI;
       std::cout <<"-----------------startp forward goalp--------------"<<std::endl;
    }
    dtheta = target_theta_final - start_pose_final.theta; 
    AERROR<<"start_pose_final.x =" << start_pose_final.x <<";start_pose_final.y =" << start_pose_final.y<<";start_pose_final.theta =" << start_pose_final.theta;
    AERROR<<"target_pose_.x =" << target_pose_.x <<";target_pose_.y =" << target_pose_.y<<";target_theta_final =" << target_theta_final;
    AERROR<<"final dx =" << dx <<",final dy ="<< dy <<",final  dt ="<< dtheta << " ,dlen  " << dlength_back;
     
    // GenerateLine(start_pose_final, length, path);
    // for (auto& pose : path) {
    //     if(dlength_back < 0 )//s forward g 
    //         pose.v = backward_speed_;
    //     else{// s behind g  
    //         pose.v = forward_speed_;
    //     }
    // }
    // ForwardBezierPath(dx,dy,dtheta,path);
    double target_x = dx * std::cos(start_pose_final.theta) + dy * std::sin(start_pose_final.theta);
    double target_y = - dx * std::sin(start_pose_final.theta) + dy * std::cos(start_pose_final.theta);
    ForwardBezierPath(target_x,target_y,dtheta,path);
    for (auto& pose : path) {
        if(dlength_back < 0 )//s forward g 
        {   
            // pose.x = -pose.x;
            // pose.y = - pose.y;
            pose.v = backward_speed_;
        }
        else{// s behind g  
            pose.v = forward_speed_;
        }
        double tmppose_x = start_pose_final.x + pose.x * std::cos(start_pose_final.theta) - pose.y * std::sin(start_pose_final.theta);
        double tmppose_y = start_pose_final.y + pose.x * std::sin(start_pose_final.theta) + pose.y * std::cos(start_pose_final.theta);
        pose.x = tmppose_x;
        pose.y = tmppose_y;
        pose.theta += start_pose_final.theta;
    }
    for (const auto& pose : path) {
        writing_file << pose.x << "," << pose.y << std::endl;
    }
    std::cout<<"FinalPath.size()=== "<<path.size()<<std::endl;
    std::cout<<"-------------------CalcuFinalPath success--------------------"<<std::endl;
   
}

void ParkingGeoPlanner::GenerateLine(const ParkingPose& start_pose, double length, ParkingPath& path) {
    double step = 0.1;
    double costh = std::cos(start_pose.theta);
    double sinth = std::sin(start_pose.theta);
    ParkingPose tmp;
    for (double s = 0.0; s <= length; s += step) {
        tmp.x = start_pose.x + s * costh;
        tmp.y = start_pose.y + s * sinth;
        tmp.theta = start_pose.theta;

        path.emplace_back(tmp);
    }
}


// flag 0 1 2 3分别对应从起点开始的第一象限 第二象限 第三象限 第四象限
void ParkingGeoPlanner::GenerateCircle(const ParkingPose& start_pose, double length, double radius, int flag, ParkingPath& path) {
    int start_index = path.size();
    double step = 0.02;
    double curvature = 1.0 / radius;
    double yaw = start_pose.theta - M_PI / 2.0;
    std::cout << "GenerateCircle length: " << length << std::endl;
    ParkingPose tmp;
    for (double s = 0.0; s <= length; s+=step) {
        double a = 2.0 / curvature * std::sin(s * curvature / 2.0);
        double alfa = (M_PI - s * curvature) / 2.0 - yaw;
        
        double c1 , c2;
        if (flag == 0) {
            c1 = -1.0;
            c2 = 1.0;
        } else if (flag == 1) {
            c1 = 1.0;
            c2 = 1.0;
        } else if (flag == 2) {
            c1 = 1.0;
            c2 = -1.0;
        } else {
            c1 = -1.0;
            c2 = -1.0;
        }

        tmp.x = c1 * a * std::cos(alfa) + start_pose.x;//-1.0
        tmp.y = c2 * a * std::sin(alfa) + start_pose.y;//1.0
        tmp.theta = start_pose.theta;
        // tmp.x = c1 * a * std::sin(alfa) + start_pose.x;//-1.0
        // tmp.y = c2 * a * std::cos(alfa) + start_pose.y;//1.0        
        path.emplace_back(tmp);
    }
    if (path.empty()) {
        return;
    }
    for (int i = start_index + 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        double yaw = std::atan2(dy, dx);
        path[i].theta = yaw;
    }

}

bool ParkingGeoPlanner::IsDoneOnceStep() {
    ParkingPose pose_A;
    float lateral_disA = 0;
    // lateral_disA_allowed_ver = 2.5;
    switch (parking_type_)
    {
    case VERLOT_right:
    { 
        float dx = parking_lot_[0].x - parking_lot_[3].x;
        float dy = parking_lot_[0].y - parking_lot_[3].y;
        float yaw = std::atan2(dy, dx);
        std::cout<<"LOt------yaw=========="<<yaw<<std::endl;
        L_safe_1 = 0.1;
        float l0_o1 = min_radius_-vehicle_width_/2.0-L_safe_1;
        float alpha = acos((min_radius_-half_lot_width_)/l0_o1);
        
        float xo1 = parking_lot_[0].x + l0_o1 * std::cos(alpha - yaw );
        float yo1 = parking_lot_[0].y - l0_o1 * std::sin(alpha - yaw );
        double ds = 1.0;
        if(yaw>=0){
            pose_A.x = xo1 - (min_radius_ - ds)* std::sin(yaw + adjust_angle_ver);
            pose_A.y = yo1 + (min_radius_ - ds) * std::cos(yaw + adjust_angle_ver);
            pose_A.theta = yaw + adjust_angle_ver;
            lateral_disA = min_radius_ * std::cos(adjust_angle_ver)- l0_o1 * std::sin(alpha);
            
        }
        else{
            pose_A.x = xo1 - (min_radius_ - ds) * std::sin(adjust_angle_ver + yaw);
            pose_A.y = yo1 + (min_radius_ - ds) * std::cos(adjust_angle_ver + yaw);
            pose_A.theta = yaw + adjust_angle_ver;
            lateral_disA = min_radius_ * std::cos(adjust_angle_ver) - l0_o1*std::sin(alpha) + vehicle_l_f_*std::sin(adjust_angle_ver);
        }

        break;
    }
    case VERLOT_left:
    {
        float dx = parking_lot_[1].x - parking_lot_[2].x;
        float dy = parking_lot_[1].y - parking_lot_[2].y;
        float yaw = std::atan2(dy, dx);
        // std::cout<<"LOt------yaw=========="<<yaw<<std::endl;
        float l1_o1 = min_radius_-vehicle_width_/2.0-L_safe_1;
        float alpha = acos((min_radius_-half_lot_width_)/l1_o1);
        float xo1 = parking_lot_[1].x + l1_o1 * std::cos(alpha + yaw );
        float yo1 = parking_lot_[1].y + l1_o1 * std::sin(alpha + yaw );
        std::cout << "parking_lot_[1].x = : " << parking_lot_[1].x << " parking_lot_[1].y = : " << parking_lot_[1].y << std::endl; 
        std::cout << "xo1 = : " << xo1 << " yo1 = : " << yo1 << " alpha = " << alpha <<std::endl; 
        if(yaw<=0){
            // std::cout<<"111111111111111111111111111111111111"<<std::endl;
            pose_A.x = xo1 - min_radius_ * std::sin(-yaw + adjust_angle_ver);
            pose_A.y = yo1 - min_radius_ * std::cos(-yaw + adjust_angle_ver);
            pose_A.theta = yaw - adjust_angle_ver;
            lateral_disA = min_radius_* std::cos(adjust_angle_ver)- l1_o1 * std::sin(alpha);
            
        }
        else{
            pose_A.x = xo1 - min_radius_ * std::sin(adjust_angle_ver - yaw);
            pose_A.y = yo1 - min_radius_ * std::cos(adjust_angle_ver - yaw);
            pose_A.theta = yaw - adjust_angle_ver;
            lateral_disA = min_radius_ * std::cos(adjust_angle_ver) - l1_o1*std::sin(alpha) + vehicle_l_f_*std::sin(adjust_angle_ver);
            
        }   
        break;
    }
    }
    pose_A_ = pose_A;
    AERROR << "parkingplan lateral_disA === "<< lateral_disA;
    if(lateral_disA < lateral_disA_allowed_ver){
        std::cout << "suit for done once" << std::endl;
        std::cout << "pose_A_.x====" <<pose_A_.x<<"\t"<<"pose_A_.y===="<<pose_A_.y<<"\t"<<"pose_A_.theta==="<<pose_A_.theta<<std::endl;
        Isdoneoncestep = true;
        return true;
    }
        

    std::cout << "not suit for done once" << std::endl;
    Isdoneoncestep = true;
    return false;
}

void ParkingGeoPlanner::ForwardBezierPath(float goal_x, float goal_y, float goal_theta, ParkingPath& path) {
    double x0 = 0;
	double y0 = 0;
	double theta0 = 0.0;
    double step0, stepf;
    double step = 0.1;
    double x1, y1, x2, y2, x3, y3, xf, yf, thetaf;
    double sig;
    xf = goal_x;
    yf = goal_y;
    if (xf>=0.0)
        sig = 1.0;
    else
        sig = -1.0;
    sig = 1.0;
    thetaf = goal_theta;
    double dis = std::sqrt(goal_x * goal_x + goal_y * goal_y);

    step0 = sig*dis/3.0;
    stepf = sig*dis/3.0;
    x1 = x0 + step0*cos(theta0);
    y1 = y0 + step0*sin(theta0);

    x3 = xf - stepf*cos(thetaf);
    y3 = yf - stepf*sin(thetaf);
    x2 = (x1 + x3) / 2.0;
    y2 = (y1 + y3) / 2.0;
        
    ParkingPose current_pose;
    ParkingPose last_pose;
    int bezierlength = std::floor(std::sqrt(xf*xf+yf*yf)/step); //instead LENGTHOFPLAN
    float t[bezierlength];
    for (int i = 0; i < bezierlength; i++) {
        t[i] = 1.0*i/ (bezierlength - 1);
    }
    for (int i = 0; i < bezierlength; i++) {
        
        current_pose.x = 4 * x1 * std::pow(1 - t[i], 3)*t[i] + 6 * x2*std::pow(1 - t[i], 2)*t[i] * t[i] + 4 * x3*(1 - t[i])*t[i] * t[i] * t[i] + xf*pow(t[i], 4);
        current_pose.y = 4 * y1 * std::pow(1 - t[i], 3)*t[i] + 6 * y2*std::pow(1 - t[i], 2)*t[i] * t[i] + 4 * y3*(1 - t[i])*t[i] * t[i] * t[i] + yf*pow(t[i], 4);
        current_pose.v = forward_speed_;
        if (i == 0){ 
            current_pose.theta = 0; 
        } else {
            last_pose.x = 4 * x1 * pow(1 - t[i-1], 3)*t[i-1] + 6 * x2*pow(1 - t[i-1], 2)*t[i-1] * t[i-1] + 4 * x3*(1 - t[i-1])*t[i-1] * t[i-1] * t[i-1] + xf*pow(t[i-1], 4);;
            last_pose.y = 4 * y1 * pow(1 - t[i-1], 3)*t[i-1] + 6 * y2*pow(1 - t[i-1], 2)*t[i-1] * t[i-1] + 4 * y3*(1 - t[i-1])*t[i-1] * t[i-1] * t[i-1] + yf*pow(t[i-1], 4);;
            double dx = current_pose.x - last_pose.x;
            double dy = current_pose.y - last_pose.y;
            current_pose.theta = atan2f(dy, dx);
        }
        path.push_back(current_pose);
    }
    std::cout<<"-------------------ForwardBezierPath success--------------------"<<std::endl;
    
}

void ParkingGeoPlanner::LocalToGlobal(const ParkingPose& current_pose, const Trajectory& local_trajectory, 
				   Trajectory& global_trajectory) {
	double sin_theta = std::sin(current_pose.theta);
	double cos_theta = std::cos(current_pose.theta);
	global_trajectory.clear();
	for(const auto& pose : local_trajectory) {
		TrajectoryPose global_pose;
		global_pose = pose;
		global_pose.x = current_pose.x + pose.x * cos_theta - pose.y * sin_theta;
		global_pose.y = current_pose.y + pose.x * sin_theta + pose.y  * cos_theta;
		global_trajectory.push_back(global_pose);
	}
}

}