#include <math.h>
#include "parking_planner.h"
int main(int argc, char** argv) {
    planning::ParkingGeoPlanner parking_planner;

    planning::ParkingPose start_pose, ver_target_poser, ver_target_posel, para_target_pose_parar, para_target_pose_paral;
    ver_target_poser.x = 15.0;
    ver_target_poser.y = -8;
    ver_target_poser.theta = 90. * M_PI / 180.;
    planning::ParkingType ver_parking_typer = planning::ParkingType::VERLOT_right;
    ver_target_posel.x = 15.0;
    ver_target_posel.y = 8;
    ver_target_posel.theta = -90. * M_PI / 180.;
    planning::ParkingType ver_parking_typel = planning::ParkingType::VERLOT_left;
    
    para_target_pose_parar.x = 15.0;
    para_target_pose_parar.y = -3.0;
    para_target_pose_parar.theta = 10. * M_PI / 180.;
    //////////
    para_target_pose_paral.x = 15.0;
    para_target_pose_paral.y = 3.0;
    para_target_pose_paral.theta = 5. * M_PI / 180.;
    planning::ParkingType para_parking_type0_r = planning::ParkingType::PARALOT0_right;
    planning::ParkingType para_parking_type0_l = planning::ParkingType::PARALOT0_left;

    planning::ParkingType para_parking_type1_r = planning::ParkingType::PARALOT1_right;
    planning::ParkingType para_parking_type1_l = planning::ParkingType::PARALOT1_left;

    Trajectory tarjectory;
    parking_planner.Init();
    int testtpye = 0;   
    //para 0 1// 2 3 ver 4 5
    switch (testtpye)
    {
    case 0:{
        parking_planner.MakePlan(start_pose,para_target_pose_parar,para_parking_type0_r, tarjectory);
        std::cout<<"parkingtype ========= 0"<<std::endl;
        break;
        }
    case 1:{
        parking_planner.MakePlan(start_pose,para_target_pose_paral,para_parking_type0_l, tarjectory);
        std::cout<<"parkingtype ========= 1"<<std::endl;
        break;
    }
    case 2:{
        parking_planner.MakePlan(start_pose,para_target_pose_parar,para_parking_type1_r, tarjectory);
        std::cout<<"parkingtype ========= 2"<<std::endl;
        break;
    }
    case 3:{
        parking_planner.MakePlan(start_pose,para_target_pose_paral,para_parking_type1_l, tarjectory);
        std::cout<<"parkingtype ========= 3"<<std::endl;
    }
    case 4:{
        parking_planner.MakePlan(start_pose,ver_target_poser,ver_parking_typer,tarjectory);
        std::cout<<"parkingtype ========= 4"<<std::endl;
        break;
    }
    case 5:{
        parking_planner.MakePlan(start_pose,ver_target_posel,ver_parking_typel,tarjectory);
        std::cout<<"parkingtype ========= 5"<<std::endl;
        break;
    }
    default:{
        std::cout<< "------tpye error------"<< std::endl;
        break;
    }
    }
    return 0;
} 