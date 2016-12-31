#define _USE_MATH_DEFINES

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

using namespace Eigen;


Eigen::Quaterniond euler2Quaternion(double roll, double pitch, double yaw ){
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    return q;
}

int main(){
    MatrixXd rotationA(3,3),rotationB(3,3);

    double rollA = M_PI/4, rollB = -M_PI/3;
    double pitchA = 0, pitchB = 0;
    double yawA = M_PI/3, yawB = 0;
    
    Eigen::Quaterniond qA = euler2Quaternion(rollA,pitchA,yawA);
    Eigen::Quaterniond qB = euler2Quaternion(rollB,pitchB,yawB);
    //Eigen::Quaterniond qA1 = euler2Quaternion1(rollA,pitchA,yawA);
    rotationA = qA.toRotationMatrix();
    rotationB = qB.toRotationMatrix();

    //Eigen::Matrix3d rmx1 = qA1.matrix();
    // Q2
    std::cout<<"\nQuestion 2\n";
    std::cout<<"Rotation matrix A is\n"; 
    std::cout << rotationA <<"\n";
    //std::cout <<"\n\n"<< rmx1 <<"\n\n";

    std::cout<<"Rotation matrix B is\n";
    std::cout << rotationB <<"\n";

    std::cout<<"matrixA*matrixB is\n";
    std::cout << rotationA*rotationB <<"\n";

    std::cout<<"matrixB*matrixA is\n";
    std::cout << rotationB*rotationA <<"\n";    
    // Q3
    std::cout<<"\nQuestion 3\n";
    std::cout<<"Quarternion A:"<<qA.w()<<" "<<qA.x()<<" "<<qA.y()<<" "<<qA.z()<<std::endl;
    std::cout<<"Quarternion B:"<<qB.w()<<" "<<qB.x()<<" "<<qB.y()<<" "<<qB.z()<<std::endl;
    
    // Q4
    std::cout<<"\nQuestion 4\n";
    Quaterniond qC = qA*qB;
    Quaterniond qD = qB*qA;
    std::cout<<"Quarternion C:"<<qC.w()<<" "<<qC.x()<<" "<<qC.y()<<" "<<qC.z()<<std::endl;
    std::cout<<"Quarternion D:"<<qD.w()<<" "<<qD.x()<<" "<<qD.y()<<" "<<qD.z()<<std::endl;
    Quaterniond qE = qA * qB.inverse();
    Quaterniond qF = qE*qB;
    std::cout<<"Quarternion E:"<<qE.w()<<" "<<qE.x()<<" "<<qE.y()<<" "<<qE.z()<<std::endl;
    std::cout<<"Quarternion F:"<<qF.w()<<" "<<qF.x()<<" "<<qF.y()<<" "<<qF.z()<<std::endl;
}
