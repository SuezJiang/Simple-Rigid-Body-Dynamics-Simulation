#include <windef.h>
#include "Eigen/Dense"
#include <Eigen/Cholesky>  
#include <Eigen/LU>  
#include <Eigen/QR>  
#include <Eigen/SVD>  

#include "../3rdLibs/glm/glm/glm.hpp"
#include "../3rdLibs/glm/glm/gtc/matrix_transform.hpp"
#include "../3rdLibs/glm/glm/gtc/type_ptr.hpp"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

struct RigidCube {
	double mass;		// M
	Eigen::Matrix3d Ibody, Ibodyinv;	// Ibody and Ibodyinv
	// states
	Eigen::Vector3d x;		// global position
	Eigen::Matrix3d R;		// global rotation
	Eigen::Vector3d P;		// linear momentum
	Eigen::Vector3d L;		// angular momentum

	// for cube, I = Ibody, Iinv = Ibodyinv
	Eigen::Vector3d v;		// linear velocity
	Eigen::Vector3d omega;	// angular velocity

	// computed quantities
	Eigen::Vector3d force;	// externel force
	Eigen::Vector3d torque; // externel torque
};