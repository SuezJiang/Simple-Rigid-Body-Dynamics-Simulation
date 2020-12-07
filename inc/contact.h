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

struct Contact {
	int a;				// a cube, for a fv contact, a gives the face
	int b;				// b cube
	double overlap;		// overlap
	Eigen::Vector3d p;	// global contact position
	Eigen::Vector3d n;	// global normal, should point from b to a
	Eigen::Vector3d ea;	// edgedirection for a
	Eigen::Vector3d eb; // edgedirection for b
	bool vf;			// true if vertex/face contact
};