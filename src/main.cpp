
#include <windef.h>
#include <glad/glad.h>  
#include <GL/glu.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include <Eigen/Cholesky>  
#include <Eigen/LU>  
#include <Eigen/QR>  
#include <Eigen/SVD>  
#include <time.h>
#include <math.h>

#include <iostream>
#include "../3rdLibs/glm/glm/glm.hpp"
#include "../3rdLibs/glm/glm/gtc/matrix_transform.hpp"
#include "../3rdLibs/glm/glm/gtc/type_ptr.hpp"

#include "../inc/my_texture.h"
#include "../inc/shader_m.h"
#include "tiny_obj_loader.h"
#include "rigid.h"
#include "contact.h"
#define STATE_SIZE 18

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The .obj .mtl and images are in Dir "model".                                                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------*/
//Here are some mouse and keyboard function. You can change that.
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
float degreeX = (360 * lastX / 400);
float degreeY = (360 * lastY / 300);
bool firstMouse = true;
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;
float OX = 0;//should be update to a new coordinate
float OY = 0;
float OZ = 0;
// camera parameter
glm::vec3 cameraPos = glm::vec3(10.0f, 10.0f, 10.0f);
glm::vec3 cameraFront = glm::vec3(-1.0f, -1.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
float currentFrame;
float yaw = -90.0;
float pitch = 0;
// parameters for rigid simulation
std::vector<RigidCube> Cubes;
int numCubes = 45;
double groundLevel = -1.0;
double wallLevel = 5.0;
// for contacts detection
std::vector<Contact> Contacts;
std::vector<std::vector<Contact>> contactsMap(numCubes);

// functions for rigid simulation
void initStates() {
	RigidCube temp;
	temp.mass = 0.5; temp.Ibody = temp.mass / 6.0 * Eigen::Matrix3d::Identity();
	temp.Ibodyinv = 6.0 / temp.mass * Eigen::Matrix3d::Identity();
	//
	temp.x = Eigen::Vector3d(0, 0, 0); temp.R = Eigen::Matrix3d::Identity();
	temp.P = Eigen::Vector3d(0, 0, 0); temp.L = Eigen::Vector3d(0, 0, 0);
	//
	temp.v = Eigen::Vector3d(0, 0, 0); temp.omega = Eigen::Vector3d(0, 0, 0);
	//
	temp.force = Eigen::Vector3d(0, 0, 0); temp.torque = Eigen::Vector3d(0, 0, 0);
	// 1
	Cubes.push_back(temp);
	// 2
	temp.x = Eigen::Vector3d(0, 1.5, 0);
	Cubes.push_back(temp);
	// 3
	temp.x = Eigen::Vector3d(1.5, 0, 0);
	Cubes.push_back(temp);
	if (numCubes >= 15) {
		temp.x = Eigen::Vector3d(-2.0, 0, -2.0);
		for (int i = 0; i < 10; i++) {
			Cubes.push_back(temp);
			temp.x[1] += 1.1;
		}
	}
	if (numCubes >= 25) {
		temp.x = Eigen::Vector3d(2.0, 0, -2.0);
		for (int i = 0; i < 10; i++) {
			Cubes.push_back(temp);
			temp.x[1] += 1.1;
		}
	}
	if (numCubes >= 35) {
		temp.x = Eigen::Vector3d(-2.0, 0, 2.0);
		for (int i = 0; i < 10; i++) {
			Cubes.push_back(temp);
			temp.x[1] += 1.1;
		}
	}
	if (numCubes >= 45) {
		temp.x = Eigen::Vector3d(2.0, 0, 2.0);
		for (int i = 0; i < 10; i++) {
			Cubes.push_back(temp);
			temp.x[1] += 1.1;
		}
	}
	// 4
	temp.x = Eigen::Vector3d(0.7, 3, 0);
	temp.R(0, 0) = sqrt(2) / 2.0; temp.R(1, 0) = sqrt(2) / 2.0; temp.R(2, 0) = 0.0;
	temp.R(0, 1) = -sqrt(2) / 2.0; temp.R(1, 1) = sqrt(2) / 2.0; temp.R(2, 1) = 0.0;
	Cubes.push_back(temp);
	// 5
	temp.x = Eigen::Vector3d(1.4, 5, 0);
	temp.R(0, 0) = sqrt(3) / 2.0; temp.R(1, 0) = 1.0 / 2.0; temp.R(2, 0) = 0.0;
	temp.R(0, 1) = -1.0 / 2.0; temp.R(1, 1) = sqrt(3) / 2.0; temp.R(2, 1) = 0.0;
	Cubes.push_back(temp);
}

// tools
void StateToArray(RigidCube* c, double* y) {
	// four states
	for (int i = 0; i < 3; i++) {
		*y++ = c->x[i];
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			*y++ = c->R(i, j);
		}
	}
	for (int i = 0; i < 3; i++) {
		*y++ = c->P[i];
	}
	for (int i = 0; i < 3; i++) {
		*y++ = c->L[i];
	}
}

void ArrayToState(RigidCube* c, double* y) {
	for (int i = 0; i < 3; i++) {
		c->x[i] = *y++;
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			c->R(i, j) = *y++;
		}
	}
	// orthognolize
	/*c->R.col(1) = c->R.col(1) - (c->R.col(1).dot(c->R.col(0))) / (c->R.col(0).squaredNorm()) * c->R.col(0);
	c->R.col(2) = c->R.col(2) - (c->R.col(2).dot(c->R.col(0))) / (c->R.col(0).squaredNorm()) * c->R.col(0)
		- (c->R.col(2).dot(c->R.col(1))) / (c->R.col(1).squaredNorm()) * c->R.col(1);
	for (int i = 0; i < 3; i++) {
		c->R.col(i) = c->R.col(i) / c->R.col(i).norm();
	}*/
	for (int i = 0; i < 3; i++) {
		c->P[i] = *y++;
	}
	for (int i = 0; i < 3; i++) {
		c->L[i] = *y++;
	}
	// also update more variables here
	c->v = c->P / c->mass;
	c->omega = c->Ibodyinv * c->L;
}

void ArrayToBodies(double x[]) {
	for (int i = 0; i < numCubes; i++) {
		ArrayToState(&Cubes[i], &x[i * STATE_SIZE]);
	}
}

void BodiesToArray(double x[]) {
	for (int i = 0; i < numCubes; i++) {
		StateToArray(&Cubes[i], &x[i * STATE_SIZE]);
	}
}

void ComputeForceAndTorque(double t, RigidCube* c, int i) {
	c->force = Eigen::Vector3d(0, -c->mass * 9.8, 0);
	c->torque = Eigen::Vector3d(0, 0, 0);
	// penalty force
	double constk = 5.0;
	for (int j = 0; j < contactsMap[i].size(); j++) {
		Eigen::Vector3d n = contactsMap[i][j].n;
		if (contactsMap[i][j].b == i) {
			n *= -1.0;
		}
		c->force += constk * contactsMap[i][j].overlap * n;
		c->torque += (contactsMap[i][j].p - c->x).cross(constk * contactsMap[i][j].overlap * n);
	}
}

Eigen::Matrix3d Star(Eigen::Vector3d omega) {
	Eigen::Matrix3d ret;
	ret << 0, -omega[2], omega[1],
		omega[2], 0, -omega[0],
		-omega[1], omega[0], 0;
	return ret;
}

void othognolizeR() {
	for (int i = 0; i < numCubes; i++) {
		RigidCube* c = &Cubes[i];
		c->R.col(1) = c->R.col(1) - (c->R.col(1).dot(c->R.col(0))) / (c->R.col(0).squaredNorm()) * c->R.col(0);
		c->R.col(2) = c->R.col(2) - (c->R.col(2).dot(c->R.col(0))) / (c->R.col(0).squaredNorm()) * c->R.col(0)
			- (c->R.col(2).dot(c->R.col(1))) / (c->R.col(1).squaredNorm()) * c->R.col(1);
		for (int i = 0; i < 3; i++) {
			c->R.col(i) = c->R.col(i) / c->R.col(i).norm();
		}
	}
}

void DdtStateToArray(RigidCube* c, double* xdot) {
	// real Ddt, v, dotR, F, t
	for (int i = 0; i < 3; i++) {
		*xdot++ = c->v[i];
	}
	Eigen::Matrix3d Rdot = Star(c->omega) * c->R;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			*xdot++ = Rdot(i, j);
		}
	}
	for (int i = 0; i < 3; i++) {
		*xdot++ = c->force[i];
	}
	for (int i = 0; i < 3; i++) {
		*xdot++ = c->torque[i];
	}
}

void Dxdt(double t, double x[], double xdot[]) {
	// using RK, should update internal state
	ArrayToBodies(x); 
	for (int i = 0; i < numCubes; i++) {
		ComputeForceAndTorque(t, &Cubes[i], i);
		DdtStateToArray(&Cubes[i], &xdot[i * STATE_SIZE]);
	}
}

// when enter this function, x0 should be equal to xFinal
void ode(double x0[], double xFinal[],int len, double t0, double t1, void(*DerivFunc)(double, double*, double*)) {
	// use a RK here
	double h = t1 - t0; // RK parameter
	double* k1 = new double[len];
	// cal k1
	DerivFunc(t0, x0, k1);
	double* temp = new double[len];
	for (int i = 0; i < len; i++) {
		temp[i] = x0[i] + k1[i] * h / 2;
	}
	// cal k2
	double* k2 = new double[len];
	DerivFunc(t0 + h / 2, temp, k2);
	for (int i = 0; i < len; i++) {
		temp[i] = x0[i] + k2[i] * h / 2;
	}
	// cal k3
	double* k3 = new double[len];
	DerivFunc(t0 + h / 2, temp, k3);
	for (int i = 0; i < len; i++) {
		temp[i] = x0[i] + k3[i] * h;
	}
	// cal k4
	double* k4 = new double[len];
	DerivFunc(t0 + h, temp, k4);
	// update xFinal
	for (int i = 0; i < len; i++) {
		xFinal[i] += h * (k1[i] / 6.0 + k2[i] / 3.0 + k3[i] / 3.0 + k4[i] / 6.0);
	}
	free(temp);
	free(k1); free(k2);
	free(k3); free(k4);
}

double transformToAxis(int c, Eigen::Vector3d axis) {
	// 8 vertices
	double projMax = -DBL_MAX;
	double projMin = DBL_MAX;
	//printf("transform to axis with %d\n",c);
	for (double cx = -0.5; cx < 1.0; cx += 1.0) {
		for (double cy = -0.5; cy < 1.0; cy += 1.0) {
			for (double cz = -0.5; cz < 1.0; cz += 1.0) {
				Eigen::Vector3d vertex = Cubes[c].x + cx * Cubes[c].R.col(0)
					+ cy * Cubes[c].R.col(1) + cz * Cubes[c].R.col(2);
				//printf("%f,%f,%f\n", vertex[0], vertex[1], vertex[2]);
				double proj = vertex.dot(axis);
				if (proj > projMax) { projMax = proj; }
				if (proj < projMin) { projMin = proj; }
			}
		}
	}
	return (projMax - projMin) / 2.0;
}

double penetrationOnAxis(int c1, int c2, Eigen::Vector3d axis, Eigen::Vector3d toCenter) {
	double oneProject = transformToAxis(c1, axis);
	double twoProject = transformToAxis(c2, axis);

	double distance = abs(toCenter.dot(axis));
	//printf("oneProject: %f, twoProject:%f, distance:%f\n", oneProject, twoProject, distance);
	//	return positive overlap
	return oneProject + twoProject - distance;
}

// generate contact between face and vertex
void fvContactGenerator(int a, int b ,Eigen::Vector3d axis, double overlap, Eigen::Vector3d toCenter) {
	if (axis.dot(toCenter) > 0) { axis *= -1.0; }
	Eigen::Vector3d vertex = Cubes[b].x + 0.5 * Cubes[b].R.col(0) + 0.5 * Cubes[b].R.col(1) + 0.5 * Cubes[b].R.col(2);
	if (Cubes[b].R.col(0).dot(axis) < 0) { vertex -= Cubes[b].R.col(0); }
	if (Cubes[b].R.col(1).dot(axis) < 0) { vertex -= Cubes[b].R.col(1); }
	if (Cubes[b].R.col(2).dot(axis) < 0) { vertex -= Cubes[b].R.col(2); }
	// create contact
	Contact temp;
	temp.a = a; temp.b = b;
	temp.overlap = overlap;
	temp.p = vertex; temp.n = axis;
	temp.vf = true;
	// push
	Contacts.push_back(temp);
	contactsMap[a].push_back(temp);
	contactsMap[b].push_back(temp);
}

void eeContactGenerator(int a, int b, int oneAxisIndex, int twoAxisIndex, Eigen::Vector3d axis, double overlap, Eigen::Vector3d toCenter) {
	// ?
	if (axis.dot(toCenter) > 0) { axis *= -1.0; }
	Eigen::Vector3d ptOnEdgeOne = Cubes[a].x;
	Eigen::Vector3d ptOnEdgeTwo = Cubes[b].x;
	for (int i = 0; i < 3; i++) {
		if (i == oneAxisIndex) {}
		else if (Cubes[a].R.col(i).dot(axis) > 0) { ptOnEdgeOne -= 0.5 * Cubes[a].R.col(i); }
		else { ptOnEdgeOne += 0.5 * Cubes[a].R.col(i); }
		if (i == twoAxisIndex) {}
		else if (Cubes[b].R.col(i).dot(axis) < 0) { ptOnEdgeTwo -= 0.5 * Cubes[b].R.col(i); }
		else { ptOnEdgeTwo += 0.5 * Cubes[b].R.col(i); }
	}
	Eigen::Vector3d axisOne = Cubes[a].R.col(oneAxisIndex);
	Eigen::Vector3d axisTwo = Cubes[b].R.col(twoAxisIndex);
	Eigen::Vector3d toSt = ptOnEdgeOne - ptOnEdgeTwo;
	// ?
	//toSt *= -1.0;
	double dpStaOne = axisOne.dot(toSt);
	double dpStaTwo = axisTwo.dot(toSt);

	double smOne = axisOne.squaredNorm();
	double smTwo = axisTwo.squaredNorm();
	double dotProductEdges = axisTwo.dot(axisOne);
	double denom = smOne * smTwo - dotProductEdges * dotProductEdges;
	double ta = (dotProductEdges * dpStaTwo - smTwo * dpStaOne) / denom;
	double tb = (smOne * dpStaTwo - dotProductEdges * dpStaOne) / denom;

	Eigen::Vector3d nearestPtOnOne = ptOnEdgeOne + axisOne * ta;
	Eigen::Vector3d nearestPtOnTwo = ptOnEdgeTwo + axisTwo * tb;
	Contact temp;
	temp.a = a; temp.b = b;
	temp.overlap = overlap;
	temp.n = axis;
	temp.p = nearestPtOnOne * 0.5 + nearestPtOnTwo * 0.5;
	temp.ea = axisOne; temp.eb = axisTwo;
	temp.vf = false;
	/*printf("%d and %d\n", a, b);
	printf("overlap is %f\n", overlap);
	printf("norm axis is %f, %f, %f\n", axis[0], axis[1], axis[2]);
	printf("ee contact point is %f %f %f\n", temp.p[0], temp.p[1], temp.p[2]);
	printf("edge %d ang edge %d\n", oneAxisIndex, twoAxisIndex);*/
	Contacts.push_back(temp);
	contactsMap[a].push_back(temp);
	contactsMap[b].push_back(temp);
}

void contactBtwCubes(int c1, int c2) {
	// first, get 15 axises needed
	std::vector<Eigen::Vector3d> axes;
	for (int i = 0; i < 3; i++) {
		axes.push_back(Cubes[c1].R.col(i));
	}
	for (int i = 0; i < 3; i++) {
		axes.push_back(Cubes[c2].R.col(i));
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			Eigen::Vector3d temp = Cubes[c1].R.col(i).cross(Cubes[c2].R.col(j));
			axes.push_back(temp);
		}
	}
	//
	double bestOverlap = DBL_MAX;
	unsigned int bestCase = 15;
	Eigen::Vector3d toCenter = Cubes[c2].x - Cubes[c1].x;
	//
	for (unsigned int i = 0; i < 15; i++) {
		Eigen::Vector3d temp = axes[i];
		// check if generated by (almost) parallel edges
		if (temp.squaredNorm() < 1e-3) { continue; }
		temp /= temp.norm();
		// do penetration on Axis
		double overlap = penetrationOnAxis(c1, c2, temp, toCenter);
		//printf("axis: %f, %f, %f\n", temp[0], temp[1], temp[2]);
		//printf("%d, %f\n", i, overlap);
		if (overlap < 0) { 
			return; 
		}
		// edge may have higher priority than fv (just guess)
		if (overlap < bestOverlap) {
			bestOverlap = overlap;
			bestCase = i;
		}
	}
	// fv or ee
	if (0 <= bestCase && bestCase <= 2) {
		fvContactGenerator(c1, c2, axes[bestCase], bestOverlap, toCenter);
		//printf("face: %d, vertex: %d\n", c1, c2);
	}
	else if (3 <= bestCase && bestCase <= 5) {
		fvContactGenerator(c2, c1, axes[bestCase], bestOverlap, toCenter * -1.0);
		//printf("face: %d, vertex: %d\n", c2, c1);
	}
	else {
		// edge-edge contact
		int oneAxisIndex = (bestCase - 6) / 3;
		int twoAxisIndex = bestCase % 3;
		eeContactGenerator(c1, c2, oneAxisIndex, twoAxisIndex, axes[bestCase], bestOverlap, toCenter);
		//printf("edge-edge: %d, %d\n", c1, c2);
	}
}

void contactWithGround(int c) {
	if (Cubes[c].x[1] - groundLevel > sqrt(3) / 2.0) {
		return;
	}

	int c2 = numCubes; // represent for ground
	double overlap = -DBL_MAX;
	Eigen::Vector3d touch;
	for (double cx = -0.5; cx < 1.0; cx += 1.0) {
		for (double cy = -0.5; cy < 1.0; cy += 1.0) {
			for (double cz = -0.5; cz < 1.0; cz += 1.0) {
				Eigen::Vector3d vertex = Cubes[c].x + cx * Cubes[c].R.col(0)
					+ cy * Cubes[c].R.col(1) + cz * Cubes[c].R.col(2);
				double in = groundLevel - vertex[1];
				if (in > overlap) {
					overlap = in;
					touch = vertex;
				}
			}
		}
	}
	if (overlap < 0) { return; }
	Contact temp;
	temp.a = c; temp.b = c2;
	temp.n = Eigen::Vector3d(0, 1, 0);
	temp.overlap = overlap;
	temp.p = touch;
	temp.vf = true;
	//printf("%d contact with ground\n",c);
	Contacts.push_back(temp);
	contactsMap[c].push_back(temp);
}

void contactWithWall(int c, int dirt, double wall) {
	if (abs(Cubes[c].x[dirt] - wall) > sqrt(3) / 2.0) {
		return;
	}

	int c2 = numCubes; // represent for ground
	double overlap = -DBL_MAX;
	Eigen::Vector3d touch;
	for (double cx = -0.5; cx < 1.0; cx += 1.0) {
		for (double cy = -0.5; cy < 1.0; cy += 1.0) {
			for (double cz = -0.5; cz < 1.0; cz += 1.0) {
				Eigen::Vector3d vertex = Cubes[c].x + cx * Cubes[c].R.col(0)
					+ cy * Cubes[c].R.col(1) + cz * Cubes[c].R.col(2);
				double in = 0.0;
				if (wall > 0) {
					in = vertex[dirt] - wall;
				}
				else {
					in = wall - vertex[dirt];
				}
				if (in > overlap) {
					overlap = in;
					touch = vertex;
				}
			}
		}
	}
	if (overlap < 0) { return; }
	Contact temp;
	temp.a = c; temp.b = c2;
	temp.n = Eigen::Vector3d(0, 0, 0);
	if (wall > 0) { temp.n[dirt] = -1; }
	if (wall < 0) { temp.n[dirt] = 1; }
	temp.overlap = overlap;
	temp.p = touch;
	temp.vf = true;
	//printf("%d contact with ground\n",c);
	Contacts.push_back(temp);
	contactsMap[c].push_back(temp);
}

// contact detector
void contactDetector() {
	// may first need to clear vectors
	Contacts.clear();
	for (int i = 0; i < numCubes; i++) {
		contactsMap[i].clear();
	}
	//first use a virtual bounding sphere to filter some unwise conparasion
	for (int i = 0; i < numCubes - 1; i++) {
		for (int j = i + 1; j < numCubes; j++) {
			Eigen::Vector3d centerOne = Cubes[i].x;
			Eigen::Vector3d centerTwo = Cubes[j].x;
			double distBtwCs = (centerOne - centerTwo).norm();
			if (distBtwCs <= sqrt(3)) {
				// focus on i and j cubes
				contactBtwCubes(i, j);
			}
		}
	}
	for (int i = 0; i < numCubes; i++) {
		// for each object, also need to check contact with ground
		contactWithGround(i);
		for (int dirt = 0; dirt <= 2; dirt += 2) {
			contactWithWall(i, dirt, wallLevel);
			contactWithWall(i, dirt, -wallLevel);
		}
	}
}

void processVF(Contact temp) {
	// rel velocity
	int a = temp.a;
	int b = temp.b;
	Eigen::Vector3d p = temp.p;
	Eigen::Vector3d padot = Cubes[a].v + Cubes[a].omega.cross(p - Cubes[a].x);
	Eigen::Vector3d pbdot(0,0,0);
	if (b == numCubes) {
		pbdot = Eigen::Vector3d(0, 0, 0);
	}
	else {
		pbdot = Cubes[b].v + Cubes[b].omega.cross(p - Cubes[b].x);
	}
	double vrel = temp.n.dot(padot - pbdot);
	if (vrel > -1e-2) { return; }
	//printf("is a collision\n");
	// colliding
	double epsilon = 0.7;
	double numerator = -(1 + epsilon) * vrel;
	double term1 = 1 / Cubes[a].mass;
	Eigen::Vector3d ra = p - Cubes[a].x;
	double term3 = temp.n.dot((Cubes[a].Ibodyinv * ra.cross(temp.n)).cross(ra));
	double term2 = 0.0;
	double term4 = 0.0;
	if (b != numCubes) {
		term2 = 1 / Cubes[b].mass;
		Eigen::Vector3d rb = p - Cubes[b].x;
		term4 = temp.n.dot((Cubes[b].Ibodyinv * rb.cross(temp.n)).cross(rb));
	}
	double j = numerator / (term1 + term2 + term3 + term4);
	//printf("j %f\n", j);
	Eigen::Vector3d momentum = j * temp.n;
	//printf("%d's original momentum: %f %f %f\n", a, Cubes[a].P[0], Cubes[a].P[1], Cubes[a].P[2]);
	Cubes[a].P += momentum;
	//printf("%d's current momentum: %f %f %f\n", a, Cubes[a].P[0], Cubes[a].P[1], Cubes[a].P[2]);
	Cubes[a].L += ra.cross(momentum);
	Cubes[a].v = Cubes[a].P / Cubes[a].mass;
	Cubes[a].omega = Cubes[a].Ibodyinv * Cubes[a].L;
	if (b != numCubes) {
		Eigen::Vector3d rb = p - Cubes[b].x;
		Cubes[b].P -= momentum;
		Cubes[b].L -= rb.cross(momentum);
		Cubes[b].v = Cubes[b].P / Cubes[b].mass;
		Cubes[b].omega = Cubes[b].Ibodyinv * Cubes[b].L;
	}
}

void processEE(Contact temp) {
	// rel velocity
	int a = temp.a;
	int b = temp.b;
	Eigen::Vector3d p = temp.p;
	Eigen::Vector3d padot = Cubes[a].v + Cubes[a].omega.cross(p - Cubes[a].x);
	Eigen::Vector3d pbdot = Cubes[b].v + Cubes[b].omega.cross(p - Cubes[b].x);
	double vrel = temp.n.dot(padot - pbdot);
	if (vrel > -1e-2) { return; }
	//printf("is a collision\n");
	// colliding
	double epsilon = 0.5;
	double numerator = -(1 + epsilon) * vrel;
	double term1 = 1 / Cubes[a].mass;
	Eigen::Vector3d ra = p - Cubes[a].x;
	double term3 = temp.n.dot((Cubes[a].Ibodyinv * ra.cross(temp.n)).cross(ra));
	double term2 = 0.0;
	double term4 = 0.0;
	term2 = 1 / Cubes[b].mass;
	Eigen::Vector3d rb = p - Cubes[b].x;
	term4 = temp.n.dot((Cubes[b].Ibodyinv * rb.cross(temp.n)).cross(rb));
	double j = numerator / (term1 + term2 + term3 + term4);
	//printf("j %f\n", j);
	Eigen::Vector3d momentum = j * temp.n;
	//printf("%d's original momentum: %f %f %f\n", a, Cubes[a].P[0], Cubes[a].P[1], Cubes[a].P[2]);
	Cubes[a].P += momentum;
	//printf("%d's current momentum: %f %f %f\n", a, Cubes[a].P[0], Cubes[a].P[1], Cubes[a].P[2]);
	Cubes[a].L += ra.cross(momentum);
	Cubes[a].v = Cubes[a].P / Cubes[a].mass;
	Cubes[a].omega = Cubes[a].Ibodyinv * Cubes[a].L;
	Cubes[b].P -= momentum;
	Cubes[b].L -= rb.cross(momentum);
	Cubes[b].v = Cubes[b].P / Cubes[b].mass;
	Cubes[b].omega = Cubes[b].Ibodyinv * Cubes[b].L;
}

void processContacts() {
	for (int i = 0; i < Contacts.size(); i++) {
		if (Contacts[i].vf) {
			processVF(Contacts[i]);
		}
		else {
			// edge-edge
			//printf("edge-edge case\n");
			processEE(Contacts[i]);
		}
	}
}


void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}
	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
	lastX = xpos;
	lastY = ypos;
	float sensitivity = 0.1;
	xoffset *= sensitivity;
	yoffset *= sensitivity;
	yaw += xoffset;
	pitch += yoffset;
	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;
	glm::vec3 front;//why not in global 
	front.x = cos(glm::radians(pitch)) * cos(glm::radians(yaw));
	front.y = sin(glm::radians(pitch));
	front.z = cos(glm::radians(pitch)) * sin(glm::radians(yaw));
	cameraFront = glm::normalize(front);
	//std::cout << yaw << " " << pitch << std::endl;
}

void processInput(GLFWwindow* window)
{
	/*currentFrame = glfwGetTime();
	deltaTime = currentFrame - lastFrame;
	lastFrame = currentFrame;*/
	float cameraSpeed = 2.0f * deltaTime; // adjust accordingly
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}
// no use
void initPMV()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(60, SCR_WIDTH / SCR_HEIGHT, 0.1, 1000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt
	(
		3, 3, 3,
		0, 0, 0,
		0, 1, 0
	);

}
// no use
void changePMV()
{


}
/*-----------------------------------------------------------------------*/



//This an function to get v, vt and vn. 
bool make_face(std::vector<float> v, std::vector<float> vt, std::vector<float> vn, std::vector<unsigned int> f,
	std::vector<glm::vec3>& points, std::vector<glm::vec3>& normals, std::vector<glm::vec2>& uvs)
{
	if (f.size() % 3 != 0)
		return false;
	// i trangles
	for (int i = 0; i < f.size()/3; i += 1)
	{
		int k = i * 3;
		// three vertexs of triangle
		for (int j = 0; j < 3; j++)
		{
			points.push_back(glm::vec3(v[f[k + j] * 3], v[f[k + j] * 3 + 1], v[f[k + j] * 3 + 2]));
			normals.push_back(glm::vec3(vn[f[k + j] * 3], vn[f[k + j] * 3 + 1], vn[f[k + j] * 3 + 2]));
			uvs.push_back(glm::vec2(vt[f[k + j] * 2], vt[f[k + j] * 2 + 1]));
		}
		
	}
}
// no use
void get_vec3(std::vector<float> list, std::vector<glm::vec3> &vec)
{
	int n = list.size() / 3;
	for (int i = 0; i < n; i++)
	{
		vec.push_back(glm::vec3(list[i], list[i + 1], list[i + 2]));
	}
}
// no use
void get_vec2(std::vector<float> list, std::vector<glm::vec2>& vec)
{
	int n = list.size() / 2;
	for (int i = 0; i < n; i++)
	{
		vec.push_back(glm::vec2(list[i], list[i + 1]));
	}
}



int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	
    gladLoadGL();  
    
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	glEnable(GL_DEPTH_TEST);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Here you need to fill construct function of class Shader. And you need to understand other funtions in Shader.//
	// Then, write code in shader_m.vs, shader_m.fs and shader_m.gs to finish the tasks.                             //
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Shader my_shader(
		"../src/shader_m.vs", 
		"../src/shader_m.fs"
	);
	//A shader for light visiable source
	Shader lampShader("../src/lamp.vs", "../src/lamp.fs");
	


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// tiny::LoadObj is a function to load obj file. The output is shape_t and material_t.                         //
	// "shape.mesh" is a mesh struct. "mesh.positions", "mesh.normals", "mesh.texcoords" corresponds to v,vn,vt.   //
	// For example:                                                                                                //
	// positions[0],positions[1],positions[2] -> v 0,0,1                                                           //
	// positions[3],positions[4],positions[5] -> v 0,1,0                                                           //
	// "mesh.indice" corresponds to f, but it is different from f. Each element is an index for all of v,vn,vt.    //
	// positions[0],positions[1],positions[2] -> v 0,0,1  positions[0],positions[1],positions[2] -> v 0,0,1        //
	// You can read tiny_obj_loader.h to get more specific information.                                            //
	//                                                                                                             //
	// I have write make_face for you.  It will return v, vt, vn in vec form (each element if for one point).      //
	// These informations can help you to do normal mapping.  (You can calculate tangent here)                     //
	// Since i did not assign uv for noraml map, you just need use vt as uv for normal map, but you will find it is//
	//  ugly. So please render a box to show a nice normal mapping. (Do normal mapping on obj and box)             //
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//// how many shapes
	//std::vector<tinyobj::shape_t> shapes;
	//// no use
	//std::vector<tinyobj::material_t> materials;
	//std::string if_load_succeed = tinyobj::LoadObj(shapes, materials,
	//	"../../model/pikaqiu.obj"
	//);
	////printf("%s\n", if_load_succeed);
	//// three list for infomation of each shape
	//std::vector<unsigned int> obj_VBO_l, obj_VAO_l, obj_NUM_l;
	//// temp
	//unsigned int tVBO, tVAO;
	//// calculation helper
	//glm::vec3 edge1, edge2, tangent, bitangent;
	//glm::vec2 deltaUV1, deltaUV2;
	//float f;
	//// fixed data for exception
	//glm::vec3 bakTangent(1, 0, 0);
	//glm::vec3 bakBitangent(0, 1, 0);
	//glm::vec3 lastT, lastB;
	//for (int i = 0; i < shapes.size(); i++)
	//{
	//	
	//	std::vector < glm::vec3 > out_vertices;
	//	std::vector < glm::vec2 > out_uvs;
	//	std::vector < glm::vec3 > out_normals;

	//	// out_vertices, out_uvs, out_normals will get v, vt and vn.
	//	make_face(shapes[i].mesh.positions, shapes[i].mesh.texcoords, shapes[i].mesh.normals, shapes[i].mesh.indices,
	//		out_vertices, out_normals, out_uvs);
	//	unsigned int tVBO, tVAO;
	//	// temp tVertices able to change if i > 1? Yes
	//	std::vector<float> tVertices;
	//	// all vertices of one shape
	//	for (int j = 0; j < out_vertices.size(); j++) {
	//		// pos
	//		tVertices.push_back(out_vertices[j].x); tVertices.push_back(out_vertices[j].y); tVertices.push_back(out_vertices[j].z);
	//		// normal
	//		tVertices.push_back(out_normals[j].x); tVertices.push_back(out_normals[j].y); tVertices.push_back(out_normals[j].z);
	//		// uvs
	//		tVertices.push_back(out_uvs[j].x); tVertices.push_back(out_uvs[j].y);
	//		// T B
	//		// one triangle one calculation
	//		if (j % 3 == 0) {
	//			edge1 = out_vertices[j + 1] - out_vertices[j];
	//			edge2 = out_vertices[j + 2] - out_vertices[j];
	//			deltaUV1 = out_uvs[j + 1] - out_uvs[j];
	//			deltaUV2 = out_uvs[j + 2] - out_uvs[j];
	//			// exception
	//			if (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y == 0) {
	//				// T
	//				tVertices.push_back(bakTangent.x); tVertices.push_back(bakTangent.y); tVertices.push_back(bakTangent.z);
	//				// B
	//				tVertices.push_back(bakBitangent.x); tVertices.push_back(bakBitangent.y); tVertices.push_back(bakBitangent.z);
	//				// record
	//				lastT = bakTangent; lastB = bakBitangent;
	//			}
	//			// do calculation 
	//			else {
	//				f = 1 / (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y);
	//				tangent.x = f * (deltaUV2.y * edge1.x - deltaUV1.y * edge2.x);
	//				tangent.y = f * (deltaUV2.y * edge1.y - deltaUV1.y * edge2.y);
	//				tangent.z = f * (deltaUV2.y * edge1.z - deltaUV1.y * edge2.z);
	//				tangent = glm::normalize(tangent);
	//				bitangent.x = f * (-deltaUV2.x * edge1.x + deltaUV1.x * edge2.x);
	//				bitangent.y = f * (-deltaUV2.x * edge1.y + deltaUV1.x * edge2.y);
	//				bitangent.z = f * (-deltaUV2.x * edge1.z + deltaUV1.x * edge2.z);
	//				bitangent = glm::normalize(bitangent);
	//				// T
	//				tVertices.push_back(tangent.x); tVertices.push_back(tangent.y); tVertices.push_back(tangent.z);
	//				// B
	//				tVertices.push_back(bitangent.x); tVertices.push_back(bitangent.y); tVertices.push_back(bitangent.z);
	//				lastT = tangent; lastB = bitangent; // record
	//			}
	//		}
	//		// the other two vertices
	//		else {
	//			tVertices.push_back(lastT.x); tVertices.push_back(lastT.y); tVertices.push_back(lastT.z);
	//			tVertices.push_back(lastB.x); tVertices.push_back(lastB.y); tVertices.push_back(lastB.z);
	//		}
	//	}
	//	// set attributes for tVAO tVBO
	//	glGenVertexArrays(1, &tVAO);
	//	glGenBuffers(1, &tVBO);
	//	glBindVertexArray(tVAO);
	//	glBindBuffer(GL_ARRAY_BUFFER, tVBO);
	//	glBufferData(GL_ARRAY_BUFFER, tVertices.size()*sizeof(float), &tVertices[0], GL_STATIC_DRAW);
	//	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)0);
	//	glEnableVertexAttribArray(0); // pos
	//	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(3 * sizeof(float)));
	//	glEnableVertexAttribArray(1); // normal
	//	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(6 * sizeof(float)));
	//	glEnableVertexAttribArray(2); // uvs
	//	glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(8 * sizeof(float)));
	//	glEnableVertexAttribArray(3); // T
	//	glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(11 * sizeof(float)));
	//	glEnableVertexAttribArray(4); // B
	//	// push to VAO,VBO,NUM list
	//	obj_VBO_l.push_back(tVBO); obj_VAO_l.push_back(tVAO); obj_NUM_l.push_back(out_vertices.size());
	//}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Render a box to show nice normal mapping.                                                                   //
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	float vertices_cube_0[] = {

		// positions          // normals           // texture coords

		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,
		 0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  0.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,

		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,
		 0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  0.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  1.0f,
		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,

		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
		-0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		-0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

		 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
		 0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
		 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		 0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,
		 0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  1.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  0.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,

		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  1.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f

	};
	// operation on cube vertices
	std::vector<glm::vec3> cube_vertices, cube_normals;
	std::vector<glm::vec2> cube_uvs;
	std::vector<float> vertices_cube_1;
	// calculation helper
	glm::vec3 edge1, edge2, tangent, bitangent;
	glm::vec2 deltaUV1, deltaUV2;
	float f;
	// fixed data for exception
	glm::vec3 bakTangent(1, 0, 0);
	glm::vec3 bakBitangent(0, 1, 0);
	glm::vec3 lastT, lastB;
	// reload cube data
	for (int j = 0; j < 36; j++) {
		cube_vertices.push_back(glm::vec3(vertices_cube_0[8 * j], vertices_cube_0[8 * j + 1], vertices_cube_0[8 * j + 2]));
		cube_normals.push_back(glm::vec3(vertices_cube_0[8 * j + 3], vertices_cube_0[8 * j + 4], vertices_cube_0[8 * j + 5]));
		cube_uvs.push_back(glm::vec2(vertices_cube_0[8 * j + 6], vertices_cube_0[8 * j + 7]));
	}
	// calculate TB, 36 vertices
	for (int j = 0; j < 36; j++) {
		vertices_cube_1.push_back(cube_vertices[j].x); vertices_cube_1.push_back(cube_vertices[j].y); vertices_cube_1.push_back(cube_vertices[j].z);
		vertices_cube_1.push_back(cube_normals[j].x); vertices_cube_1.push_back(cube_normals[j].y); vertices_cube_1.push_back(cube_normals[j].z);
		vertices_cube_1.push_back(cube_uvs[j].x); vertices_cube_1.push_back(cube_uvs[j].y);
		// each triangle
		if (j % 3 == 0) {
			edge1 = cube_vertices[j + 1] - cube_vertices[j];
			edge2 = cube_vertices[j + 2] - cube_vertices[j];
			deltaUV1 = cube_uvs[j + 1] - cube_uvs[j];
			deltaUV2 = cube_uvs[j + 2] - cube_uvs[j];
			if (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y == 0) {
				vertices_cube_1.push_back(bakTangent.x); vertices_cube_1.push_back(bakTangent.y); vertices_cube_1.push_back(bakTangent.z);
				vertices_cube_1.push_back(bakBitangent.x); vertices_cube_1.push_back(bakBitangent.y); vertices_cube_1.push_back(bakBitangent.z);
				lastT = bakTangent; lastB = bakBitangent;
			}
			else {
				f = 1 / (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y);
				tangent.x = f * (deltaUV2.y * edge1.x - deltaUV1.y * edge2.x);
				tangent.y = f * (deltaUV2.y * edge1.y - deltaUV1.y * edge2.y);
				tangent.z = f * (deltaUV2.y * edge1.z - deltaUV1.y * edge2.z);
				tangent = glm::normalize(tangent);
				bitangent.x = f * (-deltaUV2.x * edge1.x + deltaUV1.x * edge2.x);
				bitangent.y = f * (-deltaUV2.x * edge1.y + deltaUV1.x * edge2.y);
				bitangent.z = f * (-deltaUV2.x * edge1.z + deltaUV1.x * edge2.z);
				bitangent = glm::normalize(bitangent);
				vertices_cube_1.push_back(tangent.x); vertices_cube_1.push_back(tangent.y); vertices_cube_1.push_back(tangent.z);
				vertices_cube_1.push_back(bitangent.x); vertices_cube_1.push_back(bitangent.y); vertices_cube_1.push_back(bitangent.z);
				lastT = tangent; lastB = bitangent;
			}
		}
		else {
			vertices_cube_1.push_back(lastT.x); vertices_cube_1.push_back(lastT.y); vertices_cube_1.push_back(lastT.z);
			vertices_cube_1.push_back(lastB.x); vertices_cube_1.push_back(lastB.y); vertices_cube_1.push_back(lastB.z);
		}
	}

	unsigned int cubeVBO, cubeVAO;
	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &cubeVBO);
	// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
	glBindVertexArray(cubeVAO);

	glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices_cube_1.size() * sizeof(float), &vertices_cube_1[0], GL_STATIC_DRAW);

	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	// texture coords
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(8 * sizeof(float)));
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(11 * sizeof(float)));
	glEnableVertexAttribArray(4);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// You need to fill this function which is defined in my_texture.h. The parameter is the path of your image.   //
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// cat face
	/*unsigned int texture1 = loadTexture("../../model/2.jpg");
	unsigned int texturePika = loadTexture("../../model/p_r.jpg");
	unsigned int textureNormal = loadTexture("../../model/normal_map.jpg");
	unsigned int textureEye = loadTexture("../../model/eye.jpg");*/
	unsigned int texture1 = loadTexture("../model/2.jpg");

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Here we defined pointlights in shader and passed some parameter for you. You can take this as an example.   //
	// Or you can change it if you like.                                                                           //
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	glm::vec3 pointLightPositions[] = {
		glm::vec3(9.0f,  6.0f,  9.0f),
		glm::vec3(6.0f, 6.0f, -6.0f),
		glm::vec3(-6.0f,  6.0f, 6.0f),
		glm::vec3(-6.0f,  6.0f, -6.0f)
	};
	my_shader.use();
	my_shader.setVec3("dirLight.direction", glm::vec3(1.01f, 1.01f, 1.01f));
	my_shader.setVec3("dirLightDir2VS", glm::vec3(1.01f, 1.01f, 1.01f));
	my_shader.setVec3("dirLight.ambient", glm::vec3(0.01f, 0.01f, 0.02f));
	my_shader.setVec3("dirLight.diffuse", glm::vec3(1.0f, 1.0f, 1.0f));
	my_shader.setVec3("dirLight.specular", glm::vec3(1.0f, 1.0f, 1.0f));
	// point light 1
	my_shader.setVec3("pointLights[0].position", pointLightPositions[0]);
	my_shader.setVec3("lightPos[0]", pointLightPositions[0]);
	my_shader.setVec3("pointLights[0].ambient", 0.05f, 0.05f, 0.05f);
	my_shader.setVec3("pointLights[0].diffuse", 0.8f, 0.8f, 0.8f);
	my_shader.setVec3("pointLights[0].specular", 1.0f, 1.0f, 1.0f);
	my_shader.setFloat("pointLights[0].constant", 1.0f);
	my_shader.setFloat("pointLights[0].linear", 0.09);
	my_shader.setFloat("pointLights[0].quadratic", 0.032);
	// point light 2
	my_shader.setVec3("pointLights[1].position", pointLightPositions[1]);
	my_shader.setVec3("lightPos[1]", pointLightPositions[1]);
	my_shader.setVec3("pointLights[1].ambient", 0.05f, 0.05f, 0.05f);
	my_shader.setVec3("pointLights[1].diffuse", 0.8f, 0.8f, 0.8f);
	my_shader.setVec3("pointLights[1].specular", 1.0f, 1.0f, 1.0f);
	my_shader.setFloat("pointLights[1].constant", 1.0f);
	my_shader.setFloat("pointLights[1].linear", 0.09);
	my_shader.setFloat("pointLights[1].quadratic", 0.032);
	// point light 3
	my_shader.setVec3("pointLights[2].position", pointLightPositions[2]);
	my_shader.setVec3("lightPos[2]", pointLightPositions[2]);
	my_shader.setVec3("pointLights[2].ambient", 0.05f, 0.05f, 0.05f);
	my_shader.setVec3("pointLights[2].diffuse", 0.6f, 0.1f, 0.8f);
	my_shader.setVec3("pointLights[2].specular", 1.0f, 1.0f, 1.0f);
	my_shader.setFloat("pointLights[2].constant", 1.0f);
	my_shader.setFloat("pointLights[2].linear", 0.09);
	my_shader.setFloat("pointLights[2].quadratic", 0.032);
	// point light 4
	my_shader.setVec3("pointLights[3].position", pointLightPositions[3]);
	my_shader.setVec3("lightPos[3]", pointLightPositions[3]);
	my_shader.setVec3("pointLights[3].ambient", 0.05f, 0.05f, 0.05f);
	my_shader.setVec3("pointLights[3].diffuse", 0.1f, 1.1f, 0.8f);
	my_shader.setVec3("pointLights[3].specular", 1.0f, 1.0f, 1.0f);
	my_shader.setFloat("pointLights[3].constant", 1.0f);
	my_shader.setFloat("pointLights[3].linear", 0.09);
	my_shader.setFloat("pointLights[3].quadratic", 0.032);
	// normal map switch
	my_shader.setBool("useNormalMap", false);

	// add cubes
	initStates();
	// two simulation arrays
	double* x0 = new double[numCubes * STATE_SIZE];
	double* xFinal = new double[numCubes * STATE_SIZE];
	BodiesToArray(xFinal);
	// time paras
	clock_t ltime = 0;
	clock_t ctime = 0;
	double duration = 0;
	double T0 = 0.0;

    while (!glfwWindowShouldClose(window))
    {
		ctime = clock();
		duration =((double)ctime - (double)ltime) / CLK_TCK;
		if (duration >= 1.0 / 24.0) {
			ltime = ctime;
			// copy xFinal back to x0
			for (int i = 0; i < STATE_SIZE * numCubes; i++) {
				x0[i] = xFinal[i];
			}
			// get state at xFinal
			ode(x0, xFinal, STATE_SIZE * numCubes, T0, T0 + 1.0 / 24.0, Dxdt);
			ArrayToBodies(xFinal);
			// orthognolize R here, otherwise may deform when rotate
			othognolizeR();
			contactDetector();
			processContacts();
			BodiesToArray(xFinal);
			T0 += 1.0 / 24.0;
		}
        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);
		//Update Camera Matrix
		glFlush();
		glEnable(GL_MULTISAMPLE);
		glEnable(GL_LIGHTING);
		glEnable(GL_COLOR_MATERIAL);
		glLightModeli(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		glm::mat4 projection = glm::perspective(0.785f, (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		glm::mat4 model = glm::mat4(1.0f); // not used
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//  Render the lamp cubes                                                                                      //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		lampShader.use();
		lampShader.setMat4("projection", projection);
		lampShader.setMat4("view", view);
		glBindVertexArray(cubeVAO);
		for (unsigned int i = 0; i < 4; i++)
		{
			model = glm::mat4(1.0f); // re-model
			model = glm::translate(model, pointLightPositions[i]);
			model = glm::scale(model, glm::vec3(0.2f)); // Make it a smaller cube
			lampShader.setMat4("model", model);
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//  Render the normal cubes                                                                                    //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		my_shader.use();
		my_shader.setInt("mat.diffuse", 0);
		my_shader.setInt("mat.specular", 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texture1);
		my_shader.setMat4("projection", projection);
		my_shader.setMat4("view", view);
		for (int i = 0; i < numCubes; i++) {
			model = glm::mat4(1.0f);
			model = glm::translate(model, glm::vec3(Cubes[i].x[0], Cubes[i].x[1], Cubes[i].x[2]));
			Eigen::Matrix3d Ro = Cubes[i].R;
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					// in glm, columns are stored as rows
					model[j][k] = Ro(k, j);
				}
			}
			/*if (i == 4) {
				for (int j = 0; j < 4; j++) {
					printf("%f %f %f %f\n", model[j][0], model[j][1], model[j][2], model[j][3]);
				}
			}*/
			my_shader.setMat4("model", model);
			my_shader.setVec3("viewPos", cameraPos);
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}
		
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//  Render an object using texture and normal map.                                                             //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//// bind Texture
		//// mat
		//my_shader.use();
		//// set sampler2D
		//my_shader.setInt("mat.diffuse", 0);
		//my_shader.setInt("mat.specular", 0);
		//my_shader.setInt("normalMap", 2);
		//// set texture unit ID
		//glActiveTexture(GL_TEXTURE0);
		//glBindTexture(GL_TEXTURE_2D, texture1);
		//glActiveTexture(GL_TEXTURE2);
		//glBindTexture(GL_TEXTURE_2D, textureNormal);

		//my_shader.setMat4("projection", projection);
		//my_shader.setMat4("view", view);
		//model = glm::mat4(1.0f);
		//my_shader.setMat4("model", model);
		//my_shader.setVec3("viewPos", cameraPos);
		//glDrawArrays(GL_TRIANGLES, 0, 36);


		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//  Render the object in .obj file. You need to set materials and wrap texture for objects.                    //
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//my_shader.use();
		//my_shader.setInt("mat.diffuse", 1);
		//my_shader.setInt("mat.specular", 1);
		//my_shader.setInt("normalMap", 2);
		//glActiveTexture(GL_TEXTURE1);
		//glBindTexture(GL_TEXTURE_2D, texturePika);
		//glActiveTexture(GL_TEXTURE2);
		//glBindTexture(GL_TEXTURE_2D, textureNormal);
		//glActiveTexture(GL_TEXTURE3);
		//glBindTexture(GL_TEXTURE_2D, textureEye);

		//my_shader.setMat4("projection", projection);
		//my_shader.setMat4("view", view);
		//model = glm::mat4(1.0f);
		//my_shader.setMat4("model", model);
		//my_shader.setVec3("viewPos", cameraPos);
		//my_shader.setVec3("viewPos2VS", cameraPos);
		//for (int i = 0; i < obj_VAO_l.size(); i++) {
		//	// body
		//	if (obj_VAO_l[i] == 3) {
		//		my_shader.setInt("mat.diffuse", 1);
		//		my_shader.setInt("mat.specular", 1);
		//	}
		//	// eye
		//	else {
		//		my_shader.setInt("mat.diffuse", 3);
		//		my_shader.setInt("mat.specular", 3);
		//	}
		//	glBindVertexArray(obj_VAO_l[i]);
		//	//printf("%d  %d\n", obj_VAO_l[i], obj_NUM_l[i]);
		//	glDrawArrays(GL_TRIANGLES, 0, obj_NUM_l[i]);
		//}

		
		/////////////////////////////////////////////////////////////////////
		
		/////////////////////////////end/////////////////////////////////////



        glfwSwapBuffers(window);
        glfwPollEvents();
    }
	free(x0); free(xFinal);
	glDeleteVertexArrays(1, &cubeVAO);
	glDeleteBuffers(1, &cubeVBO);
	/*for (int i = 0; i < obj_VAO_l.size(); i++) {
		glDeleteVertexArrays(1, &obj_VAO_l[i]);
		glDeleteBuffers(1, &obj_VBO_l[i]);
	}*/
    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

