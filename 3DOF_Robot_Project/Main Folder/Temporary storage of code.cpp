//Temporarily storing code
#include <cmath>
#include <cstdio>
#include <cstring>

#include <iostream>
#include <fstream>
#include <strstream>

#include <windows.h>			// for keyboard input

#include "3D_graphics.h"
#include "my_project.h"

using namespace std;

//Global non-changing variables
const double PI = atan(1) * 4;
const double g = 9.81;
double m[3 + 1] = { -1.0, 100.0, 100.0, 100.0 };		//Mass of arms
double l[3 + 1] = { -1.0, 16.5, 17.5, 17.5 };			//Length of arms

void sim_step(double dt, double &yaw, double &pitch2, double &pitch3)
{
	int i;
	const int N = 6;									// number of state variables (order of the system)
	static double t;									// current time (seconds)
	static double xtemp[N + 1];							// state vector x[N](t)	Temporarily store and output to x[6+1]
	double xd[N + 1];										// derivative vector at time t
	const int M = 1;									// number of inputs
	static double u[M + 1];								// input vector
	static int init = 0;

	// initialization section
	if (!init) {
		t = 0.0;										// initial time

		xtemp[0] = -1.0;								// not used
		xtemp[1] = 0.0;									// initial theta1 (yaw)
		xtemp[2] = -PI / 6;								// initial theta2 (pitch2)
		xtemp[3] = xtemp[2];								// initial theta3 (pitch3)
		xtemp[4] = 0.0;									// initial vheta1
		xtemp[5] = 0.0;									// initial vheta2
		xtemp[6] = 0.0;									// initial vheta3

		init = 1;
	}

	calculate_inputs(xtemp, t, N, u, M);					// calculate u
	calculate_Xd(xtemp, t, N, u, M, xd);						// calculate x-derivatives

	for (i = 1; i <= N; i++) xtemp[i] = xtemp[i] + xd[i] * dt;	// Euler step

	t = t + dt;											// increment time

	// output to draw_3D_graphics
	yaw = xtemp[1];								// Rotates Base Position
	pitch2 = xtemp[2];								// Pitch First Arm Position
	pitch3 = xtemp[3];								// Pitch Second Arm Position
}

void calculate_inputs(const double X[], double t, int N, double U[], int M)
{
	double F[3 + 1];									// force matrix
	// (mass and length in global)

	// unpack
	double theta1 = X[1];
	double theta2 = X[2];
	double theta3 = X[3];
	double vheta1 = X[4];
	double vheta2 = X[5];
	double vheta3 = X[6];

	// these forces stabilize the arms
	//	F[1] = 0;
	//	F[2] = m[2] * g * l[2] / 2 * cos(theta2) + m[3] * g * (l[2] * cos(theta2) + l[3] / 2 * cos(theta2 + theta3));
	//	F[3] = m[3] * g * l[3] / 2 * cos(theta2 + theta3);

	F[1] = 0;
	F[2] = 0;
	F[3] = 0;

	// update/pack input u[] matrix
	for (int i = 1; i <= 3; i++) U[i] = F[i];
}

void calculate_Xd(const double X[], double t, int N, const double U[], int M, double Xd[])
{
	double Ma[3 + 1][3 + 1];							// inertia matrix
	double G[3 + 1];									// gravity matrix
	double C[3 + 1];									// corriolis effect matrix
	double F[3 + 1];

	double x[3 + 1], v[3 + 1];							// state variables: x[1] = yaw, x[2] = pitch2 x[3] = pitch3
	double dx[3 + 1], dv[3 + 1];						// angular derivatives

	// unpack state variables & inputs
	for (int i = 1; i <= 3; i++) x[i] = X[i];
	for (int i = 1; i <= 3; i++) v[i] = X[i + 3];
	for (int i = 1; i <= 3; i++) F[i] = U[i];

	// defining the parameters
	double D = (l[2] + l[3]) * sin((PI - abs(x[3])) / 2);	//Distance from end of arm 1 to tip of arm 3
	double R = D * cos(x[2] + x[3] / 2);					//Distance of D in x/y plane
	double det = 0.0;
	double Minv[3 + 1][3 + 1] = { 0.0 };

	// dynamic model (all of the physics)
	ComputeMassMatrix(m, x, R, l, Ma);
	ComputeGravityMatrix(m, x, l, G);
	ComputeCoriolisMatrix(m, x, v, l, C);
	ComputeDeterminant(Ma, det);
	ComputeInvertedMatrix(Ma, det, Minv);

	// velocity kinematic
	for (int i = 1; i <= 3; i++) dx[i] = v[i];

	dv[1] = Minv[1][1] * (F[1] - C[1] - G[1]) + Minv[1][2] * (F[2] - C[2] - G[2]) + Minv[1][3] * (F[3] - C[3] - G[3]);
	dv[2] = Minv[1][2] * (F[1] - C[1] - G[1]) + Minv[2][2] * (F[2] - C[2] - G[2]) + Minv[2][3] * (F[3] - C[3] - G[3]);
	dv[3] = Minv[1][3] * (F[1] - C[1] - G[1]) + Minv[2][3] * (F[2] - C[2] - G[2]) + Minv[3][3] * (F[3] - C[3] - G[3]);
	/*
	dv[1] = Minv[1][1] * (F[1] + C[1] - G[1]) + Minv[1][2] * (F[2] - C[2] - G[2]) + Minv[1][3] * (F[3] - C[3] - G[3]);
	dv[2] = Minv[2][1] * (F[1] + C[1] - G[1]) + Minv[2][2] * (F[2] - C[2] - G[2]) + Minv[2][3] * (F[3] - C[3] - G[3]);
	dv[3] = Minv[3][1] * (F[1] + C[1] - G[1]) + Minv[3][2] * (F[2] - C[2] - G[2]) + Minv[3][3] * (F[3] - C[3] - G[3]);
	*/
	// update/pack xd[]
	for (int i = 1; i <= 3; i++) Xd[i] = dx[i];
	for (int i = 1; i <= 3; i++) Xd[i + 3] = dv[i];

	//TO-DO: Figure out why the physics doesn't work

}

void ComputeMassMatrix(double m[3 + 1], double x[3 + 1], double R, double l[3 + 1], double Ma[3 + 1][3 + 1])
{
	Ma[1][1] = (m[1] * R * R) / 2 + (m[2] * l[2] * l[2] * cos(x[2]) * cos(x[2])) / 3 + (m[3] * l[3] * l[3] * cos(x[2] - x[3]) * cos(x[2] - x[3])) / 3 + (m[3] * l[2] * l[2] * cos(x[2]) * cos(x[2])) + (m[3] * l[2] * l[3] * cos(x[2] - x[3])*cos(x[2]));
	Ma[2][2] = (m[2] * l[2] * l[2]) / 3 + (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3]));
	Ma[2][3] = (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3])) / 3;
	Ma[3][3] = (m[3] * l[3] * l[3]) / 3;
	Ma[1][2] = Ma[1][3] = Ma[2][1] = Ma[3][1] = 0.0;
	Ma[3][2] = Ma[2][3];
	/*
	Ma[1][1] = (m[1] * R * R) / 2 + (m[2] * l[2] * l[2] * cos(x[2]) * cos(x[2])) / 3 + (m[3] * l[3] * l[3] * cos(x[2] + x[3]) * cos(x[2] + x[3])) / 3 + (m[3] * l[2] * l[2] * cos(x[2]) * cos(x[2])) + (m[3] * l[2] * l[3] * cos(x[2] + x[3])*cos(x[2]));
	Ma[2][2] = (m[2] * l[2] * l[2]) / 3 + (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3]));
	Ma[2][3] = (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3])) / 3;
	Ma[3][3] = (m[3] * l[3] * l[3]) / 3;
	Ma[1][2] = Ma[1][3] = Ma[2][1] = Ma[3][1] = 0.0;
	Ma[3][2] = Ma[2][3];
	*/
}

void ComputeGravityMatrix(double m[3 + 1], double x[3 + 1], double l[3 + 1], double G[3 + 1])
{
	/*
	G[1] = 0;
	G[2] = ((m[2] * g * l[2] * cos(x[2])) / 2 + (m[3] * g * l[3] * cos(x[2] + x[3])) / 2 + (m[3] * g * l[2] * cos(x[2])));
	G[3] = (m[3] * g * l[3] * cos(x[2] + x[3])) / 2;		// Should this be negative?
	*/
	G[1] = 0;
	G[2] = (m[2] * g * l[2] * cos(x[2])) / 2 + (m[3] * g * l[3] * cos(x[2] + x[3])) / 2 + (m[3] * g * l[2] * cos(x[2]));
	G[3] = (m[3] * g * l[3] * cos(x[2] + x[3])) / 2;		// Should this be negative?
}

void ComputeCoriolisMatrix(double m[3 + 1], double x[3 + 1], double v[3 + 1], double l[3 + 1], double C[3 + 1])
{
	/*
	C[1] = (((-4 / 3)*(m[2] * l[2] * l[2] * sin(2 * abs(x[2]))) - (m[3] * l[3] * l[3] * sin(2 * (abs(x[2]) + abs(x[3])))) / 3 - (m[3] * l[2] * l[3] * sin(2 * abs(x[2]) + abs(x[3])))) * v[2] * v[1] + ((-1 / 3)*(m[3] * l[3] * l[3] * sin(2 * (abs(x[2]) + abs(x[3])))) - (m[3] * l[2] * l[3] * cos(x[2])*sin(abs(x[2]) + abs(x[3])))) * v[3] * v[1]);
	C[2] = (((-1)*(m[3] * l[2] * l[3] * sin(abs(x[3]))) * v[2] * v[3] + (-1 / 2)*(m[3] * l[2] * l[3] * sin(abs(x[3]))) * v[3] * v[3] + ((1 / 6)*(m[2] * l[2] * l[2] * sin(2 * abs(x[2]))) + (1 / 6)*(m[3] * l[3] * l[3] * sin(2 * (abs(x[2]) + abs(x[3])))) + (1 / 2)*(m[3] * l[2] * l[2] * sin(2 * abs(x[2]))) + (1 / 2)*(m[3] * l[2] * l[3] * sin(2 * abs(x[2]) + abs(x[3]))))*v[1] * v[1]));
	C[3] = (((1 / 2)*(m[3] * l[2] * l[3] * sin(abs(x[3])))) * v[2] * v[2] + ((1 / 6)*(m[3] * l[3] * l[3] * sin(2 * (abs(x[2]) + abs(x[3])))) + (1 / 2)*(m[3] * l[2] * l[3] * cos(x[2])*sin(abs(x[2]) + abs(x[3]))))*v[1] * v[1]);
	*/
	C[1] = ((-4 / 3)*(m[2] * l[2] * l[2] * sin(2 * x[2])) - (m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) / 3 - (m[3] * l[2] * l[3] * sin(2 * x[2] + x[3]))) * v[2] * v[1] + ((-1 / 3)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) - (m[3] * l[2] * l[3] * cos(x[2])*sin(x[2] + x[3]))) * v[3] * v[1];
	C[2] = ((-1)*(m[3] * l[2] * l[3] * sin(x[3])) * v[2] * v[3] + (-1 / 2)*(m[3] * l[2] * l[3] * sin(x[3])) * v[3] * v[3] + ((1 / 6)*(m[2] * l[2] * l[2] * sin(2 * x[2])) + (1 / 6)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) + (1 / 2)*(m[3] * l[2] * l[2] * sin(2 * x[2])) + (1 / 2)*(m[3] * l[2] * l[3] * sin(2 * x[2] + x[3])))*v[1] * v[1]);
	C[3] = ((1 / 2)*(m[3] * l[2] * l[3] * sin(x[3]))) * v[2] * v[2] + ((1 / 6)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) + (1 / 2)*(m[3] * l[2] * l[3] * cos(x[2])*sin(x[2] + x[3])))*v[1] * v[1];

}

void ComputeDeterminant(double Ma[3 + 1][3 + 1], double & det)
{
	det = Ma[1][1] * (Ma[2][2] * Ma[3][3] - Ma[3][2] * Ma[2][3]) - Ma[1][2] * (Ma[2][1] * Ma[3][3] - Ma[3][1] * Ma[2][3]) + Ma[1][3] * (Ma[2][1] * Ma[3][2] - Ma[3][1] * Ma[2][2]);
	/*	for (int i = 1; i <= 3; i++)
	{
	if (i == 1) det += Ma[1][1] * (Ma[2][2] * Ma[3][3] - Ma[3][2] * Ma[2][3]);
	if (i == 2) det -= Ma[1][2] * (Ma[2][1] * Ma[3][3] - Ma[3][1] * Ma[2][3]);
	if (i == 3) det += Ma[1][3] * (Ma[2][1] * Ma[3][2] - Ma[3][1] * Ma[2][2]);
	}
	*/
}

void ComputeInvertedMatrix(double Ma[3 + 1][3 + 1], double det, double Minv[3 + 1][3 + 1])
{
	double temp;
	Minv[1][1] = (Ma[2][2] * Ma[3][3] - Ma[3][2] * Ma[2][3]);
	Minv[1][2] = -(Ma[1][3] * Ma[3][2] - Ma[1][2] * Ma[3][3]);
	Minv[1][3] = (Ma[1][2] * Ma[2][3] - Ma[1][3] * Ma[2][2]);
	Minv[2][1] = -(Ma[2][3] * Ma[3][1] - Ma[2][1] * Ma[3][3]);
	Minv[2][2] = (Ma[1][1] * Ma[3][3] - Ma[1][3] * Ma[3][1]);
	Minv[2][3] = -(Ma[2][1] * Ma[1][3] - Ma[1][1] * Ma[2][3]);
	Minv[3][1] = (Ma[2][1] * Ma[3][2] - Ma[3][1] * Ma[2][2]);
	Minv[3][2] = -(Ma[3][1] * Ma[1][2] - Ma[1][1] * Ma[3][2]);
	Minv[3][3] = (Ma[1][1] * Ma[2][2] - Ma[2][1] * Ma[1][2]);

	temp = Minv[1][2];
	Minv[1][2] = Minv[2][1];
	Minv[2][1] = temp;

	temp = Minv[1][3];
	Minv[1][3] = Minv[3][1];
	Minv[3][1] = temp;

	temp = Minv[2][3];
	Minv[2][3] = Minv[3][2];
	Minv[3][2] = temp;

	Minv[1][1] /= det;
	Minv[1][2] /= det;
	Minv[1][3] /= det;
	Minv[2][1] /= det;
	Minv[2][2] /= det;
	Minv[2][3] /= det;
	Minv[3][1] /= det;
	Minv[3][2] /= det;
	Minv[3][3] /= det;
}

void stabilize(double X[], double m[], double l[], double theta1, double theta2, double theta3, double U[])
{
	U[1] = 0.0;
	U[2] = m[2] * g * l[2] / 2 * cos(theta2) + m[3] * g * (l[2] * cos(theta2) + l[3] / 2 * cos(theta2 + theta3));
	U[3] = m[3] * g * l[3] / 2 * cos(theta2 + theta3);
	X[4] = 0.0;
	X[5] = 0.0;
	X[6] = 0.0;
}

void locateObject(double objThetas[3 + 1], double x, double y, double z)
{
	//Inverse Kinematics
	double r;
	double D;
	objThetas[1] = atan2(y, x);
	r = sqrt(x*x + y*y);
	D = sqrt((z - l[1])*(z - l[1]) + r * r);
	objThetas[3] = acos((D * D - l[3] * l[3] - l[2] * l[2]) / (2 * l[2] * l[3]));
	//	objThetas[3] =	acos((((z - l[1])*(z - l[1]) + x*x + y*y - l[3] * l[3] - l[2] * l[2])) / (2 * l[2] * l[3]));
	objThetas[2] = atan2(r, z - l[1]) - atan2(l[2] + l[3] * cos(objThetas[3]), l[3] * sin(objThetas[3]));

	//TO-DO: Figure out why it doesn't point exactly to the right spot (maybe a sign problem?)
}

void pointAtObject(double dt, double objTheta[3 + 1], double & yaw, double & pitch2, double & pitch3)
{
	if (yaw != objTheta[1])		yaw += (objTheta[1] - yaw)*dt;
	if (pitch2 != objTheta[2])	pitch2 += (objTheta[2] - pitch2)*dt;
	if (pitch3 != objTheta[3])	pitch3 += (objTheta[3] - pitch3)*dt;

	if (abs(objTheta[1] - yaw) < 0.01 && abs(objTheta[2] - pitch2) < 0.01 && abs(objTheta[3] - pitch3) < 0.01)
	{
		yaw = objTheta[1];
		pitch2 = objTheta[2];
		pitch3 = objTheta[3];
	}

	//TO-DO:	Make end effector point directly at object 
	//			if we rotate the arm a lot in one direction
	//			it'll undo the rotation before pointing at the object
}