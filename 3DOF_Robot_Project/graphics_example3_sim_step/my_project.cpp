
// example of Euler's algorithm for a 2nd order system

#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0

#include <conio.h> // console I/O functions such as getch()

#include "my_project.h"

using namespace std;

double PI = atan(1) * 4;
double g = 9.81;

void sim_step(double dt, double &yaw, double &pitch2, double &pitch3)
{
	int i;
	const int N = 6; // number of state variables (order of the system)
	static double t; // current time (seconds)
	static double xtemp[N + 1];	//state vector x[N](t)	Temporarily store and output to x[6+1]
	double xd[N+1]; // derivative vector at time t
	const int M = 1; // number of inputs
	static double u[M+1]; // input vector
	static int init = 0;

	// initialization section
	if (!init) {
		// #1) set initial conditions (IC)
		t = 0.0; // initial time (s)
		xtemp[0] = -1.0;				// not used
		xtemp[1] = 0.0;					// initial theta1
		xtemp[2] = PI / 12 - PI / 6;	// initial theta2
		xtemp[3] = -PI / 12;			// initial theta3
		xtemp[4] = 0.0;					// initial vheta1
		xtemp[5] = 0.0;					// initial vheta2
		xtemp[6] = 0.0;					// initial vheta3
		init = 1;
	}
		
	// new approach for assignment #5
	calculate_inputs(xtemp,t,N,u,M); // controller calculate u
	calculate_Xd(xtemp,t,N,u,M,xd);

	// make sure you calculate all your derivatives first before
	// applying euler step

	// step #4 apply Euler's equation
	// calculate x(t+dt)
	// same for your project
	for(i=1;i<=N;i++) xtemp[i] = xtemp[i] + xd[i]*dt;

	// don't forget to increment time
	t = t + dt; // increment time

	// set the output of interest
	yaw = xtemp[1];		// Rotates Base Position
	pitch2 = xtemp[2];	// Pitch First Arm Position
	pitch3 = xtemp[3];	// Pitch Second Arm Position
}


// U is and output so we don't use a const modifier (ie full call by reference)
void calculate_inputs(const double X[], double t, int N, double U[], int M)
{

	// defining the parameters & input
	double m[3 + 1] = { 0.0, 1.0, 1.0, 1.0 }; // mass matrix of all the arms
	double l[3 + 1] = { 0.0, 2.0, 2.0, 2.0 }; // length matrix of the arms
	double F[3 + 1]; // force matrix

	// Unpack state variables & pack inputs
	double theta1 = X[1];
	double theta2 = X[2];
	double theta3 = X[3];
	double vheta1 = X[4];
	double vheta2 = X[5];
	double vheta3 = X[6];

	F[1] = 0;		//Leave them separate so we can give them different inputs individually
//	F[2] = m[2] * g * l[2] / 2 * cos(theta2) + m[3] * g * (l[2] * cos(theta2) + l[3] / 2 * cos(theta2 + theta3));
//	F[3] = m[3] * g * l[3] / 2 * cos(theta2 + theta3);
	F[2] = 0;
	F[3] = 0;

	for (int i = 1; i <= 3; i++) U[i] = F[i];
}


// note U is and input for this function hence the const modifier
void calculate_Xd(const double X[], double t, int N, const double U[], int M, double Xd[])
{
	double Ma[3 + 1][3 + 1]; // inertia matrix
	double G[3 + 1]; // gravity vector matrix
	double C[3 + 1]; // corriolis effect matrix
	double F[3 + 1];

	//x[] are relative angles (x[1] = theta1, x[2] = theta2 x[3] = theta3)
	//v[] are relative velocities
	double x[3 + 1], v[3 + 1]; // state variables
	double dx[3 + 1], dv[3 + 1]; // derivatives

	// unpack state variables & inputs
	for (int i = 1; i <= 3; i++) x[i] = X[i];
	for (int i = 1; i <= 3; i++) v[i] = X[i + 3];
	for (int i = 1; i <= 3; i++) F[i] = U[i];

	// defining the parameters
	double m[3 + 1] = { 0.0, 1.0, 1.0, 1.0 }; // mass matrix of all the arms
	double l[3 + 1] = { 0.0, 2.0, 2.0, 2.0 }; // length matrix of the arms 
	double D = (l[2] + l[3]) * sin((PI - x[3]) / 2);	//Distance from end of arm 1 to tip of arm 3
	double R = D * cos(x[2] + x[3]);	//Distance of D in x/y plane
	double det = 0.0;
	double Minv[3 + 1][3 + 1] = { 0.0 };

	// Dynamic model
	ComputeMassMatrix(m, x, R, l, Ma);
	/*
	cout << "\n";
	for (int i = 1; i <= 3; i++)
	{
	for (int j = 1; j <= 3; j++)
	{
	cout << "Ma[" << i << "][" << j << "] = " << Ma[i][j] << " ";
	}
	cout << endl;
	}
	*/
	//Compute Gravity Matrix
	ComputeGravityMatrix(m, x, l, G);

	//	for (int i = 1; i <= 3; i++)
	//		cout << "G[" << i << "] = " << G[i] << " ";

	//Compute Coriolis Matrix
	ComputeCoriolisMatrix(m, x, v, l, C);

	//	cout << endl;
	//	for (int i = 1; i <= 3; i++)
	//		cout << "C[" << i << "] = " << C[i] << " ";

	//	cout << endl << endl;

	//Compute Mass Matrix Determinant
	ComputeDeterminant(Ma, det);

	//	cout << "det = " << det << endl;

	//Compute Inverse Mass Matrix
	ComputeInvertedMatrix(Ma, det, Minv);

	/*	cout << "\n";
	for (int i = 1; i <= 3; i++)
	{
	for (int j = 1; j <= 3; j++)
	{
	cout << "Minv[" << i << "][" << j << "] = " << Minv[i][j] << " ";
	}
	cout << endl;
	}
	*/
	// Velocity kinematic
	for (int i = 1; i <= 3; i++) dx[i] = v[i];

	dv[1] = Minv[1][1] * (F[1] - C[1] - G[1]) + Minv[1][2] * (F[2] - C[2] - G[2]) + Minv[1][3] * (F[3] - C[3] - G[3]);
	dv[2] = Minv[1][2] * (F[1] - C[1] - G[1]) + Minv[2][2] * (F[2] - C[2] - G[2]) + Minv[2][3] * (F[3] - C[3] - G[3]);
	dv[3] = Minv[1][3] * (F[1] - C[1] - G[1]) + Minv[2][3] * (F[2] - C[2] - G[2]) + Minv[3][3] * (F[3] - C[3] - G[3]);

	// pack the state variables
	for (int i = 1; i <= 3; i++) Xd[i] = dx[i];
	for (int i = 1; i <= 3; i++) Xd[i + 3] = dv[i];
}

void ComputeMassMatrix(double m[3 + 1], double x[3 + 1], double R, double l[3 + 1], double Ma[3 + 1][3 + 1])
{
	Ma[1][1] = (m[1] * R * R) / 2 + (m[2] * l[2] * l[2] * cos(x[2]) * cos(x[2])) / 3 + (m[3] * l[3] * l[3] * cos(x[2] + x[3]) * cos(x[2] + x[3])) / 3 + (m[3] * l[2] * l[2] * cos(x[2]) * cos(x[2])) + (m[3] * l[2] * l[3] * cos(x[2] + x[3])*cos(x[2]));
	Ma[2][2] = (m[2] * l[2] * l[2]) / 3 + (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3]));
	Ma[2][3] = (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3])) / 3;
	Ma[3][3] = (m[3] * l[3] * l[3]) / 3;
	Ma[1][2] = Ma[1][3] = Ma[2][1] = Ma[3][1] = 0.0;
	Ma[3][2] = Ma[2][3];

}

void ComputeGravityMatrix(double m[3 + 1], double x[3 + 1], double l[3 + 1], double G[3 + 1])
{
	double g = 9.81;

	G[1] = 0;
	G[2] = (m[2] * g * l[2] * cos(x[2])) / 2 + (m[3] * g * l[3] * cos(x[2] + x[3])) / 2 + (m[3] * g * l[2] * cos(x[2]));
	G[3] = (m[3] * g * l[3] * cos(x[2] + x[3])) / 2;
}

void ComputeCoriolisMatrix(double m[3 + 1], double x[3 + 1], double v[3 + 1], double l[3 + 1], double C[3 + 1])
{
	C[1] = ((-4 / 3)*(m[2] * l[2] * l[2] * sin(2 * x[2])) - (m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) / 3 - (m[3] * l[2] * l[3] * sin(2 * x[2] + x[3]))) * v[2] * v[1] + ((-1 / 3)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) - (m[3] * l[2] * l[3] * cos(x[2])*sin(x[2] + x[3]))) * v[3] * v[1];
	C[2] = ((-1)*(m[3] * l[2] * l[3] * sin(x[3])) * v[2] * v[3] + (-1 / 2)*(m[3] * l[2] * l[3] * sin(x[3])) * v[3] * v[3] + ((1 / 6)*(m[2] * l[2] * l[2] * sin(2 * x[2])) + (1 / 6)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) + (1 / 2)*(m[3] * l[2] * l[2] * sin(2 * x[2])) + (1 / 2)*(m[3] * l[2] * l[3] * sin(2 * x[2] + x[3])))*v[1] * v[1]);
	C[3] = ((1 / 2)*(m[3] * l[2] * l[3] * sin(x[3]))) * v[2] * v[2] + ((1 / 6)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) + (1 / 2)*(m[3] * l[2] * l[3] * cos(x[2])*sin(x[2] + x[3])))*v[1] * v[1];
}

void ComputeDeterminant(double Ma[3 + 1][3 + 1], double & det)
{
	for (int i = 1; i <= 3; i++)
	{
		if (i == 1) det += Ma[1][1] * (Ma[2][2] * Ma[3][3] - Ma[3][2] * Ma[2][3]);
		if (i == 2) det -= Ma[1][2] * (Ma[2][1] * Ma[3][3] - Ma[3][1] * Ma[2][3]);
		if (i == 3) det += Ma[1][3] * (Ma[2][1] * Ma[3][2] - Ma[3][1] * Ma[2][2]);
	}

}

void ComputeInvertedMatrix(double Ma[3 + 1][3 + 1], double det, double Minv[3 + 1][3 + 1])
{
	Minv[1][1] = (Ma[2][2] * Ma[3][3] - Ma[3][2] * Ma[2][3]) / det;
	Minv[1][2] = (Ma[1][3] * Ma[3][2] - Ma[1][2] * Ma[3][3]) / det;
	Minv[1][3] = (Ma[1][2] * Ma[2][3] - Ma[1][3] * Ma[2][2]) / det;
	Minv[2][1] = (Ma[2][3] * Ma[3][1] - Ma[2][1] * Ma[3][3]) / det;
	Minv[2][2] = (Ma[1][1] * Ma[3][3] - Ma[1][3] * Ma[3][1]) / det;
	Minv[2][3] = (Ma[2][1] * Ma[1][3] - Ma[1][1] * Ma[2][3]) / det;
	Minv[3][1] = (Ma[2][1] * Ma[3][2] - Ma[3][1] * Ma[2][2]) / det;
	Minv[3][2] = (Ma[3][1] * Ma[1][2] - Ma[1][1] * Ma[3][2]) / det;
	Minv[3][3] = (Ma[1][1] * Ma[2][2] - Ma[2][1] * Ma[1][2]) / det;
}