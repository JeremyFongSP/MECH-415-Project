#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0

#include <conio.h> // console I/O functions such as getch()

#include "Function.h" // include Functions

using namespace std;



void calculate_inputs(const double X[], double t, int N, double U[], int M)
{
	//x[] are relative angles (x[1] = theta1, x[2] = theta2 x[3] = theta3)
	//v[] are relative velocities 
	double x[3 + 1], v[3 + 1]; // state variables

	// defining the parameters & input

	double m[3 + 1] = { 0.0, 1.0, 1.0, 1.0 }; // mass matrix of all the arms
	double l[3 + 1] = { 0.0, 2.0, 2.0, 2.0 }; // length matrix of the arms 
	double F[3 + 1]; // force matrix

	// unpack state variables & pack inputs
	for (int i = 1; i <= 3; i++) x[i] = X[i];

	for (int i = 1; i <= 3; i++) v[i] = X[i + 3];

	F[1] = 0;		//Leave them separate so we can give them different inputs individually
	F[2] = 0;
	F[3] = 0;

	for (int i = 1; i <= 3; i++) U[i] = F[i];
}

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
	double m[3 + 1] = { 0.0, 10.0, 10.0, 10.0 }; // mass matrix of all the arms
	double l[3 + 1] = { 0.0,  2.0, 2.0, 2.0 }; // length matrix of the arms 
	double R = 4.0;
	double g = 9.81;
	double det = 0.0;
	double Minv[3 + 1][3 + 1] = { 0.0 };

	//Compute Mass Matrix
	Ma[1][1] = (m[1] * R * R) / 2 + (m[2] * l[2] * l[2] * cos(x[2]) * cos(x[2])) / 3 + (m[3] * l[3] * l[3] * cos(x[2] + x[3]) * cos(x[2] + x[3])) / 3 + (m[3] * l[2] * l[2] * cos(x[2]) * cos(x[2])) + (m[3] * l[2] * l[3] * cos(x[2] + x[3])*cos(x[2]));
	Ma[2][2] = (m[2] * l[2] * l[2]) / 3 + (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3]));
	Ma[2][3] = (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3])) / 3;
	Ma[3][3] = (m[3] * l[3] * l[3]) / 3;
	Ma[1][2] = Ma[1][3] = Ma[2][1] = Ma[3][1] = 0;
	Ma[3][2] = Ma[2][3];

	cout << "\n";
	for (int i = 1; i <= 3; i++)
	{
		for (int j = 1; j <= 3; j++)
		{
			cout << "Ma[" << i << "][" << j << "] = " << Ma[i][j] << " ";
		}
		cout << endl;
	}
		

	//Compute Gravity Matrix
	G[1] = 0;
	G[2] = (m[2] * g * l[2] * cos(x[2])) / 2 + (m[3] * g * l[3] * cos(x[2] + x[3])) / 2 + (m[3] * g * l[2] * cos(x[2]));
	G[3] = (m[3] * g * l[3] * cos(x[2] + x[3])) / 2;

	for (int i = 1; i <= 3; i++)
		cout << "G[" << i << "] = " << G[i] << " ";

	//Compute Coriolis Matrix
	C[1] = ((-4 / 3)*(m[2] * l[2] * l[2] * sin(2 * x[2])) - (m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) / 3 - (m[3] * l[2] * l[3] * sin(2 * x[2] + x[3]))) * v[2] * v[1] + ((-1 / 3)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) - (m[3] * l[2] * l[3] * cos(x[2])*sin(x[2] + x[3]))) * v[3] * v[1];
	C[2] = ((-1)*(m[3] * l[2] * l[3] * sin(x[3])) * v[2] * v[3] + (-1 / 2)*(m[3] * l[2] * l[3] * sin(x[3])) * v[3] * v[3] + ((1 / 6)*(m[2] * l[2] * l[2] * sin(2 * x[2])) + (1 / 6)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) + (1 / 2)*(m[3] * l[2] * l[2] * sin(2 * x[2])) + (1 / 2)*(m[3] * l[2] * l[3] * sin(2 * x[2] + x[3])))*v[1] * v[1]);
	C[3] = ((1 / 2)*(m[3] * l[2] * l[3] * sin(x[3]))) * v[2] * v[2] + ((1 / 6)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) + (1 / 2)*(m[3] * l[2] * l[3] * cos(x[2])*sin(x[2] + x[3])))*v[1] * v[1];

	cout << endl;
	for (int i = 1; i <= 3; i++)
		cout << "C[" << i << "] = " << C[i] << " ";

	cout << endl << endl;
	//Compute Mass Matrix Determinant
	for (int i = 1; i <= 3; i++)
	{
		if (i == 2)
		{
			det -= Ma[1][i] * (Ma[1][(i + 1) % 3] * Ma[2][(i + 2) % 3] - Ma[1][(i + 2) % 3] * Ma[2][(i + 1) % 3]);
		}
		else
		{
			det += Ma[1][i] * (Ma[1][(i + 1) % 3] * Ma[2][(i + 2) % 3] - Ma[1][(i + 2) % 3] * Ma[2][(i + 1) % 3]);
		}
	}

	//Compute Inverse Mass Matrix
	for (int i = 1; i <= 3; i++)
	{
		for (int j = 1; j <= 3; j++)
		{
			Minv[i][j] = ((Ma[(j + 1) % 3][(i + 1) % 3] * Ma[(j + 2) % 3][(i + 2) % 3]) - (Ma[(j + 1) % 3][(i + 2) % 3] * Ma[(j + 2) % 3][(i + 1) % 3])) / det;
		}
	}


	for (int i = 1; i <= 3; i++) dx[i] = v[i];
	for (int i = 1; i <= 3; i++) dv[i] = Ma[i][1] * (F[1] - C[1] - G[1]) + Ma[i][2] * (F[2] - C[2] - G[2]) + Ma[i][3] * (F[3] - C[3] - G[3]);

	// pack the state variables
	for (int i = 1; i <= 3; i++) Xd[i] = dx[i];
	for (int i = 1; i <= 3; i++) Xd[i + 3] = dv[i];

}