#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0

#include <conio.h> // console I/O functions such as getch()

using namespace std;


void calculate_inputs(const double X[], double t, int order, double U[], int input);

// calculates the derivative of the state variables for the project
void calculate_Xd(const double X[], double t, int order, const double U[], int input, double Xd[]);

int main()
{
	const int order = 2;
	double t = 0.0;
	double x[order + 1];
	double xd[order + 1];
	double dt = 0.01;
	double tf = 10.0;
	const int input = 3;
	double u[input + 1];

	ofstream fout("question 1.csv");

	// set initial conditions
	x[1] = 1.0;
	x[2] = 0.0;
	x[3] = 0.0;

	fout << "Time t" << "," << "theta 1" << "," << "theta 2" << "," << "theta 3";

	while (t < tf)
	{
		fout << t;
		for (int i = 1; i <= order; i++) fout << "," << x[i];
		calculate_inputs(x, t, order, u, input);
		calculate_Xd(x, t, order, u, input, xd);
		for (int i = 1; i <= order; i++) x[i] = x[i] + xd[i] * dt;
		t = t + dt;
		if (t<tf) fout << "\n";
	}
	fout.close();
	return 0;
}

void calculate_inputs(const double X[], double t, int order, double U[], int input)
{
	double x[3], v[3]; // state variables

	// defining the parameters & input
	double m[3] = { 1.0, 1.0, 1.0 }; // mass matrix of all the arms
	double l[3] = { 2.0, 2.0, 2.0 }; // length matrix of the arms 
	double F[3] = { 0.0 }; // force matrix

	// unpack state variables & pack inputs
	for (int i = 1; i <= 3; i++) x[i] = X[i];

	for (int i = 1; i <= 3; i++) v[i] = X[i + 3];

	for (int i = 1; i <= 3; i++) U[i] = F[i];
}

void calculate_Xd(const double X[], double t, int order, const double U[], int input, double Xd[])
{
	double M[3][3]; // inertia matrix
	double G[3]; // gravity vector matrix
	double C[3]; // corriolis effect matrix
	double F[3];

	double x[3], v[3]; // state variables

	double dx[3], dv[3]; // derivatives

	// defining the parameters
	double m[3] = { 1.0, 1.0, 1.0 }; // mass matrix of all the arms
	double l[3] = { 2.0, 2.0, 2.0 }; // length matrix of the arms 
	double R = 0.0;
	double g = 9.81;
	double det;
	double Minv[3][3] = { 0.0 };

	M[1][1] = (m[1] * R * R)/2 + (m[2] * l[2] * l[2] * cos(x[2]) * cos(x[2]))/3 + (m[3] * l[3] * l[3] * cos(x[2] + x[3]) * cos(x[2] + x[3]))/3 + (m[3] * l[2] * l[2] * cos(x[2]) * cos(x[2])) + (m[3] * l[2] * l[3] * cos(x[2] + x[3])*cos(x[2]));
	M[2][2] = (m[2] * l[2] * l[2])/3 + (m[3] * l[3] * l[3])/3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3]));
	M[2][3] = (m[3] * l[3] * l[3])/3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3]))/3;
	M[3][3] = (m[3] * l[3] * l[3])/3;
	M[1][2] = M[1][3] = M[2][1] = M[3][1] = 0;
	M[3][2] = M[2][3];

	G[1] = 0;
	G[2] = (m[2] * g * l[2] * cos(x[2]))/2 + (m[3] * g * l[3] * cos(x[2] + x[3]))/2 + (m[3] * g * l[2] * cos(x[2]));
	G[3] = (m[3] * g * l[3] * cos(x[2] + x[3]))/2;

	C[1] = ((-4/3)*(m[2] * l[2] * l[2] * sin(2 * x[2])) - (m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3])))/3 - (m[3] * l[2] * l[3] * sin(2 * x[2] + x[3]))) * v[2] * v[1] + ((-1/3)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) - (m[3] * l[2] * l[3] * cos(x[2])*sin(x[2] + x[3]))) * v[3] * v[1];
	C[2] = ((-1)*(m[3] * l[2] * l[3] * sin(x[3])) * v[2] * v[3] - (-1/2)*(m[3] * l[2] * l[3] * sin(x[3])) * v[3] * v[3] + ((1/6)*(m[2] * l[2] * l[2] * sin(2 * x[2])) + (1/6)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) + (1/2)*(m[3] * l[2] * l[2] * sin(2 * x[2])) + (1/2)*(m[3] * l[2] * l[3] * sin(2 * x[2] + x[3])))*v[1] * v[1]);
	C[3] = ((1/2)*(m[3] * l[2] * l[3] * sin(x[3]))) * v[2] * v[2] + ((1/6)*(m[3] * l[3] * l[3] * sin(2 * (x[2] + x[3]))) + (1/2)*(m[3] * l[2] * l[3] * cos(x[2])*sin(x[2] + x[3])))*v[1] * v[1];

	for (int i = 1; i <= 3; i++)
	{
		det = M[1][i] * (M[1][(i + 1) % 3] * M[2][(i + 2) % 3] - M[1][(i + 2) % 3] * M[2][(i + 1) % 3]);
	}

	for (int i = 1; i <= 3; i++)
	{
		for (int j = 1; j <= 3; j++)
		{
			Minv[i][j] = ((M[(j + 1) % 3][(i + 1) % 3] * M[(j + 2) % 3][(i + 2) % 3]) - (M[(j + 1) % 3][(i + 2) % 3] * M[(j + 2) % 3][(i + 1) % 3])) / det;
		}
	}

	//for (i = 0; i < 3; i++)
	//	determinant = determinant + (mat[0][i] * (mat[1][(i + 1) % 3] * mat[2][(i + 2) % 3] - mat[1][(i + 2) % 3] * mat[2][(i + 1) % 3]));

	//cout << "\n\ndeterminant: " << determinant;

	//cout << "\n\nInverse of matrix is: \n";
	//for (i = 0; i < 3; i++){
	//	for (j = 0; j < 3; j++)
	//		cout << ((mat[(j + 1) % 3][(i + 1) % 3] * mat[(j + 2) % 3][(i + 2) % 3]) - (mat[(j + 1) % 3][(i + 2) % 3] * mat[(j + 2) % 3][(i + 1) % 3])) / determinant << "\t";


	// unpack state variables & inputs
	for (int i = 1; i <= 3; i++) x[i] = X[i];

	for (int i = 1; i <= 3; i++) v[i] = X[i + 3];

	for (int i = 1; i <= 3; i++) F[i] = U[i];

	for (int i = 1; i <= 3; i++) dx[i] = v[i];

	for (int i = 1; i <= 3; i++)
	{
		dv[i] = M[i][1] * (F[1] - C[1] - G[1]) + M[i][2] * (F[2] - C[2] - G[2]) + M[i][3] * (F[3] - C[3] - G[3]);
	}

	// pack the state variables
	for (int i = 1; i <= 3; i++) Xd[i] = dx[i];
	for (int i = 1; i <= 3; i++) Xd[i + 3] = dv[i];

	//vd = (-k*x - b*v + F) / m; // vd = a

	// pack the state derivatives
	//Xd[1] = xd;
	//Xd[2] = vd;
}


// state variables are the following
/*
xd = [ C1*(l3*C2+l2*C2)  -S1*(l3*S23+l2*S2) -l3*S23*S1
	  -S1*(l3*C23+l2*C2) -C1*(l3*S23+l2*S2) -l3*S23*C1
	           0            l3*C23+l2*C2      l3*C23   ]
*/