
// example of Euler's algorithm for a 2nd order system

#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0

#include <conio.h> // console I/O functions such as getch()

using namespace std;

// similar to assignment #4 quesion #3
void calculate_Xd(const double X[], double t, int N, double Xd[]);

// similar to assignment #5 
// please compare in detail to the function above noting differences and similarities

// U is and output so we don't use a const modifier (ie full call by reference)
void calculate_inputs(const double X[], double t, int N, double U[], int M);

// note U is and input for this function hence the const modifier
void calculate_Xd(const double X[], double t, int N, const double U[], int M, double Xd[]);

int main()
{
	int i;
	const int N = 2; // number of state variables (order of the system)
	double t; // current time (seconds)
	double x[N+1];  // state vector x[N](t)
	double xd[N+1]; // derivative vector at time t
	double dt; // time step (s)
	double tf; // final time (s)
	const int M = 1; // number of inputs
	double u[M+1]; // input vector

	// excel
	ofstream fout("sim2.csv"); // output file

	// alteranative for matlab
//	ofstream fout("sim2.dat"); // output file

	// #1) set initial conditions (IC)
	t = 0.0; // initial time (s)
	x[1] = 1.0; // initial position
	x[2] = 0.0; // initial velocity

	// #2) set simulation parameters
	dt = 0.01; // time step
	tf = 10.0; // final time (s)

	while(t < tf) {

		// save time and states into a file
		fout << t;
		for(i=1;i<=N;i++) fout << "," << x[i];
		
		// matlab alternative
//		for(i=1;i<=N;i++) fout << " " << x[i];

		// calculate the derivative vector at time t
//		calculate_Xd(x,t,N,xd); // previous approach (should give same answer as new approach)
		
		// new approach for assignment #5
		calculate_inputs(x,t,N,u,M);
		calculate_Xd(x,t,N,u,M,xd);

		// make sure you calculate all your derivatives first before
		// applying euler step

		// step #4 apply Euler's equation
		// calculate x(t+dt)
		// same for your project
		for(i=1;i<=N;i++) x[i] = x[i] + xd[i]*dt;

		// don't forget to increment time
		t = t + dt; // increment time

		if(t<tf) fout << "\n";

	}

	fout.close();

	return 0;
}


// U is and output so we don't use a const modifier (ie full call by reference)
void calculate_inputs(const double X[], double t, int N, double U[], int M)
{
		double m,k,b; // parameters
		double x,v; // state variables
		double F; // input

		// define parameters
		m = 1.0;
		b = 0.0;
		k = 1.0;

		// unpacking states
		x = X[1];
		v = X[2];

		// define the inputs
		F = 0.0;

		// alternative #1
//		F = sin(t);

		// alternative #2
//		F = -5.0*x - 5.0*v;

		// pack input vector
		U[1] = F;
}


// note U is and input for this function hence the const modifier
void calculate_Xd(const double X[], double t, int N, const double U[], int M, double Xd[])
{
		double m,k,b; // parameters
		double x,v; // state variables
		double xd,vd; // derivatives
		double F; // input

		// unpacking states
		x = X[1];
		v = X[2];

		// unpacking inputs
		F = U[1];
	
		// m = 1, k = 1, b= 0
		// m*xdd = -k*x - b*xd
		// x1 = x
		// x2 = xd
		// m*x2d = - k*x1 - b*x2
		// x1d = x2
		// m = 1, k=1, b = 0
		// step #3) calculate derivatives (call the xd function)
		// different for your project

		// define parameters
		m = 1.0;
		b = 0.0;
		k = 1.0;

		xd = v;  // xd = v
		vd = (-k*x - b*v + F)/m; // vd = a

		// pack the state derivatives
		Xd[1] = xd;
		Xd[2] = vd;
}


void calculate_Xd(const double X[], double t, int N, double Xd[])
{
		double m,k,b; // parameters
		double x,v; // state variables
		double xd,vd; // derivatives
		double F; // input

		// unpacking
		x = X[1];
		v = X[2];
	
		// m = 1, k = 1, b= 0
		// m*xdd = -k*x - b*xd
		// x1 = x
		// x2 = xd
		// m*x2d = - k*x1 - b*x2
		// x1d = x2
		// m = 1, k=1, b = 0
		// step #3) calculate derivatives (call the xd function)
		// different for your project

		// model parameters
		m = 1.0;
		b = 0.0;
		k = 1.0;

		F = 0.0;
//		F = sin(t); // alternative

		xd = v;  // xd = v
		vd = (-k*x - b*v + F)/m; // vd = a

		// pack the state derivatives
		Xd[1] = xd;
		Xd[2] = vd;
}


