#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0

#include <conio.h> // console I/O functions such as getch()

#include "Function.h"

using namespace std;

int main()
{
	const int N = 6;
	double t = 0.0;
	double x[N + 1];
	double xd[N + 1];
	double dt = 0.01;
	double tf = 10.0;
	const int m = 3;
	double u[m + 1];

	ofstream fout("question 1.csv");

	// set initial conditions
	for (int i = 1; i <= N; i++)
	{
		if (i <= 3) x[i] = 1.0; // initial angle position for theta 1, 2 and 3
		else x[i] = 0.0; // initial velocity for theta 1, 2 and 3
	}

	fout << "Time t" << "," << "theta 1" << "," << "theta 2" << "," << "theta 3" << "," << "velocity theta 1" << "," << "velocity theta 2" << "," << "velocity theta 3" << "\n";

	while (t < tf)
	{
		fout << t;
		for (int i = 1; i <= N; i++) fout << "," << x[i];
		calculate_inputs(x, t, N, u, m);
		calculate_Xd(x, t, N, u, m, xd);
		for (int i = 1; i <= N; i++) x[i] = x[i] + xd[i] * dt; // euler equation
		t = t + dt;
		if (t<tf) fout << "\n";
	}
	fout.close();
	return 0;
}