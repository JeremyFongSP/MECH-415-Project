#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions
#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0
#include "Function.h"

using namespace std;

const double PI = atan(1) * 4;

int main()
{
	const int N = 6;
	double t = 0.0;
	double x[N + 1];
	double xd[N + 1];
	double dt = 0.01;
	double tf = 5;
	const int m = 3;
	double u[m + 1];

	ofstream fout("question 1.csv");

	if (!fout) cout << "\nCould not write file, file is currently opened" << endl;
	else
	{
		// set initial conditions
/*		for (int i = 1; i <= N; i++)
		{
			if (i <= 3) x[i] = 1.0; // initial angle position for theta 1, 2 and 3
			else x[i] = 0.0; // initial velocity for theta 1, 2 and 3
		}
*/
		x[1] = 0.0;			//Leave as separate so we can change them more easily
		x[2] = -PI/2;
		x[3] = 0.0;
		x[4] = 0.0;
		x[5] = 0.0;
		x[6] = 0.0;

		fout << "Time t" << "," << "theta 1" << "," << "theta 2" << "," << "theta 3" << "," << "omega 1" << "," << "omega 2" << "," << "omega 3" << "\n";

		while (t < tf)
		{
			fout << t;
			for (int i = 1; i <= N; i++) fout << "," << x[i];
			calculate_inputs(x, t, N, u, m);
			calculate_Xd(x, t, N, u, m, xd);

			//cout values before change
			cout << endl;
			for (int i = 1; i <= N; i++){
				cout << "X[" << i << "] = " << x[i] << " ";
			}
			cout << endl;
			for (int i = 1; i <= N; i++){
				cout << "Xd[" << i << "] = " << xd[i] << " ";
			}
			cout << endl;

			for (int i = 1; i <= N; i++) x[i] = x[i] + xd[i] * dt; // euler equation

			//cout values after change
			for (int i = 1; i <= N; i++){
				cout << "X[" << i << "]' = " << x[i] << " ";
			}
			cout << endl;

			t = t + dt;
			if (t<tf) fout << "\n";
		}
		fout.close();
	}

	cout << "done!\n";
	return 0;
}