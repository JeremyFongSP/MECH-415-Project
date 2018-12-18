#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions
#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0
#include "functions.h"

using namespace std;

const double PI = atan(1) * 4;

int main()
{
	const int N = 6;
	double t = 0.0;
	double x[N + 1];
	double xd[N + 1];
	double dt = 0.01;
	double tf = 20;
	const int m = 3;
	double u[m + 1];

	double theta1 = 0.0;
	double theta2 = PI / 3;
	double theta3 = -PI / 3;

	ofstream fout("question 1.csv");

	if (!fout) cout << "\nCould not write file, file is currently opened" << endl;
	else
	{
		x[1] = theta1;			//Leave as separate so we can change them more easily
		x[2] = theta2;
		x[3] = theta3;
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
/*			cout << endl;
			for (int i = 1; i <= N; i++){
				cout << "X[" << i << "] = " << x[i] << " ";
			}
			cout << endl;
			for (int i = 1; i <= N; i++){
				cout << "Xd[" << i << "] = " << xd[i] << " ";
			}
			cout << endl;
*/
			for (int i = 1; i <= N; i++) x[i] = x[i] + xd[i] * dt; // euler equation

			//cout values after change
/*			for (int i = 1; i <= N; i++){
				cout << "X[" << i << "]' = " << x[i] << " ";
			}
			cout << endl;
*/
			t = t + dt;
			if (t<tf) fout << "\n";
		}
		fout.close();
	}

	cout << "done!\n";
	return 0;
}