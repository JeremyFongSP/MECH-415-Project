#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions
#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0
#include "Function.h"

using namespace std;

int main()
{
	const int N = 6;
	double t = 0.0;
	double x[N + 1];
	double xd[N + 1];
	double dt = 0.01;
	double tf = 3;
	const int m = 3;
	double u[m + 1];
	double invm[4][4];
	double c[4];

	ofstream fout("question 1.csv");

	if (!fout) cout << "\nCould not write file, file is currently opened" << endl;
	else
	{
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

/*
	//fout << "Time" << "," << "theta 1" << "," << "theta 2" << "," << "theta 3" << "," << "velocity theta 1" << "," << "velocity theta 2" << "," << "velocity theta 3" << "\n";

	while (t < tf)
	{
		fout << t;
		//for (int i = 1; i <= N; i++) fout << "," << x[i];

		calculate_inputs(x, t, N, u, m);
		calculate_Xd(x, t, N, u, m, xd, invm, c);
		for (int i = 1; i <= N; i++) x[i] = x[i] + xd[i] * dt; // euler equation
		//for (int i = 1; i <= N; i++) fout << "," << xd[i];
		for (int i = 1; i <= 3; i++)
		{
			for (int j = 1; j <= 3; j++)
			{
				cout << invm[i][j] << "\t";
			}
			cout << "\n";
		}
		//for (int i = 1; i <= 3; i++) cout << c[i] << " \t";
		cout << "\n";
		t = t + dt;
		if (t<tf) fout << "\n";
	}
	fout.close();
	cout << "done!\n";
*/
	return 0;
}