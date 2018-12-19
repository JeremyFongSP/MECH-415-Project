#include <cmath>
#include <cstdio>
#include <cstring>

#include <iostream>
#include <fstream>
#include <strstream>

#include <windows.h>			// for keyboard input

#include "3D_graphics.h"
#include "my_project.h"
#include "ran.h"

using namespace std;

//------------------------------------------------ Global --------------------------------------------------------------
const double PI =	atan(1) * 4;
const double g	=	9.81;
double m[3 + 1] =	{ -1.0, 10.0, 10.0, 12.0 };			//Mass of arms
double l[3 + 1] =	{ -1.0, 16.5, 16.5, 21.0 };			//Length of arms
ofstream fout_myproject("My_project_Debug.csv");

//---------------------------------------------- End Global -----------------------------------------------------------

//------------------------------------------------ Classes -------------------------------------------------------------

Body::Body(mesh *Pm, double Px, double Py, double Pz, double pitch, double yaw, double roll)
{
	this->Px = Px;
	this->Py = Py;
	this->Pz = Pz;
	this->pitch = pitch;
	this->yaw = yaw;
	this->roll = roll;
	this->Pm = Pm;
}

void Body::draw()
{
	Pm->draw(Px, Py, Pz, yaw, pitch, roll);
}

Arm::Arm(double length, double mass, mesh *Pm, double x, double y, double z, double pitch, double yaw, double roll) : Body(Pm, x, y, z, pitch, yaw, roll)
{
	this->length = length;
	this->mass = mass;
}

int Object::N;	//Initialize static N for object count

Object::Object(double radius, mesh *Pm, double x, double y, double z, double pitch, double yaw, double roll) : Body(Pm, x, y, z, pitch, yaw, roll)
{
	this->radius = radius;
	NumObj = ++N;	//Count number of objects on the field
}

void Object::sim_fall(double dt)
{
	if (Pz > 1e-4)	Pz -= g*dt;
//	checkGoal();							//TO-DO2:	if reaches goal, add 1 point
}

ObjectWorld::ObjectWorld(int N, mesh *Pm1, int M, mesh *Pm2)
{
	this->Pm1 = Pm1;
	this->Pm2 = Pm2;
	this->N = N;
	this->M = M;

	for (int i = 1; i <= N; i++)
		Pn[i] = new Object(5.0, Pm1, rand() % 20, rand() % 20, 0.0, 0.0, 0.0, 0.0);
	for (int i = 1; i <= N; i++)
		Pm[i] = new Object(5.0, Pm2, rand() % 20, rand() % 20, 0.0, 0.0, 0.0, 0.0);
}

ObjectWorld::~ObjectWorld()
{
	for (int i = 1; i <= N; i++)
	{
		if (Pn[i] == nullptr)		//safe deletes
			exit(1);
		else
		{
			delete Pn[i];
			Pn[i] = nullptr;
		}
	}

	for (int i = 1; i <= M; i++)
	{
		if (Pm[i] == nullptr)
			exit(1);
		else
		{
			delete Pm[i];
			Pm[i] = nullptr;
		}
	}
}

void ObjectWorld::draw()
{
	for (int i = 1; i <= N; i++)
		Pn[i]->draw();
	for (int i = 1; i <= M; i++)
		Pm[i]->draw();
}


//-------------------------------------------------- End Classes -----------------------------------------------------------

//--------------------------------------------------- Functions ------------------------------------------------------------

void sim_step(double dt, double &yaw, double &pitch2, double &pitch3)
{

	int i;
	const int N = 3;									// number of state variables (order of the system)
	static double t;									// current time (seconds)
	static double xtemp[N + 1];							// state vector x[N](t)	Temporarily store and output to x[6+1]
	double xd[N+1];										// derivative vector at time t
	const int M = 2;									// number of inputs
	static double u[M+1];								// input vector
	static int init = 0;

	// initialization section
	if (!init) {
		t = 0.0;										// initial time
		xtemp[0] = -1.0;								// not used
		xtemp[1] = 0.0;									// initial theta1 (yaw)
		xtemp[2] = -PI/6;								// initial theta2 (pitch2)
		xtemp[3] = PI/6;								// initial theta3 (pitch3)
		xtemp[4] = 0.0;									// initial vheta1
		xtemp[5] = 0.0;									// initial vheta2
		xtemp[6] = 0.0;									// initial vheta3
		init = 1;
	}

	calculate_inputs(xtemp,t,N,u);						// calculate u
	calculate_Xd(xtemp,t,N,u,M,xd);						// calculate x-derivatives

	for(i=1;i<=N;i++) xtemp[i] = xtemp[i] + xd[i]*dt;	// Euler step

	t = t + dt;											// increment time

	// output to draw_3D_graphics
	yaw =		xtemp[1];								// Rotates Base Position
	pitch2 =	xtemp[2];								// Pitch First Arm Position
	pitch3 =	xtemp[3] + xtemp[2];					// Pitch Second Arm Position
}

void calculate_inputs(const double X[], double t, int N, double U[])
{
	double F[3 + 1] = { 0.0 };							// force matrix
	double fs = 2e-4;									// force step
	double friction_coef = 1e-4;						// force friction depends on current velocity
	static double ObjTheta[3 + 1] = { 0.0 };

	// unpack
	double theta1 = X[1];
	double theta2 = X[2];
	double theta3 = X[3];
//	double vheta1 = X[4];
//	double vheta2 = X[5];
//	double vheta3 = X[6];

	//Keystrokes: W - A - S - D - Q - E
	if (KEY(0x44))	F[1] = fs;
	if (KEY(0x41))	F[1] = -fs;
	if (KEY(0x53))	F[2] = fs;
	if (KEY(0x57))	F[2] = -fs;
	if (KEY(0x51))	F[3] = fs;
	if (KEY(0x45))	F[3] = -fs;


//TO-DO: Add a stop + bounce back if arm goes too far (kind of like collision)

	//these forces stabilize the arms
//	F[1] = 0;
//	F[2] = m[2] * g * l[2] / 2 * cos(theta2) + m[3] * g * (l[2] * cos(theta2) + l[3] / 2 * cos(theta2 + theta3));
//	F[3] = m[3] * g * l[3] / 2 * cos(theta2 + theta3);
	

	// Update velocity input (w/ friction which depends on velocity)
	for (int i = 1; i <= 3; i++)
	{
		U[i] = U[i] + F[i] - friction_coef*U[i];
	}
}

void calculate_Xd(const double X[], double t, int N, const double U[], int M, double Xd[])
{
//	double Ma[3 + 1][3 + 1];							// inertia matrix
//	double G[3 + 1];									// gravity matrix
//	double C[3 + 1] = { 0.0 };							// corriolis effect matrix
//	double F[3 + 1];

	double x[3 + 1], v[3 + 1];							// state variables: x[1] = yaw, x[2] = pitch2 x[3] = pitch3
	double dx[3 + 1];	// , dv[3 + 1];					// angular derivatives
	static double vp[3 + 1] = { 0.0 };					// Previous velocity
	// unpack state variables & inputs
	for (int i = 1; i <= 3; i++) x[i] = X[i];
//	for (int i = 1; i <= 3; i++) v[i] = X[i + 3];

	for (int i = 1; i <= 3; i++) v[i] = U[i];

	// defining the parameters
//	double D = (l[2] + l[3]) * sin((PI - abs(x[3])) / 2);	//Distance from end of arm 1 to tip of arm 3
//	double R = D * cos(x[2] + x[3] / 2);					//Distance of D in x/y plane
//	double det = 0.0;
//	double Minv[3 + 1][3 + 1] = { 0.0 };

	// dynamic model (all of the physics)
//	ComputeMassMatrix(m, x, R, l, Ma);
//	ComputeDeterminant(Ma, det);
//	ComputeInvertedMatrix(Ma, det, Minv);
//	ComputeCoriolisMatrix(m, x, v, l, C);
//	ComputeGravityMatrix(m, x, l, G);

	// velocity kinematic
	for (int i = 1; i <= 3; i++)	dx[i] = v[i];

//	dv[1] = Minv[1][1] * (F[1] - C[1] - G[1]) + Minv[1][2] * (F[2] - C[2] - G[2]) + Minv[1][3] * (F[3] - C[3] - G[3]);
//	dv[2] = Minv[2][1] * (F[1] - C[1] - G[1]) + Minv[2][2] * (F[2] - C[2] - G[2]) + Minv[2][3] * (F[3] - C[3] - G[3]);		//The only things that affect the physics is M22, M23, M32, M33
//	dv[3] = Minv[3][1] * (F[1] - C[1] - G[1]) + Minv[3][2] * (F[2] - C[2] - G[2]) + Minv[3][3] * (F[3] - C[3] - G[3]);

	// update/pack xd[]
	for (int i = 1; i <= 3; i++) Xd[i] = dx[i];
//	for (int i = 1; i <= 3; i++) Xd[i + 3] = dv[i];

//TO-DO: Figure out why the physics doesn't work

}

void ComputeMassMatrix(double m[3 + 1], double x[3 + 1], double R, double l[3 + 1], double Ma[3 + 1][3 + 1])
{

	Ma[1][1] = (m[1] * R * R) / 2 + (m[2] * l[2] * l[2] * cos(x[2] *x[2])) / 3 + (m[3] * l[3] * l[3] * cos((x[2] + x[3]) * (x[2] + x[3]))) / 3 + (m[3] * l[2] * l[2] * cos(x[2]* x[2])) + (m[3] * l[2] * l[3] * cos(x[2] + x[3])*cos(x[2]));
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

	for (int i = 1; i <= 3; i++)
	{
		for (int j = 1; j <= 3; j++)
		{
			fout_myproject << Ma[i][j] << ",";
		}
		fout_myproject << "\n";
	}
	*/

}

void ComputeGravityMatrix(double m[3 + 1], double x[3 + 1], double l[3 + 1], double G[3 + 1])
{
	/*
	G[1] = 0;
	G[2] = (m[2] * g * l[2] * cos(x[2])) / 2 + (m[3] * g * l[3] * cos(x[2] + x[3])) / 2 + (m[3] * g * l[2] * cos(x[2]));
	G[3] = (m[3] * g * l[3] * cos(x[2] + x[3])) / 2;		// Should this be negative?
	*/
	G[1] = 0;
	G[2] = (m[2] * g * l[2] * cos(x[2])) / 2 + (m[3] * g * l[3] * cos(x[3] + x[2])) / 2 + (m[3] * g * l[2] * cos(x[2]));
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
	/*
	for (int i = 1; i <= 3; i++)
	{
		for (int j = 1; j <= 3; j++)
		{
			fout_myproject << Minv[i][j] << ",";
		}
		fout_myproject << "\n";
	}
	*/
}

void locateObject(double objThetas[3+1], double x, double y, double z)
{
	//Inverse Kinematics
	double ztemp = z + 3;
	double r;
	double D;
	objThetas[1] =	atan2(y, x);
	r			 =	sqrt(x*x + y*y);
	D			 =	sqrt((ztemp - l[1])*(z - l[1]) + r * r);
	objThetas[3] =	PI - acos((D * D - l[3] * l[3] - l[2] * l[2]) / -(2 * l[2] * l[3]));
	objThetas[2] =	atan2(-(ztemp - l[1]), r) - atan2(l[3] * sin(objThetas[3]), l[2] + l[3] * cos(objThetas[3]));

//TO-DO: Make classes so that we can just pass objects instead of inputs
}

void pointAtObject(double dt, double objTheta[3 + 1], double & yaw, double & pitch2, double & pitch3)
{

	if (yaw != objTheta[1])		yaw	   += (objTheta[1] - yaw)*dt;
	if (pitch2 != objTheta[2])	pitch2 += (objTheta[2] - pitch2)*dt;
	if ((pitch3) != (objTheta[3]+objTheta[2]))	pitch3 += ((objTheta[3]+objTheta[2]) - pitch3)*dt;

	if (abs(objTheta[1] - yaw) < 0.1 && abs(objTheta[2] - pitch2) < 0.1 && abs(objTheta[3] - pitch3) < 0.1)
	{
		yaw =		objTheta[1];
		pitch2 =	objTheta[2];
		pitch3 =	objTheta[3]+objTheta[2];
	}

//TO-DO:	Make end effector point directly at object 
//			if we rotate the arm a lot in one direction
//			it'll undo the rotation before pointing at the object
}

void checkPickup(Body end_effector, Object & obj)
{
	bool in_rangeX = false;
	bool in_rangeY = false;
	bool in_rangeZ = false;
	static bool grabbing = false;
	if (KEY(0x52)) grabbing = false;

	if (end_effector.Px < obj.Px + obj.radius && end_effector.Px > obj.Px - obj.radius) in_rangeX = true;
	if (end_effector.Py < obj.Py + obj.radius && end_effector.Py > obj.Py - obj.radius) in_rangeY = true;
	if (end_effector.Pz < obj.Pz + obj.radius && end_effector.Pz > obj.Pz - obj.radius) in_rangeZ = true;

	if (in_rangeX && in_rangeY && in_rangeZ)
	{
		if (KEY(0x47))	{
			grabbing = !grabbing;		//Toggle grab/not grab
			obj.is_grabbed = !obj.is_grabbed;
			Sleep(200);
		}

		if (grabbing && obj.is_grabbed)
		{
			obj.Px = end_effector.Px;
			obj.Py = end_effector.Py;
			obj.Pz = end_effector.Pz;
			obj.pitch = end_effector.pitch;
			obj.yaw = end_effector.yaw;
		}
	}
}

void resolveCollision(Object & one, Object & two)
{
	bool in_rangeX = false;
	bool in_rangeY = false;
	bool in_rangeZ = false;
	long int s = -3;
	double r = 1;

	if (one.Px - (one.radius + r) < two.Px + (two.radius + r) && one.Px + (one.radius + r) > two.Px - (two.radius + r)) in_rangeX = true;
	if (one.Py - (one.radius + r) < two.Py + (two.radius + r) && one.Py + (one.radius + r) > two.Py - (two.radius + r)) in_rangeY = true;
	if (one.Pz - (one.radius + r) < two.Pz + (two.radius + r) && one.Pz + (one.radius + r) > two.Pz - (two.radius + r)) in_rangeZ = true;


	if (in_rangeX && in_rangeY && in_rangeZ)
	{
		if (one.is_grabbed)
		{
			two.Px += ((ran(s) * 4.0) - 2.0);
			two.Py += ((ran(s) * 4.0) - 2.0);
		}
		else if (two.is_grabbed)
		{
			one.Px += ((ran(s) * 4.0) - 2.0);
			one.Py += ((ran(s) * 4.0) - 2.0);
		}
		else
		{
			two.Px += ((ran(s) * 4.0) - 2.0);
			two.Py += ((ran(s) * 4.0) - 2.0);
			one.Px += ((ran(s) * 4.0) - 2.0);
			one.Py += ((ran(s) * 4.0) - 2.0);
		}
		//TO-DO:	Weird random (is it actually random?)

	}
}
//---------------------------------------------- End Functions -----------------------------------------------------------