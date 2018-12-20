#include <cmath>
#include <cstdio>
#include <cstring>

#include <iostream>
#include <fstream>
#include <strstream>
#include <windows.h>

#include "3D_graphics.h"
#include "my_project.h"
#include "ran.h"
#include "timer.h"

using namespace std;

//------------------------------------------------ Global --------------------------------------------------------------
const double PI =	atan(1) * 4;
const double g	=	9.81;
double l[3 + 1] =	{ -1.0, 16.5, 16.5, 25.0 };					//Arm Length for convenience
ofstream fout_myproject("My_project_Debug.csv");				//Debugging

//---------------------------------------------- End Global -----------------------------------------------------------

//------------------------------------------------ Classes -------------------------------------------------------------

//BODY: CONSTRUCTOR
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

//BODY: OVERLOADED CONSTRUCTOR
Body::Body(mesh *Pm)
{
	Px = 0.0;
	Py = 0.0;
	Pz = 0.0;
	pitch = 0.0;
	yaw = 0.0;
	roll = 0.0;
	this->Pm = Pm;
}

//BODY: GET DISTANCE
double Body::get_distance(Body obj)
{
	return sqrt(abs((obj.Px - Px) * (obj.Px - Px) + (obj.Py - Py) * (obj.Py - Py) + (obj.Pz - Pz) * (obj.Pz - Pz)));
}

//BODY: DRAW
void Body::draw()
{
	Pm->draw(Px, Py, Pz, yaw, pitch, roll);
}

//ARM: CONSTRUCTOR
Arm::Arm(double length, double mass, mesh *Pm, double x, double y, double z, double pitch, double yaw, double roll) : Body(Pm, x, y, z, pitch, yaw, roll)
{
	this->length = length;
	this->mass = mass;
}

//Initialize static N for object count
int Object::N;	

//OBJECT: CONSTRUCTOR
Object::Object(double radius, mesh *Pm, double x, double y, double z, double pitch, double yaw, double roll) : Body(Pm, x, y, z, pitch, yaw, roll)
{
	this->radius = radius;
	previous_x = x;
	previous_y = y;
	previous_z = z;
	NumObj = ++N;												//Count number of objects on the field
}

//OBJECT: OVERLOADED CONSTRUCTOR
Object::Object(double radius, mesh *Pm) : Body(Pm)
{
	this->radius = radius;
	previous_z = 0.0;
	NumObj = ++N;												//Count number of objects on the field
}

//OBJECT: SIMULATING FALL
void Object::sim_fall(double dt)
{
	if (Pz > 1e-4)	Pz -= g*dt;									//Steady Fall, no real acceleration
}

//OBJECT: SIMULATING ROAMING (SWIMMING)
void Object::sim_roam(double dt)
{
	static double t0 = high_resolution_time();
	static double t = high_resolution_time() - t0;

	if (!is_grabbed && !in_cage)								//Constantly swim if not grabbed or in cage
	{
		Pz = 5 * sin(1.2*t) + previous_z;
		Px = 5 * cos(1.2*t) + previous_x;
		Py = 5 * sin(1.2*t) + previous_y;
		t += dt;
	}
}

//OBJECT: DELETE OBJECT
void Object::delete_obj()
{
	Px = 200.0;													//Move Far away instead of delete
	Py = 200.0;
	Pz = -200.0;
}

//FISHWORLD: CONSTRUCTOR
FishWorld::FishWorld(int nb)
{
	static mesh fm("fish1.x");
	this->nb = nb;

	fm.Roll_0 = PI;

	for (int i = 1; i <= nb; i++)
	{
		pf[i] = new Object(3.0, &fm);
	}
}

//FISHWORLD: DESTRUCTOR
FishWorld::~FishWorld()
{
	for (int i = 1; i <= nb; i++)
	{
		if (pf[i] == nullptr)									//Safe delete
			exit(1);
		else
		{
			delete pf[i];
			pf[i] = nullptr;
		}
	}
}

//FISHWORLD: DRAW
void FishWorld::draw()
{
	for (int i = 1; i <= nb; i++) 
	{
		pf[i]->Pm->draw(pf[i]->Px, pf[i]->Py, pf[i]->Pz, pf[i]->yaw, pf[i]->pitch, pf[i]->roll);
	}
}

//-------------------------------------------------- End Classes -----------------------------------------------------------

//--------------------------------------------------- Functions ------------------------------------------------------------

void sim_step(double dt, double &yaw, double &pitch2, double &pitch3, double ObjTheta[3+1])
{
	int i;
	const int N = 3;											// number of state variables (order of the system)
	static double t;											// current time (seconds)
	static double xtemp[N + 1];									// state vector temporarily store and output to yaw/pitch2/pitch3
	double xd[3 + 1];											// derivative vector at time t
	const int M = 3;											// number of inputs
	static double u[M + 1];										// input vector
	static int init = 0;

	// initialization section
	if (!init) {
		t = 0.0;												// initial time
		xtemp[0] = -1.0;										// not used
		xtemp[1] = 0.0;											// initial theta1 (yaw)
		xtemp[2] = -PI/6;										// initial theta2 (pitch2)
		xtemp[3] = PI/6;										// initial theta3 (pitch3)
		xtemp[4] = 0.0;											// initial vheta1
		xtemp[5] = 0.0;											// initial vheta2
		xtemp[6] = 0.0;											// initial vheta3
		init = 1;
	}

	calculate_inputs(xtemp,dt,u,ObjTheta);						// calculate u
	calculate_Xd(xtemp,u,xd);									// calculate x-derivatives

	for(i=1;i<=N;i++) xtemp[i] = xtemp[i] + xd[i]*dt;			// Euler step

	t = t + dt;													// increment time

	// Output to draw_3D_graphics
	yaw =		xtemp[1];										// Rotates Base Position
	pitch2 =	xtemp[2];										// Pitch First Arm Position
	pitch3 =	xtemp[3] + xtemp[2];							// Pitch Second Arm Position
}

void calculate_inputs(const double X[], double dt, double U[], double ObjTheta[3+1])
{
	double F[3 + 1] = { 0.0 };									// force matrix
	double fs = 2e-4;											// force step
	double friction_coef = 1e-4;								// force friction depends on current velocity

	//Velocity Input Keystrokes: W - A - S - D - Q - E
	if (KEY(0x44))	F[1] = fs;
	if (KEY(0x41))	F[1] = -fs;
	if (KEY(0x53))	F[2] = fs;
	if (KEY(0x57))	F[2] = -fs;
	if (KEY(0x51))	F[3] = fs;
	if (KEY(0x45))	F[3] = -fs;
	//YAW LINK 1 = A & D
	//PITCH LINK 2 = W & S
	//PITCH LINK 3 = Q & E

	//KEY: 1 - 2 - 3 (the difference is when locating the object)
	if (KEY(0x31) || KEY(0x32) || KEY(0x33)) pointAtObject(dt, X, ObjTheta, F[1], F[2], F[3]);

	// Update velocity input (w/ friction which depends on velocity)
	for (int i = 1; i <= 3; i++)
	{
		U[i] = U[i] + F[i] - friction_coef*U[i];
	}
}

void calculate_Xd(const double X[], const double U[], double Xd[])
{
	double x[3 + 1], v[3 + 1];
	double dx[3 + 1];

	// unpack state variables & inputs
	for (int i = 1; i <= 3; i++)	x[i] = X[i];
	for (int i = 1; i <= 3; i++)	v[i] = U[i];

	// update/pack xd[]
	for (int i = 1; i <= 3; i++)	Xd[i] = v[i];
}

void locateObject(double objThetas[3 + 1], Object obj)
{
	//Inverse kinematics for object location in yaw, pitch, roll
	double r;
	double D;
	objThetas[1] = atan2(obj.Py, obj.Px);
	r = sqrt(obj.Px*obj.Px + obj.Py*obj.Py);
	D = sqrt((obj.Pz - l[1])*(obj.Pz - l[1]) + r * r);
	objThetas[3] = PI - acos((D * D - l[3] * l[3] - l[2] * l[2]) / -(2 * l[2] * l[3]));
	objThetas[2] = atan2(-(obj.Pz - l[1]), r) - atan2(l[3] * sin(objThetas[3]), l[2] + l[3] * cos(objThetas[3]));

	//Output on-screen text
	text_xy(":::ZONING ON TARGET:::", 850, 200, 20);
	text_xy("Target Location: ", 800.0, 250.0, 14);
	text_xy((int)obj.Px, 1000, 250, 14);
	text_xy((int)obj.Py, 1070.0, 250.0, 14);
	text_xy((int)obj.Pz, 1140.0, 250.0, 14);
	text_xy("Objective distance: ", 800.0, 300.0, 14);
}

void pointAtObject(double dt, const double X[3+1], double objTheta[3 + 1], double & yaw, double & pitch2, double & pitch3)
{
	//Update velocities to point at located object
	double fs = 2e-4;
	if ((objTheta[1] - X[1]) > 0.01)							yaw = fs;
	if ((objTheta[1] - X[1]) < -0.01)							yaw = -fs;
	if ((objTheta[2] - X[2]) > 0.01)							pitch2 = fs;
	if ((objTheta[2] - X[2]) < -0.01)							pitch2 = -fs;
	if (((objTheta[3] + objTheta[2]) - (X[3]+X[2])) > 0.01)		pitch3 = fs;
	if (((objTheta[3] + objTheta[2]) - (X[3]+X[2])) < -0.01)	pitch3 = -fs;
}

void checkPickup(Body end_effector, Object & obj)
{
	bool in_rangeX = false;
	bool in_rangeY = false;
	bool in_rangeZ = false;
	static bool grabbing = false;
	if (KEY(0x52)) grabbing = false;

	//If in range in all axes
	if (end_effector.Px < obj.Px + obj.radius && end_effector.Px > obj.Px - obj.radius) in_rangeX = true;
	if (end_effector.Py < obj.Py + obj.radius && end_effector.Py > obj.Py - obj.radius) in_rangeY = true;
	if (end_effector.Pz < obj.Pz + obj.radius && end_effector.Pz > obj.Pz - obj.radius) in_rangeZ = true;

	//Allow grabbing
	if (in_rangeX && in_rangeY && in_rangeZ)
	{
		if (KEY(0x47))	{
			grabbing = !grabbing;								//Toggle grabbing/!grabbing
			obj.is_grabbed = !obj.is_grabbed;
			if (!obj.is_grabbed)
			{
				obj.previous_z = obj.Pz;
				obj.previous_y = obj.Py;
				obj.previous_x = obj.Px;
			}
			Sleep(200);
		}

		if (grabbing && obj.is_grabbed)							//Move object with arm while grabbing
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
	static long int s = -3;
	double r = 1;
		
	//If in range in all axes
	if (one.Px - (one.radius + r) < two.Px + (two.radius + r) && one.Px + (one.radius + r) > two.Px - (two.radius + r)) in_rangeX = true;
	if (one.Py - (one.radius + r) < two.Py + (two.radius + r) && one.Py + (one.radius + r) > two.Py - (two.radius + r)) in_rangeY = true;
	if (one.Pz - (one.radius + r) < two.Pz + (two.radius + r) && one.Pz + (one.radius + r) > two.Pz - (two.radius + r)) in_rangeZ = true;

	//Move objects randomly when colliding if not in cage
	if (in_rangeX && in_rangeY && in_rangeZ && !one.in_cage && !two.in_cage)
	{
		if (one.is_grabbed)
		{
			two.Px += ((ran(s) * 4.0) - 2.0);
			two.Py += ((ran(s) * 4.0) - 2.0);
			s--;
		}
		else if (two.is_grabbed)
		{
			one.Px += ((ran(s) * 4.0) - 2.0);
			one.Py += ((ran(s) * 4.0) - 2.0);
			s--;
		}
		else
		{
			two.Px += ((ran(s) * 4.0) - 2.0);
			two.Py += ((ran(s) * 4.0) - 2.0);
			one.Px += ((ran(s) * 4.0) - 2.0);
			one.Py += ((ran(s) * 4.0) - 2.0);
			s--;
		}
	}
}

void fish_caught(Body one, Object &two, int &score)
{
	bool in_rangeX = false;
	bool in_rangeY = false;
	bool in_rangeZ = false;
	double scale = 5.0;
	double cage_length = 2 * scale;
	double cage_height = 5 * scale;

	//If in range of cage in all axes
	if (((one.Px - cage_length)  < (two.Px - two.radius)) && ((one.Px + cage_length) > (two.Px + two.radius))) in_rangeX = true;
	if (((one.Py - cage_length)  < (two.Py - two.radius)) && ((one.Py + cage_length) > (two.Py + two.radius))) in_rangeY = true;
	if (((one.Pz)  < (two.Pz - two.radius)) && ((one.Pz + cage_height) > (two.Pz + two.radius))) in_rangeZ = true;

	//Update score
	if (in_rangeX && in_rangeY && in_rangeZ && !two.in_cage)
	{
		score++;
		two.in_cage = true;
	}
}
// ----------------------------------- End Functions -------------------------------------------------------

// ----------------------------------- ARCHIVED FUNCTIONS ---------------------------------------------------
//									FOR GRAVITY AND CORIOLIS
/*
void ComputeMassMatrix(double m[3 + 1], double x[3 + 1], double R, double l[3 + 1], double Ma[3 + 1][3 + 1])
{
	Ma[1][1] = (m[1] * R * R) / 2 + (m[2] * l[2] * l[2] * cos(x[2] * x[2])) / 3 + (m[3] * l[3] * l[3] * cos((x[2] + x[3]) * (x[2] + x[3]))) / 3 + (m[3] * l[2] * l[2] * cos(x[2] * x[2])) + (m[3] * l[2] * l[3] * cos(x[2] + x[3])*cos(x[2]));
	Ma[2][2] = (m[2] * l[2] * l[2]) / 3 + (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3]));
	Ma[2][3] = (m[3] * l[3] * l[3]) / 3 + (m[3] * l[2] * l[2]) + (m[3] * l[2] * l[3] * cos(x[3])) / 3;
	Ma[3][3] = (m[3] * l[3] * l[3]) / 3;
	Ma[1][2] = Ma[1][3] = Ma[2][1] = Ma[3][1] = 0.0;
	Ma[3][2] = Ma[2][3];
}

void ComputeGravityMatrix(double m[3 + 1], double x[3 + 1], double l[3 + 1], double G[3 + 1])
{
	G[1] = 0;
	G[2] = (m[2] * g * l[2] * cos(x[2])) / 2 + (m[3] * g * l[3] * cos(x[3] + x[2])) / 2 + (m[3] * g * l[2] * cos(x[2]));
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
	det = Ma[1][1] * (Ma[2][2] * Ma[3][3] - Ma[3][2] * Ma[2][3]) - Ma[1][2] * (Ma[2][1] * Ma[3][3] - Ma[3][1] * Ma[2][3]) + Ma[1][3] * (Ma[2][1] * Ma[3][2] - Ma[3][1] * Ma[2][2]);
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
}
*/