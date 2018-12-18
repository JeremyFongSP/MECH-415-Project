#include <conio.h>
#include <iostream>
#include <cmath> 
#include <fstream>
#include <windows.h>		// for keyboard input

#include "timer.h"			// for measuring time
#include "rotation.h"		// for computing rotation functions
#include "3D_graphics.h"	// for DirectX 3D graphics
#include "my_project.h"
#include "graphic_setup.h"

/*
//Initialization for each mesh
static double Px, Py, Pz, pitch, yaw, roll;
static double Px2, Py2, Pz2, pitch2, yaw2, roll2;
static double Px3, Py3, Pz3, pitch3, yaw3, roll3;
static double Bgx, Bgy, Bgz, Bgpitch, Bgyaw, Bgroll;
static double Pxobj1, Pyobj1, Pzobj1, pitchobj1, yawobj1, rollobj1;
static double Pxobj2, Pyobj2, Pzobj2, pitchobj2, yawobj2, rollobj2;
static int init = 0;

static double t;											// clock time from t0
static double t0;											// initial clock time

static double T, fps, tp, dt, dt2;							// dt used for sim stepping; dt2 used to move arm

static mesh m1("arm1.x");								//the mesh here gets constructed only once since  its a static local variable, so no need to put in the initialization section
static mesh m2("arm2.x");									//You can keep adding max of 8 mesh files.
static mesh m3("arm3.x");
static mesh bg("background.x");
static mesh obj1("plane.x");
static mesh obj2("car.x");
*/

simulation::simulation(char *mesh_name, double position[3], double rotation[3]) {
	init();
	char name[500], txt[] = ".txt";
	strcpy(name, mesh_name);
	strcat(name, txt);
	mesh name(name);

	//static mesh m1("arm1.x");
	//static mesh m2("arm2.x");
	//static mesh m3("arm3.x");
	//static mesh bg("background.x");
	//static mesh obj1("plane.x");
	//static mesh obj2("car.x");



}

simulation::~simulation() {
	deinit();
}

int simulation::init() {
	t0 = high_resolution_time();							// initial clock time (s)
	tp = 0.0;
	t = high_resolution_time() - t0;						//Time since the program started (s)


}

int simulation::deinit() {

}

void simulation::draw_sim(double position[3], double rotation[3]){


	//m1.draw(Px, Py, Pz, yaw, pitch, roll);
	//m2.draw(Px2, Py2, Pz2, yaw2, pitch2, roll2);
	//m3.draw(Px3, Py3, Pz3, yaw3, pitch3, roll3);

	////Draw Background
	//bg.draw(Bgx, Bgy, Bgz, Bgyaw, Bgpitch, Bgroll);

	////Draw Objects
	//obj1.draw(Pxobj1, Pyobj1, Pzobj1, yawobj1, pitchobj1, rollobj1);
	//obj2.draw(Pxobj2, Pyobj2, Pzobj2, yawobj2, pitchobj2, rollobj2);
}

void key_inut() {


	//if (KEY(VK_UP))		pitch3 -= PI*dt2 / 2;
	//if (KEY(VK_DOWN))	pitch3 += PI*dt2 / 2;
	//if (KEY(VK_RIGHT))	yaw += PI*dt2 / 2;
	//if (KEY(VK_LEFT))	yaw -= PI*dt2 / 2;

}