
class Body
{
public:
	double Px, Py, Pz;
	double pitch, yaw, roll;
	mesh *Pm;

	Body(mesh *Pm, double Px, double Py, double Pz, double pitch, double yaw, double roll);
	void draw();
};

class Arm : public Body
{
public:
	double length;
	double mass;

	Arm(double length, double mass, mesh *Pm, double x, double y, double z, double pitch, double yaw, double roll);

};

class Object : public Body
{
public:
	double radius;
	bool is_grabbed = false;
	int NumObj;
	static int N;

	Object(double radius, mesh *Pm, double x, double y, double z, double pitch, double yaw, double roll);
	void sim_fall(double dt);		//Objects drawn to the ground
};

class fish
{
public:
	int nb;
	Object *fishes[10];
	double t0, t;

	fish(int nb);
	~fish();

	void draw();
	void input();
};

class ObjectWorld		//For random objects generation
{
public:
	int N;				//mesh obj 1
	int M;				//mesh obj 2
	Object *Pn[10];
	Object *Pm[10];
	mesh *Pm1, *Pm2;

	ObjectWorld(int N, mesh *Pm1, int M, mesh *Pm2);
	~ObjectWorld();
	void draw();
};


// U is and output so we don't use a const modifier (ie full call by reference)
void calculate_inputs(const double X[], double t, int N, double U[]);
// note U is and input for this function hence the const modifier
void calculate_Xd(const double X[], double t, int N, const double U[], int M, double Xd[]);
void sim_step(double dt, double &yaw, double &pitch2, double &pitch3);
void ComputeMassMatrix(double m[3 + 1], double x[3 + 1], double R, double l[3 + 1], double Ma[3 + 1][3 + 1]);
void ComputeGravityMatrix(double m[3 + 1], double x[3 + 1], double l[3 + 1], double G[3 + 1]);
void ComputeCoriolisMatrix(double m[3 + 1], double x[3 + 1], double v[3 + 1], double l[3 + 1], double C[3 + 1]);
void ComputeDeterminant(double Ma[3 + 1][3 + 1], double & det);
void ComputeInvertedMatrix(double Ma[3 + 1][3 + 1], double det, double Minv[3 + 1][3 + 1]);
void locateObject(double objThetas[3 + 1], double x, double y, double z);
void pointAtObject(double dt, double objTheta[3 + 1], double & yaw, double & pitch2, double & pitch3);
void checkPickup(Body End_Effector, Object & obj);
void resolveCollision(Object & one, Object & two);
void fish_catched(Body one, Object &two, int &score);
