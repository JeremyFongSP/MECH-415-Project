//Prototypes
class Body
{
public:
	double Px, Py, Pz;
	double pitch, yaw, roll;
	mesh *Pm;

	Body(mesh *Pm, double Px, double Py, double Pz, double pitch, double yaw, double roll);
	Body(mesh *Pm);
	double get_distance(Body obj);
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
	bool in_cage = false;
	double previous_x;
	double previous_y;
	double previous_z;
	int NumObj;
	static int N;

	Object(double radius, mesh *Pm, double x, double y, double z, double pitch, double yaw, double roll);
	Object(double radius, mesh *Pm);
	void sim_fall(double dt);
	void sim_roam(double dt);
	void delete_obj();
};

class FishWorld
{
public:
	int nb;
	Object *pf[5];

	FishWorld(int nb);
	~FishWorld();
	void draw();
};

void calculate_inputs(const double X[], double dt, double U[], double ObjTheta[3+1]);
void calculate_Xd(const double X[], const double U[], double Xd[]);
void sim_step(double dt, double &yaw, double &pitch2, double &pitch3, double ObjTheta[3+1]);
void ComputeMassMatrix(double m[3 + 1], double x[3 + 1], double R, double l[3 + 1], double Ma[3 + 1][3 + 1]);
void ComputeGravityMatrix(double m[3 + 1], double x[3 + 1], double l[3 + 1], double G[3 + 1]);
void ComputeCoriolisMatrix(double m[3 + 1], double x[3 + 1], double v[3 + 1], double l[3 + 1], double C[3 + 1]);
void ComputeDeterminant(double Ma[3 + 1][3 + 1], double & det);
void ComputeInvertedMatrix(double Ma[3 + 1][3 + 1], double det, double Minv[3 + 1][3 + 1]);
void locateObject(double objThetas[3 + 1], Object obj);
void pointAtObject(double dt, const double X[3 + 1], double objTheta[3 + 1], double & yaw, double & pitch2, double & pitch3);
void checkPickup(Body End_Effector, Object & obj);
void resolveCollision(Object & one, Object & two);
void fish_caught(Body one, Object &two, int &score);
