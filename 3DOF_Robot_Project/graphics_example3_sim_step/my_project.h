
// similar to assignment #4 quesion #3
//void calculate_Xd(const double X[], double t, int N, double Xd[]);

// similar to assignment #5 
// please compare in detail to the function above noting differences and similarities

// U is and output so we don't use a const modifier (ie full call by reference)
void calculate_inputs(const double X[], double t, int N, double U[], int M);

// note U is and input for this function hence the const modifier
void calculate_Xd(const double X[], double t, int N, const double U[], int M, double Xd[]);

void sim_step(double dt, double &yaw, double &pitch2, double &pitch3);

void ComputeMassMatrix(double m[3 + 1], double x[3 + 1], double R, double l[3 + 1], double Ma[3 + 1][3 + 1]);

void ComputeGravityMatrix(double m[3 + 1], double x[3 + 1], double l[3 + 1], double G[3 + 1]);

void ComputeCoriolisMatrix(double m[3 + 1], double x[3 + 1], double v[3 + 1], double l[3 + 1], double C[3 + 1]);

void ComputeDeterminant(double Ma[3 + 1][3 + 1], double & det);

void ComputeInvertedMatrix(double Ma[3 + 1][3 + 1], double det, double Minv[3 + 1][3 + 1]);