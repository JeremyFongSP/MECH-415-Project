// unpacking of the parameters and packing of the input force
void calculate_inputs(const double X[], double t, int N, double U[], int M);

// calculates the derivative of the state variables for the project
void calculate_Xd(const double X[], double t, int N, const double U[], int M, double Xd[]);

void ComputeMassMatrix(double m[3 + 1], double x[3 + 1], double R, double l[3 + 1], double Ma[3 + 1][3 + 1]);

void ComputeGravityMatrix(double m[3 + 1], double x[3 + 1], double l[3 + 1], double G[3 + 1]);

void ComputeCoriolisMatrix(double m[3 + 1], double x[3 + 1], double v[3 + 1], double l[3 + 1], double C[3 + 1]);

void ComputeDeterminant(double Ma[3 + 1][3 + 1], double & det);

void ComputeInvertedMatrix(double Ma[3 + 1][3 + 1], double det, double Minv[3 + 1][3 + 1]);
