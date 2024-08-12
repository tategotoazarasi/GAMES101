#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

int main() {
	Vector3f p(2.0f, 1.0f, 1.0f);
	Matrix3f m;
	m << cos(M_PIf / 4), -sin(M_PIf / 4), 1,
	        sin(M_PIf / 4), cos(M_PIf / 4), 2,
	        0, 0, 1;
	cout << m * p << endl;
	return 0;
}