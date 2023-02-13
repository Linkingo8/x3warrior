#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class LQR
{
public:
    MatrixXd A, B, Q, R, K, P;
    int n, m;

    LQR(){}

    void computeGain();
    VectorXd control(const VectorXd &x);

};

// int main()
// {
//     MatrixXd A(6, 6);
//     A << 1, 0, 0, 0, 0, 0,
//         0, 1, 0, 0, 0, 0,
//         0, 0, 1, 0, 0, 0,
//         0, 0, 0, 1, 0, 0,
//         0, 0, 0, 0, 1, 0,
//         0, 0, 0, 0, 0, 1;

//     MatrixXd B(6, 2);
//     B << 1, 0,
//         0, 1,
//         1, 0,
//         0, 1,
//         1, 0,
//         0, 1;

//     MatrixXd Q(6, 6);
//     Q << 1, 0, 0, 0, 0, 0,
//         0, 1, 0, 0, 0, 0,
//         0, 0, 1, 0, 0, 0,
//         0, 0, 0, 1, 0, 0,
//         0, 0, 0, 0, 1, 0,
//         0, 0, 0, 0, 0, 1;

//     MatrixXd R(2, 2);
//     R << 1, 0,
//         0, 1;

//     LQR lqr(A, B, Q, R);

//     cout << "K = \n" << lqr.K << endl;

//     return 0;
// }
