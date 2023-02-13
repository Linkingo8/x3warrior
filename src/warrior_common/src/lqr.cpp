#include "warrior_common/lqr.hpp"

VectorXd LQR::control(const VectorXd &x)
{
    return -K * x;
}


void LQR::computeGain()
{
    for(int i = 0;i<100;i++)
    K = (-R + B.transpose() * Q * B).inverse() * B.transpose() * Q * A;
//     P = (A.transpose() * Q * A + R).inverse();
//     K = (B.transpose() * P * A).transpose();
}