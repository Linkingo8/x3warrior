#include "warrior_common/lqr.hpp"

VectorXd LQR::control(const VectorXd &x)
{
    return -K * x;
}


void LQR::computeGain()
{
    P = (A.transpose() * Q * A + R).inverse();
    K = (B.transpose() * P * A).transpose();
}