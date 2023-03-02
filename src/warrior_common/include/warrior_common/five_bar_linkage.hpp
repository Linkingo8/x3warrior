#ifndef _FIVE_BAR_LINKAGE_HPP_
#define _FIVE_BAR_LINKAGE_HPP_
#include <iostream>
#include <Eigen/Dense>
#include <memory>

#ifndef PI
#define PI 3.14159265358979323846f
#endif
/// improvement：pass the L of leg as parameter
namespace five_bar_linkage
{
struct bar_length
{
    double L1,L2,L3,L4,L5;
    double L0,q0;
    bar_length() 
    {
        memset(this,0,sizeof(bar_length));
    }
};
struct five_bar_linkage_param
{
    double xb, yb, xd, yd, xc, yc, xm, ym;
    double Lbd;
    double A0, B0, C0;
    double q1, q2, q3, q4;
    double L0, q0;
    double J_array[4];
    double deg0, deg1, deg2, deg3, deg4;
    five_bar_linkage_param() 
    {
        memset(this,0,sizeof(five_bar_linkage_param));
    }
};
class FiveBar
{
private:

    five_bar_linkage_param linkage_param_;
    bar_length bar_length_;
public:
    FiveBar(double L1,double L2,double L3,double L4,double L5);
    ~FiveBar();
    void virtualLegCalc(double q1, double q4);
    bar_length* exportBarLength(void);
    five_bar_linkage_param* exportLinkageParam(void);
};
}
#endif