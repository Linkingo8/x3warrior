#include "warrior_common/five_bar_linkage.hpp"
using namespace five_bar_linkage;
FiveBar::FiveBar(double L1,double L2,double L3,double L4,double L5)
{
    bar_length_.L1 = L1;
    bar_length_.L2 = L2;
    bar_length_.L3 = L3;
    bar_length_.L4 = L4;
    bar_length_.L5 = L5;
    std::cout << bar_length_.L5 << std::endl;
}

FiveBar::~FiveBar()
{
}
/// @brief calculate the length and theata of virtual leg
/// @param linkage_param_ five bar linkage parammeters
/// @param q1 fai1
/// @param q4 fai4
void FiveBar::virtualLegCalc(double q1, double q4)
{
   /* 更新数据 */
    linkage_param_.q1 = q1;
    linkage_param_.q4 = q4;

    /*  计算膝关节坐标 */
    linkage_param_.xb = bar_length_.L1 * cos(q1);
    linkage_param_.yb = bar_length_.L1 * sin(q1);
    linkage_param_.xd = bar_length_.L4 * cos(q4) + bar_length_.L5;
    linkage_param_.yd = bar_length_.L4 * sin(q4);

    /* 计算解算所需中间系数 */
    linkage_param_.Lbd = sqrt(pow(linkage_param_.xd - linkage_param_.xb, 2) + pow(linkage_param_.yd - linkage_param_.yb, 2));
    linkage_param_.A0 = 2 * bar_length_.L2 * (linkage_param_.xd - linkage_param_.xb);
    linkage_param_.B0 = 2 * bar_length_.L2 * (linkage_param_.yd - linkage_param_.yb);
    linkage_param_.C0 = pow(bar_length_.L2, 2) + pow(linkage_param_.Lbd, 2) - pow(bar_length_.L3, 2);

    /* 解算左膝关节角 */
    linkage_param_.q2 = 2 * atan2(linkage_param_.B0 + sqrt(pow(linkage_param_.A0, 2) + pow(linkage_param_.B0, 2) - pow(linkage_param_.C0, 2)), linkage_param_.A0 + linkage_param_.C0);

    /* 得到足端坐标 */
    linkage_param_.xc = linkage_param_.xb + bar_length_.L2 * cos(linkage_param_.q2);
    linkage_param_.yc = linkage_param_.yb + bar_length_.L2 * sin(linkage_param_.q2);

    /* 解算右膝关节角 */
    linkage_param_.q3 = atan2(linkage_param_.yc - linkage_param_.yd, linkage_param_.xc - linkage_param_.xd);

    /* 写出髋部中点 */
    linkage_param_.xm = bar_length_.L5 / 2;
    linkage_param_.ym = 0;

    /* 计算虚拟腿连杆角度与长度（虚拟模型） */
    linkage_param_.L0 = sqrt(pow(linkage_param_.xc - linkage_param_.xm, 2) + pow(linkage_param_.yc - linkage_param_.ym, 2));
    linkage_param_.q0 = atan2(linkage_param_.yc - linkage_param_.ym, linkage_param_.xc - linkage_param_.xm);
    bar_length_.L0 = linkage_param_.L0;
    bar_length_.q0 = linkage_param_.q0;
    // std::cout << "bar_length_.L0 :" << bar_length_.L0 << std::endl;
    // std::cout << "bar_length_.q0 :" << bar_length_.q0 << std::endl;
}

bar_length* FiveBar::exportBarLength(void)
{
    bar_length *bar = &bar_length_;
    return bar;
}

five_bar_linkage_param* FiveBar::exportLinkageParam(void)
{
    five_bar_linkage_param* param = &linkage_param_;
    return param;

}
