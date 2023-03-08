
#include "warrior_common/vmc.hpp"

VMC::VMC(/* args */)
{
}

VMC::~VMC()
{
}

void VMC::getDataOfLeg(five_bar_linkage::bar_length* real_leg_data, five_bar_linkage::five_bar_linkage_param* bar_length_calc_date)
{
    bar_length_ = real_leg_data;
    linkage_calc_date_ = bar_length_calc_date;
}

void VMC::VMCControllerCalc(void)
{
    /* 写出此时的雅可比矩阵（虚拟力控制的模型） */
    J_array_[0] = (bar_length_->L1 * sin(linkage_calc_date_->q1 - linkage_calc_date_->q2) * cos(linkage_calc_date_->q0 - linkage_calc_date_->q3)) / (linkage_calc_date_->L0 * sin(linkage_calc_date_->q3 - linkage_calc_date_->q2));
    J_array_[1] = (bar_length_->L1 * sin(linkage_calc_date_->q1 - linkage_calc_date_->q2) * sin(linkage_calc_date_->q0 - linkage_calc_date_->q3)) / sin(linkage_calc_date_->q3 - linkage_calc_date_->q2);
    J_array_[2] = (bar_length_->L4 * sin(linkage_calc_date_->q3 - linkage_calc_date_->q4) * cos(linkage_calc_date_->q0 - linkage_calc_date_->q2)) / (linkage_calc_date_->L0 * sin(linkage_calc_date_->q3 - linkage_calc_date_->q2));
    J_array_[3] = (bar_length_->L4 * sin(linkage_calc_date_->q3 - linkage_calc_date_->q4) * sin(linkage_calc_date_->q0 - linkage_calc_date_->q2)) / sin(linkage_calc_date_->q3 - linkage_calc_date_->q2);
    /*赋值给矩阵*/
    vmc_output_.vmc_Jacobian(0,0) = J_array_[0];
    vmc_output_.vmc_Jacobian(0,1) = J_array_[1];
    vmc_output_.vmc_Jacobian(1,0) = J_array_[2];    
    vmc_output_.vmc_Jacobian(1,1) = J_array_[3];
}

void VMC::setFTp(double Tp,double F)
{
    vmc_output_.vmc_F(0,0) = Tp;
    vmc_output_.vmc_F(1,0) = F;
    // std::cout <<"vmc_F:    \n"<< vmc_output_.vmc_F << std::endl;
}

void VMC::calcT(void)
{
    vmc_output_.vmc_T = vmc_output_.vmc_Jacobian  * vmc_output_.vmc_F;
    std::cout <<"vmc_T:    \n"<< vmc_output_.vmc_T << std::endl;
}

double VMC::exportT1(void)
{
    return vmc_output_.vmc_T(0,0);
}
double VMC::exportT2(void)
{
    return vmc_output_.vmc_T(1,0);
}
