#ifndef _VMC_HPP_
#define _VMC_HPP_
#include "warrior_common/five_bar_linkage.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <memory>
using namespace std;
using namespace Eigen;
struct vmc_output
{
    MatrixXd vmc_Jacobian;
    MatrixXd vmc_F;
    MatrixXd vmc_T;
    vmc_output():vmc_Jacobian(2,2),vmc_F(2,1),vmc_T{2,1}{}
};

class VMC
{
private:
    five_bar_linkage::bar_length *bar_length_;
    five_bar_linkage::five_bar_linkage_param *linkage_calc_date_;
    double J_array_[4]{0};
    vmc_output vmc_output_;
public:
    VMC();
    ~VMC();
    void getDataOfLeg(five_bar_linkage::bar_length* real_leg_data, five_bar_linkage::five_bar_linkage_param* bar_length_calc_date);
    void VMCControllerCalc(void);

};
#endif


