#include "warrior_common/lqr.hpp"

VectorXd LQR::control(const VectorXd &x)
{
    return -K * x;
}


void LQR::computeGain()
{
    
}

MatrixXd LQR::care(void)
{
    const size_t dim_x = A.rows();
    // Set Hamilton Matrix
    Eigen::MatrixXd Ham(2*dim_x, 2*dim_x);
    Ham << A, -B*R.inverse()*B.transpose(), -Q, -A.transpose();
    // calc eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);
    if (Eigs.info() != Eigen::Success) abort();
    // extract stable eigenvectors into 'eigvec'
    Eigen::MatrixXcd eigvec(2*dim_x, dim_x);
    int j = 0;
    // store those with negative real number
    for(int i = 0; i < 2*dim_x; ++i){
        if(Eigs.eigenvalues()[i].real() < 0){
            eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2*dim_x, 1);
            ++j;
        }
    }
      // calc S with stable eigen vector matrix
      Eigen::MatrixXcd U(dim_x, dim_x);
      Eigen::MatrixXcd V(dim_x, dim_x);

      U = eigvec.block(0,0,dim_x,dim_x);
      V = eigvec.block(dim_x,0,dim_x,dim_x);

      return (V * U.inverse()).real();

      
}

  Eigen::MatrixXd LQR::calcGainK()
  {
      /**
      * Calculate LQR Gain K
      * Solves Riccati Equation using Arimoto Potter Method
      * 
      * author: Horibe Takamasa
      */

      // https://www.mathworks.com/help/control/ref/lqr.html
      // http://www.kostasalexis.com/lqr-control.html
      Eigen::MatrixXd  S = care();
      return R.inverse() * (B.transpose() * S);
  }
