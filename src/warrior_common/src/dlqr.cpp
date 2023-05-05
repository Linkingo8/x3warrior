#include "warrior_common/dlqr.hpp"
using namespace Eigen;
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

/* constructor */
dlqr::dlqr(Eigen::MatrixXd A, Eigen::MatrixXd B, 
           int dim_state_, int dim_control_, double T_,
           SYSTEM_TYPE system_type_)
{
    dim_state = dim_state_;
    dim_control = dim_control_;
    T = T_;
    Q.resize(dim_state, dim_state);
    R.resize(dim_control, dim_control);

    /* error checking */
    if((A.rows()) != dim_state)
        std::cout << "A is in wrong rows !!!" << std::endl; 
    if((A.cols()) != dim_state)
        std::cout << "A is in wrong cols !!!" << std::endl; 
    if((B.rows()) != dim_state)
        std::cout << "B is in wrong rows !!!" << std::endl; 
    if((B.cols()) != dim_control)
        std::cout << "B is in wrong cols !!!" << std::endl; 

    /* discretization */
    if(system_type_ == DISCRETE)
    {
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_state, dim_state);
        //Ad = (I + 0.5 * T * A) * (I - 0.5 * T * A).inverse();
        Ad = (I - 0.5 * T * A).inverse() * (I + 0.5 * T * A);
        Bd = (I - 0.5 * T * A).inverse() * B * T;
        #ifdef DLQR_APPROXIMATE_MODE
            Bd = B * T;
        #endif
    }
    else
    {
        Ad = A;
        Bd = B;
    }

    #ifdef DLQR_TEST_PRINT_Ad
        std::cout << "----------------------------------------- Ad -----------------------------------------" << std::endl;
        std::cout << (Ad.format(HeavyFmt)) << std::endl;
    #endif
    #ifdef DLQR_TEST_PRINT_Bd
        std::cout << "----------------------------------------- Bd -----------------------------------------" << std::endl;
        std::cout <<(Bd.format(HeavyFmt)) << std::endl;
    #endif

    std::cout << "dlqr Birth Done" << std::endl;
}

/* destructor */
dlqr::~dlqr()
{
    std::cout << "dlqr Die ..." << std::endl;
}

void dlqr::dlqrInit()
{
    /* ************************************ option start ************************************ */
    /* Q init */
    Q << 
         500,           0,         0,           0,           0,           0,
           0,          500,         0,           0,           0,           0,
           0,           0,        500,           0,           0,           0,
           0,           0,         0,           500,           0,           0,
           0,           0,         0,           0,         1500,           0,
           0,           0,         0,           0,           0,         8000;
    #ifdef DLQR_TEST_PRINT_Q
        std::cout << "----------------------------------------- Q -----------------------------------------" << std::endl;
        std::cout << Q << std::endl;
    #endif

    /* R init */
    R << 
           8,     0,
             0,   10;
    #ifdef DLQR_TEST_PRINT_R
        std::cout << "----------------------------------------- R -----------------------------------------" << std::endl;
        std::cout << R << std::endl;
    #endif
    P = Q;
    P_1 = Q;
    err = 10 * DLQR_TOLERANCE;
    /* ************************************ option end ************************************ */
}

void dlqr::dlqrIterRun()
{
    Eigen::MatrixXd Ad_temp = Ad;
    Eigen::MatrixXd Bd_temp = Bd;
    
    Eigen::MatrixXd AdT = Ad.transpose();
    Eigen::MatrixXd BdT = Bd.transpose();

    int iteration_num = 0;
    Eigen::MatrixXd::Index maxRow, maxCol;
    std::chrono::system_clock::time_point beforeTime_ns = std::chrono::system_clock::now();
    while(iteration_num < DLQR_MAX_ITERATION)
    {
        iteration_num++;
        // P_1 = Q + Ad.transpose() * P * Ad - Ad.transpose() * P * Bd * (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
        P_1 = Q + AdT * P * Ad_temp -
             AdT * P * Bd_temp * (R + BdT * P * Bd_temp).inverse() * BdT * P * Ad_temp;
        // Eigen::MatrixXd P_err = P_1 - P;
        // err = fabs(P_err.maxCoeff(&maxRow, &maxCol));
        err = fabs((P_1 - P).maxCoeff());
        P = P_1;
        if(err < DLQR_TOLERANCE)
            return;
    }
    std::chrono::system_clock::time_point endTime_ns = std::chrono::system_clock::now();
    double duration = (endTime_ns - beforeTime_ns).count() / 1e6; 
    std::cout << "运行周期" << duration << std::endl;

    K = (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
    #ifdef DLQR_TEST_PRINT_K
        std::cout << "----------------------------------------- K -----------------------------------------" << std::endl;
        std::cout <<K<< std::endl;
        std::cout << "iteration time" << iteration_num << std::endl;
    #endif
    
    
    #ifdef DLQR_TEST_PRINT_ITERATION
        std::cout << "----------------------------------------- ITERATION -----------------------------------------" << std::endl;
        std::cout << iteration_num << std::endl;
    #endif
    #ifdef DLQR_TEST_PRINT_ULTI_ERROR
        std::cout << "----------------------------------------- ULTI_ERROR -----------------------------------------" << std::endl;
        std::cout << err << std::endl;
    #endif
}

void dlqr::dlqrArimotoPotterRun()
{
    std::chrono::system_clock::time_point beforeTime_ns = std::chrono::system_clock::now();
    const uint dim_x = Ad.rows();
    const uint dim_u = Bd.cols();

    // set Hamilton matrix
    Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    Ham <<  Ad
            ,-Bd * R.inverse() * Bd.transpose()
            ,-Q
            ,-Ad.transpose();

    // calc eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);

    // check eigen values
    // std::cout << "eigen values：\n" << Eigs.eigenvalues() << std::endl;
    // std::cout << "eigen vectors：\n" << Eigs.eigenvectors() << std::endl;

    // extract stable eigenvectors into 'eigvec'
    Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dim_x, dim_x);
    int j = 0;
    for (int i = 0; i < 2 * dim_x; ++i) {
        if (Eigs.eigenvalues()[i].real() < 0.) {
        eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
        ++j;
        }
    }

    // calc P with stable eigen vector matrix
    Eigen::MatrixXcd Vs_1, Vs_2;
    Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
    Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
    P = (Vs_2 * Vs_1.inverse()).real();
    
    K = R.inverse() * (Bd.transpose() * P);
    #ifdef DLQR_TEST_PRINT_K
        std::cout << "----------------------------------------- K -----------------------------------------" << std::endl;
        std::cout <<K<< std::endl;
    #endif
    std::chrono::system_clock::time_point endTime_ns = std::chrono::system_clock::now();
    double duration = (endTime_ns - beforeTime_ns).count() / 1e6; 
    // std::cout << "运行周期" << duration << std::endl;
}