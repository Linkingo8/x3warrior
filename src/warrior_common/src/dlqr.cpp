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
    if(system_type_ == CONTINUOUS)
    {
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_state, dim_state);
        Ad = (I + 0.5 * T * A) * (I - 0.5 * T * A).inverse();
        //Ad = (I - 0.5 * T * A).inverse() * (I + 0.5 * T * A);
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
        2000,           0,         0,           0,           0,           0,
           0,          10,         0,           0,           0,           0,
           0,           0,        10,           0,           0,           0,
           0,           0,         0,           1,           0,           0,
           0,           0,         0,           0,         100,           0,
           0,           0,         0,           0,           0,         100;
    #ifdef DLQR_TEST_PRINT_Q
        std::cout << "----------------------------------------- Q -----------------------------------------" << std::endl;
        std::cout << Q << std::endl;
    #endif

    /* R init */
    R << 
           100,     0,
             0,   100;
    #ifdef DLQR_TEST_PRINT_R
        std::cout << "----------------------------------------- R -----------------------------------------" << std::endl;
        std::cout << R << std::endl;
    #endif
    /* ************************************ option end ************************************ */
}

void dlqr::dlqrRun()
{
    Eigen::MatrixXd AdT = Ad.transpose();
    Eigen::MatrixXd BdT = Bd.transpose();

    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd P_1;
    Eigen::MatrixXd P_err;
    int iteration_num = 0;
    double err =   10 * DLQR_TOLERANCE;
    Eigen::MatrixXd::Index maxRow, maxCol;

    while(err > DLQR_TOLERANCE && iteration_num < DLQR_MAX_ITERATION)
    {

        iteration_num++;
        // P_1 = Q + Ad.transpose() * P * Ad - Ad.transpose() * P * Bd * (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
        std::chrono::system_clock::time_point beforeTime_ns = std::chrono::system_clock::now();
        P_1 = Q + AdT * P * Ad - AdT * P * Bd * (R + BdT * P * Bd).inverse() * BdT * P * Ad;
        std::chrono::system_clock::time_point endTime_ns = std::chrono::system_clock::now();
        Eigen::MatrixXd P_err = P_1 - P;
        err = fabs(P_err.maxCoeff(&maxRow, &maxCol));
        double duration = (endTime_ns - beforeTime_ns).count() / 1e6; 
        std::cout << "运行周期" << duration << std::endl;
        P = P_1;
    }

    // if(iteration_num < DLQR_MAX_ITERATION)
    // {
    //     K = (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
    //     #ifdef DLQR_TEST_PRINT_K
    //         std::cout << "----------------------------------------- K -----------------------------------------" << std::endl;
    //         std::cout <<K<< std::endl;
    //         std::cout << "iteration time" << iteration_num << std::endl;
    //     #endif
    // }
    // else
    //     std::cout << "DLQR Solve failed !!!" << std::endl;
    
    #ifdef DLQR_TEST_PRINT_ITERATION
        std::cout << "----------------------------------------- ITERATION -----------------------------------------" << std::endl;
        std::cout << iteration_num << std::endl;
    #endif
    #ifdef DLQR_TEST_PRINT_ULTI_ERROR
        std::cout << "----------------------------------------- ULTI_ERROR -----------------------------------------" << std::endl;
        std::cout << err << std::endl;
    #endif
}