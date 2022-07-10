#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <opencv2/core/core.hpp>

#include "tool_fun.h"
#include "data_tpye.h"

//#include "ArmourFinder.h"
//#include "ArmourFinder.h"
using namespace cv;
using namespace std;
using namespace ceres;

#include "particle_filter.h"


//#define PLOT_FITTING

extern ParticleFilter pf;
extern ParticleFilter pf_param_loader;
const string pf_path = "../config/filter_param.yaml";

extern Mat plot_all;
class SolvePara {


public:

    bool is_sine_found = false;      //拟合正弦关系是否完成标志位
    bool is_phi_start = false;       //拟合初相位是否start标志位
    bool is_phi_found = false;       //拟合初相位是否完成标志位
    long long start_time = 0;        //预测的起始时间
    //待优化参数
    static double params_fitting[4];
    static double PHI[1];


    ///pos_fun C取0时角速度函数的积分
    ///\param t 自变量t
    ///\param phi 相位
    ///\return 角度
    double pos_fun(float t){
        return ( params_fitting[3] * t ) - ( params_fitting[0] / params_fitting[1] * cos(params_fitting[1] * t + PHI[0] ));
    }


    const int rows = 1000;
    const int cols = 600;
    Mat plot =Mat::zeros(rows,cols, CV_8UC3);
    Mat plot_phi =Mat::zeros(rows,300, CV_8UC3);
    int cnt = 0;
    void solvePara(vector<time_angle>& spd){


        if(!is_sine_found){sample_num = sin_sample_num;}
                     else {sample_num = phi_sample_num;}
        int start_idx = spd.size()-sample_num-1;
        if(start_idx<=0){return;}
        is_phi_found = false;
        fit_start_time = spd[start_idx].x;  //记录起始时间
        /// 构建最小二乘问题
        ceres::Problem sineProblem;
        ceres::Problem phiProblem;

//        cv::FileStorage spd_write_1 ("../spd_1_50.yaml", cv::FileStorage::WRITE);
//        cv::FileStorage spd_write_2 ("../spd_2_50.yaml", cv::FileStorage::WRITE);
//        cv::FileStorage diff_write_1 ("../diff_1.yaml", cv::FileStorage::WRITE);
//        cv::FileStorage diff_write_2 ("../diff_2.yaml", cv::FileStorage::WRITE);

        for (int i = start_idx; i != (spd.size()-1); i++) {

            //距离起始时间差 ns
            double cur_time = (double (spd[i].x - fit_start_time)/1000);   //s
            //相邻两帧之间的时间差 ns
            tao = (double(spd[i].x - spd[i-1].x)/1000);                 //s

            if(!is_sine_found){
                /// 向问题中添加误差项,通过循环添加了N个误差项
                sineProblem.AddResidualBlock(
                        // 使用自动求导，模板参数：误差类型，输出维度r，s_1
                        new ceres::AutoDiffCostFunction<CURVE_SINE_FITTING_COST, 1, 4>(
                                new CURVE_SINE_FITTING_COST(cur_time, spd[i].y)
//                                new CURVE_SINE_FITTING_COST(cur_time, cur_spd)
                        ),
//                        new ceres::CauchyLoss(2),            // 核函数
                        new ceres::CauchyLoss(1),            // 核函数
                        params_fitting                // 待估计参数
                );
            }

            if(!is_phi_found && is_sine_found){
                phiProblem.AddResidualBlock(
                        // 使用自动求导，模板参数：误差类型，输出维度r，s_1
                        new ceres::AutoDiffCostFunction<CURVE_PHI_FITTING_COST, 1, 1>(
                                new CURVE_PHI_FITTING_COST(cur_time, spd[i].y, params_fitting[0], params_fitting[1], params_fitting[3])
//                                    new CURVE_PHI_FITTING_COST(cur_time, spd[i].y, tao)
                        ),
                        new ceres::CauchyLoss(2),
                        PHI                // 待估计参数
                );
                is_phi_start = true;
            }
        }

        if(!is_sine_found){
            ///配置求解器
            ceres::Solver::Options sine_options;                 // 这里有很多配置项可以填
//            sine_options.linear_solver_type = ceres::DENSE_QR;   // 增量方程如何求解

            ///设置取值范围
            sineProblem.SetParameterLowerBound(params_fitting,0,0.75);
            sineProblem.SetParameterUpperBound(params_fitting,0,1.1);
            sineProblem.SetParameterLowerBound(params_fitting,1,1.75);
            sineProblem.SetParameterUpperBound(params_fitting,1,2.1);
            sineProblem.SetParameterLowerBound(params_fitting,2,0);
            sineProblem.SetParameterUpperBound(params_fitting,2,CV_2PI);
            sineProblem.SetParameterLowerBound(params_fitting,3,0.5);
            sineProblem.SetParameterUpperBound(params_fitting,3,2.2);

            ceres::Solver::Summary sineSummary;                 // 优化信息
            ceres::Solve(sine_options, &sineProblem, &sineSummary);     // 开始优化
//            cout<<sineSummary.FullReport()<<endl;
        }


        //测试拟合效果
        if(!is_sine_found){
            float RMSE = evalRMSE(spd,params_fitting[2]);
            if (abs(2.090-params_fitting[3]-params_fitting[0])<0.2 && (  params_fitting[1] > 1.78 && params_fitting[1] < 2.1 ) && RMSE<0.45){
                //拟合成功
                is_sine_found = true;
                cout << "---------------------ERR---------------\n" << 2.090-params_fitting[3]-params_fitting[0] << endl;
                cout << "---------------------A--T---------------\n" << params_fitting[0] << endl;
                cout << "---------------------W--T---------------\n" << params_fitting[1] << endl;
                cout << "---------------------phi--T---------------\n" << params_fitting[2] << endl;
                cout << "---------------------B--T---------------\n" << params_fitting[3] << endl;

#ifdef PLOT_FITTING
                int start_idx = spd.size()-sample_num-1;
                fit_start_time = spd[start_idx].x;  //记录起始时间
                plot =Mat::zeros(rows,cols, CV_8UC3);
                for (int i = start_idx; i != (spd.size()-1); i++) {

                    //距离起始时间差 ns
                    double cur_time = (double (spd[i].x - fit_start_time)/1000);   //s
                    //相邻两帧之间的时间差 ns
                    tao = (double(spd[i].x - spd[i-1].x)/1000);                 //s
                    auto  value = params_fitting[0] * ceres::sin(params_fitting[1] * cur_time + params_fitting[2]) + params_fitting[3];

                    ////记录下每帧
                    cv::circle(plot,Point2f(fmod(cnt,cols), rows-(spd[i].y/5*rows)),1, Scalar(0,255,0));
                    cv::circle(plot,Point2f(fmod(cnt,cols), rows-(value/5*rows)),1, Scalar(0,0,255));
                    cnt++;
                    if(fmod(cnt,cols) == 0){plot =Mat::zeros(rows,cols, CV_8UC3);}
                    imshow("plot_test_ture",plot);
//                imshow("plot_2",plot_2);
                    waitKey(1);
                }
#endif

            }else{
//                cout << "fit err!" << 2.090 - AWB[2] - AWB[0] << endl;
                cout << "---------------------ERR---------------\n" << 2.090-params_fitting[3]-params_fitting[0] << endl;
                cout << "---------------------A--F---------------\n" << params_fitting[0] << endl;
                cout << "---------------------W--F---------------\n" << params_fitting[1] << endl;
                cout << "---------------------phi-F----------------\n" << params_fitting[2] << endl;
                cout << "---------------------B--F---------------\n" << params_fitting[3] << endl;

#ifdef PLOT_FITTING
                int start_idx = spd.size()-sample_num-1;
                fit_start_time = spd[start_idx].x;  //记录起始时间
                plot =Mat::zeros(rows,cols, CV_8UC3);
                for (int i = start_idx; i != (spd.size()-1); i++) {

                    //距离起始时间差 ns
                    double cur_time = (double (spd[i].x - fit_start_time)/1000);   //s
                    //相邻两帧之间的时间差 ns
                    tao = (double(spd[i].x - spd[i-1].x)/1000);                 //s
                    auto  value = params_fitting[0] * ceres::sin(params_fitting[1] * cur_time + params_fitting[2]) + params_fitting[3];

                    ////记录下每帧
                    cv::circle(plot,Point2f(fmod(cnt,cols), rows-(spd[i].y/5*rows)),1, Scalar(0,255,0));
                    cv::circle(plot,Point2f(fmod(cnt,cols), rows-(value/5*rows)),1, Scalar(0,0,255));
                    cnt++;
                }
                if(fmod(cnt,cols) == 0){plot =Mat::zeros(rows,cols, CV_8UC3);}
                imshow("plot_test_false",plot);
//                imshow("plot_2",plot_2);
                waitKey(1);
#endif

            }
        }

        if(!is_phi_found && is_sine_found && is_phi_start ){
            PHI[0] =CV_PI;
            /// 配置求解器
            ceres::Solver::Options phi_options;                 // 这里有很多配置项可以填

            //设置上下限
            phiProblem.SetParameterLowerBound(PHI,0,0);
            phiProblem.SetParameterUpperBound(PHI,0,CV_2PI);
            ceres::Solver::Summary phiSummary;                 // 优化信息
            ceres::Solve(phi_options, &phiProblem, &phiSummary);     // 开始计算

            float RMSE = evalRMSE(spd,PHI[0]);
            if(RMSE<0.5)
            {is_phi_found = true;}

//            cout<<"-----------phi---------------\n"<<PHI[0]<<endl;
#ifdef PLOT_FITTING
            int start_idx = spd.size()-sample_num-1;
            fit_start_time = spd[start_idx].x;  //记录起始时间
            plot_phi =Mat::zeros(rows,cols, CV_8UC3);
            for (int i = start_idx; i != (spd.size()-1); i++) {

                //距离起始时间差 ns
                double cur_time = (double (spd[i].x - fit_start_time)/1000);   //s
                auto  value = params_fitting[0] * ceres::sin(params_fitting[1] * cur_time + PHI[0]) + params_fitting[3];

                ////记录下每帧
                cv::circle(plot_phi,Point2f(fmod(cnt,cols), rows-(spd[i].y/5*rows)),1, Scalar(0,255,0));
                cv::circle(plot_phi,Point2f(fmod(cnt,cols), rows-(value/5*rows)),1, Scalar(0,0,255));
                cnt++;
                if(fmod(cnt,cols) == 0){plot_phi =Mat::zeros(rows,cols, CV_8UC3);}
                putText(plot_phi, to_string(RMSE) ,Point(100,100),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2);
                imshow("plot_phi",plot_phi);
                waitKey(1);
            }
#endif

//            cout<<phiSummary.FullReport()<<endl;
            start_time = fit_start_time;
        }
    }

    double evalRMSE(vector<time_angle>& spd , double phi)
    {
        double rmse_sum = 0;
        double rmse = 0;
        int start_idx = spd.size()-sample_num-1;
        fit_start_time = spd[start_idx].x;  //记录起始时间
        for (int i = start_idx; i != (spd.size()-1); i++) {

            //距离起始时间差 ns
            double cur_time = (double (spd[i].x - fit_start_time)/1000);   //s
            //相邻两帧之间的时间差 ns
            tao = (double(spd[i].x - spd[i-1].x)/1000);                 //s
            auto  value = params_fitting[0] * ceres::sin(params_fitting[1] * cur_time + phi) + params_fitting[3];
            rmse_sum +=pow((value - spd[i].y),2);

        }
        rmse = sqrt(rmse_sum / sample_num);
        return rmse;

    }

private:

    int sample_num = 0;
    const int sin_sample_num = 200;             //参数拟合使用的样本数目
    const int phi_sample_num = 50;             //相位拟合

    double tao = -1;                 //相邻两帧时间差
    long long fit_start_time = 0;    //采样开始时间

//    // 代价函数的计算模型 q=1,s_1=3  a*sin(w*t)+b
//    struct CURVE_SINE_FITTING_COST
//    {
//        CURVE_SINE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
//        // 残差的计算
//        template <typename T>
//        bool operator() (
//            const T* const awb,     // 模型参数，有3维
//            T* residual ) const     // 残差
//        {
//            auto  value = ( awb[0]* sin(awb[1] * T ( _x )) + awb[2] );
//            residual[0] = T ( _y ) - value;
//            return true;
//        }
//        const double _x, _y;    // x,y数据
//    };

    // 代价函数的计算模型 q=1,s_1=4  a*sin(w*t+θ)+b
    struct CURVE_SINE_FITTING_COST
    {
        CURVE_SINE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
        // 残差的计算
        template <typename T>
        bool operator() (
                const T*  params,     // 模型参数，有4维
                T* residual ) const     // 残差
        {
            // f(x) = a * sin(ω * t + θ) + b
            auto  value = params[0] * ceres::sin(params[1] * T (_x) + params[2]) + params[3];
            residual[0] = T ( _y ) - value;
            return true;
        }
        const double _x, _y;    // x,y数据
    };

    // 代价函数的计算模型 q=1,s_1=1  a*sin(w*t+θ)+b
    struct CURVE_PHI_FITTING_COST
    {
        CURVE_PHI_FITTING_COST (double x, double y, double A, double W, double B) : _x (x), _y (y), _A(A), _W(W), _B(B){}

        // 残差的计算
        template <typename T>
        bool operator() (
                const T* const phi,     // 模型参数，有1维
                T* residual ) const     // 残差
        {
            // f(x) = a * sin(ω * t + θ) + b
            auto  value = T (_A) * ceres::sin(T(_W) * T (_x) + phi[0]) + T(_B);
            residual[0] = T ( _y ) - value;
            return true;
        }
        const double _x, _y, _A, _W, _B;
    };

//    // 代价函数的计算模型 相位计算
//    struct CURVE_PHI_FITTING_COST
//    {
//        CURVE_PHI_FITTING_COST ( double x, double y, float tau ) : _x ( x ), _y ( y ), _tau(tau) {}
//        // 残差的计算
//        float _tau;
//        template <typename T>
//        bool operator() (
//            const T* const phi,     // 模型参数，有1维
//            T* residual ) const     // 残差
//        {
//            //B*tau+A/W(-cos(phi+W*t)+cos(phi+W(-tao+t)))
//            auto value = params_fitting[3]*_tau + params_fitting[0]/params_fitting[1]*(-ceres::cos(phi[0]+(params_fitting[1]*_x)) + ceres::cos(phi[0]+params_fitting[1]*(-_tau+_x)));
//            residual[0] = T ( _y ) - value;
//            return true;
//        }
//        const double _x, _y;    // x,y数据
//    };
};



