//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct Pose {
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};

int main() {

    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for (int n = 0; n < poseNums; ++n) {
        double theta = n * 2 * M_PI / (poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R, t));
    }

    // 随机数生成 1 个 三维特征点（gt）
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3

    ///提升2： 将start_frame_id设置为可变量
    for(int start_frame_id = 0; start_frame_id<= poseNums -3; start_frame_id++) {
        //int start_frame_id = 3;
        int end_frame_id = poseNums;

        ///提升1： 加入测量噪声
        double n = 0.2;
        //while(n<=2.0){
        std::normal_distribution<double> noise_pdf(0., n / 1000);
        for (int i = start_frame_id; i < end_frame_id; ++i) {   //一共10帧 序号0-9
            Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
            Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

            double x = Pc.x();
            double y = Pc.y();
            double z = Pc.z();

            camera_pose[i].uv = Eigen::Vector2d(x / z, y / z);    //观测到的在图像上的位置，通过图像位置复原出深度
            camera_pose[i].uv[0] += noise_pdf(generator);  //加噪音
            camera_pose[i].uv[1] += noise_pdf(generator);  //加噪音
        }

        /// TODO::homework; 请完成三角化估计深度的代码
        // 遍历所有的观测数据，并三角化
        //由于fx, fy =1.0，内参矩阵为I
        //相当于现在的相机归一化坐标就是图像坐标系下的观测值

        int size = end_frame_id - start_frame_id; //每一次观测对应2行
        Eigen::MatrixXd D(size * 2, 4);

        for (int j = start_frame_id; j < end_frame_id; j++) {
            //世界到相机坐标系的t,得到每一个观测的投影矩阵P
            Eigen::Matrix3d r_cw = camera_pose[j].Rwc.transpose();
            Eigen::Vector3d t_cw = -camera_pose[j].Rwc.transpose() * camera_pose[j].twc;
            Eigen::MatrixXd P(3, 4);
            P.block<3, 3>(0, 0) = r_cw;
            P.block<3, 1>(0, 3) = t_cw;
            double u = camera_pose[j].uv[0];
            double v = camera_pose[j].uv[1];
            D.block<1, 4>(2 * (j - start_frame_id), 0) = u * P.block<1, 4>(2, 0) - P.block<1, 4>(0, 0);
            D.block<1, 4>(2 * (j - start_frame_id) + 1, 0) = v * P.block<1, 4>(2, 0) - P.block<1, 4>(1, 0);
        }
        // 遍历所有的观测数据，并三角化
        Eigen::Vector3d P_est;           // 结果保存到这个变量
        P_est.setZero();
        /* your code begin */
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(D.transpose() * D, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix4d V = svd.matrixV();
        P_est = V.block<3, 1>(0, 3) / V(3, 3);

        //最小奇异值和第二小奇异值比值
        std::cout << "====================================" << std::endl;
        //std::cout << "noise = " << n << "/1000" << std::endl;
        std::cout << "startFrame: " << start_frame_id << std::endl;
        std::cout << "minimal singular value is " << svd.singularValues()[3] << std::endl;
        std::cout << "second smallest singular value is " << svd.singularValues()[2] << std::endl;
        double proportion = svd.singularValues()[3] / svd.singularValues()[2];
        std::cout << "Proportion:  smallest / second smallest = " << proportion << std::endl;

        /* your code end */

        std::cout << "ground truth: \n" << Pw.transpose() << std::endl;
        std::cout << "your result: \n" << P_est.transpose() << std::endl;

        //n += 0.2;
        // TODO:: 请如课程讲解中提到的判断三角化结果好坏的方式，绘制奇异值比值变化曲线
        //}  //提升1
    } //提升2

    return 0;
}
