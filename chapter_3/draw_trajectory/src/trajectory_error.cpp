/*
 * File: trajectory_error.cpp
 * Project: draw_trajectory
 * Created Date: Friday, October 9th 2020, 10:52:16 pm
 * -----
 * Last Modified:
 * Modified By:
 * -----
 * Good Good Study, Day Day Up
 * ----------------------------------------------------------
 */

#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

using Trajectory = std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;

std::string groud_path = "../groundtruth.txt";
std::string estimate_path = "../estimated.txt";

int main()
{
    std::ifstream g_fin(groud_path);
    std::ifstream e_fin(estimate_path);

    if(!g_fin || !e_fin) {
        std::cerr << "trajectory file read error" << std::endl;
        return -1;
    }

    Trajectory g_trj;
    while(!g_fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        g_fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p(Eigen::Quaterniond(qx, qy, qz, qw), Eigen::Vector3d(tx, ty, tz));
        g_trj.push_back(p);
    }
    Trajectory e_trj;
    while(!e_fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        e_fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p(Eigen::Quaterniond(qx, qy, qz, qw), Eigen::Vector3d(tx, ty, tz));
        e_trj.push_back(p);
    }
    if(e_trj.size() == 0 || (e_trj.size() != g_trj.size())) {
        std::cerr << "size error" << std::endl;
        return -1;
    }

    double rmse = 0.0;
    for(size_t i = 0; i < e_trj.size(); i++) {
        double error = (g_trj[i].inverse() * e_trj[i]).log().norm();
        rmse += error * error;
    }
    rmse /= double(e_trj.size());
    rmse = std::sqrt(rmse);
    std::cout << "RMSE is " << rmse << std::endl;

    return 0;
}