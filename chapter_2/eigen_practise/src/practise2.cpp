/*
 * File: pra
 * Project: src
 * Created Date: Saturday, September 26th 2020, 7:57:30 pm
 * Author: Chen Hao (hao.chen@cloudminds.com)
 * -----
 * Last Modified:
 * Modified By:
 * -----
 * Copyright (c) 2020 CloudMinds
 *
 * Good Good Study, Day Day Up
 * ----------------------------------------------------------
 */

#include <iostream>
#include <Eigen/Dense>

int main()
{
    Eigen::Matrix<double, 100, 100> A = Eigen::MatrixXd::Random(100, 100);
    A = A * A.transpose();

    Eigen::Matrix<double, 100, 1> x;
    Eigen::Matrix<double, 100, 1> b = Eigen::MatrixXd::Random(100, 1);

    // QR
    x = A.colPivHouseholderQr().solve(b);
    std::cout << "QR" << std::endl;
    std::cout << x << std::endl;

    // Cholesky
    x = A.ldlt().solve(b);
    std::cout << std::endl << "Cholesky" << std::endl;
    std::cout << x << std::endl;

    return 0;
}