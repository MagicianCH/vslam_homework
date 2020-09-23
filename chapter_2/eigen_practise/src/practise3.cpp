/*
 * File: practise3.cpp
 * Project: eigen_practise
 * Created Date: Wednesday, September 23rd 2020, 9:01:08 pm
 * Author:
 * -----
 * Last Modified:
 * Modified By:
 * -----
 * Good Good Study, Day Day Up
 * ----------------------------------------------------------
 */

/*
设有小萝卜 1 一号和小萝卜二号位于世界坐标系中。小萝卜一号的位姿为: q 1 = [0.55, 0.3, 0.2, 0.2], t 1 =[0.7, 1.1, 0.2] T (q 的第一项为实部)。
这里的 q 和 t 表达的是 T cw ,也就是世界到相机的变换关系。小萝卜二号的位姿为 q 2 = [−0.1, 0.3, −0.7, 0.2], t 2 = [−0.1, 0.4, 0.8] T 。
现在,小萝卜一号看到某个点在自身的坐标系下,坐标为 p 1 = [0.5, −0.1, 0.2] T ,求该向量在小萝卜二号坐标系下的坐标。
*/

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

int main()
{
    Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2);
    Eigen::Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
    q1.normalize();
    q2.normalize();

    Eigen::Vector3d t1(0.7, 1.1, 0.2);
    Eigen::Vector3d t2(-0.1, 0.4, 0.8);

    Eigen::Vector3d p1(0.5, -0.1, 0.2);

    Eigen::Isometry3d T_1w(q1);
    T_1w.pretranslate(t1);
    Eigen::Isometry3d T_2w(q2);
    T_2w.pretranslate(t2);

    // p2 = T_2w * T_w1 * p1
    Eigen::Vector3d p2 = T_2w * T_1w.inverse() * p1;
    std::cout << "p2 is " << std::endl << p2 << std::endl;

    return 0;
}