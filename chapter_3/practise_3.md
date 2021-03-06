# 第三章作业

## 1. 群的性质

### 1.1 $\{\mathbb{Z}, + \}$是否为群？

$\{\mathbb{Z}, +\}$ 是群。

(1) 满足封闭性，任意两个整数的和仍是整数

(2)满足结合性，$\forall{a_1, a_2, a_3} \in \mathbb{Z}, (a_1+a_2)+a_3 = a_1+(a_2+a_3)$

(3)具有幺元1

(4)具有逆，任意整数的逆为其相反数

### 1. 2 $\{\mathbb{N}, + \}$是否为群。

$\{\mathbb{N}, + \}$不是群，没有逆。

## 2. 验证向量叉乘的李代数性质

(1) 封闭性。根据向量叉乘的定义， $\forall{X = (x_1, x_2, x_3), Y= (y_1, y_2, y_3)} \in \mathbb{R^3}$
$$
X\times Y = (x_2y_3-y_2x_3, x_3y_1-y_3x_1, x_1y_2-y_2x_1) \in \mathbb{R^3}
$$
(2) 双线性。同样根据向量叉乘定义，
$$
\begin{aligned}
& [aX	+bY, Z] = [(ax_1+by_2, ax_2+by_2, ax_3+by_3)^T, (z_1,z_2, z_3)^T] \\
& =(ax_2z_3+by_2z_3-ax_3z_2-by_3z_2, ax_3z_1+by_3z_1-ax_1z_3-by_1z_3, ax_1z_2+by1z2-ax_2z_1-by_2z_1)^T \\[]
& =a[X,Z]+b[Y,Z]
\end{aligned}
$$
同理可证$[Z, aX+bY] = a[Z,X]+b[Z,Y]$。

(3) 自反性。根据向量叉乘的几何意义，
$$
X \times X = ||X||*||X||*sin(<X,X>) = ||X||*||X||*sin(0) = 0
$$
满足自反性。

(4) 雅可比等价。根据向量叉乘的定义，
$$
\begin{aligned}
& \ \ \ \  [X, [Y,Z]] + [Y,[Z,X]]+{[Z,[X,Y]]} \\
& =X \times(Y  \times Z) +Y\times (Z\times X) + Z \times (X \times Y) \\
& ...\\
& =0
\end{aligned}
$$

## 3. 推导$SE(3)$的指数映射

$se(3)$上的指数映射为
$$
\begin{aligned}
\exp \left(\boldsymbol{\xi}^{\wedge}\right) &=\left[\begin{array}{cc}
\sum_{n=0}^{\infty} \frac{1}{n !}\left(\phi^{\wedge}\right)^{n} & \sum_{n=0}^{\infty} \frac{1}{(n+1) !}\left(\phi^{\wedge}\right)^{n} \rho \\
0^{\mathrm{T}} & 1
\end{array}\right] \\
& \triangleq\left[\begin{array}{cc}
\boldsymbol{R} & \boldsymbol{J} \boldsymbol{\rho} \\
\mathbf{0}^{\mathrm{T}} & 1
\end{array}\right]=\boldsymbol{T}
\end{aligned}
$$
其中R的部分已经由$so(3)$指数映射推导得到，这里不再推导，一下推倒平移部分，主要是$J$的推导，内容如下：
$$
\begin{aligned}
\sum_{n=0}^{\infty} \frac{1}{(n+1) !}\left(\phi^{\wedge}\right)^{n} &=I+\frac{1}{2 !} \theta a^{\wedge}+\frac{1}{3 !} \theta^{2}\left(a^{\wedge}\right)^{2}+\frac{1}{4 !} \theta^{3}\left(a^{\wedge}\right)^{3}+\frac{1}{5 !} \theta^{4}\left(a^{\wedge}\right)^{4} \cdots \\
&=\frac{1}{\theta}\left(\frac{1}{2 !} \theta^{2}-\frac{1}{4 !} \theta^{4}+\cdots\right)\left(a^{\wedge}\right)+\frac{1}{\theta}\left(\frac{1}{3 !} \theta^{3}-\frac{1}{5} \theta^{5}+\cdots\right)\left(a^{\wedge}\right)^{2}+I \\
&=\frac{1}{\theta}(1-\cos \theta)\left(a^{\wedge}\right)+\frac{\theta-\sin \theta}{\theta}\left(a a^{T}-I\right)+I \\
&=\frac{\sin \theta}{\theta} I+\left(1-\frac{\sin \theta}{\theta}\right) a a^{T}+\frac{1-\cos \theta}{\theta} a^{\wedge} \triangleq J
\end{aligned}
$$

## 4. 伴随

对于任意向量$\boldsymbol{v}$,有
$$
\begin{aligned}
(R a)^{\wedge} v &=(R a) \times v \\
&=(R a) \times\left(R R^{-1} v\right) \\
&=R\left[a \times\left(R^{-1} v\right)\right] \\
&=R a^{\wedge} R^{-1} v \\
&=R a^{\wedge} R^{T} v
\end{aligned}
$$
因此有
$$
(R a)^{\wedge} v = R a^{\wedge} R^{T} v
$$
则：
$$
(R a)^{\wedge}  = R a^{\wedge} R^{T} 
$$
另$p = \theta a$,
$$
\begin{aligned}
R \exp \left(\theta a^{\wedge}\right) R^{T} &=R\left(\cos \theta I+(1-\cos \theta) a a^{T}+\sin \theta a^{\wedge}\right) R^{T} \\
&=\cos \theta I+(1-\cos \theta) R a(R a)^{T}+\sin \theta R a^{\wedge} R^{T} \\
&=\cos \theta I+(1-\cos \theta) R a(R a)^{T}+\sin \theta(R a)^{\wedge} \\
&=\exp \left(\theta(R a)^{\wedge}\right) \\
&=\exp \left((R p)^{\wedge}\right)
\end{aligned}
$$

## 5. 轨迹的描绘

### 5.1 $T_{WC}$的物理意义?

$T_{WC}$的物理意义为相机坐标系到世界坐标系的变换。

$T_{WC}$实际描述的是相机坐标系和世界坐标系的相对变换过系，如果以世界坐标系$W$为参考系，那么这个相对变换关系随时间的变化自然就是相机的轨迹。

## 5.2 画出轨迹

添加如下代码：

```c++
ifstream fin(trajectory_file);
    if(!fin) {
        std::cout << "cant open trajectory file" << std::endl;
        return -1;
    }
    while(!fin.eof()) {
        double t, tx, ty, tz, qx, qy, qz, qw;
        fin >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Quaterniond q(qx, qy, qz,qw);
        q.normalize();
        Sophus::SE3d p(q, Eigen::Vector3d(tx, ty, tz));
        poses.push_back(p);
    }
```

## 6. 求轨迹误差

代码如下：

```c++
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
```

结果截图如下：

![rmse](./images/error.png)