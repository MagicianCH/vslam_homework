# 第二章作业

## 1. 熟悉Eigen矩阵运算

设线性方程组 $ Ax = b $，$A$为方阵，设其阶数为$n$，增广矩阵为$B = (A|b)$

### 1.1 什么条件下，$x$有唯一解

$rank(A) = rank(B) = n$, 即A和B的秩相等且为n

### 1.2 高斯消元法的原理是什么?

高斯消元法首先对增广矩阵进行初等行变换，把B变成一个三角矩阵，然后从后往前依次带入得到最终解。

### 1.2 QR分解的原理是什么？



## 2. 几何运算练习

设有小萝卜一号和小萝卜二号位于世界坐标系中。

小萝卜一号的位姿为: $q_1 = [0.55, 0.3, 0.2, 0.2], t_1 =
[0.7, 1.1, 0.2]^T$ (q 的第一项为实部)。这里的 q 和 t 表达的是 $T_{cw}$,也就是世界到相机的变换关系。

小萝卜二号的位姿为 $q_2 = [−0.1, 0.3, −0.7, 0.2], t_2 = [−0.1, 0.4, 0.8]^T$。

现在,小萝卜一号看到某个点在自身的坐标系下,坐标为 p 1 = [0.5, −0.1, 0.2] T ,求该向量在小萝卜二号坐标系下的坐标。

**主要代码如下：**

```c++
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
```

**运行结果如下：**

![practise3](./images/practise3.png)

## 3. 旋转的表达

### 3.1 设有旋转矩阵 $R$,证明 $R^TR = I$ 且$det R = +1$ 

由于旋转矩阵都是正交矩阵，因此$R^T = R^{-1}$，故$R^TR = R^{-1}R = I$。

设$R$把正交基$I$转为$I^{'}$，即$I^{'} = RI$, 由于$detI = detI^{'} = 1$, 故$detR = 1$

### 3.2 设有四元数 q,我们把虚部记为 ε,实部记为 η,那么 q = (ε, η)。请说明 ε 和 η 的维度

$\epsilon$为3维，$\eta$为标量。

### 3.3 证明

定义$q^{+}$和$q^{\oplus}$为
$$
q^{+} =  \left[
\matrix{
  \eta1+ \epsilon^{\times} & \epsilon\\
  -\epsilon^{T} & \eta\\
}
\right], 
q^{\oplus} =  \left[
\matrix{
  \eta1- \epsilon^{\times} & \epsilon\\
  -\epsilon^{T} & \eta\\
}
\right]
$$
其中运算$\times$和$\wedge$相同，即取$\epsilon$的反对称矩阵(它们都成叉积的矩阵运算形式),1 为单位矩阵。请证明对任意单位四元数 q 1 , q 2 ,四元数乘法可写成矩阵乘法:
$$
q_1q_2 = q_1^{+}q_2
$$
或者
$$
q_1q_2 = q_2^{\oplus}q_1
$$
证明如下：



## 4. 罗德里格斯公式证明

罗德里格斯公式定义为:
$$
R = cos{\theta}I + (1 - cos{\theta})nn^T + sin{\theta}n^{\wedge}
$$
证明如下：



## 5. 四元数运算性质的验证



## 6. 熟悉C++11

```c++
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class A {
public:
	A(const int& i ) : index(i) {}
	int index = 0;
};

int main() {
    A a1(3), a2(5), a3(9);
    vector<A> avec{a1, a2, a3};
    std::sort(avec.begin(), avec.end(), [](const A&a1, const A&a2) {return a1.index<a2.index;});
    for ( auto& a: avec ) cout<<a.index<<" ";
    cout<<endl;
    return 0;
}
```

第15行用到了lambda表达式，`std::sort`函数中传入一个自定义比较函数。

第16行用到范围for循环，其中还用到auto自动类型推倒。