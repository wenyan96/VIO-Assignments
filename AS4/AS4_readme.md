# AS4

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled.png)

![AS4%2054b6699fe329471baad5a7c11fb69b9f/_05-28-2021_22.48_2.jpg](AS4%2054b6699fe329471baad5a7c11fb69b9f/_05-28-2021_22.48_2.jpg)

![AS4%2054b6699fe329471baad5a7c11fb69b9f/_05-28-2021_22.48_1.jpg](AS4%2054b6699fe329471baad5a7c11fb69b9f/_05-28-2021_22.48_1.jpg)

如图所示,最左边为原信息矩阵, 最右边为相机姿态1被边缘化后的信息矩阵(紫+红)

可以看到,边缘化后L1和L2之间产生了关联. 根据课上提到的内容,在VSLAM中应该尽量避免这种情况, 为了保持稀疏性,会丢弃一些观测. 所以可以丢弃掉相机姿态1上对L1和L2的观测, 这样得到的信息矩阵就为最右侧紫色记号笔的部分

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%201.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%201.png)

https://wiseodd.github.io/techblog/2018/03/11/fisher-information/

目的: 明确Fisher Information Matrix, 协方差矩阵, Hessian矩阵之间的关系

### Fisher Information Matrix

假定有一个通过参数向量θ参数化的模型, 它描述了一个概率分布p(x|θ). 我们学习θ的方法是使与θ有关的似然概率p(x|θ)最大化. 为了评价对θ估计的好坏,会定义一个score function:

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%202.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%202.png)

这个score function是对数似然函数的梯度.

那么该模型得分的期望值是0

Proof: 

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%203.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%203.png)

该模型得分的协方差为: 

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%204.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%204.png)

这个协方差(上式)可以被看作一种信息,而且,这个式子就是Fisher Information的定义,即**得分函数的协方差**. 由于我们假设θ为一向量,所以Fisher Information以矩阵形式存在,及Fisher Information Matrix

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%205.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%205.png)

但是, 似然函数通常会十分复杂,难以计算出期望值, 所以可以使用经验分布来近似F中的期望值,.下式被成为经验Fisher

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%206.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%206.png)

Empirical distribution ^q(x), which is given by training data X={x1,x2,⋯,xN}

### Fisher and Hessian

对数似然函数Hessian的负期望等于Fisher信息矩阵F。

**Proof:** 

对数似然的Hessian是通过对它的梯度求Jacobian得出

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%207.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%207.png)

第二行利用了分数的求导法则

对该Hessian求期望值

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%208.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%208.png)

为什么第一部分等于0?

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%209.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%209.png)

Di,j表示二阶偏导:

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2010.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2010.png)

所以,可得: **对数似然Hessian的期望*(-1) = F**

**小结:**

Fisher信息矩阵被定义为得分函数的协方差。它是一个曲率(求了二阶导)矩阵，可以解释为对数似然的Hessian矩阵的期望取负值(negative expected Hessian of log likelihood function)。因此，F的直接应用是在二阶优化方法中替代H。

⇒ ⇒ ⇒

负对数似然的Hessian矩阵的期望就是F矩阵(这个负是加在上式H的下标)

论文relationship between the hessian and covariance matrix for gaussian random variables

### Hessian and Covariance

考虑一个**高斯随机向量θ**,  θ*为均值,可以显出概率密度函数为:

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2011.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2011.png)

将目标函数objective function定义为负对数形式:

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2012.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2012.png)

可以看到,这个目标函数为θ的二次函数. 所以求二阶偏导后,可以得到Hessian矩阵,如下图. 从此处,可以看出负对数Hessian矩阵等于对应协方差矩阵的逆.

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2013.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2013.png)

对于高斯随机变量, **目标函数的二阶导对θ都是常数(θ的二次函数)**. 此外,Hessian在不知道mean vector θ*时就可以被求出来.

### Summary

对于符合高斯分布的变量,取概率密度函数的负对数作为目标函数a(前提), 其信息矩阵和协方差的逆之间的关系如下:

- 首先, 对于任一种概率分布,信息矩阵等于目标函数a的Hessian(二阶导)的期望值;
- 在满足高斯分布的情况下, 目标函数是变量的二次函数,所以其二阶导一定为常数, 也就是说,**信息矩阵等于目标函数a的Hessian矩阵**
- 对目标函数a, 本来就有Hessian等于协方差的逆. 所以最终可以推出: **信息矩阵= 协方差矩阵的逆**

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2014.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2014.png)

代码修改如下:

```cpp
for (int i = 0; i < poseNums; ++i) {
            Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();  
            Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);   //transform the feature points from world coord to cam coord

            double x = Pc.x();
            double y = Pc.y();
            double z = Pc.z();
            double z_2 = z * z;
            Eigen::Matrix<double,2,3> jacobian_uv_Pc;
            jacobian_uv_Pc<< fx/z, 0 , -x * fx/z_2,
                    0, fy/z, -y * fy/z_2;
            Eigen::Matrix<double,2,3> jacobian_Pj = jacobian_uv_Pc * Rcw;
            Eigen::Matrix<double,2,6> jacobian_Ti;
            jacobian_Ti << -x* y * fx/z_2, (1+ x*x/z_2)*fx, -y/z*fx, fx/z, 0 , -x * fx/z_2,
                            -(1+y*y/z_2)*fy, x*y/z_2 * fy, x/z * fy, 0,fy/z, -y * fy/z_2;

            //for the whole H, the first 6*PoseNums* 6*PoseNums elements are for camera pose
            H.block(i*6,i*6,6,6) += jacobian_Ti.transpose() * jacobian_Ti;
            // TODO: 请补充完整作业信息矩阵块的计算
	    H.block(6*poseNums+3*j, 6*poseNums+3*j, 3,3) += jacobian_Pj.transpose() * jacobian_Pj;
	    H.block(i*6, 6*poseNums+3*j, 6, 3) += jacobian_Ti.transpose() * jacobian_Pj;
            H.block(6*poseNums + 3*j, i*6, 3, 6) += jacobian_Pj.transpose() * jacobian_Ti;
        }
    }
```

输出结果: 奇异值分解之后,最后7维的结果接近0, 即零空间的维度为7

![AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2015.png](AS4%2054b6699fe329471baad5a7c11fb69b9f/Untitled%2015.png)

```cpp
jacobian_uv_Pc<< fx/z, 0 , -x * fx/z_2,
                    0, fy/z, -y * fy/z_2;
jacobian_Pj = jacobian_uv_Pc * Rcw;           
jacobian_Ti << -x* y * fx/z_2, (1+ x*x/z_2)*fx, -y/z*fx, fx/z, 0 , -x * fx/z_2,
               -(1+y*y/z_2)*fy, x*y/z_2 * fy, x/z * fy, 0,fy/z, -y * fy/z_2;
```

**这两个雅克比的求解过程: 参考<slam十四讲> S186**

jacobian_uv_Pc表示像素平面的误差对相机坐标系下的空间点P'坐标x'y'z'求导. u=fx * x' / z' + c_x;  v= fy * y' / z' + c_y

jacobian_Pj 表示像素平面的误差对世界坐标系下的空间点P坐标xyz求导   因为P' = RP + t, P'对P求导为R, 再由链式法则可得到结果

jacobian_Ti 表示 像素坐标误差对相机位姿求导