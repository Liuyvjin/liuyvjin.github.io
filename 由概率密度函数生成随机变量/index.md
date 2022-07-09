# 由概率密度函数生成随机变量


<!--more-->

# 1. 问题引入
在编程时，标准库往往只会提供生成均匀分布随机变量的函数，但实际问题中，经常会遇到非均匀分布的随机变量。

例如下述问题：
{{< admonition type=question title="478. 在圆内随机生成点" open=true >}}
给定圆的半径和圆心的位置，实现函数 `randPoint`，在圆中产生均匀随机点。
实现 `Solution` 类：

* `Solution(double radius, double x_center, double y_center)` 用圆的半径 `radius` 和圆心的位置 `(x_center, y_center)` 初始化对象。
* `randPoint()` 返回圆内的一个随机点。圆周上的一点被认为在圆内。答案作为数组返回 `[x, y]` 。

来源：[力扣（LeetCode）](https://leetcode.cn/problems/generate-random-point-in-a-circle)
{{< /admonition >}}

对于本题，容易想到极坐标，随机产生半径 $R$ 和角度 $\theta$ 。其中随机变量 $\theta$ 是符合均匀分布的，直接可以用连续均匀分布类 `uniform_real_distribution` 生成。

但随机变量 $R$ 不符合均匀分布，直观来看，在圆的面积中，同一半径的点出现在一个圆周上，因此半径 $x$ 出现的概率应该是周长除以面积。据此可写出**概率密度函数** $f(x)$ 如下，其中 $r$ 为最大半径：

$$f(x) = \frac{2\pi x }{ \pi r^2} = \frac{2x }{ r^2}$$

顺便计算一下**概率分布函数** $F(x)$，后面推导要用:

$$F(x)=\int_{0}^{x}f(\xi)d\xi = \frac{ x^2 }{ r^2} $$

因此本题考察的问题实质是：**如何用标准库提供的均匀分布，生成符合概率分布函数 $\bold{F(x)}$ 或概率密度函数 $f(x)$ 的随机分布。**

# 2. 随机变量的生成方法

简单介绍两种通用方法：

## 2.1 舍选抽样法
<div style="text-align: center">
<img src="https://pic.leetcode-cn.com/1654366052-ueMIcC-image.png" width="70%" />
</div>

也就是说，我们可以按照均匀分布产生一个 $x_0\in [0,\ r]$，接着再按均匀分布产生一个 $y\in [0,\ f_{max}(x)]$，根据 $y$ 和概率密度函数 $f(x_0)$ 的关系来决定是否保留本次产生的 $x_0$，即：
* 若 $y \leq f(x_0)$，保留 $x_0$
* 若 $y > f(x_0)$，舍去 $x_0$

因为 $y$ 是均匀分布，因此 $x_0$ 能被保留的概率恰好是 $f(x_0)$，即这样产生的随机变量符合概率密度函数 $f(x)$ 。参考代码见最后。

## 2.2 反变换法

若已知随机变量的概率分布函数 $F(x)$，$y$ 是 $[0,1]$ 区间的均匀分布随机数， 则具有概率分布函数 $F(x)$ 的随机数可以用下式得到：

$$x=F^{-1}(y)$$

简单来说，要得到随机分布可以按照以下步骤：

* (1) 求出概率分布函数 $F(x)$
* (2) 求概率分布函数的反函数 $F^{-1}(x)$
* (3) 把 $[0,1]$ 区间内均匀分布的随机数代入 $x = F^{-1}(y)$

**证明：**
根据定义：
$$P(\ y\leq F(x_0)\ ) = F(x_0) = P(\ x\leq x_0\ )$$
因为 $F(x_0)$ 是单调递增的函数，因此其反函数也单调递增。所以 $y\leq F(x_0)$ 两边同时施加反函数，大小关系不变：
$$P(\ F^{-1}(y) \leq x_0\ ) = P(\ y\leq F(x_0)\ ) = P(\ x\leq x_0\ )$$
比较左右两式有：
$$F^{-1}(y) = x$$

**本题的应用：**
根据上述方法，本题的概率分布函数为：
$$F(x)= \frac{ x^2 }{ r^2} $$

其反函数为：
$$F^{-1}(y)= \sqrt{ y\times r^2} = r\sqrt{y}$$


# 3. 参考代码
**反变换法**
```cpp
class Solution {
public:
    double x_c, y_c, radius2;
    double _2pi = 4*acos(0.0);
    default_random_engine generator;
    uniform_real_distribution<double> distribution;

    Solution(double radius, double x_center, double y_center):y_c(y_center), x_c(x_center) {
        radius2 = radius * radius;
        distribution = uniform_real_distribution<double> (0.0, 1.0); // 均匀分布
    }

    vector<double> randPoint() {
        double theta = distribution(generator) * _2pi;
        double r = sqrt(distribution(generator) * radius2);
        return {x_c + cos(theta) * r, y_c + sin(theta) * r};
    }
};
```
**舍选抽样法**
```cpp
class Solution {
public:
    double x_c, y_c, r, radius2;
    double _2pi = 4*acos(0.0);
    uniform_real_distribution<double> dist;
    default_random_engine generator;

    Solution(double radius, double x_center, double y_center):y_c(y_center), x_c(x_center), r(radius), radius2(radius * radius), dist(0.0, 1.0) {}

    vector<double> randPoint() {
        double theta = dist(generator) * _2pi;
        double r = randomR();
        return {x_c + cos(theta) * r, y_c + sin(theta) * r};
    }
    // 随机产生 R
    double randomR(){
        while(true){
            double x = dist(generator) * r;     // x 的取值范围 [0, r]
            double y = dist(generator) * 2 / r; // f(x) 的取值范围 [0, 2/r]
            if(y <= 2 * x / radius2) return x;  // y <= f(x) 选，否则舍
        }
    }
};

```




