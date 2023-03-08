# GTSAM学习

<!--more-->

# 一. [自定义 Factor](https://zhuanlan.zhihu.com/p/359095869)

在处理 robotics 问题时, 大部分都是非线性问题, 所以主要考虑 `gtsam/nonlinear/NonlinearFactor.h`, GTSAM factor 中的类继承关系示意图如下

{{<mermaid>}}
graph TD
factor --> A(NonlinearFactor)
A --> b(NoiseModelFactor1) & c(NoiseModelFactor2) & d(...)
{{</mermaid>}}

## 如何自定义 Factor

以点云配准(`ICP`) 问题为例, 写一个名为 `IcpFactor` 的因子, 用来优化不同坐标系下两帧点云的配准结果:

$$
T^*=min{\Sigma^N_i||Tp_i-q_i||^2_W}
$$

$$
T\in SE(3), p_i, q_i\in \R^3,W\in \R^{3\times 3}
$$

其中, $p$ 为初始点云, $q'=Tp$表示 `prediction`, $q$ 表示 `measurement`, 待优化变量为 $T$, 每一次测量都可以构成一个 `unary factor`

{{<mermaid>}}
graph TB
    A((T))---B[q_1] & C[q_2] & D[...] & E[q_i]
{{</mermaid>}}

## 代码

这里继承自 `NoiseModelFator1`，使用 `Pose3`表示待优化变量 $T$ , 使用 `Point3`分别表示 $p, q$

可以看出, 自定义 `Factor` 需要继承 `NoiseModelFactor`, 并且根据具体误差形式重写 `evaluteError` 函数.

其中, 我们使用 `Pose3` 中的函数 `transformFrom` 函数计算旋转之后的点 $q'$ 以及 $q'$ 对 $T$ 的导数 $Hpose$, 也即是 error 对 $T$ 的导数.

```cpp
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam
{
class IcpFactor : public NoiseModelFactor1<Pose3>
{
public:
    /**
     * @brief Construct a new Icp Factor object
     *
     * @param noiseModel
     * @param key
     * @param measurement
     * @param p
     */
    IcpFactor(const SharedNoiseModel& noiseModel, Key key,
              const Point3& measurement, const Point3& p)
      : NoiseModelFactor1(noiseModel, key), measurement_(measurement), p_(p)
    {
    }

    /**
     * @brief Destroy the Icp Factor object
     *
     */
    virtual ~IcpFactor()
    {
    }

    /**
     * @brief
     *
     * @param x
     * @param H
     * @return Vector
     */
    virtual Vector evaluateError(
        const Pose3& T, boost::optional<Matrix&> H = boost::none) const override
    {
        // construct prediction and compute jacobian
        gtsam::Matrix36 Hpose;
        const Point3    prediction = T.transformFrom(p_, Hpose);

        if (H)
        {
            *H = Hpose;
        }

        return prediction - measurement_;
    }

private:
    Point3 measurement_;
    Point3 p_;
};

}  // namespace gtsam
```

# 二. Python examples

## 1. Simple Rotation Example

这个示例将对单个变量(只有一个 factor)执行相对简单的优化

```Python
import gtsam
from gtsam.symbol_shorthand import X

if __name__ == '__main__':
    """
    Step 1: 创建一个 factor 来表达一元约束

    "prior": 是从传感器获得的一个测量值(measurement), 测量值上有一个噪声模型

    "Key": 是一个标签, 或者说变量对应的符号, 用于将部分状态(存储在“RotValues”中)
    与特定 factors 相关联, 这些状态需要用 Key 进行标记和查找，并且 Key 应该是唯一的。

    通常, 创建一个 factor 需要:

    - 一个 key 或者一个 keys 的集合, 标记相关的 Variables
    - 一个测量值
    - 一个有正确维度的测量模型, 用于构造 factor
    """
    prior = gtsam.Rot2.fromAngle(np.deg2rad(30))
    prior.print('goal angle')
    model = gtsam.noiseModel.Isotropic.Sigma(dim=1, sigma=np.deg2rad(1))
    key = X(1)
    factor = gtsam.PriorFactorRot2(key, prior, model)

    """
    Step 2: 创建一个图容器, 并且将 factor 添加进去

    在优化之前, 所有的 factors 都要被加到图容器中, 这提供了定义一个约束系统的
    必要的顶层功能

    在本例中, 只有一个 factor, 但是在一些特定的场合中, 许多 factors 需要被添加
    """
    graph = gtsam.NonlinearFactorGraph()
    graph.push_back(factor)
    graph.print('full graph\n')

    """
    Step 3: 创建一个初始估计

    一个初始值对开始优化是必要的. 这个初始状态值是 "Values" 实例, 与字典的结构类似,
    它将 keys (在第一步中创建的标签) 映射到特定的 values 上.

    这个初始估计值将被用作优化的一个线性化点, 所以图中所有的 variables 都在这个结构中
    有一个对应的值是非常重要的.
    """
    initial = gtsam.Values()
    initial.insert(key, gtsam.Rot2.fromAngle(np.deg2rad(20)))
    initial.print('initial estimate:')

    """
    Step 4: 优化

    在以一个约束图以及初始估计定义问题之后, 执行优化只需要简单地调用一个通用的优化函数.
    这将会产生一个新的 RotValues 结构, 包含优化的最终状态.
    """
    result = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
    result.print('final result:')
```

运行结果:

```
goal angle: 0.523599
full graph:
size: 1

Factor 0: PriorFactor on x1
  prior mean: : 0.523599
isotropic dim=1 sigma=0.0174533

initial estimate:
Values with 1 values:
Value x1: (gtsam::Rot2)
: 0.349066

final result:
Values with 1 values:
Value x1: (gtsam::Rot2)
: 0.523599
```

## 2. Odometry Example

这是一个简单的机器人运动示例, 包含一个先验和两个里程计测量值

```Python
from __future__ import print_function

import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np

# 创建噪声模型
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


def main():
    # 创建一个空的非线性因子图
    graph = gtsam.NonlinearFactorGraph()

    # 为初始状态添加先验因子, 需要有初始状态的变量符号(key), 先验值, 噪声模型
    priorMean = gtsam.Pose2(0.0, 0.0, 0.0)  # prior at origin
    graph.add(gtsam.PriorFactorPose2(1, priorMean, PRIOR_NOISE))

    # 添加一个里程计因子, 为了简化, 所有的里程计因子使用相同的噪声模型
    # 里程计因子为二元, 因此每个因子需要两个状态的变量符号
    odometry = gtsam.Pose2(2.0, 0.0, 0.0)
    graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, ODOMETRY_NOISE))
    print("\nFactor Graph:\n{}".format(graph))

    # 设定每个状态的初始估计, 为了演示效果, 这些变量被故意设为错误的值
    initial = gtsam.Values()
    initial.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
    initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
    initial.insert(3, gtsam.Pose2(4.1, 0.1, 0.1))
    print("\nInitial Estimate:\n{}".format(initial))

    # 优化, 使用 Levenberg-Marquardt optimization
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()
    print("\nFinal Result:\n{}".format(result))

    # 计算并输出所有变量边缘化之后的后验概率分布
    marginals = gtsam.Marginals(graph, result)
    for i in range(1, 4):
        print("X{} covariance:\n{}\n".format(i,
                                             marginals.marginalCovariance(i)))

    for i in range(1, 4):
        gtsam_plot.plot_pose2(0, result.atPose2(i), 0.5,
                              marginals.marginalCovariance(i))
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    main()
```

## 3. Planar Manipulator Example

```Python
"""
具有 GTSAM 姿态和指数映射乘积的三连杆机械手的运动学。
Kinematics of three-link manipulator with GTSAM poses and product of exponential maps.
"""
import math
import unittest
from functools import reduce

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import gtsam
import gtsam.utils.plot as gtsam_plot
from gtsam import Pose2
from gtsam.utils.test_case import GtsamTestCase


def vector3(x, y, z):
    """Create 3D double numpy array."""
    return np.array([x, y, z], dtype=float)


def compose(*poses):
    """Compose all Pose2 transforms given as arguments from left to right."""
    return reduce((lambda x, y: x.compose(y)), poses)


def vee(M):
    """Pose2 vee operator."""
    return vector3(M[0, 2], M[1, 2], M[1, 0])


def delta(g0, g1):
    """Difference between x,y, theta components of SE(2) poses."""
    return vector3(g1.x() - g0.x(), g1.y() - g0.y(), g1.theta() - g0.theta())


def trajectory(g0, g1, N=20):
    """ Create an interpolated trajectory in SE(2), treating x,y, and theta separately.
        g0 and g1 are the initial and final pose, respectively.
        N is the number of *intervals*
        Returns N+1 poses
    """
    e = delta(g0, g1)
    return [Pose2(g0.x()+e[0]*t, g0.y()+e[1]*t, g0.theta()+e[2]*t) for t in np.linspace(0, 1, N)]


class ThreeLinkArm(object):
    """Three-link arm class."""

    def __init__(self):
        self.L1 = 3.5
        self.L2 = 3.5
        self.L3 = 2.5
        self.xi1 = vector3(0, 0, 1)
        self.xi2 = vector3(self.L1, 0, 1)
        self.xi3 = vector3(self.L1+self.L2, 0, 1)
        self.sXt0 = Pose2(0, self.L1+self.L2 + self.L3, math.radians(90))

    def fk(self, q):
        """ Forward kinematics.
            Takes numpy array of joint angles, in radians.
        """
        sXl1 = Pose2(0, 0, math.radians(90))
        l1Zl1 = Pose2(0, 0, q[0])
        l1Xl2 = Pose2(self.L1, 0, 0)
        l2Zl2 = Pose2(0, 0, q[1])
        l2Xl3 = Pose2(self.L2, 0, 0)
        l3Zl3 = Pose2(0, 0, q[2])
        l3Xt = Pose2(self.L3, 0, 0)
        return compose(sXl1, l1Zl1, l1Xl2, l2Zl2, l2Xl3, l3Zl3, l3Xt)

    def jacobian(self, q):
        """ Calculate manipulator Jacobian.
            Takes numpy array of joint angles, in radians.
        """
        a = q[0]+q[1]
        b = a+q[2]
        return np.array([[-self.L1*math.cos(q[0]) - self.L2*math.cos(a)-self.L3*math.cos(b),
                          -self.L1*math.cos(a)-self.L3*math.cos(b),
                          - self.L3*math.cos(b)],
                         [-self.L1*math.sin(q[0]) - self.L2*math.sin(a)-self.L3*math.sin(b),
                          -self.L1*math.sin(a)-self.L3*math.sin(b),
                          - self.L3*math.sin(b)],
                         [1, 1, 1]], float)

    def poe(self, q):
        """ Forward kinematics.
            Takes numpy array of joint angles, in radians.
        """
        l1Zl1 = Pose2.Expmap(self.xi1 * q[0])
        l2Zl2 = Pose2.Expmap(self.xi2 * q[1])
        l3Zl3 = Pose2.Expmap(self.xi3 * q[2])
        return compose(l1Zl1, l2Zl2, l3Zl3, self.sXt0)

    def con(self, q):
        """ Forward kinematics, conjugation form.
            Takes numpy array of joint angles, in radians.
        """
        def expmap(x, y, theta):
            """Implement exponential map via conjugation with axis (x,y)."""
            return compose(Pose2(x, y, 0), Pose2(0, 0, theta), Pose2(-x, -y, 0))

        l1Zl1 = expmap(0.0, 0.0, q[0])
        l2Zl2 = expmap(0.0, self.L1, q[1])
        l3Zl3 = expmap(0.0, self.L1+self.L2, q[2])
        return compose(l1Zl1, l2Zl2, l3Zl3, self.sXt0)

    def ik(self, sTt_desired, e=1e-9):
        """ Inverse kinematics.
            Takes desired Pose2 of tool T with respect to base S.
            Optional: mu, gradient descent rate; e: error norm threshold
        """
        q = np.radians(vector3(30, -30, 45))  # well within workspace
        error = vector3(100, 100, 100)

        while np.linalg.norm(error) > e:
            error = delta(sTt_desired, self.fk(q))
            J = self.jacobian(q)
            q -= np.dot(np.linalg.pinv(J), error)

        # return result in interval [-pi,pi)
        return np.remainder(q+math.pi, 2*math.pi)-math.pi

    def manipulator_jacobian(self, q):
        """ Calculate manipulator Jacobian.
            Takes numpy array of joint angles, in radians.
            Returns the manipulator Jacobian of differential twists. When multiplied with
            a vector of joint velocities, will yield a single differential twist which is
            the spatial velocity d(sTt)/dt * inv(sTt) of the end-effector pose.
            Just like always, differential twists can be hatted and multiplied with spatial
            coordinates of a point to give the spatial velocity of the point.
        """
        l1Zl1 = Pose2.Expmap(self.xi1 * q[0])
        l2Zl2 = Pose2.Expmap(self.xi2 * q[1])
        # l3Zl3 = Pose2.Expmap(self.xi3 * q[2])

        p1 = self.xi1
        # p1 = Pose2().Adjoint(self.xi1)

        sTl1 = l1Zl1
        p2 = sTl1.Adjoint(self.xi2)

        sTl2 = compose(l1Zl1, l2Zl2)
        p3 = sTl2.Adjoint(self.xi3)

        differential_twists = [p1, p2, p3]
        return np.stack(differential_twists, axis=1)

    def plot(self, fignum, q):
        """ Plot arm.
            Takes figure number, and numpy array of joint angles, in radians.
        """
        fig = plt.figure(fignum)
        axes = fig.gca()

        sXl1 = Pose2(0, 0, math.radians(90))
        p1 = sXl1.translation()
        gtsam_plot.plot_pose2_on_axes(axes, sXl1)

        def plot_line(p, g, color):
            q = g.translation()
            line = np.append(p[np.newaxis], q[np.newaxis], axis=0)
            axes.plot(line[:, 0], line[:, 1], color)
            return q

        l1Zl1 = Pose2(0, 0, q[0])
        l1Xl2 = Pose2(self.L1, 0, 0)
        sTl2 = compose(sXl1, l1Zl1, l1Xl2)
        p2 = plot_line(p1, sTl2, 'r-')
        gtsam_plot.plot_pose2_on_axes(axes, sTl2)

        l2Zl2 = Pose2(0, 0, q[1])
        l2Xl3 = Pose2(self.L2, 0, 0)
        sTl3 = compose(sTl2, l2Zl2, l2Xl3)
        p3 = plot_line(p2, sTl3, 'g-')
        gtsam_plot.plot_pose2_on_axes(axes, sTl3)

        l3Zl3 = Pose2(0, 0, q[2])
        l3Xt = Pose2(self.L3, 0, 0)
        sTt = compose(sTl3, l3Zl3, l3Xt)
        plot_line(p3, sTt, 'b-')
        gtsam_plot.plot_pose2_on_axes(axes, sTt)


# Create common example configurations.
Q0 = vector3(0, 0, 0)
Q1 = np.radians(vector3(-30, -45, -90))
Q2 = np.radians(vector3(-90, 90, 0))


class TestPose2SLAMExample(GtsamTestCase):
    """Unit tests for functions used below."""

    def setUp(self):
        self.arm = ThreeLinkArm()

    def assertPose2Equals(self, actual, expected, tol=1e-2):
        """Helper function that prints out actual and expected if not equal."""
        equal = actual.equals(expected, tol)
        if not equal:
            raise self.failureException(
                "Poses are not equal:\n{}!={}".format(actual, expected))

    def test_fk_arm(self):
        """Make sure forward kinematics is correct for some known test configurations."""
        # at rest
        expected = Pose2(0, 2*3.5 + 2.5, math.radians(90))
        sTt = self.arm.fk(Q0)
        self.assertIsInstance(sTt, Pose2)
        self.assertPose2Equals(sTt, expected)

        # -30, -45, -90
        expected = Pose2(5.78, 1.52, math.radians(-75))
        sTt = self.arm.fk(Q1)
        self.assertPose2Equals(sTt, expected)

    def test_jacobian(self):
        """Test Jacobian calculation."""
        # at rest
        expected = np.array([[-9.5, -6, -2.5], [0, 0, 0], [1, 1, 1]], float)
        J = self.arm.jacobian(Q0)
        np.testing.assert_array_almost_equal(J, expected)

        # at -90, 90, 0
        expected = np.array([[-6, -6, -2.5], [3.5, 0, 0], [1, 1, 1]], float)
        J = self.arm.jacobian(Q2)
        np.testing.assert_array_almost_equal(J, expected)

    def test_con_arm(self):
        """Make sure POE is correct for some known test configurations."""
        # at rest
        expected = Pose2(0, 2*3.5 + 2.5, math.radians(90))
        sTt = self.arm.con(Q0)
        self.assertIsInstance(sTt, Pose2)
        self.assertPose2Equals(sTt, expected)

        # -30, -45, -90
        expected = Pose2(5.78, 1.52, math.radians(-75))
        sTt = self.arm.con(Q1)
        self.assertPose2Equals(sTt, expected)

    def test_poe_arm(self):
        """Make sure POE is correct for some known test configurations."""
        # at rest
        expected = Pose2(0, 2*3.5 + 2.5, math.radians(90))
        sTt = self.arm.poe(Q0)
        self.assertIsInstance(sTt, Pose2)
        self.assertPose2Equals(sTt, expected)

        # -30, -45, -90
        expected = Pose2(5.78, 1.52, math.radians(-75))
        sTt = self.arm.poe(Q1)
        self.assertPose2Equals(sTt, expected)

    def test_ik(self):
        """Check iterative inverse kinematics function."""
        # at rest
        actual = self.arm.ik(Pose2(0, 2*3.5 + 2.5, math.radians(90)))
        np.testing.assert_array_almost_equal(actual, Q0, decimal=2)

        # -30, -45, -90
        sTt_desired = Pose2(5.78, 1.52, math.radians(-75))
        actual = self.arm.ik(sTt_desired)
        self.assertPose2Equals(self.arm.fk(actual), sTt_desired)
        np.testing.assert_array_almost_equal(actual, Q1, decimal=2)

    def test_manipulator_jacobian(self):
        """Test Jacobian calculation."""
        # at rest
        expected = np.array([[0, 3.5, 7], [0, 0, 0], [1, 1, 1]], float)
        J = self.arm.manipulator_jacobian(Q0)
        np.testing.assert_array_almost_equal(J, expected)

        # at -90, 90, 0
        expected = np.array(
            [[0, 0, 3.5], [0, -3.5, -3.5], [1, 1, 1]], float)
        J = self.arm.manipulator_jacobian(Q2)
        np.testing.assert_array_almost_equal(J, expected)


def run_example():
    """ Use trajectory interpolation and then trajectory tracking a la Murray
        to move a 3-link arm on a straight line.
    """
    # Create arm
    arm = ThreeLinkArm()

    # Get initial pose using forward kinematics
    q = np.radians(vector3(30, -30, 45))
    sTt_initial = arm.fk(q)

    # Create interpolated trajectory in task space to desired goal pose
    sTt_goal = Pose2(2.4, 4.3, math.radians(0))
    poses = trajectory(sTt_initial, sTt_goal, 50)

    # Setup figure and plot initial pose
    fignum = 0
    fig = plt.figure(fignum)
    axes = fig.gca()
    axes.set_xlim(-5, 5)
    axes.set_ylim(0, 10)
    gtsam_plot.plot_pose2(fignum, arm.fk(q))

    # For all poses in interpolated trajectory, calculate dq to move to next pose.
    # We do this by calculating the local Jacobian J and doing dq = inv(J)*delta(sTt, pose).
    for pose in poses:
        sTt = arm.fk(q)
        error = delta(sTt, pose)
        J = arm.jacobian(q)
        q += np.dot(np.linalg.inv(J), error)
        arm.plot(fignum, q)
        plt.pause(0.01)

    plt.pause(10)


if __name__ == "__main__":
    run_example()
    unittest.main()

```

