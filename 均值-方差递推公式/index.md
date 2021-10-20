# 均值, 方差递推公式


<!--more-->

## 问题描述

在一般的数学统计过程中, 为了求得方差和均值, 需要预先知道所有的数据项. 但是在大数据, 流式处理场景, 无法预知所有数据项, 若采用一般的计算方案, 需要缓存所有的数据, 并且每次计算都要遍历一次, 这非常耗费资源.
因此, 有必要使用递推的方法, 通过之前的均值, 方差, 数据量, 以及当前的样本值来计算出当前的均值和方差.

设:

* $n$ : 当前采样数
* $x_i$ : 第 $i$ 项采样的数据值
* $\bar{x}_i$ : 前 $i$ 项的平均值
* $v_i^2$ : 前 $i$ 项的方差估计值

则问题描述为: 已知 $n, x_{n}, \bar{x}_{n-1}, v_{n-1}^2, (n > 0)$, 求 $\bar{x}_n, v_n^2$.

## 方差与均值
均值的估计量为:

$$
\bar{x} = \frac{\sum_{i=1}^n x_i}{n}
$$

在知道总体均值 $\mu$ 时, 有总体方差计算公式:

$$
\sigma^2 = \frac{\sum_{i=1}^N (x_i - \mu)^2}{N}
$$

实际工作中, 总体均数难以得到时, 应用样本统计量代替总体均值, 方差的无偏估计量如下:

$$
S^2 = \frac{\sum_{i=1}^n (x_i - \bar{x})^2}{n-1}
$$

{{< admonition >}}
注意方差的无偏估计量分母为 $n-1$, 但是在实际操作中, 有时为了计算简便, 可以用 $n$ 代替.

`numpy.var()`函数可用于计算方差, 它使用的默认分母就是 $n$. 可以通过参数`ddof`来改变分母为 $n-ddof$.
{{< /admonition >}}


## 均值和方差的递推公式

**均值递推公式:**
<div>
$$
\bar{x}_n = \frac{\sum_{i=1}^n x_i}{n} = \frac{\sum_{i=1}^{n-1} x_i + x_n}{n}
    = \frac{(n-1)\bar{x}_{n-1}+x_n}{n}
$$
</div>

**方差递推公式:**

若采用分母为 $n$ 的公式, 设 $d\bar{x} = -\bar{x}\_{n} + \bar{x}\_{n-1}$:

<div>
$$
\begin{split}
v_n^2 = &\frac{\sum_{i=1}^n (x_i - \bar{x}_n)^2}{n} \\
    = & \frac{\sum_{i=1}^{n-1} (x_i - \bar{x}_n)^2 + (x_n-\bar{x}_n)^2}{n}\\
    = & \frac{\sum_{i=1}^{n-1} (x_i - \bar{x}_{n-1}+d\bar{x})^2 + (x_n-\bar{x}_n)^2}{n}\\
    = & \frac{\sum_{i=1}^{n-1} \left[ \underbrace{(x_i - \bar{x}_{n-1}) ^2 }_{=(n-1)v_{n-1}^2} +
        \underbrace{2(x_i - \bar{x}_{n-1})}_{\text{=0}}d\bar{x} +d\bar{x}^2 \right] + (x_n-\bar{x}_n)^2}{n}  \\ \\
    = & \frac{(n-1)v_{n-1}^2 + (n-1)d\bar{x}^2 + (x_n-\bar{x}_n)^2}{n}, (n\geq 1)\\
\end{split}
$$

若采用无偏估计, 分母为 $n-1$ 的公式:
<div>
$$
v_n^2 = \frac{(n-2)v_{n-1}^2 + (n-1)d\bar{x}^2 + (x_n-\bar{x}_n)^2}{n-1}, (n\geq 2)\\
$$
</div>


## Python 实现

```python
def new_mean_var(self, x, old_mean, old_var, n)->int:
    """
    均值以及方差递推公式, 已知n个样本下的均值和方差, 计算加入第 n+1 个样本后的均值和方差

    Args:
        x (float): 新的采样值
        old_mean (float): n个样本时的均值
        old_var (float): n个样本时的方差
        n (int): 已统计样本数 n >= 0
    """
    new_mean =  (n * old_mean + x) / (n + 1)
    dmean = old_mean - new_mean
    new_var = (n * old_var + n * dmean**2 + (x - new_mean)**2) / (n + 1)
    return new_mean, new_var

if __name__ == '__main__':
    a = np.arange(0, 20)
    mean = 0
    var = 0
    for i in range(len(a)):
        print(f'np   mean: {np.mean(a[:i+1])}, var: {np.var(a[:i+1])}')
        mean, var = new_mean_var(a[i], var, mean, i)
        print(f'anas mean: {mean}, var: {var}')
```

