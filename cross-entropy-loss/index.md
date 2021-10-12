# Cross Entropy Loss详解

pytorch中的`CrossEntropyLoss`究竟干了什么?
<!--more-->

## 1 logits
一般多分类模型的原始输出称为[`logits`]^(batchsize x class_nums), 可将其理解为没有标准化的概率.
`logits`不必在$0\sim 1$之间, 也不必满足每个样本各种情况概率相加等于1.

## 2 softmax
`softmax` 可以将logits标准化, 设其中一个样本的logit为:
$$
\begin{equation}
logit = [a_1, a_2, a_3, \cdots, a_c]
\end{equation}
$$

若将logits解释成没有归一化的对数概率:
$$\begin{equation} a_i = \ln{b_i}, i=1\sim c \end{equation}$$
那么对其做指数操作并归一化，就可以得到标准化的概率. 因此softmax的公式为:
$$ \begin{equation}
q_i = \frac{e^{a_i}}{\sum_{j=1}^{c}e^{a_j}}
\end{equation} $$

## 3 交叉熵损失 Cross Entropy Loss
在有监督多分类问题中, `one-hot`编码相当于样本的真实概率分布`p`, 而模型预测的logit经过softmax得到了预测的概率分布`q`, 因此可以方便地采用交叉熵作为损失函数:
$$ \begin{equation}
Loss = \sum_j^c p_j \ln \frac{p_j}{q_j}
\end{equation} $$
而`one-hot`编码中只有一项为1, 不妨设$p_k=1$, 则损失函数变为:
$$ \begin{equation}
Loss = -\ln q_k
\end{equation} $$

## 4 反向传播
在梯度的反向传播中需要求loss对模型输出的导数, 也即: $\frac{\partial L}{\partial a_i}, i=1\sim c$.
<div>
$$
\begin{equation}
\begin{split}
\frac{\partial L}{\partial a_i} =&  \frac{-\partial \ln q_k}{\partial a_i}= -\frac{1}{q_k} \frac{\partial q_k}{\partial a_i} \\ \\
=&\begin{cases}
   { \begin{split}
    -\frac{1}{q_k} \frac{\partial \frac{e^{a_k}}{\sum e^{a_j}} }{\partial a_k} =
    -\frac{1}{q_k} \frac{e^{a_k}\sum e^{a_j}-(e^{a_k})^2}{(\sum e^{a_j})^2} =
    -\frac{1}{q_k}\cdot q_k (1-q_k) = q_k-1 \end{split}} &\text{, if } i=k \\ \\
   { \begin{split}
    -\frac{1}{q_k} \frac{\partial \frac{e^{a_k}}{\sum e^{a_j}} }{\partial a_i} =
    \frac{1}{q_k} \frac{e^{a_k}e^{a_i}}{(\sum e^{a_j})^2} =
    \frac{1}{q_k}\cdot q_k \cdot q_i = q_i \end{split}}  &\text{, if } i\neq k
\end{cases}
\end{split}
\end{equation}
$$
</div>

## 5 [torch.nn.CrossEntropyLoss](https://pytorch.org/docs/stable/generated/torch.nn.CrossEntropyLoss.html) 究竟干了什么
`CrossEntropyLoss` 输入为 logits, 输出为交叉熵损失。

也就是包含 softmax 和 交叉熵$-ln()$ 两个步骤:
$$
\begin{equation}
loss(x,class)=−\ln\left(\frac{e^{x[class]}}{\sum_j e^{x[j]}} \right)=-x[class]+\ln \left( \sum_j e^{x[j]} \right)
\end{equation}
$$

把两步操作写成一个op, 目的是为了让反向传播梯度更加稳定, 从式(6)中可以看出, softmax的梯度是$p(1-p)$, 而交叉熵的梯度是$-1/p$, 后者在$p\to 0$的时候并不稳定, 而将两者相乘可以抵消掉分母的$p$.

{{< admonition >}}
还有另一种计算交叉熵损失的步骤使用 `LogSoftmax` + [`NLLLoss`]^(negative log likelihood loss) 能够到达跟`CrossEntropyLoss`一样的效果.

其中 `LogSoftmax()(logits)` 等效于 `log(Softmax()(logits))`, 即通过logits先计算标准化概率, 再取对数概率$y$:
$$
y_i = \ln \left( \frac{e^{x[i]}}{\sum_j e^{x[j]}} \right)
$$

`NLLLoss`的作用是根据标签选择对应的对数概率, 并添加负号:
$$
loss(y, class) = -y[class]
$$
两种方法殊途同归, 虽然使用的是不同函数, 但本质是同一个公式, 并且`LogSoftmax`也将两个操作写成了一个op.

参考代码如下:
```python
import torch

input = torch.tensor([  [ 1.1316, -0.7943,  0.1328],
                        [ 0.2654,  0.1978, -0.0265],
                        [-2.8054,  0.2867, -0.1517]])
target = torch.tensor([0, 1, 2])

log = torch.log
softmax = torch.nn.Softmax(dim=1)
logsoftmax = torch.nn.LogSoftmax(dim=1)
nlll = torch.nn.NLLLoss()
crossentropyloss = torch.nn.CrossEntropyLoss()

print(softmax(input))
print(log(softmax(input)))
print(nll(log(softmax(input)), target))
print(crossentropyloss(input, target))
```
{{< /admonition >}}



