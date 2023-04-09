# OSQP使用指南

## OSQP数学推导及理论

后续再写



## OSQP 举例

找到下式的最小值

$f(x)=\frac{1}{2}x_1^2+x_2^2-x_1x_2-2x_1-6x_2$

需满足以下约束

$x_1+x_2\leq2\\
-x_1+2x_2\leq2\\
2x_1+x_2\leq3$

在QP问题中，QP问题的定义形式：

$f(x)=\frac{1}{2}x^THx+f^Tx$

所以上面的问题可以对等为

$f(x)=\frac{1}{2} \left[
 \begin{matrix}
   x_1 & x_2
  \end{matrix}
  \right]\left[
 \begin{matrix}
   1 &-1\\-1&2
  \end{matrix}
  \right] \left[
 \begin{matrix}
   x_1 \\ x_2
  \end{matrix}
  \right]
  -\left[
 \begin{matrix}
   -2 \\ -6
  \end{matrix}
  \right]^T \left[
 \begin{matrix}
   x_1 \\ x_2
  \end{matrix}
  \right]$

constrain 的定义为

$Ax\leq b$

所以constrain可写为

$\left[
 \begin{matrix}
   1&1 \\ -1&2\\2&1
  \end{matrix}
  \right]
  \left[
 \begin{matrix}
   x_1 \\ x_2
  \end{matrix}
  \right] \leq \left[
 \begin{matrix}
   2 \\ 2\\3
  \end{matrix}
  \right]$


故系数矩阵为

H  =[1 -1;-1 2]

f = [-2;-6]

A =[1 1;-1 2;2 1]

b =[2;2;3]

$Ax\leq b$

