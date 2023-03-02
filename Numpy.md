# Numpy
对于numpy各种库函数的基本介绍和用法实例展示
## 1.基本操作

### 1. numpy.array()
* 构造一个numpy矩阵数组，中括号的数量与形式来确定矩阵的维度
* 参数：既可以是具体的数字，也可以是一个列表
* 实例：numpy.array([\[1,4,5],[2,5,7]])
### 2.numpy.ones()
* 构造一个全为1的行向量
* 实例：numpy.ones(3)
### 3.numpy.zeros()
* 构造一个全为0的行向量
* 实例：numpy.zero(3)
### 4.numpy.sum()
* 对numpy数组矩阵求和
* 参数：不加任何参数是对整体求和，ndim=0表示以行为单位
* 实例：numpy.sum(a,ndim=0)
### 5.numpy.all()
* 判断numpy数组中的所有元素是否满足条件
* 实例：numpy.all(a>0)
### 6.numpy.any()
* 判断numpy数组中的某一个元素是否满足条件
* 实例：numpy.any(a>0)
### 7.numpy.arrange()
* 快速生成一组numpy行向量
* 实例：numpy.arrange(1,3)