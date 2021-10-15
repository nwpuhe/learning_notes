<<深度学习与计算机视觉实战>>

<<斯坦福大学 `cs231n`>>

<<动手深度学习>>

<<Machine Learning>>





## 深度学习与计算机视觉实战

- 计算 `softmax` 函数
- 用来计算似然函数
- 解决概率计算中概率结果大占绝对优势的问题

```python
import numpy
import math


def softmax(in_matrix):
    m, n = numpy.shape(in_matrix)
    out_matrix = numpy.mat(numpy.zeros((m, n)))
    soft_sum = 0
    for idx in range(0, n):
        out_matrix[0, idx] = math.exp(in_matrix[0, idx])
        soft_sum += out_matrix[0, idx]

    for idx in range(0, n):
        out_matrix[0, idx] = out_matrix[0, idx] / soft_sum

    return out_matrix


a = numpy.array([[1, 2, 1, 2, 1, 1, 3]])
print(softmax(a))
```

