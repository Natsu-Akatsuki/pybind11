import numpy as np
from add_plus import add_plus

if __name__ == '__main__':
    # create a 2D numpy matrix
    arr_np = np.arange(10000).reshape(200, 50)
    print("处理前：前三个元素分别为：", arr_np[0][0], " ", arr_np[0][1], " ", arr_np[0][2])
    # 对矩阵进行逐元素的+1
    arr_np = add_plus(arr_np)
    print("处理后：前三个元素分别为：", arr_np[0][0], " ", arr_np[0][1], " ", arr_np[0][2])
