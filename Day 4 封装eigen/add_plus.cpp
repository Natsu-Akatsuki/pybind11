#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace pybind11::literals;

#define pass_ref  // pass_value or pass_ref

// 方法一：传值
#ifdef pass_value

Eigen::MatrixXi add_plus(Eigen::MatrixXi arr_np) {

    /*
    for (int i = 0; i < arr_np.rows(); i++) {
        for (int j = 0; j < arr_np.cols(); j++)
            arr_np(i, j) = arr_np(i, j) + 1;
    }
     */

    // Eigen也支持向量化操作
    arr_np.array() += 1;
    return arr_np;
}

#endif

// 方法二：传引入
#ifdef pass_ref

Eigen::MatrixXi add_plus(Eigen::MatrixXi &arr_np) {
    arr_np.array() += 1;
    return arr_np;
}

#endif

// 使用关键词和默认参数
PYBIND11_MODULE(add_plus, m) {
    m.doc() = "给矩阵进行逐元素的+1";
    m.def("add_plus", &add_plus, "给矩阵进行逐元素的+1");
}