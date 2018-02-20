#include <iostream>
#include <GU/GU_Detail.h>
#include "math.hpp"

int main()
{
    constexpr int rows = 10;
    constexpr int cols = 3;
    Eigen::MatrixXd m(rows, cols);
    double * data = m.data();
    for (int i=0; i< rows*cols; ++i)
        data[i] = (double)i;

    // const Eigen::VectorXd & d = m.row(0);
    // UT_Vector3 v(d.data());
    std::cout << m << '\n';
    // std::cout << d << '\n';
    // std::cout << v.x() << v.y() << v.z() << '\n';


    Eigen::VectorXd d(cols);
    d.segment<cols>(0) = m.row(1);
    std::cout << d << '\n';

    Eigen::VectorXd r =  m * d;
    std::cout << r << '\n';    


    return 0;

}

