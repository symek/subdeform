#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <GU/GU_Detail.h>

// using Matrix    = Eigen::MatrixXd;
// using Vector    = Eigen::VectorXd;

int main()
{
    Eigen::MatrixXd m(3,3);
    double * data = m.data();
    for (int i=0; i< 3*3; ++i)
        data[i] = i;

    // const Eigen::VectorXd & d = m.row(0);
    // UT_Vector3 v(d.data());
    std::cout << m << '\n';
    // std::cout << d << '\n';
    // std::cout << v.x() << v.y() << v.z() << '\n';


    Eigen::VectorXd d(3);
    d.segment<3>(0) = m.row(1);
    std::cout << d << '\n';

    Eigen::VectorXd r = d.transpose() * m;
    std::cout << r << '\n';    


    return 0;

}

