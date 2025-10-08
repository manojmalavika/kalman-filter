#include <iostream>
#include <Eigen/Dense>


int main(){
    // Matrix x d => Matrix of arbitary size, where entry entry is a double
    Eigen::MatrixXd A(2, 2); 
    A << 1, 2, 
         3, 4;

    // OR declare a fixed size matrix.
    // Fixed size is preferred for small size matrices,  less than 4x4
    Eigen::Matrix2d B;

    // Similarly for vector
    Eigen::VectorXd c(3);
    std::cout << "C: " << c <<std::endl; // always 0 initilized by default?
    c << 1, 2, 3;
    std::cout << "C: \n" << c <<std::endl;

    Eigen::Vector2d b(5, 6);
    Eigen::Vector2d x = A.inverse() * b;

    std::cout << "Solution : \n" << x << std::endl;
    std::cout << "A * x:\n" << A * x << std::endl;
    return 0;
}