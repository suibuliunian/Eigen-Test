//main.cpp

#include <iostream>
#include <fstream>
#include <vector>

//#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

using namespace std;
int main()
{
    //1. matrix normal operation
    Eigen::MatrixXd m1(2,2);
    m1(0,0) = 1;
    m1(0,1) = 2;
    m1(1,0) = 3;
    m1(1,1) = m1(0,0) + m1(0,1) + m1(1,0);
    cout << " Matrix(2x2) m1 " << endl;
    cout << m1 << endl;
    cout << endl;
    

    //2. random matrix 
    Eigen::MatrixXd m2 = Eigen::Matrix3d::Random(3,3);
    cout << " Matrix(3x3) m2 Random " << endl;
    cout << m2 << endl;
    cout << " m2 inverse " << endl;
    cout << m2.inverse() << endl;
    cout << " m2 transpose " << endl;  
    cout << m2.transpose() << endl;
    cout << " m2 conjugate " << endl;  
    cout << m2.conjugate() << endl;
    cout << " m2 adjoint " << endl;  
    cout << m2.adjoint() << endl;
    cout << " m2 sum " << endl;  
    cout << m2.sum() << endl;
    cout << " m2 prod " << endl;  
    cout << m2.prod() << endl;
    cout << " m2 mean " << endl;  
    cout << m2.mean() << endl;
    cout << " m2 trace " << endl;  
    cout << m2.trace() << endl;
    
    int minRow,minCol,maxRow,maxCol;
    cout << " m2 min = " << endl;  
    cout << m2.minCoeff(&minRow,&minCol) << endl;
    cout << " minRow = " << minRow << " minCol = " << minCol << endl;
    cout << " m2 max = " << endl;  
    cout << m2.maxCoeff(&maxRow,&maxCol) << endl;
    cout << " maxRow = " << maxRow << " maxCol = " << maxCol << endl;
    cout << endl;

    //3. constant matrix
    Eigen::MatrixXd m3 = Eigen::Matrix3d::Constant(3,3,1.2);
    cout << " Matrix(3x3) m3 Constant " << endl;
    cout << m3 << endl;
    cout << endl;

    //4. block matrix manipulation
    int rows = 5, cols = 6;
    Eigen::MatrixXf m4(rows,cols);
    m4 << (Eigen::Matrix3f() <<1,2,3,4,5,6,7,8,9).finished(),
           Eigen::MatrixXf::Zero(3,cols - 3),
           Eigen::MatrixXf::Zero(rows - 3,3),
           Eigen::MatrixXf::Identity(rows - 3,cols - 3);
    cout << " Matrix(5x6) m4 block manipulation " << endl;
    cout << m4 << endl;
    cout << " m4 size = " << endl;
    cout << m4.size() << endl;
    cout << " m4 row = " << endl;
    cout << m4.rows() << endl;
    cout << " m4 col = " << endl;
    cout << m4.cols() << endl;
    cout << endl;

    //5. vector manipulation
    Eigen::Vector3d v1(1,2,3);
    Eigen::Vector3d v2(4,5,6);
    
    //Eigen::VectorXd v3(3);
    //Eigen::VectorXd v4(3);
    
    //v3 << 1.1,2.2,3.3;
    //v4 << 4.4,5.5,6.6;

    cout << " v1 = " << endl << v1 << endl;
    cout << " v2 = " << endl << v2 << endl;
    //cout << " v3 = " << endl << v3 << endl;
    //cout << " v4 = " << endl << v4 << endl;

    cout << " v1 .* v2 = " << v1.dot(v2) << endl;
    cout << " v1 x* v2 = " << v1.cross(v2) << endl;

    //1.rotation vector to  rotation matrix
    Eigen::AngleAxisd rotationVector(M_PI/4,Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
    rotationMatrix = rotationVector.toRotationMatrix();
    cout << "rotationMatrix \n" << rotationMatrix<<endl;
    cout << endl;

    //2.rotation vector to quaterniond
    Eigen::Quaterniond q = Eigen::Quaterniond( rotationVector );
    cout << "rotation quaternion \n" << q.coeffs() << endl;
    cout << endl;

    //3.rotaion vector to eulerAngles
    Eigen::Vector3d eulerAngle = rotationVector.matrix().eulerAngles(0,1,2);
    cout << "eulerAngle roll pitch yaw\n" << 180*eulerAngle/M_PI << endl;

    return 0;
}
