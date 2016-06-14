#include "obb.h"
#include <iostream>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/OrderingMethods>

void OBB::buildOBB(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd& obbV, Eigen::MatrixXi& obbF){
    std::cout << "trying to build oob" << std::endl;
    Eigen::MatrixXd eU, eV, eUVt;

    auto mean           = V.colwise().sum();
    Eigen::MatrixXd D   = V.rowwise() - mean;
    std::cout << " D size " <<  D.rows() << ", " << D.cols() << std::endl;
    D = D.transpose() * D;

//    std::cout << " D size " <<  D.rows() << ", " << D.cols() << std::endl;
//    std::cout << " D matrix " << std::endl << D << std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd( D, Eigen::ComputeThinU | Eigen::ComputeThinV );

    std::cout << "svd done " << std::endl;
    eU = svd.matrixU();
    eV = svd.matrixV();
    auto evalues = svd.singularValues();


//    eUVt = eU * eV.transpose();
//    eU = eU * eV.transpose();
    std::cout << "matrices done " << std::endl;
    std::cout << eU << std::endl;
    std::cout << eV << std::endl;
    std::cout << evalues << std::endl;

//    std::cout << eV << std::endl;
//    std::cout << eUVt << std::endl;
    Eigen::Vector3d r,u,f, min, max;
    double dmin = std::numeric_limits<double>::max();
    double dmax = std::numeric_limits<double>::min();
    r   << eU( 0, 0 ), eU( 1, 0 ), eU( 2, 0 );
    u   << eU( 0, 1 ), eU( 1, 1 ), eU( 2, 1 );
    f   << eU( 0, 2 ), eU( 1, 2 ), eU( 2, 2 );
    r.normalize();
    u.normalize();
    f.normalize();
    min << dmin,dmin,dmin;
    max << dmax,dmax,dmax;
    std::cout << "ruf done " << std::endl;

    for( int i = 0; i < V.rows(); ++i ){
        Eigen::Vector3d point;
        point << V( i, 0 ), V( i, 1 ), V( i, 2 );
        Eigen::Vector3d p_prime( r.dot( point ), u.dot( point ), f.dot( point ));
        min << std::min( min( 0 ), p_prime( 0 )), std::min( min( 1 ), p_prime( 1 )), std::min( min( 2 ), p_prime( 2 ));
        max << std::max( max( 0 ), p_prime( 0 )), std::max( max( 1 ), p_prime( 1 )), std::max( max( 2 ), p_prime( 2 ));
    }
    std::cout << "min_max done " << std::endl;

    Eigen::Vector3d center = (min + max) * 0.5;
    Eigen::Vector3d ext    = (max - min) * 0.5;
    Eigen::Vector3d pos;
    pos << eU.row(0).dot(center), eU.row(1).dot(center), eU.row(2).dot(center);
    //m_pos.Set( m_rot[0].Dot(center), m_rot[1].Dot(center), m_rot[2].Dot(center) );

    std::cout << "center : " << center << std::endl << "size " << ext << std::endl;

    obbV.resize( 8, 3 );
    obbF.resize( 6, 4 );

    obbV.row(0) = pos - r * ext( 0 ) - u * ext( 1 ) - f * ext( 2 );     // LDB
    obbV.row(1) = pos + r * ext( 0 ) - u * ext( 1 ) - f * ext( 2 );     // RDB
    obbV.row(2) = pos + r * ext( 0 ) - u * ext( 1 ) + f * ext( 2 );     // RDF
    obbV.row(3) = pos - r * ext( 0 ) - u * ext( 1 ) + f * ext( 2 );     // LDF
    obbV.row(4) = pos - r * ext( 0 ) + u * ext( 1 ) - f * ext( 2 );     // LUB
    obbV.row(5) = pos + r * ext( 0 ) + u * ext( 1 ) - f * ext( 2 );     // RUB
    obbV.row(6) = pos + r * ext( 0 ) + u * ext( 1 ) + f * ext( 2 );     // RUF
    obbV.row(7) = pos - r * ext( 0 ) + u * ext( 1 ) + f * ext( 2 );     // LUF

    std::cout << "obbV done " << std::endl;

    obbF << 0, 4, 5, 1,
            4, 7, 6, 5,
            0, 3, 7, 4,
            7, 3, 2, 6,
            3, 0, 1, 2,
            5, 6, 2, 1;

    std::cout << "obbF done " << std::endl;
}
