#ifndef OBB_H
#define OBB_H

#include <Eigen/Dense>

namespace OBB{

    void buildOBB( Eigen::MatrixXd& points, Eigen::MatrixXi& F ,
                   Eigen::MatrixXd &obbV, Eigen::MatrixXi &obbF);
}


#endif // OBB_H
