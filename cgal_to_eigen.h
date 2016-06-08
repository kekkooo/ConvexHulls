#ifndef CGAL_TO_EIGEN_H
#define CGAL_TO_EIGEN_H

#include <commontypedefs.h>
#include <Eigen/Dense>

namespace Utilities{
    void PolyhedronToDenseMatrices( Polyhedron_3& poly, Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

}

#endif // CGAL_TO_EIGEN_H
