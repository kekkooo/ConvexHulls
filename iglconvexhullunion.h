#ifndef IGLCONVEXHULLUNION_H
#define IGLCONVEXHULLUNION_H

#include <Eigen/Dense>
#include <segmentation.h>

class IglConvexHullUnion
{
public:
    IglConvexHullUnion();

public:
    void getConvexHullUnion( Eigen::MatrixXd &points_in, const Segmentation::Segments &s,
                             Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out );

};

#endif // IGLCONVEXHULLUNION_H
