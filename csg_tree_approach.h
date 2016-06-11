#ifndef CSG_TREE_APPROACH_H
#define CSG_TREE_APPROACH_H

#include <convexhullcreator.h>

namespace CSGTree{

    void getConvexHullUnion(Eigen::MatrixXd& points_in, const Segmentation::Segments &segments,
                             Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

}


#endif // CSG_TREE_APPROACH_H
