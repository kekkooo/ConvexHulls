#include "csg_tree_approach.h"
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <convexhullcreator.h>
#include <igl/copyleft/cgal/CSGTree.h>

typedef igl::copyleft::cgal::CSGTree CsgTree;

namespace CSGTree{

void getConvexHullUnion(Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                        Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out){



    std::vector<Eigen::MatrixXd> points_vector;
    std::vector<Eigen::MatrixXi> faces_vector;
    std::vector<CsgTree> csgTrees;

    ConvexHullCreator::getConvexHulls( points_in, segments, points_vector, faces_vector );

    csgTrees.resize( points_vector.size( ));
    for( int i = 0; i < points_vector.size(); ++i ){
        csgTrees[i] = { points_vector[i], faces_vector[i] };
    }






}

}
