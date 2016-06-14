#ifndef CONVEXHULLCREATOR_H
#define CONVEXHULLCREATOR_H

#include <eigen3/Eigen/Dense>
//#include <CGAL/Exact_integer.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/Exact_integer.h>
//#include <CGAL/Homogeneous.h>
//#include <CGAL/Polyhedron_items_with_id_3.h>
//#include <CGAL/algorithm.h>
//#include <CGAL/Polyhedron_3.h>
//#include <CGAL/convex_hull_3.h>
//#include <CGAL/Nef_polyhedron_3.h>

#include <commontypedefs.h>
#include <segmentation.h>

namespace ConvexHullCreator{

    // TODO :
    // I'm building the bounding boxes using the exact constructions,
    // it's a waste of time, I can use inexact constructions and then upcast only for the nef polyhedra construction

    void buildConvexHulls(const Segmentation::SegmentedPointList& points, std::vector< Polyhedron_3>& hulls );

    void getConvexHulls(        Eigen::MatrixXd& points_in, const Segmentation::Segments& s,
                                std::vector<Eigen::MatrixXd>& points_out, std::vector<Eigen::MatrixXi>& faces_out );

    void getConvexHull(         Eigen::MatrixXd& points_in, Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

    void getConvexHull(         Eigen::MatrixXd& points_in, const Segmentation::Segments& s,
                                Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

    void getConvexHullUnion(    Eigen::MatrixXd& points_in, const Segmentation::Segments& s,
                                Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

    void getOOBs(               Eigen::MatrixXd& points_in, const Segmentation::Segments& s,
                                Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

    void remesh(                Double_Polyhedron_3 &in);

    void toInexactPolyhedron(   Polyhedron_3& in , Double_Polyhedron_3 &out);

}

#endif // CONVEXHULLCREATOR_H
