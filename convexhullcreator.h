#ifndef CONVEXHULLCREATOR_H
#define CONVEXHULLCREATOR_H

#include <Eigen/Dense>
#include <commontypedefs.h>
#include <segmentation.h>

namespace ConvexHullCreator{

    typedef std::map< size_t, std::vector<CGAL_Point_3> > SegmentedPointList;

    // TODO :
    // I'm building the bounding boxes using the exact constructions,
    // it's a waste of time, I can use inexact constructions and then upcast only for the nef polyhedra construction

    void buildConvexHulls(const Segmentation::SegmentedPointList& points, std::vector< Polyhedron_3>& hulls );

    void getConvexHulls(Eigen::MatrixXd& points_in, const Segmentation::Segments& segments,
                                std::vector<Eigen::MatrixXd>& points_out, std::vector<Eigen::MatrixXi>& faces_out );

    void getConvexHull(         Eigen::MatrixXd& points_in, Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

    void getConvexHull(Eigen::MatrixXd& points_in, const Segmentation::Segments& segments,
                                Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

    void getConvexHullUnion(Eigen::MatrixXd& points_in, const Segmentation::Segments& segments,
                                Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

    void getOOBs(               Eigen::MatrixXd& points_in, const Segmentation::Segments& s,
                                Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out );

    void remesh(Double_Polyhedron_3 &in, double target_edge_length = 1.0 );

    void toInexactPolyhedron(   Polyhedron_3& in , Double_Polyhedron_3 &out);

}

#endif // CONVEXHULLCREATOR_H
