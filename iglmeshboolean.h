#ifndef IGLMESHBOOLEAN_H
#define IGLMESHBOOLEAN_H

#include <Eigen/Dense>

#include <segmentation.h>

namespace iglMeshBoolean{

struct Hull{
    Eigen::MatrixXd points;
    Eigen::MatrixXi faces;

    bool operator < (const Hull& other) const{
        return points.rows() < other.points.rows();
    }
};

enum MeshBoolean{ CGAL, CORK };
enum MultiThread{ MT, NO_MT };

void getConvexHullUnionWithCGAL(Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                                Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out);

void getConvexHullUnionWithCGAL_mt(Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                                Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out);

void getConvexHullUnionWithCORK(Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                                Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out);

void getConvexHullUnionWithCORK_mt(Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                                Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out);


void getConvexUnion( Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                     Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out, MeshBoolean library, MultiThread mt );

void getConvexHullUnion(Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                        Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out, MeshBoolean library );

void getConvexHullUnion_mt( Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                            Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out, MeshBoolean library);

}

#endif // IGLMESHBOOLEAN_H
