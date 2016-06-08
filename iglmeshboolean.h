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

void getConvexHullUnion(Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                        Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out);

void getConvexHullUnion_mt( Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                            Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out);

}

#endif // IGLMESHBOOLEAN_H
