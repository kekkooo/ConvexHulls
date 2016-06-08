#include "iglmeshboolean.h"
#include <igl/copyleft/cgal/mesh_boolean.h>
#include<convexhullcreator.h>
#include <thread>

typedef std::chrono::high_resolution_clock myclock;

namespace iglMeshBoolean{

static const int num_threads = 4;

void getConvexHullUnion(Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                        Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out){

    std::vector<Eigen::MatrixXd> points_vector;
    std::vector<Eigen::MatrixXi> faces_vector;
    std::vector<Hull> hulls;

    ConvexHullCreator::getConvexHulls( points_in, segments, points_vector, faces_vector );

    igl::MeshBooleanType boolean_op = igl::MESH_BOOLEAN_TYPE_UNION;

    Hull zero;
    zero.points = points_vector[0];
    zero.faces  = faces_vector[0];
    hulls.push_back(zero);

    for( size_t i = 1; i < points_vector.size() ; ++i ){
        auto t0 = myclock::now();

        Eigen::MatrixXd points;
        Eigen::MatrixXi faces;
        Eigen::VectorXi J,I;
        Hull hull;

        igl::copyleft::cgal::mesh_boolean( hulls[i-1].points, hulls[i-1].faces,
                                           points_vector[i], faces_vector[i], boolean_op,
                                           hull.points, hull.faces, J );

        hulls.push_back(hull);

        auto t1  = myclock::now();
        long span = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        std::cout << "union " << i - 1 << " took"  << span << "millis" << std::endl;
    }

    points_out = std::move( hulls.back().points );
    faces_out = std::move( hulls.back().faces );
}

void getConvexHullUnion_mt(Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                           Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out){
    std::vector<Eigen::MatrixXd> points_vector;
    std::vector<Eigen::MatrixXi> faces_vector;
    std::vector<Hull> hulls, unions;

    ConvexHullCreator::getConvexHulls( points_in, segments, points_vector, faces_vector );

    igl::MeshBooleanType boolean_op = igl::MESH_BOOLEAN_TYPE_UNION;
    int parts = num_threads;

    hulls.resize( points_vector.size( ));
    for( size_t i = 0; i < points_vector.size(); ++i ){
        std::cout << "( " << points_vector[i].rows() << ", " << faces_vector[i].rows() << std::endl;
        hulls[i].points = std::move( points_vector[i]);
        hulls[i].faces  = std::move( faces_vector[i]);
    }

    for( size_t i = 0; i < hulls.size(); ++i ){
        std::cout << "( " << hulls[i].points.rows() << ", " << hulls[i].faces.rows() << std::endl;
    }

    std::sort( hulls.begin(), hulls.end() );
    unions.push_back( hulls[0] );

    for( size_t i = 1; i < points_vector.size() ; ++i ){
        auto t0 = myclock::now();

        Eigen::MatrixXd points;
        Eigen::MatrixXi faces;
        Eigen::VectorXi J,I;
        Hull hull;

        igl::copyleft::cgal::mesh_boolean( unions[i-1].points, unions[i-1].faces,
                                           hulls[i].points, hulls[i].faces, boolean_op,
                                           hull.points, hull.faces, J );

        unions.push_back(hull);

        auto t1  = myclock::now();
        long span = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        std::cout << "union " << i - 1 << " took"  << span << "millis" << std::endl;
    }

    points_out = std::move( unions.back().points );
    faces_out = std::move( unions.back().faces );

}

}
