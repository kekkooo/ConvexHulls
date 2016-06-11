#include "iglmeshboolean.h"

#include <thread>
#include <algorithm>

#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/copyleft/cork/mesh_boolean.h>

#include<convexhullcreator.h>

typedef std::chrono::high_resolution_clock myclock;

namespace iglMeshBoolean{

static const int num_threads = 4;

void getConvexHullUnion(Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                        Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out, MeshBoolean library){

    std::vector<Eigen::MatrixXd> points_vector;
    std::vector<Eigen::MatrixXi> faces_vector;
    std::vector<Hull> hulls, unions;

    ConvexHullCreator::getConvexHulls( points_in, segments, points_vector, faces_vector );

    igl::MeshBooleanType boolean_op = igl::MESH_BOOLEAN_TYPE_UNION;
    int parts = num_threads;

    hulls.resize( points_vector.size( ));
    for( size_t i = 0; i < points_vector.size(); ++i ){
        std::cout << "( " << points_vector[i].rows() << ", " << faces_vector[i].rows() << std::endl;
        hulls[i].points =  points_vector[i];
        hulls[i].faces  = faces_vector[i];
    }

    std::cout << "created hulls list" << std::endl;

    std::sort( hulls.begin(), hulls.end() );

    std::cout << "sorted" << std::endl;

    bool done = false;
    while( !done ){
        std::cout << "working on #hulls = " << hulls.size()  << std::endl;
        assert( hulls.size() > 1 );

        size_t numPairs = hulls.size() / 2;

        unions.clear();
        unions.resize( numPairs );

        for( int i = 0; i < numPairs; ++i ){
            size_t hull_index  = i * 2;
            const Hull& hull_a = hulls[hull_index];
            const Hull& hull_b = hulls[hull_index+1];
//            std::cout << "working on hulls " << i << " and " << i + 1 << std::endl;
//            std::cout << "( " << hull_a.points.size() << ", " << hull_a.faces.size() << " )" << std::endl
//                      << "( " << hull_b.points.size() << ", " << hull_b.faces.size() << " )" << std::endl;
            Eigen::VectorXi J;

            switch(library){
                case MeshBoolean::CGAL : {
                    igl::copyleft::cgal::mesh_boolean( hull_a.points, hull_a.faces,
                                                       hull_b.points, hull_b.faces, boolean_op,
                                                       unions[i].points, unions[i].faces, J );
                    break;
                }
                case MeshBoolean::CORK :{
                    igl::copyleft::cork::mesh_boolean( hull_a.points, hull_a.faces,
                                                       hull_b.points, hull_b.faces, boolean_op,
                                                       unions[i].points, unions[i].faces );
                    break;
                }
            }
        }
        std::cout << "created #unions = " << unions.size()  << std::endl;
        if( hulls.size() % 2 == 1 ){
            unions.push_back( hulls.back() );
        }
        done = unions.size() == 1;
        if( !done ){
            unions.swap(hulls);
        }
        std::cout << "end loop " << std::endl;
    }
    assert( unions.size() == 1 );

    points_out = std::move( unions.back().points );
    faces_out = std::move( unions.back().faces );
}


void thread_job_cgal( const std::vector<Hull>& hulls, std::vector<Hull>& unions, size_t start, size_t end, size_t thread_id ){
    for( int i = start; i < end; ++i ){
        size_t hull_index  = i * 2;
        const Hull& hull_a = hulls.at( hull_index );
        const Hull& hull_b = hulls.at( hull_index+1 );

        Eigen::VectorXi _;

        igl::copyleft::cgal::mesh_boolean( hull_a.points, hull_a.faces,
                                           hull_b.points, hull_b.faces, igl::MESH_BOOLEAN_TYPE_UNION,
                                           unions[i].points, unions[i].faces, _ );
    }
}

void thread_job_cork( const std::vector<Hull>& hulls, std::vector<Hull>& unions, size_t start, size_t end, size_t thread_id ){
    for( int i = start; i < end; ++i ){
        size_t hull_index  = i * 2;
        const Hull& hull_a = hulls.at( hull_index );
        const Hull& hull_b = hulls.at( hull_index+1 );


        igl::copyleft::cork::mesh_boolean( hull_a.points, hull_a.faces,
                                           hull_b.points, hull_b.faces, igl::MESH_BOOLEAN_TYPE_UNION,
                                           unions[i].points, unions[i].faces );
    }
}

void getConvexHullUnion_mt(Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                           Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out, MeshBoolean library){
    std::vector<Eigen::MatrixXd> points_vector;
    std::vector<Eigen::MatrixXi> faces_vector;
    std::vector<Hull> hulls, unions;

    ConvexHullCreator::getConvexHulls( points_in, segments, points_vector, faces_vector );

    igl::MeshBooleanType boolean_op = igl::MESH_BOOLEAN_TYPE_UNION;
    int parts = num_threads;

    hulls.resize( points_vector.size( ));
    for( size_t i = 0; i < points_vector.size(); ++i ){
        std::cout << "( " << points_vector[i].rows() << ", " << faces_vector[i].rows() << std::endl;
        hulls[i].points =  points_vector[i];
        hulls[i].faces  = faces_vector[i];
    }
    std::cout << "created hulls list" << std::endl;

//    std::sort( hulls.begin(), hulls.end() );
//    std::cout << "sorted" << std::endl;

    bool done = false;
    while( !done ){
        std::cout << "working on #hulls = " << hulls.size()  << std::endl;
        assert( hulls.size() > 1 );

        int numPairs = hulls.size() / 2;

        unions.clear();
        unions.resize( numPairs );

        // compute the actual number of threads needed and how many pairs must be computed
        // by each thread. If #pairs < #threads, just use less threads
        int actual_num_threads  = std::min( numPairs, num_threads);
        int pairs_per_thread    = numPairs / actual_num_threads;
        int remainder           = numPairs % actual_num_threads;
        bool run_mt             = pairs_per_thread >= 1;

        std::cout << " num pairs " << numPairs << "remainder " << remainder
                  << " # no threads" << actual_num_threads << " ppp " << pairs_per_thread << std::endl;

        // if a single pair remained, just use the main thread.
        if( run_mt ){
            int last_end = 0;

            std::vector< std::thread > threads(actual_num_threads - 1);

            // launch threads with #pairs_per_thread plus an additional pair if it is needed
            // to consume parts from the remainder.
            for( int i = 0; i < actual_num_threads - 1; ++i ){
                int start = last_end;
                last_end = start + pairs_per_thread + ( i < remainder ? 1 : 0 );

//                std::cout << "thread " << i << "will process data from " << start << " to " << last_end << std::endl;
                switch(library){
                    case MeshBoolean::CGAL : {
                        threads[i] = std::thread( thread_job_cgal, std::ref(hulls), std::ref( unions ), start, last_end, i );
                        break;
                    }
                    case MeshBoolean::CORK : {
                        threads[i] = std::thread( thread_job_cork, std::ref(hulls), std::ref( unions ), start, last_end, i );
                        break;
                    }
                }

            }
//            std::cout << "main thread " << "will process data from " << last_end << " to " << numPairs << std::endl;
            switch(library){
                case MeshBoolean::CGAL : {
                    thread_job_cgal( hulls, unions, last_end, numPairs, actual_num_threads - 1 );
                    break;
                }
                case MeshBoolean::CORK : {
                    thread_job_cork( hulls, unions, last_end, numPairs, actual_num_threads - 1 );
                    break;
                }
            }

            //Join threads
            for ( int i = 0; i < actual_num_threads - 1; ++i ) { threads[i].join(); }
        }else{
            std::cout << "going single thread " << std::endl;
            switch(library){
                case MeshBoolean::CGAL : {
                    thread_job_cgal( hulls, unions, 0, numPairs, 0 );
                    break;
                }
                case MeshBoolean::CORK : {
                    thread_job_cork( hulls, unions, 0, numPairs, 0 );
                    break;
                }
            }
        }

        std::cout << "created #unions = " << unions.size()  << std::endl;
        // if the number of hulls is odd, the last one will be added as union by itself
        if( hulls.size() % 2 == 1 ){
            unions.push_back( hulls.back() );
        }
        done = unions.size() == 1;
        if( !done ){
            unions.swap(hulls);
        }
        std::cout << "end loop " << std::endl;
    }
    assert( unions.size() == 1 );

    points_out = std::move( unions.back().points );
    faces_out = std::move(  unions.back().faces );
}

void getConvexUnion(Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                    Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out, MeshBoolean library, MultiThread mt){

    switch(mt){
    case MultiThread::MT : {
        getConvexHullUnion_mt( points_in, segments, points_out, faces_out, library );
        break;
    }
    case MultiThread::NO_MT : {
        getConvexHullUnion( points_in, segments, points_out, faces_out, library );
        break;
    }
    }
}

void getConvexHullUnionWithCGAL(Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                                Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out){
    getConvexUnion(points_in, segments, points_out, faces_out, MeshBoolean::CGAL, MultiThread::NO_MT );
}

void getConvexHullUnionWithCGAL_mt(Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                                   Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out){
    getConvexUnion(points_in, segments, points_out, faces_out, MeshBoolean::CGAL, MultiThread::MT );
}

void getConvexHullUnionWithCORK(Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                                Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out){
    getConvexUnion(points_in, segments, points_out, faces_out, MeshBoolean::CORK, MultiThread::NO_MT );
}

void getConvexHullUnionWithCORK_mt(Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                                   Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out){
    getConvexUnion(points_in, segments, points_out, faces_out, MeshBoolean::CORK, MultiThread::MT );
}

}
