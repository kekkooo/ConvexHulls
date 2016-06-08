#include "convexhullcreator.h"
#include <chrono>

#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>

#include <polyhedron_converter.h>
#include <cgal_to_eigen.h>

typedef std::chrono::high_resolution_clock myclock;

void buildCGALPoints( const Eigen::MatrixXd &points_in, std::vector<CGAL_Point_3>& points_out ){
    for( long i = 0; i < points_in.rows(); ++i ){
        points_out.push_back(( CGAL_Point_3( points_in( i, 0 ), points_in( i, 1 ), points_in( i, 2 )) ));
    }
}

void BuildSegmentedPointList( const Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                              ConvexHullCreator::SegmentedPointList& segmentedPoints ){    
    for( const auto& item : segments ){
        std::vector<CGAL_Point_3> points;
        for( size_t id : item.second ){
            points.push_back( CGAL_Point_3( points_in( id, 0 ), points_in( id, 1 ), points_in( id, 2 )));
        }
        segmentedPoints[item.first] = std::move( points );
    }
}

void ConvexHullCreator::getConvexHull( Eigen::MatrixXd &points_in, Eigen::MatrixXd &points_out,
                                       Eigen::MatrixXi &faces_out){
    auto t0 = myclock::now();
    Polyhedron_3 hull;
    std::vector<CGAL_Point_3> points( points_in.rows( ));

    buildCGALPoints( points_in, points );
    CGAL::convex_hull_3( points.begin(), points.end(), hull );    
    Utilities::PolyhedronToDenseMatrices( hull, points_out, faces_out );

    auto t3  = myclock::now();
    long span4 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count();
    std::cout << "all process took " << span4 << "millis" << std::endl;
}

void ConvexHullCreator::getConvexHull( Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                                       Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out){
    auto t0 = myclock::now();

    size_t no_segments = segments.size();
    std::vector<Polyhedron_3>   hulls( no_segments );
    SegmentedPointList          seg_points;

    BuildSegmentedPointList( points_in, segments, seg_points );

    // build the convex_hulls
    for( const auto& item : seg_points ){
        Polyhedron_3 hull;
        auto ta = myclock::now();
        std::cout << "building convex hull for segment " << item.first << std::endl;
        std::cout << "with " << item.second.size() << " points " << std::endl;

        const auto& points = item.second;
        CGAL::convex_hull_3( points.begin(), points.end(), hull );
        hulls.push_back( hull );

        auto tb  = myclock::now();
        long ab = std::chrono::duration_cast<std::chrono::milliseconds>(tb - ta).count();
        std::cout << "took " << ab << "millis" << std::endl;
    }

    size_t no_faces = 0, no_vertices = 0;
    for( const auto& hull : hulls ){
        no_faces    += hull.size_of_facets();
        no_vertices += hull.size_of_vertices();
    }
    points_out.resize( no_vertices, 3 );
    faces_out.resize(  no_faces, 3 );

    // puts all the convex hulls into a single eigen matrix
    size_t vid = 0, fid = 0;
    for(  auto& hull : hulls ){
        for( Polyhedron_3::Vertex_iterator vit = hull.vertices_begin(); vit != hull.vertices_end(); ++vit, ++vid ){
            vit->id() = vid;
            points_out.row(vid) << CGAL::to_double( (*vit).point().x()),
                                   CGAL::to_double( (*vit).point().y()),
                                   CGAL::to_double( (*vit).point().z());
        }

        for( auto fit = hull.facets_begin(); fit != hull.facets_end(); ++fit, ++fid ){
            assert( fit->is_triangle());
            auto h = (*fit).facet_begin();
            size_t v0 = h->vertex()->id();      ++h;
            size_t v1 = h->vertex()->id();      ++h;
            size_t v2 = h->vertex()->id();      ++h;
            assert( h  == (*fit).facet_begin( ));
    //        std::cout << fid << ") " << v0 << ", " << v1 << ", " << v2 << std::endl;
            faces_out.row(fid) << v0, v1, v2;
        }
    }

    auto t1  = myclock::now();
    long span = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "all process took " << span << "millis" << std::endl;
}

void ConvexHullCreator::getConvexHullUnion( Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                                            Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out ){
    auto t0 = myclock::now();

    size_t no_segments = segments.size();
    //std::vector<Polyhedron_3_no_id>                 hulls( no_segments );
    std::vector<Nef_Polyhedron_3>                   nef_hulls;
    std::map< size_t, std::vector<CGAL_Point_3> >   seg_points;

    BuildSegmentedPointList( points_in, segments, seg_points );

    auto ta0 = myclock::now();
    // build the convex_hulls
    for( const auto& item : seg_points ){
        Polyhedron_3_no_id hull;
        auto ta = myclock::now();
        std::cout << "building convex hull for segment " << item.first << std::endl;
        std::cout << "with " << item.second.size() << " points " << std::endl;

        const auto& points = item.second;
        CGAL::convex_hull_3( points.begin(), points.end(), hull );

        if( hull.is_closed() ){
            //hulls.push_back( hull );
            Nef_Polyhedron_3 nef_hull( hull );
            nef_hulls.push_back( nef_hull );
        }

        auto tb  = myclock::now();
        long ab = std::chrono::duration_cast<std::chrono::milliseconds>(tb - ta).count();
        std::cout << "took " << ab << "millis" << std::endl;
    }
    auto tb0  = myclock::now();
    long ab0 = std::chrono::duration_cast<std::chrono::milliseconds>(tb0 - ta0).count();
    std::cout << "Computation of all convex hulls took " << ab0 << "millis" << std::endl;

    // inefficient way to do it
    Polyhedron_3        exact_convex_union;
    Double_Polyhedron_3 convex_union;
    Nef_Polyhedron_3 nef_union( Nef_Polyhedron_3::EMPTY );

    size_t iter = 0;
    for( auto& nh : nef_hulls ) {
        auto ta = myclock::now();

        nef_union = nef_union + nh;

        auto tb  = myclock::now();
        long ab = std::chrono::duration_cast<std::chrono::milliseconds>(tb - ta).count();
        std::cout << "union " << iter++ << "done in "  << ab << "millis" << std::endl;

    }
    nef_union.convert_to_Polyhedron( exact_convex_union );
    std::cout << "union hull converted to polyhedron " << std::endl;
    //toInexactPolyhedron( exact_convex_union, convex_union );
    //std::cout << "union hull converted to inexact polyhedron " << std::endl;

    //    remesh( convex_union, 10.0 );
//    std::cout << "remeshed " << std::endl;

    //Utilities::PolyhedronToDenseMatrices( convex_union, points_out, faces_out );
    Utilities::PolyhedronToDenseMatrices( exact_convex_union, points_out, faces_out );

    auto t1  = myclock::now();
    long span = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "all process took " << span << "millis" << std::endl;

}

//void ConvexHullCreator::remesh(Double_Polyhedron_3& in, double target_edge_length ){

//    std::cout << "trying to remesh " << std::endl;
//    std::cout << "a polygon of " << in.size_of_vertices() << " vertices and "
//                                 << in.size_of_facets() << "faces " << std::endl;
//    std::cout << in.size_of_vertices();

//    CGAL::Polygon_mesh_processing::split_long_edges( edges(in), 10.0, in);
//    std::cout << "split long edeges done" << std::endl;
//    CGAL::Polygon_mesh_processing::isotropic_remeshing( faces(in), 10.0, in,
//                                                        CGAL::Polygon_mesh_processing::parameters::protect_constraints(true));
//}

void ConvexHullCreator::toInexactPolyhedron( Polyhedron_3 &in, Double_Polyhedron_3 &out){
    poly_copy<Double_Polyhedron_3, Polyhedron_3>( out, in );
}

void ConvexHullCreator::getConvexHulls( Eigen::MatrixXd &points_in, const Segmentation::Segments& segments,
                                        std::vector<Eigen::MatrixXd> &points_out,
                                        std::vector<Eigen::MatrixXi> &faces_out){
    auto t0 = myclock::now();

    size_t no_segments = segments.size();
    std::vector<Polyhedron_3>                       hulls( no_segments );
    std::map< size_t, std::vector<CGAL_Point_3> >   seg_points;

    std::cout << "there are " << no_segments << " segments " << std::endl;
    Segmentation::buildSegmentedPointsList( points_in, segments, seg_points );


    buildConvexHulls( seg_points, hulls );
    points_out.reserve( no_segments );
    faces_out.reserve( no_segments );
    for( auto& hull : hulls ){
        Eigen::MatrixXd vertices( hull.size_of_vertices(), 3 );
        Eigen::MatrixXi faces( hull.size_of_facets(), 3 );
        Utilities::PolyhedronToDenseMatrices(hull, vertices, faces );
        points_out.push_back(vertices);
        faces_out.push_back(faces);
    }

    /*
    // build the convex_hulls
    for( const auto& item : seg_points ){
        std::cout << "building convex hull for segment " << item.first << std::endl;
        const auto& points = item.second;
        Polyhedron_3 hull;
        CGAL::convex_hull_3( points.begin(), points.end(), hull );
        hulls.push_back( hull );
    }

    size_t no_faces = 0, no_vertices = 0;
    for( auto& hull : hulls ){
        Eigen::MatrixXd vertices( hull.size_of_vertices(), 3 );
        Eigen::MatrixXi faces( hull.size_of_facets(), 3 );

        size_t vid = 0, fid = 0;
        for( Polyhedron_3::Vertex_iterator vit = hull.vertices_begin(); vit != hull.vertices_end(); ++vit, ++vid ){
            vit->id() = vid;
            vertices.row(vid) << CGAL::to_double( (*vit).point().x()),
                                 CGAL::to_double( (*vit).point().y()),
                                 CGAL::to_double( (*vit).point().z());
        }

        for( auto fit = hull.facets_begin(); fit != hull.facets_end(); ++fit, ++fid ){
            assert( fit->is_triangle());
            auto h = (*fit).facet_begin();
            size_t v0 = h->vertex()->id();      ++h;
            size_t v1 = h->vertex()->id();      ++h;
            size_t v2 = h->vertex()->id();      ++h;
            assert( h  == (*fit).facet_begin( ));
    //        std::cout << fid << ") " << v0 << ", " << v1 << ", " << v2 << std::endl;
            faces.row(fid) << v0, v1, v2;
        }

        faces_out.push_back( faces );
        points_out.push_back( vertices );
    }

    auto t1  = myclock::now();
    long span = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "all process took " << span << "millis" << std::endl;
    */
}

void ConvexHullCreator::buildConvexHulls( const Segmentation::SegmentedPointList &seg_points,
                                          std::vector<Polyhedron_3> &hulls){
    hulls.clear();
    hulls.reserve( seg_points.size( ));
    // build the convex_hulls
    for( const auto& item : seg_points ){
        auto ta = myclock::now();
        std::cout << "building convex hull for segment " << item.first << std::endl;

        const auto& points = item.second;
        std::cout << "with " << points.size() << " points " << std::endl;

        Polyhedron_3 hull;
        CGAL::convex_hull_3( points.begin(), points.end(), hull );
        hulls.push_back( hull );

        auto tb  = myclock::now();
        long ab = std::chrono::duration_cast<std::chrono::milliseconds>(tb - ta).count();
        std::cout << "took " << ab << "millis" << std::endl;
    }

}
