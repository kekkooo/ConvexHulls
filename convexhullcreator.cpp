#include "convexhullcreator.h"
#include <chrono>

#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>

#include <polyhedron_converter.h>

typedef std::chrono::high_resolution_clock myclock;

void buildCGALPoints( const Eigen::MatrixXd &points_in, std::vector<CGAL_Point_3>& points_out ){
    for( long i = 0; i < points_in.rows(); ++i ){
        points_out.push_back(( CGAL_Point_3( points_in( i, 0 ), points_in( i, 1 ), points_in( i, 2 )) ));
    }
}

void PolyhedronToDenseVertexMatrix( Polyhedron_3& poly, Eigen::MatrixXd& points_out ){
    size_t no_vertices = poly.size_of_vertices();
    points_out.resize( no_vertices, 3 );

    size_t vertexId = 0;
    for( auto vertexIterator = poly.vertices_begin(); vertexIterator != poly.vertices_end(); ++vertexIterator, ++vertexId ){
        vertexIterator->id() = vertexId;
        points_out.row(vertexId) <<  CGAL::to_double( (*vertexIterator).point().x()),
                                CGAL::to_double( (*vertexIterator).point().y()),
                                CGAL::to_double( (*vertexIterator).point().z());
    }
}

void PolyhedronToDenseFacesMatrix( Polyhedron_3& poly, Eigen::MatrixXi& faces_out ){
    size_t no_faces    = poly.size_of_facets();
    faces_out.resize(  no_faces, 3 );

    size_t faceId = 0;
    for( auto faceIterator = poly.facets_begin(); faceIterator != poly.facets_end(); ++faceIterator, ++faceId ){
        assert( faceIterator->is_triangle());
        auto faceVertexIterator = (*faceIterator).facet_begin();
        size_t v0 = faceVertexIterator->vertex()->id();      ++faceVertexIterator;
        size_t v1 = faceVertexIterator->vertex()->id();      ++faceVertexIterator;
        size_t v2 = faceVertexIterator->vertex()->id();      ++faceVertexIterator;
        assert( faceVertexIterator  == (*faceIterator).facet_begin( ));
//        std::cout << fid << ") " << v0 << ", " << v1 << ", " << v2 << std::endl;
        faces_out.row( faceId ) << v0, v1, v2;
    }
}

void PolyhedronToDenseMatrices( Polyhedron_3& poly, Eigen::MatrixXd& points_out, Eigen::MatrixXi& faces_out ){
    PolyhedronToDenseVertexMatrix( poly, points_out );
    PolyhedronToDenseFacesMatrix( poly, faces_out );
}

void BuildSegmentedPointList( const Eigen::MatrixXd &points_in, const Segmentation::Segments &segments,
                              ConvexHullCreator::SegmentedPointList& segmentedPoints ){
    // build the segmented list of points
    for( const auto& item : segments ){
        std::vector<CGAL_Point_3> points;
        for( size_t id : item.second ){
            points.push_back( CGAL_Point_3( points_in( id, 0 ), points_in( id, 1 ), points_in( id, 2 )));
        }
        segmentedPoints[item.first] = std::move( points );
    }
}



void ConvexHullCreator::getConvexHull( Eigen::MatrixXd &points_in,
                                       Eigen::MatrixXd &points_out,
                                       Eigen::MatrixXi &faces_out){
    auto t0 = myclock::now();

    Polyhedron_3 hull;
    std::vector<CGAL_Point_3> points( points_in.rows( ));
    buildCGALPoints( points_in, points );


//    std::vector<CGAL_Point_3> points( points_in.rows( ));
//    for( long i = 0; i < points_in.rows(); ++i ){
//        points.push_back(( CGAL_Point_3( points_in( i, 0 ), points_in( i, 1 ), points_in( i, 2 )) ));
//    }

    CGAL::convex_hull_3( points.begin(), points.end(), hull );    

    PolyhedronToDenseMatrices( hull, points_out, faces_out );

//    size_t vid = 0;
//    for( Polyhedron_3::Vertex_iterator vit = hull.vertices_begin(); vit != hull.vertices_end(); ++vit, ++vid ){
//        vit->id() = vid;
//        points_out.row(vid) <<  CGAL::to_double( (*vit).point().x()),
//                                CGAL::to_double( (*vit).point().y()),
//                                CGAL::to_double( (*vit).point().z());
//    }

//    std::cout << "points added " << no_vertices << std::endl;

//    size_t fid = 0;
//    for( auto fit = hull.facets_begin(); fit != hull.facets_end(); ++fit, ++fid ){
//        assert( fit->is_triangle());
//        auto h = (*fit).facet_begin();
//        size_t v0 = h->vertex()->id();      ++h;
//        size_t v1 = h->vertex()->id();      ++h;
//        size_t v2 = h->vertex()->id();      ++h;
//        assert( h  == (*fit).facet_begin( ));
////        std::cout << fid << ") " << v0 << ", " << v1 << ", " << v2 << std::endl;
//        faces_out.row(fid) << v0, v1, v2;
//    }

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

//    // build the segmented list of points
//    for( const auto& item : segments ){
//        std::vector<CGAL_Point_3> points;
//        for( size_t i : item.second ){
////            std::cout << "adding point " << i;
//            points.push_back( CGAL_Point_3( points_in( i, 0 ), points_in( i, 1 ), points_in( i, 2 )));
////            std::cout << " = " << points.back() << std::endl;
//        }
//        seg_points[item.first] = std::move( points );
//    }

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


    size_t no_faces = 0, no_vertices = 0;
    for( const auto& hull : hulls ){
        no_faces    += hull.size_of_facets();
        no_vertices += hull.size_of_vertices();
    }
    points_out.resize( no_vertices, 3 );
    faces_out.resize(  no_faces, 3 );


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

    size_t no_segments = s.size();
    std::vector<Polyhedron_3_no_id>                 hulls( no_segments );
    std::vector<Nef_Polyhedron_3>                   nef_hulls;
    std::map< size_t, std::vector<CGAL_Point_3> >   seg_points;
//    std::vector<CGAL_Point_3>                       points( points_in.rows( ));


    BuildSegmentedPointList( points_in, segments, seg_points );

//    // build the segmented list of points
//    for( const auto& item : s ){
//        std::vector<CGAL_Point_3> points;
//        for( size_t i : item.second ){
////            std::cout << "adding point " << i;
//            points.push_back( CGAL_Point_3( points_in( i, 0 ), points_in( i, 1 ), points_in( i, 2 )));
////            std::cout << " = " << points.back() << std::endl;
//        }
//        seg_points[item.first] = std::move( points );
//    }

    auto ta0 = myclock::now();
    // build the convex_hulls
    for( const auto& item : seg_points ){
        auto ta = myclock::now();
        std::cout << "building convex hull for segment " << item.first << std::endl;
        const auto& points = item.second;
        std::cout << "with " << points.size() << " points " << std::endl;
        Polyhedron_3_no_id hull;
        CGAL::convex_hull_3( points.begin(), points.end(), hull );

        if( hull.is_closed() ){
            hulls.push_back( hull );
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
    toInexactPolyhedron( exact_convex_union, convex_union );
    std::cout << "union hull converted to inexact polyhedron " << std::endl;
    remesh( convex_union, 10.0 );
    std::cout << "remeshed " << std::endl;

    PolyhedronToDenseMatrices(convex_union, points_out, faces_out );

//    // Build the union of the convex hulls
//    size_t no_vertices = convex_union.size_of_vertices();
//    size_t no_faces    = convex_union.size_of_facets();

//    points_out.resize( no_vertices, 3 );
//    faces_out.resize(  no_faces, 3 );

//    size_t vid = 0;
//    for( Double_Polyhedron_3::Vertex_iterator vit = convex_union.vertices_begin(); vit != convex_union.vertices_end(); ++vit, ++vid ){
//        vit->id() = vid;
//        points_out.row(vid) <<  (*vit).point().x(),
//                                (*vit).point().y(),
//                                (*vit).point().z();
//    }
////    std::cout << "points added " << no_vertices << std::endl;

//    size_t fid = 0;
//    for( auto fit = convex_union.facets_begin(); fit != convex_union.facets_end(); ++fit, ++fid ){
//        assert( fit->is_triangle());
//        auto h = (*fit).facet_begin();
//        size_t v0 = h->vertex()->id();      ++h;
//        size_t v1 = h->vertex()->id();      ++h;
//        size_t v2 = h->vertex()->id();      ++h;
//        assert( h  == (*fit).facet_begin( ));
////        std::cout << fid << ") " << v0 << ", " << v1 << ", " << v2 << std::endl;
//        faces_out.row(fid) << v0, v1, v2;
//    }

    auto t1  = myclock::now();
    long span = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "all process took " << span << "millis" << std::endl;

}

void ConvexHullCreator::remesh(Double_Polyhedron_3& in, double target_edge_length ){

    std::cout << "trying to remesh " << std::endl;
    std::cout << "a polygon of " << in.size_of_vertices() << " vertices and "
                                 << in.size_of_facets() << "faces " << std::endl;
//    std::vector<Double_Polyhedron_3::Facet_handle> new_facets;
//    std::vector<Double_Polyhedron_3::Vertex_handle> new_verts;

    std::cout << in.size_of_vertices();

    CGAL::Polygon_mesh_processing::split_long_edges( edges(in), 10.0, in);
    std::cout << "split long edeges done" << std::endl;
    CGAL::Polygon_mesh_processing::isotropic_remeshing( faces(in), 10.0, in,
                                                        CGAL::Polygon_mesh_processing::parameters::protect_constraints(true));

//    CGAL::Polygon_mesh_processing::refine( in,
//                                           faces(in),
//                                           std::back_inserter(new_facets),
//                                           std::back_inserter(new_verts),
//                                           CGAL::Polygon_mesh_processing::parameters::density_control_factor(3.0));
////    CGAL::Polygon_mesh_processing::fair()
}

void ConvexHullCreator::toInexactPolyhedron( Polyhedron_3 &in, Double_Polyhedron_3 &out){
    poly_copy<Double_Polyhedron_3, Polyhedron_3>( out, in );
}

void ConvexHullCreator::getConvexHulls( Eigen::MatrixXd &points_in, const Segmentation::Segments& s,
                                        std::vector<Eigen::MatrixXd> &points_out,
                                        std::vector<Eigen::MatrixXi> &faces_out){
    auto t0 = myclock::now();

    size_t no_segments = s.size();
    std::vector<Polyhedron_3>                       hulls( no_segments );
    std::map< size_t, std::vector<CGAL_Point_3> >   seg_points;

    std::cout << "there are " << no_segments << " segments " << std::endl;
    Segmentation::buildSegmentedPointsList( points_in, s, seg_points );

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

}

//void ConvexHullCreator::getOOBs( Eigen::MatrixXd &points_in, const Segmentation::Segments &s,
//                                 Eigen::MatrixXd &points_out, Eigen::MatrixXi &faces_out){

//    auto t0 = myclock::now();

//    size_t no_segments = s.size();
//    std::vector<Polyhedron_3>                       hulls( no_segments );
//    std::map< size_t, std::vector<CGAL_Point_3> >   seg_points;

//    std::cout << "there are " << no_segments << " segments " << std::endl;
//    Segmentation::buildSegmentedPointsList( points_in, s, seg_points );

//    buildConvexHulls( seg_points, hulls );

//    size_t ov = 0, of = 0;
//    points_out.resize( no_segments * 8, 3 );
//    faces_out.resize(    no_segments * 6, 4 );
//    for( auto& hull : hulls ){
//        Eigen::MatrixXd v( 8, 3 );
//        Eigen::MatrixXi f( 6, 4 );

//        Eigen::MatrixXd hv( hull.size_of_vertices(), 3 );
//        Eigen::MatrixXi hf( hull.size_of_facets(), 3 );

//        size_t vid = 0, fid = 0;
//        for( Polyhedron_3::Vertex_iterator vit = hull.vertices_begin(); vit != hull.vertices_end(); ++vit, ++vid ){
//            vit->id() = vid;
//            hv.row(vid) <<  CGAL::to_double( (*vit).point().x( )),
//                            CGAL::to_double( (*vit).point().y( )),
//                            CGAL::to_double( (*vit).point().z( ));
//        }

//        for( auto fit = hull.facets_begin(); fit != hull.facets_end(); ++fit, ++fid ){
//            assert( fit->is_triangle());
//            auto h = (*fit).facet_begin();
//            size_t v0 = h->vertex()->id();      ++h;
//            size_t v1 = h->vertex()->id();      ++h;
//            size_t v2 = h->vertex()->id();      ++h;
//            assert( h  == (*fit).facet_begin( ));
//    //        std::cout << fid << ") " << v0 << ", " << v1 << ", " << v2 << std::endl;
//            hf.row( fid ) << v0, v1, v2;
//        }

//        OBB::buildOBB( hv, hf, v, f );

//        size_t offset = ov;
//        points_out.row( ov++ ) = v.row( 0 );  points_out.row( ov++ ) = v.row( 1 );
//        points_out.row( ov++ ) = v.row( 2 );  points_out.row( ov++ ) = v.row( 3 );
//        points_out.row( ov++ ) = v.row( 4 );  points_out.row( ov++ ) = v.row( 5 );
//        points_out.row( ov++ ) = v.row( 6 );  points_out.row( ov++ ) = v.row( 7 );

//        f = f.array() + offset;

//        faces_out.row( of++ ) = f.row( 0 ); faces_out.row( of++ ) = f.row(1);
//        faces_out.row( of++ ) = f.row( 2 ); faces_out.row( of++ ) = f.row(3);
//        faces_out.row( of++ ) = f.row( 4 ); faces_out.row( of++ ) = f.row(5);
//    }

//    assert( ov == points_out.rows() );
//    assert( of == faces_out.rows() );

//    auto t1  = myclock::now();
//    long span = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
//    std::cout << "all process took " << span << "millis" << std::endl;
//}

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
