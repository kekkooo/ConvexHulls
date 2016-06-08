#include "cgal_to_eigen.h"

namespace Utilities{

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

void PolyhedronToDenseMatrices( Polyhedron_3 &poly, Eigen::MatrixXd &points_out,
                                Eigen::MatrixXi &faces_out){
    PolyhedronToDenseVertexMatrix( poly, points_out );
    PolyhedronToDenseFacesMatrix( poly, faces_out );
}

}
