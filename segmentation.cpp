#include "segmentation.h"
#include <iostream>
#include <fstream>

namespace Segmentation {

void buildSegmentedPointsList( Eigen::MatrixXd &points_in, const Segments &segments,
                                             SegmentedPointList &segmentedPointList){
    for( const auto& item : segments ){
        std::vector<CGAL_Point_3> points;
        for( size_t i : item.second ){
            ////std::cout << "adding point " << i;
            points.push_back( CGAL_Point_3( points_in( i, 0 ), points_in( i, 1 ), points_in( i, 2 )));
    //            //std::cout << " = " << points.back() << std::endl;
        }
        segmentedPointList[item.first] = std::move( points );
    }
}




void loadSegments( std::string filename, Segments& segments, const Eigen::MatrixXi& f ) {
    segments.clear();
    std::ifstream infile( filename.c_str() );
    assert( infile.good() );

    std::string header, kind;
    size_t num_verts, num_segments, vid, segid;

    // Read file from animation data. The file format has an header and then VID and SegID
    if( f.rows() == 0 ) {
        infile  >> header;
        assert( std::strcmp( header.c_str(), "SEGMENT" ) == 0);

        infile >> num_verts >> num_segments;
        infile >> kind;
        //std::cout << header << "   " << kind << std::endl;
        assert( std::strcmp( kind.c_str(), "MAX" ) == 0);
        //std::cout << "there are " << num_verts << " vertices ";
        while( infile >> vid >> segid ){
            if( segments.count(segid) == 0 ){
                segments[segid] = std::set<size_t>();
            }
            segments[segid].insert( vid );
            ////std::cout << vid << " " << segid << std::endl;
        }
    } else {
        // Read file from Princeton Dataset. The file format has f.size() rows with just SegID.
        const uint size = f.rows();

        for( uint i = 0; i < size; ++i ) {
            infile >> segid;
            if( segments.count(segid) == 0 ){
                segments[segid] = std::set<size_t>();
            }
            for( uint j = 0; j < f.cols(); ++j ) {
                segments[segid].insert( f( i, j ) );
            }
        }

    }
    infile.close();
}




void joinSegments(Segments &segments, Eigen::MatrixXd &v, Eigen::MatrixXi &f ){
    // for each vertex, if all its neighbors belong to the same segment
    Segments to_add;
    std::vector<int> vert_to_segment( v.rows(), -1 );

    // create vertex to segment correspondance
    for( const auto&  item : segments ){
        for( size_t vid : item.second ){
            ////std::cout << "vertex " << vid << " maps to segment " << item.first << std::endl;
            vert_to_segment[vid] = item.first;
        }
    }

//    //std::cout << f;

    for( int i = 0; i < f.rows(); ++i ){
        // make kocho happy,
        // rewrite this as a for loop

        size_t v0 = f( i, 0 );  int s0 = vert_to_segment[v0];   assert( s0 != -1 );
        size_t v1 = f( i, 1 );  int s1 = vert_to_segment[v1];   assert( s1 != -1 );
        size_t v2 = f( i, 2 );  int s2 = vert_to_segment[v2];   assert( s2 != -1 );

        if( to_add.count( s0 ) == 0 ){
            to_add[s0] = std::set<size_t>();
        }
        to_add[s0].insert(v1);
        to_add[s0].insert(v2);

        if( to_add.count( s1 ) == 0 ){
            to_add[s1] = std::set<size_t>();
        }
        to_add[s1].insert(v0);
        to_add[s1].insert(v2);

        if( to_add.count( s2 ) == 0 ){
            to_add[s2] = std::set<size_t>();
        }
        to_add[s2].insert(v0);
        to_add[s2].insert(v1);
    }

    for( const auto&  item : to_add ){
        for( size_t vid : item.second ){
            assert( segments.count(item.first) != 0 );
            segments[item.first].insert( vid );
        }
    }

}

} // namespace Segmentation
