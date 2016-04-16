#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include<string>
#include <Eigen/Dense>
#include <commontypedefs.h>

namespace Segmentation
{
    typedef std::map< size_t, std::set< size_t> >                       Segments;
    typedef std::map< size_t, std::vector<CGAL_Point_3> >               SegmentedPointList;

    void buildSegmentedPointsList(Eigen::MatrixXd &points_in, const Segments& segments, SegmentedPointList& segmentedPointList );
    void loadSegments( std::string filename, Segments& segments );
    void joinSegments( Segments& segments, Eigen::MatrixXd& v, Eigen::MatrixXi& f );

}

#endif // SEGMENTATION_H
