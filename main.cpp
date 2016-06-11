//#include <QCoreApplication>

#include <iostream>
#include <fstream>
#include <vector>

#include <igl/grad.h>
#include <igl/jet.h>
#include <igl/massmatrix.h>
#include <igl/per_vertex_normals.h>
#include <igl/readDMAT.h>
#include <igl/readOFF.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/repdiag.h>
#include <igl/viewer/Viewer.h>
#include <igl/slice.h>
#include <igl/AABB.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/barycentric_coordinates.h>
#include <igl/barycentric_to_global.h>
#include <igl/invert_diag.h>
#include <igl/principal_curvature.h>
#include <igl/doublearea.h>
#include <igl/copyleft/cgal/CSGTree.h>
#include <igl/signed_distance.h>

#include <convexhullcreator.h>
#include <segmentation.h>
#include <common_paths.h>
#include <iglmeshboolean.h>

using namespace Eigen;
using namespace std;

typedef std::chrono::high_resolution_clock myclock;

// trimesh
Eigen::MatrixXd VT;
Eigen::MatrixXi FT;

// convex hull mesh
Eigen::MatrixXd VH;
Eigen::MatrixXi FH;

// OOBs
Eigen::MatrixXd VO;
Eigen::MatrixXi FO;
Eigen::MatrixXd PQC0, PQC1, PQC2; // quad edges

igl::viewer::Viewer viewer;

#define DANA
//#define SAVE_PARTS

#ifdef CAPSULE
    PathsStruct current_model = CommonPaths::capsule;
#endif
#ifdef ASURA
    PathsStruct current_model = CommonPaths::getAsura();
#endif
#ifdef HAND
    PathsStruct current_model = CommonPaths::hand;
#endif
#ifdef DANA
    PathsStruct current_model = CommonPaths::getDana();
#endif
#ifdef JEFF
     PathsStruct current_model = CommonPaths::getJeff();
#endif

void saveParts( const Segmentation::Segments& seg ){
    std::vector<Eigen::MatrixXd> points_out;
    std::vector<Eigen::MatrixXi> faces_out;
    ConvexHullCreator::getConvexHulls( VT, seg, points_out, faces_out );

    assert( points_out.size() == faces_out.size( ));
    for( size_t i = 0; i < points_out.size(); ++i ){
        string filename = current_model.hull_folder + current_model.hull_name_template + to_string(i) + ".obj";
        igl::writeOBJ( filename, points_out[i], faces_out[i] );
    }
}

//void setOOBs( ){
//    // Add edges of the Oriented Bounding Box
//    igl::slice( VO, FO.col(0).eval(), 1, PQC0);
//    igl::slice( VO, FO.col(1).eval(), 1, PQC1);
//    igl::slice( VO, FO.col(2).eval(), 1, PQC2);

//    viewer.data.add_edges( PQC0, PQC1, Eigen::RowVector3d( 0, 0, 0 ));
//    viewer.data.add_edges( PQC1, PQC2, Eigen::RowVector3d( 0, 0, 0 ));
//    viewer.data.add_edges( PQC2, PQC3, Eigen::RowVector3d( 0, 0, 0 ));
//    viewer.data.add_edges( PQC3, PQC0, Eigen::RowVector3d( 0, 0, 0 ));
//}

int main(int argc, char *argv[]){

    std::cout << "model paths:" << std::endl
              << current_model.mesh_to_load << std::endl
              << current_model.mesh_to_save << std::endl
              << current_model.segmentation << std::endl;


    igl::readOBJ( current_model.mesh_to_load , VT, FT );    
////    igl::readOFF( mesh_to_load , VT, FT );

    Segmentation::Segments seg;
    Segmentation::loadSegments( current_model.segmentation, seg );
    Segmentation::joinSegments( seg, VT, FT );    

    const auto &key_down = [&seg]( igl::viewer::Viewer &viewer, unsigned char key, int mod )->bool{
        bool convexhull_available = false;
        switch(key)
        {

        case 'c':
        case 'C':{
            std::cout << "CGAL approach" << std::endl;
            ConvexHullCreator::getConvexHullUnion( VT, seg, VH, FH );

            std::cout << FO.rows() << ", " << FO.cols() <<std::endl;
            std::cout << FO;

#ifdef SAVE_PARTS
            saveParts( seg );
#endif
            igl::writeOBJ( current_model.mesh_to_save, VH, FH);
            convexhull_available = true;
            break;
        }
        case 'b':
        case 'B':{
            std::cout << "IGL::mesh_boolean approach" << std::endl;
            auto t0 = myclock::now();

//            iglMeshBoolean::getConvexHullUnion_mt(VT, seg, VH, FH );
            iglMeshBoolean::getConvexHullUnionWithCGAL_mt(VT, seg, VH, FH);

            auto t1  = myclock::now();
            long span = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
            std::cout << "whole process took " << span << "millis" << std::endl;

            igl::writeOBJ( current_model.mesh_to_save, VH, FH);
            convexhull_available = true;

            break;
        }
        case 'i':
        case 'I':{
            std::cout << "IGL::CsgTree approach" << std::endl;            
            break;
        }
        case 'k':
        case 'K':{
            std::cout << "IGL::Cork approach" << std::endl;
            auto t0 = myclock::now();

            iglMeshBoolean::getConvexHullUnionWithCORK( VT, seg, VH, FH );

            auto t1  = myclock::now();
            long span = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
            std::cout << "whole process took " << span << "millis" << std::endl;

            igl::writeOBJ( current_model.mesh_to_save, VH, FH);
            convexhull_available = true;
            break;
        }
        }

        if(convexhull_available){            

//            Eigen::VectorXd S;
//            Eigen::VectorXi I;
//            Eigen::MatrixXd C, N;
//            Eigen::MatrixXd Colors;
//            igl::signed_distance( VT, VH, FH, igl::SIGNED_DISTANCE_TYPE_WINDING_NUMBER, S, I, C, N );
//            std::cout << "distance computed" << std::endl;
//            // Compute per-vertex colors
//            igl::jet( S, true, Colors );
//            viewer.data.clear();
//            viewer.data.set_mesh( C, FT );
//            viewer.data.set_colors( Colors );
            igl::slice( VH, FH.col(0).eval(), 1, PQC0 );
            igl::slice( VH, FH.col(1).eval(), 1, PQC1 );
            igl::slice( VH, FH.col(2).eval(), 1, PQC2 );
            viewer.data.add_edges(PQC0, PQC1, Eigen::RowVector3d(0,0,0));
            viewer.data.add_edges(PQC1, PQC2, Eigen::RowVector3d(0,0,0));
            viewer.data.add_edges(PQC1, PQC0, Eigen::RowVector3d(0,0,0));
        }

        return true;
    };


    // Use original normals as pseudo-colors
    MatrixXd N;
    igl::per_vertex_normals( VT, FT, N );
    std::cout << "normals ok" << std::endl;

    MatrixXd C = N.rowwise().normalized().array()*0.5+0.5;
    std::cout << "colors ok" << std::endl;

    viewer.data.set_mesh( VT, FT );
    std::cout << "mesh has been set" << std::endl;

//    setOOBs();

    viewer.data.set_colors(C);
    viewer.callback_key_down = key_down;
    return viewer.launch();
}
