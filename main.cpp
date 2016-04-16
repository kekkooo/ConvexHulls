//#include <QCoreApplication>

#include <iostream>
#include <fstream>

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

#include <convexhullcreator.h>
#include <segmentation.h>
#include <obb.h>

using namespace Eigen;
using namespace std;

// trimesh
Eigen::MatrixXd VT;
Eigen::MatrixXi FT;

// convex hull mesh
Eigen::MatrixXd VH;
Eigen::MatrixXi FH;

// OOBs
Eigen::MatrixXd VO;
Eigen::MatrixXi FO;
Eigen::MatrixXd PQC0, PQC1, PQC2, PQC3; // quad edges

igl::viewer::Viewer viewer;

#define CAPSULE
//#define SAVE_PARTS

void improveTriangulation( MatrixXd &V, MatrixXi &F ){

    VectorXd area;
    igl::doublearea(V, F, area);
    area /= 2.0;
    double area_avg   = area.mean();
    double sigma      = sqrt((( area.array() - area_avg ) / area_avg ).square().mean( ));
    VectorXd zscore   = (area.array() - area_avg ) / sigma;

    std::cout << "mean area " << area_avg << " sigma : " << sigma << std::endl;
    for( int i = 0; i < F.rows(); ++i ){
        std::cout << "face " << i << " has area " << area(i) << " and zscore " << zscore(i) << std::endl;
    }

}

int main(int argc, char *argv[]){

#ifdef NICOLA
    string mesh_to_load = "./Data/Dana.obj";
    string mesh_to_save = "./Data/dana_chs2.obj";
    string segmentation = "./Data/Dana.segm";
#endif

#ifdef CAPSULE
    string mesh_to_load = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/CapsuleSegmentation/Capsula.obj";
    string mesh_to_save = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/CapsuleSegmentation/Capsula_chs_union.obj";
    string segmentation = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/CapsuleSegmentation/Capsula.segm";
#ifdef SAVE_PARTS
    string hull_folder  = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/CapsuleSegmentation/";
    string hull_name_template = "CapsulaPart";
#endif
#endif

#ifdef ASURA
    string mesh_to_load = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/AsuraSegmentation/AsuraKekko.obj";
    string mesh_to_save = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/AsuraSegmentation/AsuraKekko_chs_union.obj";
    string segmentation = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/AsuraSegmentation/AsuraKekko.segm";
#ifdef SAVE_PARTS
    string hull_folder  = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/AsuraSegmentation/";
    string hull_name_template = "AsuraPart";
#endif
#endif

#ifdef DANA
    string mesh_to_load = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/Dana/dana.obj";
    string mesh_to_save = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/Dana/dana_chs2.obj";
    string segmentation = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/Dana/dana.segm";
#endif

#ifdef NOISY_DANA
    string mesh_to_load = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/dana_noise.obj";
    string mesh_to_save = "/Users/francescousai/Dropbox/AnimationQuadLayout/Experiments/dana_noise_chs.obj";
    string segmentation = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/dana.segm";
#endif


    igl::readOBJ( mesh_to_load , VT, FT );
//    OBB::buildOBB( VT, FT, VO, FO );

//    igl::readOFF( mesh_to_load , VT, FT );

    Segmentation::Segments seg;
    Segmentation::loadSegments( segmentation, seg );
    Segmentation::joinSegments( seg, VT, FT );

//    ConvexHullCreator::getConvexHull( VT, VH, FH );
//    ConvexHullCreator::getConvexHull( VT, seg, VH, FH );
    ConvexHullCreator::getConvexHullUnion( VT, seg, VH, FH );
    //improveTriangulation( VH, FH );

//    OBB::buildOBB( VH, FH, VO, FO );
    ConvexHullCreator::getOOBs(VT, seg, VO, FO);

    std::cout << FO.rows() << ", " << FO.cols() <<std::endl;
//    return 0;
    std::cout << FO;


#ifdef SAVE_PARTS
    std::vector<Eigen::MatrixXd> points_out;
    std::vector<Eigen::MatrixXi> faces_out;
    ConvexHullCreator::getConvexHulls( VT, seg, points_out, faces_out );

    assert( points_out.size() == faces_out.size( ));
    for( size_t i = 0; i < points_out.size(); ++i ){
        string filename = hull_folder + hull_name_template + to_string(i) + ".obj";
        igl::writeOBJ( filename, points_out[i], faces_out[i] );
    }
#endif

//    igl::writeOBJ(mesh_to_save, VH, FH);


    // Use original normals as pseudo-colors
    MatrixXd N;
    igl::per_vertex_normals( VH, FH, N );
    std::cout << "normals ok" << std::endl;

    MatrixXd C = N.rowwise().normalized().array()*0.5+0.5;
    std::cout << "colors ok" << std::endl;

    viewer.data.set_mesh( VH, FH );
//    viewer.data.set_mesh( VO, FO );
    std::cout << "mesh has been set" << std::endl;

    // Add edges of the Oriented Bounding Box
    igl::slice( VO, FO.col(0).eval(), 1, PQC0); igl::slice( VO, FO.col(1).eval(), 1, PQC1);
    igl::slice( VO, FO.col(2).eval(), 1, PQC2); igl::slice( VO, FO.col(3).eval(), 1, PQC3);

    viewer.data.add_edges( PQC0, PQC1, Eigen::RowVector3d(0,0,0));
    viewer.data.add_edges( PQC1, PQC2, Eigen::RowVector3d(0,0,0));
    viewer.data.add_edges( PQC2, PQC3, Eigen::RowVector3d(0,0,0));
    viewer.data.add_edges( PQC3, PQC0, Eigen::RowVector3d(0,0,0));

    viewer.data.set_colors(C);
    std::cout << "colors has been set" << std::endl;
//    viewer.callback_key_down = key_down;


return viewer.launch();



}
