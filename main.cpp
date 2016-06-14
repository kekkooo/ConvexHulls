#include <iostream>
#include <fstream>

#include <convexhullcreator.h>
#include <segmentation.h>
#include <obb.h>

#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/writeOBJ.h>


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

#define DANA
//#define SAVE_PARTS


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
    string mesh_to_load = "/export/home/botero/mkovacic/Devel/ConvexHulls/Mesh/Dana.obj";
    string mesh_to_save = "/export/home/botero/mkovacic/Devel/ConvexHulls/Mesh/Dana_chs2.obj";
    string segmentation = "/export/home/botero/mkovacic/Devel/ConvexHulls/Mesh/Dana.segm";

//      string mesh_to_load = "/export/home/botero/mkovacic/Devel/ConvexHulls/Mesh/282.off";
//      string mesh_to_save = "/export/home/botero/mkovacic/Devel/ConvexHulls/Mesh/282_chs2.obj";
//      string segmentation = "/export/home/botero/mkovacic/Devel/ConvexHulls/Mesh/282.seg";

#endif

#ifdef NOISY_DANA
    string mesh_to_load = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/dana_noise.obj";
    string mesh_to_save = "/Users/francescousai/Dropbox/AnimationQuadLayout/Experiments/dana_noise_chs.obj";
    string segmentation = "/Users/francescousai/Dropbox/AnimationQuadLayout/Mesh/dana.segm";
#endif


    igl::readOBJ( mesh_to_load , VT, FT );
//    OBB::buildOBB( VT, FT, VO, FO );

    //igl::readOFF( mesh_to_load , VT, FT );





    Segmentation::Segments seg;
    //Segmentation::loadSegments( segmentation, seg, FT ); // Princeton dataset
    Segmentation::loadSegments( segmentation, seg, Eigen::MatrixXi() );
    Segmentation::joinSegments( seg, VT, FT );

//    ConvexHullCreator::getConvexHull( VT, VH, FH );
//    ConvexHullCreator::getConvexHull( VT, seg, VH, FH );
    ConvexHullCreator::getConvexHullUnion( VT, seg, VH, FH );
    //improveTriangulation( VH, FH );

//    OBB::buildOBB( VH, FH, VO, FO );

    igl::writeOBJ( mesh_to_save, VH, FH );

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

return 0;

}
