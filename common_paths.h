#ifndef COMMON_PATHS_H
#define COMMON_PATHS_H

#include <string>;

#define base_path "/Users/francescousai/Dropbox/AnimationQuadLayout/"


struct PathsStruct{
    std::string mesh_to_load;
    std::string mesh_to_save;
    std::string segmentation;
    std::string hull_folder;
    std::string hull_name_template;

    PathsStruct( std::string toLoad, std::string toSave, std::string segmentationFile,
                 std::string hullFolder, std::string hullNameTemplate ){
        this->mesh_to_load = toLoad;
        this->mesh_to_save = toSave;
        this->segmentation = segmentationFile;
        this->hull_folder = hullFolder;
        this->hull_name_template = hull_name_template;
    }
};

class CommonPaths{
public:

    static PathsStruct& getAsura();
    static PathsStruct& getCapsule();
    static PathsStruct& getDana();
    static PathsStruct& getHand();
    static PathsStruct& getJeff();

};

#endif // COMMON_PATHS_H
