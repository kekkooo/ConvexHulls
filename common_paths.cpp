#include "common_paths.h"

PathsStruct &CommonPaths::getAsura(){
    static PathsStruct x = PathsStruct(
        base_path  "Mesh/AsuraSegmentation/AsuraKekko.obj",
        base_path  "Mesh/AsuraSegmentation/AsuraKekko_chs_union.obj",
        base_path  "Mesh/AsuraSegmentation/AsuraKekko.segm",
        base_path  "Mesh/AsuraSegmentation/",
        "AsuraPart" );
    return x;

}

PathsStruct &CommonPaths::getCapsule(){
    static PathsStruct x = PathsStruct(
        base_path  "Mesh/CapsuleSegmentation/Capsula.obj",
        base_path  "Mesh/CapsuleSegmentation/Capsula_chs_union.obj",
        base_path  "Mesh/CapsuleSegmentation/Capsula.segm",
        base_path  "AnimationQuadLayout/Mesh/CapsuleSegmentation/",
        "CapsulaPart" );
    return x;

}

PathsStruct &CommonPaths::getDana(){
    static PathsStruct x = PathsStruct(
        base_path  "Mesh/DanaSegmentation/dana.obj",
        base_path  "Mesh/DanaSegmentation/dana_chs2.obj",
        base_path  "Mesh/DanaSegmentation/dana.segm",
        base_path  "Mesh/DanaSegmentation/",
        "DanaPart");
    return x;

}

PathsStruct &CommonPaths::getHand(){
    static PathsStruct x = PathsStruct(
        base_path  "Mesh/HandSegmentation/Hand.obj",
        base_path  "Mesh/HandSegmentation/Hand_chs2.obj",
        base_path  "Mesh/HandSegmentation/Hand.segm",
        base_path  "Mesh/HandSegmentation/",
        "HandPart");
    return x;

}

PathsStruct &CommonPaths::getJeff(){
    static PathsStruct x = PathsStruct(
        base_path  "Mesh/JeffSegmentation/Jeff.obj",
        base_path  "Mesh/JeffSegmentation/Jeff_chs2.obj",
        base_path  "Mesh/JeffSegmentation/Jeff.segm",
        base_path  "Mesh/JeffSegmentation/",
        "JeffPart" );
    return x;

}
