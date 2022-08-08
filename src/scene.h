#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include "glm/glm.hpp"
#include "utilities.h"
#include "sceneStructs.h"

using namespace std;

class Scene {
private:
    ifstream fp_in;
    int loadMaterial(string materialid);
    int loadGeom(string objectid);
    int loadCamera();
public:
    Scene(string filename);
    ~Scene();

    int loadObj(string filename);
    std::vector<Geom> geoms;
    std::vector<Material> materials;
    std::vector<std::vector<Triangle>> tris;
    std::vector<Geom> objects;
    RenderState state;
};
