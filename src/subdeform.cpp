#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <GU/GU_Detail.h>
#include <CMD/CMD_Args.h>
#include "utils.hpp"


bool create_shape_matrix(const std::string &restfile, 
    const StringVec &shapefiles, Matrix &matrix)
{
    GU_Detail rest;
    if(!rest.load(restfile.c_str()).success()) {
        std::cerr << "Can't open rest file " << restfile << '\n';
        return false;
    }

    const int npoints = rest.getNumPoints();
    matrix.conservativeResize(npoints*3, shapefiles.size());
    GA_Offset ptoff;
    int shapenum = 0;
    GU_Detail shape;
    for (auto file: shapefiles) {
        if(!shape.load(file.c_str()).success()) {
            std::cerr << "Can't open shape file, ignoring it: " << file << '\n';
            continue;
        }
        if(npoints != shape.getNumPoints()) {
            std::cerr << "Point count doesn't match, ignoring this file: " << file << '\n';
            continue;
        }
        GA_FOR_ALL_PTOFF(&rest, ptoff) {
            const GA_Index   rest_index = rest.pointIndex(ptoff);
            const UT_Vector3 rest_pos   = rest.getPos3(ptoff);
            const GA_Offset  shape_off  = shape.pointOffset(rest_index);
            const UT_Vector3 shape_pos  = shape.getPos3(shape_off);
            const UT_Vector3 shape_delta(rest_pos - shape_pos);
            matrix(3*rest_index+0, shapenum) = shape_delta.x();
            matrix(3*rest_index+1, shapenum) = shape_delta.y();
            matrix(3*rest_index+2, shapenum) = shape_delta.z(); 
        }
        shapenum++;
    }

    return true;
}

int main(int argc, char *argv[])
{
    if (argc == 1) {
        std::cerr << "Usage: \n\t ./subdeform rest.bgeo deform.*.bgeo [deform.matrix]" << '\n';
        return 1;
    }

    const std::string restfile(argv[1]);
    StringVec shapefiles;
    for(int i=2; i<argc; ++i) {   
        std::string file(argv[i]);
        shapefiles.push_back(file);
    }

    if (shapefiles.size() == 0) {
        std::cerr << "No shape files found." << '\n';
        return 1;
    }

    std::cout << "Using rest file: " <<  restfile << '\n';
    std::cout << "Using " << shapefiles.size() <<  " shapes: " \
        << shapefiles[0] << "...\n";


    Matrix shapes_matrix;
    if (!create_shape_matrix(restfile, shapefiles, shapes_matrix)) {
        std::cerr << "Can't create shape matrix." << '\n';
        return 1;   
    }

    const std::string matrix_file("../tmp/shapes.matrix");
    if(!write_matrix(shapes_matrix, matrix_file))
        return 1;

    // check;
    Matrix second_matrix;
    if(!read_matrix(matrix_file, second_matrix)) {
        std::cerr << "Can't read matrix" << '\n';
        return 1;
    }
    
    return 0;
}
    