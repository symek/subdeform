#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <GU/GU_Detail.h>
#include <CMD/CMD_Args.h>
#include <hboost/program_options.hpp>
#include "utils.hpp"

namespace po = hboost::program_options;

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
            const UT_Vector3 shape_delta(shape_pos - rest_pos);
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
    try 
    {

        po::options_description options("Subdeform options.");
        options.add_options()
            ("help,h",  "Help")
            ("rest,r",   po::value<std::string>()->required(), "Rest input file.")
            ("shapes,s", po::value<StringVec>()->multitoken()->required(), "Input shape files")
            ("skin,k",   po::value<StringVec>()->multitoken(), "Input skin  files");

        po::variables_map result;        
        po::store(po::parse_command_line(argc, argv, options), result);
        po::notify(result);

        if(!result.count("shapes")) {
            std::cerr << "No shape files found." << '\n';
            return 1;
        } 
        
        auto & shapefiles = result["shapes"].as<StringVec>();
        auto & restfile   = result["rest"].as<std::string>();
        StringVec skinfiles;
        if(result.count("skin")) {
            skinfiles = result["skin"].as<StringVec>();
            std::cout << "Using " << shapefiles.size() <<  " skins: " \
            << skinfiles[0] << "...\n";
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
        if(!write_matrix(shapes_matrix, matrix_file.c_str()))
            return 1;

        // check;
        Matrix second_matrix;
        if(!read_matrix(matrix_file.c_str(), second_matrix)) {
            std::cerr << "Can't read matrix" << '\n';
            return 1;
        } else {
            std::cout << "Matrix seems to be fine. " << '\n';   
        }

    } catch (const std::exception &ex) {
        std::cerr << ex.what() << '\n';
    }
    
    return 0;
}
    