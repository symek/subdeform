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

bool create_shape_matrix(const std::string & restfile, const StringVec &skinfiles,
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
    GU_Detail skin;
    StringVec::const_iterator it;
    for (it=shapefiles.begin(); it!= shapefiles.end(); ++it, ++shapenum) {
        auto & file = *it;
        if(!shape.load(file.c_str()).success()) {
            std::cerr << "Can't open shape file, ignoring it: " << file << '\n';
            // shapenum++;
            continue;
        }
        auto & skinfile = skinfiles.at(shapenum); 
        if(!skin.load(skinfile.c_str()).success()) {
            std::cerr << "Can't open skin file, ignoring it: " << skinfile << '\n';
            // shapenum++;
            continue;
        }

        if(npoints != shape.getNumPoints() || npoints != skin.getNumPoints()) {
            std::cerr << "Point count doesn't match, ignoring these files: " << file << ": " << skinfile<< '\n';
            // shapenum++;
            continue;
        }

        GA_FOR_ALL_PTOFF(&rest, ptoff) {
            const GA_Index   rest_index = rest.pointIndex(ptoff);
            // const UT_Vector3 rest_pos   = rest.getPos3(ptoff);
            const GA_Offset  skin_off   = skin.pointOffset(rest_index);
            const UT_Vector3 skin_pos   = skin.getPos3(skin_off);
            const GA_Offset  shape_off  = shape.pointOffset(rest_index);
            const UT_Vector3 shape_pos  = shape.getPos3(shape_off);
            const UT_Vector3 shape_delta(shape_pos - skin_pos);
            matrix(3*rest_index+0, shapenum) = shape_delta.x();
            matrix(3*rest_index+1, shapenum) = shape_delta.y();
            matrix(3*rest_index+2, shapenum) = shape_delta.z(); 
        }
        // shapenum++;
    }

    return true;
}


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
            ("rest,r",   po::value<std::string>()->required(),  "Rest input file.")
            ("output,o", po::value<std::string>()->required(),  "Output file (.matrix).")
            ("shapes,s", po::value<StringVec>()->multitoken()->required(), "Input shape files")
            ("skin,k",   po::value<StringVec>()->multitoken(),  "Input skin files")
            ("ortho,t",  po::value<bool>(),                     "Orthogonalize PCA")
            ("var,v",    po::value<double>(),                   "PCA Variance");

        po::variables_map result;        
        po::store(po::parse_command_line(argc, argv, options), result);
        po::notify(result);

        if(!result.count("shapes")) {
            std::cerr << "No shape files found." << '\n';
            return 1;
        } 
        
        auto & shapefiles  = result["shapes"].as<StringVec>();
        auto & restfile    = result["rest"].as<std::string>();
        auto & matrix_file = result["output"].as<std::string>();
        StringVec            skinfiles;

        if(result.count("skin")) {
            skinfiles = result["skin"].as<StringVec>();
            std::cout << "Using " << shapefiles.size() <<  " skins: " \
            << skinfiles[0] << "...\n";
        }

        if (skinfiles.size() != 0 && (shapefiles.size() != skinfiles.size())) {
            std::cerr << "Shapes and skin files don't match." << '\n';
            return 1;
        }

        std::cout << "Using rest file: " <<  restfile << '\n';
        std::cout << "Using " << shapefiles.size() <<  " shapes: " \
            << shapefiles[0] << "...\n";

        /// Create matrix from skin and deforemed sequence
        Matrix shapes_matrix;
        if (skinfiles.size() != 0) {
            if (!create_shape_matrix(restfile, skinfiles, shapefiles, shapes_matrix)) {
                std::cerr << "Can't create shape matrix." << '\n';
                return 1;  
            }
        } else {
            if (!create_shape_matrix(restfile, shapefiles, shapes_matrix)) {
                std::cerr << "Can't create shape matrix." << '\n';
                return 1;   
            }
        }

        /// Save
        if (result.count("var")) {
            Matrix pca_matrix;
            const double variance      = result["var"].as<double>();
            const bool   orthogonalize = result["ortho"].as<bool>();
            if(!computePCA(shapes_matrix, pca_matrix, variance, orthogonalize)) {
                std::cerr << "Can't compute PCA matrix." << '\n';
                return 1;
            }

            if(!write_matrix(pca_matrix, matrix_file.c_str())) {
                std::cerr << "Can't write matrix to file: " << matrix_file << '\n';
                return 1;
            }

        } else {
            if(!write_matrix(shapes_matrix, matrix_file.c_str())) {
                std::cerr << "Can't write matrix to file: " << matrix_file << '\n';
                return 1;
            }
        }


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
    