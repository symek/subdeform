#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <GU/GU_Detail.h>
#include <hboost/program_options.hpp>
#include "math.hpp"

namespace po = hboost::program_options;
using namespace subdeform;


inline void compute_rotation(UT_Vector3 & rest_tanu ) {

}

bool compute_psd(const GU_Detail & rest, const GU_Detail & shape, \
    const GU_Detail & skin, const int shape_index, Matrix & matrix)
{
    GA_ROHandleV3 rest_tu_h(rest.findFloatTuple(GA_ATTRIB_POINT, "tangentu", 3));
    GA_ROHandleV3 rest_tv_h(rest.findFloatTuple(GA_ATTRIB_POINT, "tangentv", 3));
    GA_ROHandleV3 skin_tu_h(skin.findFloatTuple(GA_ATTRIB_POINT, "tangentu", 3));
    GA_ROHandleV3 skin_tv_h(skin.findFloatTuple(GA_ATTRIB_POINT, "tangentv", 3)); 

    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(&rest, ptoff) {
        const GA_Index   rest_index = rest.pointIndex(ptoff);
        const UT_Vector3 rest_pos   = rest.getPos3(ptoff);
        UT_Vector3 rest_tu          = rest_tu_h.get(ptoff);
        UT_Vector3 rest_tv          = rest_tv_h.get(ptoff);

        const GA_Offset  skin_off   = skin.pointOffset(rest_index);
        const UT_Vector3 skin_pos   = skin.getPos3(skin_off);
        UT_Vector3 skin_tu          = skin_tu_h.get(skin_off);
        UT_Vector3 skin_tv          = skin_tv_h.get(skin_off);

        UT_Matrix3D m1, m2, m3;
        const int r1 = m1.dihedral(skin_tu, rest_tu);
        const int r2 = m2.dihedral(skin_tv, rest_tv);
        m3 = m1 * m2;

        const GA_Offset  shape_off  = shape.pointOffset(rest_index);
        const UT_Vector3 shape_pos  = shape.getPos3(shape_off);

        UT_Vector3 shape_delta(shape_pos - skin_pos);
        shape_delta *= m3;

        matrix(3*rest_index+0, shape_index) = shape_delta.x();
        matrix(3*rest_index+1, shape_index) = shape_delta.y();
        matrix(3*rest_index+2, shape_index) = shape_delta.z(); 
    }
    return true;
}

bool compute_delta(const GU_Detail & rest, const GU_Detail & shape, \
    const GU_Detail & skin, const int shape_index, Matrix & matrix)
{
    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(&rest, ptoff) {
        const GA_Index   rest_index = rest.pointIndex(ptoff);
        const GA_Offset  skin_off   = skin.pointOffset(rest_index);
        const UT_Vector3 skin_pos   = skin.getPos3(skin_off);
        const GA_Offset  shape_off  = shape.pointOffset(rest_index);
        const UT_Vector3 shape_pos  = shape.getPos3(shape_off);
        const UT_Vector3 shape_delta(shape_pos - skin_pos);
        matrix(3*rest_index+0, shape_index) = shape_delta.x();
        matrix(3*rest_index+1, shape_index) = shape_delta.y();
        matrix(3*rest_index+2, shape_index) = shape_delta.z(); 
    }
    return true;
}

bool create_shape_matrix(const std::string & restfile, const StringVec &skinfiles,
    const StringVec &shapefiles, const bool psd, Matrix &matrix)
{
    GU_Detail rest;
    if(!rest.load(restfile.c_str()).success()) {
        std::cerr << "Can't open rest file: " << restfile << '\n';
        return false;
    } else {
        std::cout << "Loading rest file: " << restfile << '\n';
    }

    const int npoints = rest.getNumPoints();
    matrix.conservativeResize(npoints*3, shapefiles.size());
    int shapenum = 0;
    GU_Detail shape_geo;
    GU_Detail skin_geo;

    StringVec::const_iterator it;
    for (it=shapefiles.begin(); it!= shapefiles.end(); ++it, ++shapenum) {
        auto & shape_file = *it;
        if(!shape_geo.load(shape_file.c_str()).success()) {
            std::cerr << "Can't open shape file, ignoring it: " << shape_file << '\n';
            continue;
        } else {
            std::cout << "Loading shape file: " << shape_file << '\n';
        }
        auto & skinfile = skinfiles.at(shapenum); 
        if(!skin_geo.load(skinfile.c_str()).success()) {
            std::cerr << "Can't open skin file, ignoring it: " << skinfile << '\n';
            continue;
        } else {
             std::cout << "Loading skin file: " << skinfile << '\n';
        }

        if(npoints != shape_geo.getNumPoints() || npoints != skin_geo.getNumPoints()) {
            std::cerr << "Points doesn't match, ignoring these files: " << shape_file << ", " << skinfile << '\n';
            continue;
        }

        // Tangents are on place
        if (rest.findFloatTuple(GA_ATTRIB_POINT, "tangentu", 3)     && 
            rest.findFloatTuple(GA_ATTRIB_POINT, "tangentv", 3)     && 
            skin_geo.findFloatTuple(GA_ATTRIB_POINT, "tangentu", 3) && 
            skin_geo.findFloatTuple(GA_ATTRIB_POINT, "tangentv", 3) && psd) {

            if(compute_psd(rest, shape_geo, skin_geo, shapenum, matrix)) {
               std::cout << "Computed pose space deformation #: " << shapenum + 1 << '\n'; 
            } else {
                std::cerr << "Can't compute pose space deformation for: " << shape_file << '\n';
            }
        // Proceed in case of lack of tangents: TODO: make them by yourself
        } else {
            if(compute_delta(rest, shape_geo, skin_geo, shapenum, matrix)) {
            std::cerr << "No tangents found, proceeding without them... " << '\n';
               std::cout << "Computed delta #: " << shapenum + 1 << '\n'; 
            } else {
                std::cerr << "Can't compute delta for: " << shape_file << '\n';
            }
        }


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
        po::options_description options("subdeform options:");
        options.add_options()
            ("help,h",                                                     "Help.") 
            ("rest,r",   po::value<std::string>()->required(),             "Rest input file.")
            ("output,o", po::value<std::string>()->required(),             "Output file (.matrix).")
            ("shape,s",  po::value<StringVec>()->multitoken()->required(), "Input shape files")
            ("skin,k",   po::value<StringVec>()->multitoken(),             "Input skin files")
            ("var,v",    po::value<double>(),                              "PCA Variance. If omitted, PCA won't be performed.")
            ("norm,n",   po::bool_switch()->default_value(true),             "Orthonormalize PCA (on by default).")
            ("psd,p",    po::bool_switch()->default_value(false),           \
                "Compute pose space deformation in case tangents vectors are presnet. (default false)");

        po::variables_map result;        
        po::store(po::parse_command_line(argc, argv, options), result);

        if (result.count("help") || argc == 1) {
            std::cout << options << '\n';
        }

        po::notify(result);

        if(!result.count("shape")) {
            std::cerr << "No shape files found." << '\n';
            return 1;
        } 
        
        auto & shapefiles  = result["shape"].as<StringVec>();
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
        const bool psd = result["psd"].as<bool>();
        Matrix shapes_matrix;
        if (skinfiles.size() != 0) {
            if (!create_shape_matrix(restfile, skinfiles, shapefiles, psd,  shapes_matrix)) {
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
            const double variance       = result["var"].as<double>();
            const bool   orthonormalize = result["norm"].as<bool>(); 
            std::cout << "Computing PCA... " << std::flush; 
            if(!computePCA(shapes_matrix, pca_matrix, variance, orthonormalize)) {
                std::cerr << "Can't compute PCA matrix." << '\n';
                return 1;
            } else {
                std::cout << "done"  << '\n';
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
            std::cerr << "Can't read matrix" << matrix_file << '\n';
            return 1;
        } else {
            std::cout << "Matrix seems to be fine... " << '\n';
            std::cout << "Points: " << second_matrix.rows() / 3 << '\n';
            std::cout << "Shapes: " << second_matrix.cols() << '\n';
            std::cout << "Size  : " << second_matrix.rows() * second_matrix.cols() * sizeof(double) / 1024 << "KB\n";  
        }

    } catch (const std::exception &ex) {
        std::cerr << ex.what() << '\n';
    }
    
    return 0;
}
    