#pragma once

using StringVec = std::vector<std::string>;
using Matrix    = Eigen::MatrixXd;
using Vector    = Eigen::VectorXd;

bool write_matrix(const Matrix & matrix, const std::string & filename) {
    const double * data = matrix.data();
    // Eigen::Map<Matrix>(data, matrix.rows(), matrix.cols()) = matrix;
    FILE *file = fopen(filename.c_str(), "wb");
    if (!file) {
        std::cerr << "Can't create matrix file: " << filename << '\n';
        return false;
    }
    const int rows = matrix.rows();
    const int cols = matrix.cols();
    fwrite((void*)&rows, sizeof(int), 1, file);
    fwrite((void*)&cols, sizeof(int), 1, file);
    fwrite((void*)data, sizeof(double), rows * cols, file);
    if (ferror(file)) {
        std::cerr << "Can't write to a file: " << filename << '\n';
        return false;
    }
    fclose(file);
    return true;
}

bool read_matrix(const std::string & filename, Matrix & matrix) {
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "Can't open matrix file: " << filename << '\n';
        return false;
    }
    int rows = 0; 
    int cols = 0;  
    fread((void*)&rows, sizeof(int), 1, file);
    fread((void*)&cols, sizeof(int), 1, file);
    matrix.conservativeResize(rows, cols);
    double * data = matrix.data();
    fread((void*)data, sizeof(double), rows * cols, file);
    if (ferror(file)) {
        std::cerr << "Can't read from a file: " << filename << '\n';
        return false;
    }
    fclose(file);
    return true;
}