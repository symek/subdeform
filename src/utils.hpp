#pragma once

using StringVec = std::vector<std::string>;
using Matrix    = Eigen::MatrixXd;
using Vector    = Eigen::VectorXd;

// Should we just use EIGEN::QRMatrix?
void orthogonalize_matrix(Matrix & matrix, int c=0) {
    if(c >= matrix.cols())
      return;
    for (int x = c; x < matrix.cols(); ++x) {
        // project out other components
        for (int y = 0; y < x; ++y) {
            double dot = matrix.col(y).dot(matrix.col(x));
            matrix.col(x) -= dot * matrix.col(y);
        }
        double norm2 = matrix.col(x).norm();
        if(norm2 >= 1e-6) {
            matrix.col(x) *= 1.0 / norm2;
        }
    }
}


bool computePCA(Matrix & matrix, Matrix & pcamatrix, 
    double variance, bool shift=false, bool orthogonalize=false) {

    Vector eigenvalues;
    const int rows = matrix.rows();
    const int cols = matrix.cols();

    if(shift) {
        for(int x = 0; x < rows; x++) {
            double mean = matrix.row(x).sum() / (double)cols;
            matrix.row(x) = (matrix.row(x).array() - mean).matrix();
        }
        matrix *= 1.0 / sqrt(cols - 1);
    }

    Eigen::JacobiSVD<Matrix> eigenSystem(matrix, Eigen::ComputeThinU);
    const Vector & singularValues = eigenSystem.singularValues();
    pcamatrix = eigenSystem.matrixU();
    eigenvalues.resize(pcamatrix.cols());
    for(int x = 0; x < pcamatrix.cols(); x++) {
        eigenvalues[x] = singularValues[x] * singularValues[x];
    }

    double sum_   = eigenvalues.sum();
    double cutoff = variance * sum_;
    double keep_ = 0;
    int pcarank  = 0;

    for(int i = 0; i < eigenvalues.size(); ++i) {
        keep_ += eigenvalues[i];
        pcarank++;
        if(keep_ >= cutoff)
            break;
    }

    if(pcarank < pcamatrix.cols()) {
        pcamatrix.conservativeResize(pcamatrix.rows(), pcarank);
    }

    if (orthogonalize)
        orthogonalize_matrix(pcamatrix);

    return true;
}


bool write_matrix(const Matrix & matrix, const char * filename) {
    const double * data = matrix.data();
    // Eigen::Map<Matrix>(data, matrix.rows(), matrix.cols()) = matrix;
    FILE *file = fopen(filename, "wb");
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

bool read_matrix(const char * filename, Matrix & matrix) {
    FILE *file = fopen(filename, "rb");
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