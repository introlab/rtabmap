#ifndef EIGEN_EXTENSIONS_H
#define EIGEN_EXTENSIONS_H

#include <Eigen/Eigen>
//#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
//#include <boost/filesystem.hpp>
#include <stdint.h>
#include <fstream>
#include <iostream>
//#include <gzstream/gzstream.h>

namespace eigen_extensions {

  inline void stdToEig(const std::vector<double>& std, Eigen::VectorXd* eig)
  {
    eig->resize(std.size());
    for(size_t i = 0; i < std.size(); ++i)
      eig->coeffRef(i) = std[i];
  }
  
  inline double stdev(const Eigen::VectorXd& vec)
  {
    double mean = vec.sum() / (double)vec.rows();
    double total = 0;
    for(int i = 0; i < vec.rows(); ++i)
      total += (vec.coeffRef(i) - mean) * (vec.coeffRef(i) - mean);
    double var = total / (double)vec.rows();
    return sqrt(var);
  }
  
  template<class S, int T, int U>
  void save(const Eigen::Matrix<S, T, U>& mat, const std::string& filename);

  template<class S, int T, int U>
  void load(const std::string& filename, Eigen::Matrix<S, T, U>* mat);

  template<class ScalarType, int Options, class IndexType>
  void save(const Eigen::SparseMatrix<ScalarType, Options, IndexType>& mat, const std::string& filename);

  template<class ScalarType, int Options, class IndexType>
  void load(const std::string& filename, Eigen::SparseMatrix<ScalarType, Options, IndexType>* mat);
  
  template<class S, int T, int U>
  void saveASCII(const Eigen::Matrix<S, T, U>& mat, const std::string& filename);

  template<class S, int T, int U>
  void loadASCII(const std::string& filename, Eigen::Matrix<S, T, U>* mat);
  
  template<class S, int T, int U>
  void serialize(const Eigen::Matrix<S, T, U>& mat, std::ostream& strm);
  
  template<class S, int T, int U>
  void deserialize(std::istream& strm, Eigen::Matrix<S, T, U>* mat);

  template<class S, int T, int U>
  void serializeASCII(const Eigen::Matrix<S, T, U>& mat, std::ostream& strm);
  
  template<class S, int T, int U>
  void deserializeASCII(std::istream& strm, Eigen::Matrix<S, T, U>* mat);  

  
  // -- SparseMatrix serialization.
  
  template<class ScalarType, int Options, class IndexType>
  void serialize(const Eigen::SparseMatrix<ScalarType, Options, IndexType>& mat, std::ostream& strm);

  template<class ScalarType, int Options, class IndexType>
  void deserialize(std::istream& strm, Eigen::SparseMatrix<ScalarType, Options, IndexType>* mat);


  // -- Scalar serialization
  //    TODO: Can you name these {de,}serialize() and still have the right
  //    functions get called when serializing matrices?
  template<class T>
  void serializeScalar(T val, std::ostream& strm);
  
  template<class T>
  void deserializeScalar(std::istream& strm, T* val);


  /************************************************************
   * Template implementations
   ************************************************************/
  
  template<class S, int T, int U>
  void serialize(const Eigen::Matrix<S, T, U>& mat, std::ostream& strm)
  {
    int bytes = sizeof(S);
    int rows = mat.rows();
    int cols = mat.cols();
    strm.write((char*)&bytes, sizeof(int));
    strm.write((char*)&rows, sizeof(int));
    strm.write((char*)&cols, sizeof(int));
    strm.write((const char*)mat.data(), sizeof(S) * rows * cols);
  }
  
  template<class S, int T, int U>
  void deserialize(std::istream& strm, Eigen::Matrix<S, T, U>* mat)
  {
    int bytes;
    int rows;
    int cols;
    strm.read((char*)&bytes, sizeof(int));
    strm.read((char*)&rows, sizeof(int));
    strm.read((char*)&cols, sizeof(int));
    assert(bytes == sizeof(S));
      
    S *buf = (S*) malloc(sizeof(S) * rows * cols);
    strm.read((char*)buf, sizeof(S) * rows * cols);
    *mat = Eigen::Map< Eigen::Matrix<S, T, U> >(buf, rows, cols);
    free(buf);    
  }
/*
  template<class S, int T, int U>
  void save(const Eigen::Matrix<S, T, U>& mat, const std::string& filename)
  {
    assert(filename.size() > 3);
    if(filename.substr(filename.size() - 3, 3).compare(".gz") == 0) {
      ogzstream file(filename.c_str());
      assert(file);
      serialize(mat, file);
      file.close();
    }
    else { 
      assert(boost::filesystem::extension(filename).compare(".eig") == 0);
      std::ofstream file(filename.c_str());
      assert(file);
      serialize(mat, file);
      file.close();
    }
  }

  template<class S, int T, int U>
  void load(const std::string& filename, Eigen::Matrix<S, T, U>* mat)
  {
    assert(filename.size() > 3);
    if(filename.substr(filename.size() - 3, 3).compare(".gz") == 0) {
      igzstream file(filename.c_str());
      assert(file);
      deserialize(file, mat);
      file.close();
    }
    else {
      assert(boost::filesystem::extension(filename).compare(".eig") == 0);
      std::ifstream file(filename.c_str());
      assert(file);
      deserialize(file, mat);
      file.close();
    }
  }
*/
  template<class ScalarType, int Options, class IndexType>
  void serialize(const Eigen::SparseMatrix<ScalarType, Options, IndexType>& mat, std::ostream& strm)
  {
    int bytes = sizeof(ScalarType);
    int type = Options;
    int outer = mat.outerSize();
    int inner = mat.innerSize();
    int nnz = mat.nonZeros();
    strm.write((char*)&bytes, sizeof(int));
    strm.write((char*)&type, sizeof(int));
    strm.write((char*)&outer, sizeof(int));
    strm.write((char*)&inner, sizeof(int));
    strm.write((char*)&nnz, sizeof(int));
    
    typedef typename Eigen::SparseMatrix<ScalarType, Options, IndexType>::InnerIterator InnerIterator;
    for(IndexType i = 0; i < mat.outerSize(); ++i) {
      int num = 0;
      for(InnerIterator it(mat, i); it; ++it)
        ++num;
      strm.write((const char*)&num, sizeof(num));
      
      for(InnerIterator it(mat, i); it; ++it) {
        int idx = it.index();
        ScalarType buf = it.value();
        strm.write((const char*)&idx, sizeof(idx));
        strm.write((const char*)&buf, sizeof(buf));
      }
    }
  }

  template<class ScalarType, int Options, class IndexType>
  void deserialize(std::istream& strm, Eigen::SparseMatrix<ScalarType, Options, IndexType>* mat)
  {
    int bytes;
    int options;
    int outer;
    int inner;
    int nnz;
    strm.read((char*)&bytes, sizeof(int));
    strm.read((char*)&options, sizeof(int));
    strm.read((char*)&outer, sizeof(int));
    strm.read((char*)&inner, sizeof(int));
    strm.read((char*)&nnz, sizeof(int));
    assert(bytes == sizeof(ScalarType));
    assert(options == Options);

    if(mat->IsRowMajor) 
      mat->resize(outer, inner);
    else
      mat->resize(inner, outer);

    mat->reserve(nnz);
    ScalarType buf;
    for(int i = 0; i < mat->outerSize(); ++i) {
      mat->startVec(i);
      int num;
      strm.read((char*)&num, sizeof(int));
      int idx;
      for(int j = 0; j < num; ++j) {
        strm.read((char*)&idx, sizeof(idx));
        strm.read((char*)&buf, sizeof(buf));
        mat->insertBackByOuterInner(i, idx) = buf;
      }
    }
    mat->finalize();
  }
/*
  template<class ScalarType, int Options, class IndexType>
  void save(const Eigen::SparseMatrix<ScalarType, Options, IndexType>& mat, const std::string& filename)
  {
    assert(boost::filesystem::extension(filename).compare(".eig") == 0);
    std::ofstream file(filename.c_str());
    assert(file);
    serialize(mat, file);
    file.close();
  }

  template<class ScalarType, int Options, class IndexType>
  void load(const std::string& filename, Eigen::SparseMatrix<ScalarType, Options, IndexType>* mat)
  {
    assert(filename.size() > 3);
    assert(boost::filesystem::extension(filename).compare(".eig") == 0);
    std::ifstream file(filename.c_str());
    assert(file);
    deserialize(file, mat);
    file.close();
  }
  */
  template<class S, int T, int U>
  void serializeASCII(const Eigen::Matrix<S, T, U>& mat, std::ostream& strm)
  {
    int old_precision = strm.precision();
    strm.precision(16);
    strm << "% " << mat.rows() << " " << mat.cols() << std::endl;
    strm << mat << std::endl;
    strm.precision(old_precision);
  }
      
  template<class S, int T, int U>
  void deserializeASCII(std::istream& strm, Eigen::Matrix<S, T, U>* mat)
  {
    // -- Read the header.
    std::string line;
    while(line.length() == 0) getline(strm, line);
    assert(line[0] == '%');
    std::istringstream iss(line.substr(1));
    int rows;
    int cols;
    iss >> rows;
    iss >> cols;
    // -- Read in the data.
    *mat = Eigen::Matrix<S, T, U>(rows, cols);
    for(int y = 0; y < rows; ++y) {
      getline(strm, line);
      std::istringstream iss(line);
      for(int x = 0; x < cols; ++x) {
        iss >> mat->coeffRef(y, x);
      }
    }
  }

  template<class S, int T, int U>
  void saveASCII(const Eigen::Matrix<S, T, U>& mat, const std::string& filename)
  {
    assert(filename.substr(filename.size() - 8).compare(".eig.txt") == 0);
    std::ofstream file;
    file.open(filename.c_str());
    assert(file);
    serializeASCII(mat, file);
    file.close();
  }
  
  template<class S, int T, int U>
  void loadASCII(const std::string& filename, Eigen::Matrix<S, T, U>* mat)
  {
    assert(filename.substr(filename.size() - 8).compare(".eig.txt") == 0);
    std::ifstream file;
    file.open(filename.c_str());
    if(!file)
      std::cerr << "File " << filename << " could not be opened.  Dying badly." << std::endl;
    assert(file);
    deserializeASCII(file, mat);
    file.close();
  }

  template<class T>
  void serializeScalar(T val, std::ostream& strm)
  {
    strm.write((char*)&val, sizeof(T));
  }
  
  template<class T>
  void deserializeScalar(std::istream& strm, T* val)
  {
    strm.read((char*)val, sizeof(T));
  }
  
}

#endif // EIGEN_EXTENSIONS_H
