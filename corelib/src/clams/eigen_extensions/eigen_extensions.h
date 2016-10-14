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
  void serialize(const Eigen::Matrix<S, T, U>& mat, std::ostream& strm);
  
  template<class S, int T, int U>
  void deserialize(std::istream& strm, Eigen::Matrix<S, T, U>* mat);

  template<class S, int T, int U>
  void serializeASCII(const Eigen::Matrix<S, T, U>& mat, std::ostream& strm);
  
  template<class S, int T, int U>
  void deserializeASCII(std::istream& strm, Eigen::Matrix<S, T, U>* mat);  

  
  // -- Scalar serialization
  //    TODO: Can you name these {de,}serialize() and still have the right
  //    functions get called when serializing matrices?
  template<class T>
  void serializeScalar(T val, std::ostream& strm);
  
  template<class T>
  void deserializeScalar(std::istream& strm, T* val);

  template<class T>
  void serializeScalarASCII(T val, std::ostream& strm);

  template<class T>
  void deserializeScalarASCII(std::istream& strm, T* val);


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

  template<class T>
  void serializeScalarASCII(T val, std::ostream& strm)
  {
	  int old_precision = strm.precision();
	  strm.precision(16);
	  strm << "% " << val << std::endl;
	  strm.precision(old_precision);
  }

  template<class T>
  void deserializeScalarASCII(std::istream& strm, T* val)
  {
	  std::string line;
	  while(line.length() == 0) getline(strm, line);
	  assert(line[0] == '%');
	  std::istringstream iss(line.substr(1));
	  iss >> *val;
  }
  
}

#endif // EIGEN_EXTENSIONS_H
