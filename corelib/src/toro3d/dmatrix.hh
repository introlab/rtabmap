#ifndef DMATRIX_HXX 
#define DMATRIX_HXX

#include <iostream>
#include <exception>

class DNotInvertibleMatrixException: public std::exception {};
class DIncompatibleMatrixException: public std::exception {};
class DNotSquareMatrixException: public std::exception {};


template <class X> struct DVector{
public:
  DVector(int n=0);
  ~DVector();
  
  DVector(const DVector&);
  DVector& operator=(const DVector&);
  
  X&  operator[](int i) {
    if ((*shares)>1) detach();
    return elems[i];
  }
  
  const X&  operator[](int i) const { return elems[i]; }
  
  X operator*(const DVector&) const;
  DVector operator+(const DVector&) const;
  DVector operator-(const DVector&) const;
  DVector operator*(const X&) const;
  
  int dim() const { return size; }
  
  void detach();
  
  static DVector<X> I(int);
  
protected:
  X * elems;
  int size;
  int * shares;
};



template <class X> class DMatrix {
public:
  DMatrix(int n=0,int m=0);
  ~DMatrix();
  
  DMatrix(const DMatrix&);
  DMatrix& operator=(const DMatrix&);
  
  X * operator[](int i) {
    if ((*shares)>1) detach();
    return mrows[i];
  }
  
  const X * operator[](int i) const { return mrows[i]; }
  
  const X det() const;
  DMatrix inv() const;
  DMatrix transpose() const;
  DMatrix operator*(const DMatrix&) const;
  DMatrix operator+(const DMatrix&) const;
  DMatrix operator-(const DMatrix&) const;
  DMatrix operator*(const X&) const;
  
  int rows() const { return nrows; }
  int columns() const { return ncols; }
  
  void detach();
  
  static DMatrix I(int);
  
protected:
  X * elems;
  int nrows,ncols;
  X ** mrows;
  
  int * shares;
};


template <class X> DVector<X> operator * (const DMatrix<X> m, const DVector<X> v);

template <class X> DVector<X> operator * (const DVector<X> v, const DMatrix<X> m);


/*************** IMPLEMENTATION ***************/

#include "dmatrix.hxx"

#endif
