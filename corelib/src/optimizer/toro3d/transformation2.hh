/**********************************************************************
 *
 * This source code is part of the Tree-based Network Optimizer (TORO)
 *
 * TORO Copyright (c) 2007 Giorgio Grisetti, Cyrill Stachniss, 
 *                         Slawomir Grzonka, and Wolfram Burgard
 *
 * TORO is licences under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *  
 *   - Noncommercial. You may not use this work for commercial purposes.
 *  
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * TORO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 **********************************************************************/

/** \file transformation2.hh
 * \brief Definition of the 2d transformations.
 *
 * Definition of the 2d transformations, the symmetrix matrix operations,
 * handling covariance, etc.
 **/

#ifndef _TRANSFORMATION2_HXX_
#define _TRANSFORMATION2_HXX_

#include <cmath>

namespace AISNavigation
{

/** \brief Template class for representing a 2D point (x and y coordinate) **/
template <class T>
struct Vector2{
  T values[2] ; ///< container for x and y

  /** Constructor **/
  Vector2(T x, T y)        {values[0]=x; values[1]=y;}
  /** Default constructor which sets x and y to 0 **/
  Vector2()                {values[0]=0; values[1]=0;}

  /** @returns Const reference to x **/
  inline const T& x()      const  {return values[0];}
  /** @returns Const reference to y **/
  inline const T& y()      const  {return values[1];}

  /** @returns Reference to x **/
  inline T& x()            {return values[0];}
  /** @returns Reference to y **/
  inline T& y()            {return values[1];}

  /** @returns Norm of the vector **/
  inline T norm2() const   {
    return values[0]*values[0]+values[1]*values[1];
  }

};

/** Operator for scalar multiplication.  **/
template <class T>
inline Vector2<T> operator * (const T& d, const Vector2<T>& v) {
  return Vector2<T>(v.values[0]*d, v.values[1]*d);
}

/** Operator for scalar multiplication.  **/
template <class T>
inline Vector2<T> operator * (const Vector2<T>& v, const T& d) {
  return Vector2<T>(v.values[0]*d, v.values[1]*d);
}

/** Operator for dot product.  **/
template <class T>
inline T operator * (const Vector2<T>& v1, const Vector2<T>& v2){
  return v1.values[0]*v2.values[0] 
    + v1.values[1]*v2.values[1];
}

/** Operator for vector addition.  **/
template <class T>
inline Vector2<T> operator + (const Vector2<T>& v1, const Vector2<T>& v2){
  return Vector2<T>(v1.values[0]+v2.values[0], 
		    v1.values[1]+v2.values[1]);
}

/** Operator for vector subtraction.  **/
template <class T>
Vector2<T> operator - (const Vector2<T>& v1, const Vector2<T>& v2){
  return Vector2<T>(v1.values[0]-v2.values[0], 
		    v1.values[1]-v2.values[1]);
}


/** \brief 2D Point (x,y) with orientation (theta)
 *
 * Tenmplate class for representing a 2D Ooint with x and y
 * coordinates and an orientation theta in the x-y-plane (theta=0 ->
 * orientation along the x axis).
**/
template <class T>
struct Pose2{
  T values[3];///< container for x, y, and theta

  /** @returns Const refernce to x **/
  inline const T& x()     const  {return values[0];}
  /** @returns Const refernce to y **/
  inline const T& y()     const  {return values[1];}
  /** @returns Const refernce to theta **/
  inline const T& theta() const  {return values[2];}

  /** @returns Refernce to x **/
  inline T& x()            {return values[0];}
  /** @returns Refernce to y **/
  inline T& y()            {return values[1];}
  /** @returns Refernce to theta **/
  inline T& theta()        {return values[2];}

  /** Default constructor which sets x, y, and theta to 0 **/
  Pose2(){
    values[0]=0.; values[1]=0.;  values[2]=0.;
  }

  /** Constructor **/
  Pose2(const T& x, const T& y, const T& theta){
    values[0]=x, values[1]=y, values[2]=theta;
  }
};

/** Operator for scalar multiplication with a pose **/
template <class T>
Pose2<T> operator * (const Pose2<T>& v, const T& d){
  Pose2<T> r;
  for (int i=0; i<3; i++){
    r.values[i]=v.values[i]*d;
  }
  return r;
}


/** \brief A class to represent 2D transformations (rotation and translation) **/
template <class T>
struct Transformation2{
  T rotationMatrix[2][2]; ///< the rotation matrix
  T translationVector[2]; ///< the translation vector

  /** Default constructor 
   * @param initAsIdentity if true (default) the transormation 
   *        is the identity, otherwise no initializtion **/
  Transformation2(bool initAsIdentity = true){
    if (initAsIdentity) {
      rotationMatrix[0][0]=1.; rotationMatrix[0][1]=0.;
      rotationMatrix[1][0]=0.; rotationMatrix[1][1]=1.;
      translationVector[0]=0.;
      translationVector[1]=0.;
    }
  }

  /** @returns Identity transformation **/
  inline static Transformation2<T> identity(){
    Transformation2<T> m(true);
    return m;
  }

  /** Constructor that sets the translation and rotation  **/
  Transformation2 (const T& x, const T& y, const T& theta){
    setRotation(theta);
    setTranslation(x,y);
  }

  /** Constructor that sets the translation and rotation  **/
  Transformation2 (const T& _theta, const Vector2<T>& trans){
	  setRotation(_theta);
	  setTranslation(trans.x(), trans.y());
  }


  /** Copy constructor  **/
  Transformation2 (const Pose2<T>& v){
    setRotation(v.theta());
    setTranslation(v.x(),v.y());
  }


  /** Get the translation  **/
  inline Vector2<T> translation() const {
    return Vector2<T>(translationVector[0], 
		      translationVector[1]);
  }
  
  /** Get the rotation  **/
  inline T rotation()    const {
    return atan2(rotationMatrix[1][0],rotationMatrix[0][0]);
  }
  
  /** Computed the Pose based on the translation and rotation  **/
  inline Pose2<T> toPoseType()   const {
    Vector2<T> t=translation();
    T r=rotation();
    Pose2<T> rv(t.x(), t.y(), r );
    return rv;
  }
  
  /** Set the translation  **/
  inline void setTranslation(const Vector2<T>& t){
    setTranslation(t.x(),t.y());
  }

  /** Set the rotation  **/
  inline void setRotation(const T& theta){
    T s=sin(theta), c=cos(theta);
    rotationMatrix[0][0]=c, rotationMatrix[0][1]=-s;
    rotationMatrix[1][0]=s, rotationMatrix[1][1]= c;
  }

  /** Set the translation  **/
  inline void setTranslation(const T& x, const T& y){
    translationVector[0]=x;
    translationVector[1]=y;
  }

  /** Computes the inveres of the transformation  **/
  inline Transformation2<T> inv() const {
    Transformation2<T> rv(*this);
    for (int i=0; i<2; i++)
      for (int j=0; j<2; j++){
	rv.rotationMatrix[i][j]=rotationMatrix[j][i];
      }

    for (int i=0; i<2; i++){
      rv.translationVector[i]=0;
      for (int j=0; j<2; j++){
	rv.translationVector[i]-=rv.rotationMatrix[i][j]*translationVector[j];
      }
    }
    return rv;
  }

};

/** Operator for transforming a Vector2 **/
template <class T>
Vector2<T> operator * (const Transformation2<T>& m, const Vector2<T>& v){
  return Vector2<T>(
		    m.rotationMatrix[0][0]*v.values[0]+
		    m.rotationMatrix[0][1]*v.values[1]+
		    m.translationVector[0],
		    m.rotationMatrix[1][0]*v.values[0]+
		    m.rotationMatrix[1][1]*v.values[1]+
		    m.translationVector[1]);
}

/** Operator for concatenating two transformations **/
template <class T>
Transformation2<T> operator * (const Transformation2<T>& m1, const Transformation2<T>& m2){
  Transformation2<T> rt;
  for (int i=0; i<2; i++)
    for (int j=0; j<2; j++){
      rt.rotationMatrix[i][j]=0.;
      for (int k=0; k<2; k++)
	rt.rotationMatrix[i][j]+=m1.rotationMatrix[i][k]*m2.rotationMatrix[k][j];
    }
  for (int i=0; i<2; i++){
    rt.translationVector[i]=m1.translationVector[i];
    for (int j=0; j<2; j++)
      rt.translationVector[i]+=m1.rotationMatrix[i][j]*m2.translationVector[j];
  }
  return rt;
}


/** \brief A class to represent symmetric 3x3 matrices **/
template <class T>
struct SMatrix3{
  T values[3][3];
  T det() const;
  SMatrix3<T> transpose() const;
  SMatrix3<T> adj() const;
  SMatrix3<T> inv() const;
};


/** Operator for symmetric matrix-pose multiplication **/
template <class T>
Pose2<T> operator * (const SMatrix3<T>& m, const Pose2<T>& p){
  Pose2<T> v;
  for (int i=0; i<3; i++){
    v.values[i]=0.;
    for (int j=0; j<3; j++)
      v.values[i]+=m.values[i][j]*p.values[j];
  }
  return v;
}

/** Operator for symmetric matrix-scalar multiplication **/
template <class T>
SMatrix3<T> operator * (const SMatrix3<T>& s, T& d){
  SMatrix3<T> m;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      m.values[i][j]=d*s.values[i][j];
  return m;
}

/** Operator forsymmetric  matrix-symmetric matrix multiplication **/
template <class T>
SMatrix3<T> operator * (const SMatrix3<T>& s1, const SMatrix3<T>& s2){
  SMatrix3<T> m;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++){
      m.values[i][j]=0.;
      for (int k=0; k<3; k++){
	m.values[i][j]+=s1.values[i][k]*s2.values[k][j];
      }
    }
  return m;
}

/** Operator for symmetric matrix-symmetric matrix addition **/
template <class T>
SMatrix3<T> operator + (const SMatrix3<T>& s1, const SMatrix3<T>& s2){
  SMatrix3<T> m;
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++){
      m.values[i][j]=s1.values[i][j]+s2.values[i][j];
    }
  return m;
}


/** Computes the determinat of the symmetric matrix **/
template <class T>
T SMatrix3<T>::det() const{
   T dp= values[0][0]*values[1][1]*values[2][2]
     +values[0][1]*values[1][2]*values[2][0]
     +values[0][2]*values[1][0]*values[2][1];
   T dm=values[2][0]*values[1][1]*values[0][2]
     +values[2][1]*values[1][2]*values[0][0]
     +values[2][2]*values[1][0]*values[0][1];
   return dp-dm;
}

/** Computes the transposed symmetric matrix **/
template <class T>
SMatrix3<T>  SMatrix3<T>::transpose() const{
   SMatrix3<T> m;
   for (int i=0; i<3; i++)
     for (int j=0; j<3; j++)
       m.values[j][i]=values[i][j];
   return m;
}

/** Computes the complement of the symmetric matrix **/
template <class T>
SMatrix3<T>  SMatrix3<T>::adj() const{
   SMatrix3<T> m;
   m.values[0][0]= values[1][1]*values[2][2]-values[2][1]*values[1][2];
   m.values[0][1]=-values[1][0]*values[2][2]+values[1][2]*values[2][0];
   m.values[0][2]= values[1][0]*values[2][1]-values[2][0]*values[1][1];
   m.values[1][0]=-values[0][1]*values[2][2]+values[2][1]*values[0][2];
   m.values[1][1]= values[0][0]*values[2][2]-values[2][0]*values[0][2];
   m.values[1][2]=-values[0][0]*values[2][1]+values[2][0]*values[0][1];
   m.values[2][0]= values[0][1]*values[1][2]-values[1][1]*values[0][2];
   m.values[2][1]=-values[0][0]*values[1][2]+values[1][0]*values[0][2];
   m.values[2][2]= values[0][0]*values[1][1]-values[1][0]*values[0][1];
   return m;
}

/** Computes the inverse (=transposed) symmetric matrix **/
template <class T>
SMatrix3<T>  SMatrix3<T>::inv() const{
   T id=1./det();
   SMatrix3<T> i=adj().transpose();
   return i*id;
}



/** \brief Tenmplate class to define the operations in 2D  **/
template <class T>
struct Operations2D{
  typedef T                  BaseType;            /**< base type of the operation typedef **/
  typedef Pose2<T>           PoseType;            /**< plain representation of the 2d pose as x,y,theta **/
  typedef Pose2<T>           ParametersType;      /**< plain representation of the 2d pose as x,y,theta **/
  typedef T                  RotationType;        /**< plain representation of the angle **/
  typedef Vector2<T>         TranslationType;     /**< plain representation of the 2D translation (x,y) **/
  typedef Transformation2<T> TransformationType;  /**< homogeneous based representation for a 2d pose, as rotation matrix + vector **/
  typedef SMatrix3<T>        CovarianceType;      /**< 3 by 3 symmetric covariance matrix for the 2D case **/
  typedef SMatrix3<T>        InformationType;     /**< 3 by 3 symmetric information matrix for the 2D case **/
};

} // namespace AISNavigation

#endif
