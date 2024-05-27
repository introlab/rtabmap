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

#ifndef _TRANSFORMATION3_HXX_
#define _TRANSFORMATION3_HXX_

#include <assert.h>
#include <cmath>

#include "dmatrix.h"

namespace AISNavigation {

template <class T>
struct Vector3 {
  T elems[3] ;

  Vector3(T x, T y, T z) {elems[0]=x; elems[1]=y; elems[2]=z;}
  Vector3()              {elems[0]=0.; elems[1]=0.; elems[2]=0.;}
  Vector3(const DVector<T>& t){}

  // translational view
  inline const T& x()     const  {return elems[0];}
  inline const T& y()     const  {return elems[1];}
  inline const T& z()     const  {return elems[2];}
  inline T& x()            {return elems[0];}
  inline T& y()            {return elems[1];}
  inline T& z()            {return elems[2];}

  // rotational view
  inline const T& roll()   const  {return elems[0];}
  inline const T& pitch() const  {return elems[1];}
  inline const T& yaw()   const  {return elems[2];}
  inline T& roll()          {return elems[0];}
  inline T& pitch()        {return elems[1];}
  inline T& yaw()          {return elems[2];}

};

template <class T>
struct Pose3 : public DVector<T>{
  Pose3();
  Pose3(const Vector3<T>& rot, const Vector3<T>& trans);
  Pose3(const T& x, const T& y, const T& z, const T& roll, const T& pitch, const T& yaw);
  Pose3(const DVector<T>& v): DVector<T>(v) {assert(v.dim()==6);} 
    
  inline operator const DVector<T>&  () {return (const DVector<T>)*this;}
  inline operator DVector<T>&        () {return *this;}

  inline const T& roll()  const  {return DVector<T>::elems[0];}
  inline const T& pitch() const  {return DVector<T>::elems[1];}
  inline const T& yaw()   const  {return DVector<T>::elems[2];}
  inline const T& x()     const  {return DVector<T>::elems[3];}
  inline const T& y()     const  {return DVector<T>::elems[4];}
  inline const T& z()     const  {return DVector<T>::elems[5];}

  inline T& roll()               {return DVector<T>::elems[0];}
  inline T& pitch()              {return DVector<T>::elems[1];}
  inline T& yaw()                {return DVector<T>::elems[2];}
  inline T& x()                  {return DVector<T>::elems[3];}
  inline T& y()                  {return DVector<T>::elems[4];}
  inline T& z()                  {return DVector<T>::elems[5];}

};


/*!
 * A Quaternion can be used to either represent a rotational axis
 * and a Rotation, or, the point which will be rotated
 */

template <class T>
struct Quaternion{

  /*!
   * Default Constructor: w=x=y=z=0;
   */
  Quaternion();
  
  /*!
   * The Quaternion representation of the point "pose"
   */
  Quaternion(const Vector3<T>& pose);
  
  /*!
   * create a Quaternion by scalar w and the imaginery parts x,y, and z.
   */
  Quaternion(const T _w, const T _x, const T _y, const T _z);
  
  /*!
   * create a rotational Quaternion, roll along x-axis, pitch along y-axis and yaw along z-axis
   */
  Quaternion(const T _roll_x_phi, const T _pitch_y_theta, const T _yaw_z_psi);
  
  /*!
   * @return the conjugated version of this quaternion
   */
  inline Quaternion<T> conjugated() const;
  
  /*!
   * @return this quaternion, but normalized
   */
  inline Quaternion<T> normalized() const;
  
  /*!
   * @return the inverse of this Quaternion
   */
  inline Quaternion<T> inverse() const;

  /*construct a quaternion on the axis/angle representation*/
  inline Quaternion(const Vector3<T>& axis, const T& angle);

  /*!
   * if this Quaternion represents a point, use this function
   * to rotate the point along <axis> with angle <alpha>
   * @param axis the rotational axis
   * @param alpha rotational angle
   */
  inline Quaternion<T> rotateThisAlong (const Vector3<T>& axis, const T alpha) const;
  
	
  /*!
   * if this Quaternion represents a rotational axis + rotation,
   * use this function to rotate another point represented as a Quaternion p
   * @param p the point to be rotated by <this>. Point is represented as a Quaternion
   * @return rotated Point (represented as a Quaternion)
   */
  inline Quaternion<T> rotatePoint(const Quaternion& p) const;
  
  /*!
   * if this Quaternion represents a rotational axis + rotation,
   * use this function to rotate another point
   * @param p the point to be rotated by <this>. 
   * @return rotated Point 
   */
  inline Vector3<T> rotatePoint(const Vector3<T>& p) const;
  
  /*!
   * if this Quaternion represents a rotational axis, add a rotation of angle <alpha>
   * along <this> axis to the Quaternion
   * @param alpha rotational value
   * @return this Quaternion with included information about the rotation along <this> axis
   */
  inline Quaternion withRotation (const T alpha) const;
	
  /*!
   * Given rotational axis x,y,z, get the rotation along these axis encoded in this Quaternion
   * @return rotation along x,y,z axis encoded in <this> Quaternion
	*/
  inline Vector3<T> toAngles() const;
  
  inline Vector3<T> axis() const;
  inline T angle() const;

	
  /*!
   * @return the norm of this Quaternion
   */
  inline T norm() const;
  
  /*!
   * @return the real part (==w) of this Quaternion
   */
  inline T re() const;
  
  /*!
   * @return the imaginery part (== (x,y,z)) of this Quaternion
   */
  inline Vector3<T> im() const;
	
  T w,x,y,z;
};



template <class T> inline Quaternion<T> operator + (const Quaternion<T> & left, const Quaternion<T>& right);
template <class T> inline Quaternion<T> operator - (const Quaternion<T> & left, const Quaternion<T>& right);
template <class T> inline Quaternion<T> operator * (const Quaternion<T> & left, const Quaternion<T>& right);
template <class T> inline Quaternion<T> operator * (const Quaternion<T> & left, const T scalar);
template <class T> inline Quaternion<T> operator * (const T scalar, const Quaternion<T>& right);
template <class T> std::ostream& operator << (std::ostream& os, const Quaternion<T>& q);

template <class T> inline T innerproduct(const Quaternion<T>& left, const Quaternion<T>& right);
template <class T> inline Quaternion<T> slerp(const Quaternion<T>& from, const Quaternion<T>& to, const T lambda);



template <class T>
struct Transformation3{
  Quaternion<T> rotationQuaternion;
  Vector3<T>    translationVector;

  Transformation3(){}

  inline static Transformation3<T> identity();

  Transformation3 (const Vector3<T>& trans, const Quaternion<T>& rot);
  Transformation3 (const Pose3<T>& v);
  Transformation3 (const T& x, const T& y, const T& z, const T& roll, const T& pitch, const T& yaw);

  inline Vector3<T> translation() const;
  inline Quaternion <T> rotation() const;

  inline Pose3<T> toPoseType() const;
  
  inline void setTranslation(const Vector3<T>& t);
  inline void setTranslation(const T& x, const T& y, const T& z);

  inline void setRotation(const Vector3<T>& r);
  inline void setRotation(const T& roll, const T& pitch, const T& yaw);
  inline void setRotation(const Quaternion<T>& q);


  inline Transformation3<T> inv() const;
  inline bool validRotation(const T& epsilon=0.001) const;

};

template <class T>
inline Vector3<T> operator * (const Transformation3<T>& m, const Vector3<T>& v);

template <class T>
inline Transformation3<T> operator * (const Transformation3<T>& m1, const Transformation3<T>& m2);

template <class T>
struct Operations3D{
  typedef T                  BaseType;
  typedef Pose3<T>           PoseType;
  typedef Quaternion<T>      RotationType;
  typedef Vector3<T>         TranslationType;
  typedef Transformation3<T> TransformationType;
  typedef DMatrix<T>         CovarianceType;
  typedef DMatrix<T>         InformationType;
  typedef Transformation3<T> ParametersType;
};

} // namespace AISNavigation
/**************************** IMPLEMENTATION ****************************/

#include "transformation3.hxx"

#endif

