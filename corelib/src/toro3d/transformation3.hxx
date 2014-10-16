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

#include <limits>

namespace AISNavigation {

template <class T>
inline Vector3<T> operator * (const T& d, const Vector3<T>& v) {
  return Vector3<T>(v.elems[0]*d, v.elems[1]*d, v.elems[2]*d);
}

template <class T>
inline Vector3<T> operator * (const Vector3<T>& v, const T& d) {
  return Vector3<T>(v.elems[0]*d, v.elems[1]*d, v.elems[2]*d);
}

template <class T>
inline T operator * (const Vector3<T>& v1, const Vector3<T>& v2){
  return v1.elems[0]*v2.elems[0] 
    + v1.elems[1]*v2.elems[1] 
    + v1.elems[2]*v2.elems[2];
}

template <class T>
inline Vector3<T> operator + (const Vector3<T>& v1, const Vector3<T>& v2){
  return Vector3<T>(v1.elems[0]+v2.elems[0], 
		    v1.elems[1]+v2.elems[1], 
		    v1.elems[2]+v2.elems[2]);
}

template <class T>
Vector3<T> operator - (const Vector3<T>& v1, const Vector3<T>& v2){
  return Vector3<T>(v1.elems[0]-v2.elems[0], 
		    v1.elems[1]-v2.elems[1],
		    v1.elems[2]-v2.elems[2]);
}



template <class T>
Pose3<T>::Pose3(): DVector<T>(6){
}

template <class T>
Pose3<T>::Pose3(const Vector3<T>& trans, const Vector3<T>& rot): DVector<T>(6){
  DVector<T>::elems[0]=rot.roll(); 
  DVector<T>::elems[1]=rot.pitch(); 
  DVector<T>::elems[2]=rot.yaw(); 
  DVector<T>::elems[3]=trans.x(); 
  DVector<T>::elems[4]=trans.y(); 
  DVector<T>::elems[5]=trans.z();
}

template <class T>
Pose3<T>::Pose3(const T& x, const T& y, const T& z, const T& r, const T& p, const T& yw): DVector<T>(6){
  DVector<T>::elems[0]=r; 
  DVector<T>::elems[1]=p; 
  DVector<T>::elems[2]=yw; 
  DVector<T>::elems[3]=x; 
  DVector<T>::elems[4]=y; 
  DVector<T>::elems[5]=z;
}


#define MY_MAX(a,b) (((a)>(b))?(a):(b))


template<class T>
Quaternion<T>::Quaternion(){
	w = 1;
	x = 0;
	y = 0;
	z = 0;
}

template<class T>
Quaternion<T>::Quaternion(const Vector3<T>& pose){
	w = 0;
	x = pose.x();
	y = pose.y();
	z = pose.z();
}

template<class T>
Quaternion<T>::Quaternion(const Vector3<T>& axis, const T& angle){
  T sa=sin(angle/2);
  T ca=cos(angle/2);
  w=ca;
  x=axis.x()*sa;
  y=axis.y()*sa;
  z=axis.z()*sa;
}

template<class T>
Quaternion<T>::Quaternion(const T _w, const T _x, const T _y, const T _z){
	w = _w;
	x = _x;
	y = _y;
	z = _z;
}

template<class T>
Quaternion<T>::Quaternion(const T phi, const T theta, const T psi){
	T sphi   = sin(phi);
	T stheta = sin(theta);
	T spsi   = sin(psi);
	T cphi   = cos(phi);
	T ctheta = cos(theta);
	T cpsi   = cos(psi);
	
	T _r[3][3] = { //create rotational Matrix
		{cpsi*ctheta, cpsi*stheta*sphi - spsi*cphi, cpsi*stheta*cphi + spsi*sphi},
		{spsi*ctheta, spsi*stheta*sphi + cpsi*cphi, spsi*stheta*cphi - cpsi*sphi},
		{    -stheta,                  ctheta*sphi,                  ctheta*cphi}
	};
	
	T _w = sqrt(MY_MAX(0, 1 + _r[0][0] + _r[1][1] + _r[2][2]))/2.0;
	T _x = sqrt(MY_MAX(0, 1 + _r[0][0] - _r[1][1] - _r[2][2]))/2.0;
	T _y = sqrt(MY_MAX(0, 1 - _r[0][0] + _r[1][1] - _r[2][2]))/2.0;
	T _z = sqrt(MY_MAX(0, 1 - _r[0][0] - _r[1][1] + _r[2][2]))/2.0;
	this->w = _w;
	this->x = (_r[2][1] - _r[1][2])>=0?fabs(_x):-fabs(_x);
	this->y = (_r[0][2] - _r[2][0])>=0?fabs(_y):-fabs(_y);
	this->z = (_r[1][0] - _r[0][1])>=0?fabs(_z):-fabs(_z);
}


template<class T>
inline Quaternion<T> Quaternion<T>::conjugated() const{
	return Quaternion<T>(w,-x,-y,-z);
}

template<class T>
inline Quaternion<T> Quaternion<T>::normalized() const{
	T n = this->norm();
	if (n > 0)
		return ((1./n) * (*this));
	else
		return Quaternion<T>(0.,0.,0.,0.);
}

template<class T>
inline Quaternion<T> Quaternion<T>::inverse() const{
	return ((1./this->norm()) * this->conjugated());
}

template<class T>
inline Quaternion<T> Quaternion<T>::rotateThisAlong(const Vector3<T>& axis, const T alpha) const{
	Quaternion<T> q(axis);
	q = q.normalized();
	q = q.withRotation(alpha);
	return q.rotatePoint(*this);
}

template<class T>
inline Quaternion<T> Quaternion<T>::rotatePoint(const Quaternion<T>& p) const{
	return (*this)*p*(this->conjugated());
}

template<class T>
inline Vector3<T> Quaternion<T>::rotatePoint(const Vector3<T>& point) const{
	Quaternion<T> p(point);
	Quaternion<T> q = this->rotatePoint(p);
	return q.im();
}

template<class T>
inline Quaternion<T> Quaternion<T>::withRotation(const T alpha) const{
	Quaternion<T> q = normalized();
	T salpha = sin(alpha/2.);
	T calpha = cos(alpha/2.);
	q.w = calpha;
	q.x = salpha * q.x;
	q.y = salpha * q.y;
	q.z = salpha * q.z;
	return q;
}


template<class T>
inline Vector3<T> Quaternion<T>::toAngles() const{
	T n = this->norm();
	T s = n > 0?2./(n*n):0.;
	
	T m00, m10, m20, m21, m22;
	T phi,theta,psi;
	
	
	T xs = this->x*s;
	T ys = this->y*s;
	T zs = this->z*s;
	
	T wx = this->w*xs;
	T wy = this->w*ys;
	T wz = this->w*zs;
	
	T xx = this->x*xs;
	T xy = this->x*ys;
	T xz = this->x*zs;
	
	T yy = this->y*ys;
	T yz = this->y*zs;
	
	T zz = this->z*zs;
	
	m00 = 1.0 - (yy + zz);
	//m11 = 1.0 - (xx + zz);
	m22 = 1.0 - (xx + yy);
	
	
	m10 = xy + wz;
	//m01 = xy - wz;
	
	m20 = xz - wy;
	//m02 = xz + wy;
	m21 = yz + wx;
	//m12 = yz - wx;
	
	phi   = atan2(m21,m22);
	theta = atan2(-m20,sqrt(m21*m21 + m22*m22));
	psi 	= atan2(m10,m00);
	
	return Vector3<T>(phi, theta, psi);
}


template<class T>
inline Vector3<T> Quaternion<T>::axis() const {
  double imNorm=sqrt(x*x+y*y+z*z);
  if (imNorm<std::numeric_limits<double>::min()){
    return Vector3<T>(0.,0.,1.);
  }
  return Vector3<T>(x/imNorm, y/imNorm, z/imNorm);
}

template<class T>
inline T Quaternion<T>::angle() const{
  Quaternion<T> q=normalized();
  double a=2*atan2(sqrt(q.x*q.x + q.y*q.y  + q.z*q.z), q.w);
  return atan2(sin(a), cos(a));
}

template<class T>
inline T Quaternion<T>::norm() const{
	return sqrt(w*w + x*x + y*y + z*z);
}


template<class T>
inline T Quaternion<T>::re() const{
	return w;
}

template<class T>
inline Vector3<T> Quaternion<T>::im() const{
	return Vector3<T>(x, y, z);
}


template<class T>
inline Quaternion<T> operator + (const Quaternion<T>& left, const Quaternion<T>& right){
	return Quaternion<T>(left.w + right.w, left.x + right.x, left.y + right.y, left.z + right.z);
}

template<class T>
inline Quaternion<T> operator - (const Quaternion<T>& left, const Quaternion<T>& right){
	return Quaternion<T>(left.w - right.w, left.x - right.x, left.y - right.y, left.z - right.z);
}

template<class T>
inline Quaternion<T> operator * (const Quaternion<T>& q1, const Quaternion<T>& q2){
  return Quaternion<T> (q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
			q1.y*q2.z - q2.y*q1.z + q1.w*q2.x + q2.w*q1.x,
			q1.z*q2.x - q2.z*q1.x + q1.w*q2.y + q2.w*q1.y,
			q1.x*q2.y - q2.x*q1.y + q1.w*q2.z + q2.w*q1.z);
}

template<class T>
inline Quaternion<T> operator * (const Quaternion<T>& q, const T s){
	return Quaternion<T>(s*q.w, s*q.x, s*q.y, s*q.z);
}

template<class T>
inline Quaternion<T> operator * (const T s, const Quaternion<T>& q){
	return Quaternion<T>(q.w*s, q.x*s, q.y*s, q.z*s);
}

template<class T>
std::ostream& operator << (std::ostream& os, const Quaternion<T>& q){
	os << q.w << " " << q.x << " " << q.y << " " << q.z << " ";
	return os;
}

template<class T>
inline T innerproduct(const Quaternion<T>& q1, const Quaternion<T>& q2){
	return q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
}

template<class T>
inline Quaternion<T> slerp(const Quaternion<T>& from, const Quaternion<T>& to, const T lambda){
	Quaternion<T> _from = from.normalized();
	Quaternion<T> _to = to.normalized();
	T _cos_omega = innerproduct(_from,_to);
	_cos_omega = (_cos_omega>1)?1:_cos_omega;
	_cos_omega = (_cos_omega<-1)?-1:_cos_omega;
	T _omega = acos(_cos_omega);
	assert (!isnan(_cos_omega));
	if (fabs(_omega) < 1e-6)
		return to;
	
	//determine right direction of slerp:
	Quaternion<T> _pq = _from - _to;
	Quaternion<T> _pmq = _from + _to;
	T _first = _pq.norm();
	T _alternativ = _pmq.norm();
	
	Quaternion<T> q1 = _from;
	Quaternion<T> q2 = (_first < _alternativ)? (Quaternion<T>) _to: -1.*(Quaternion<T>)_to;
	//now calculate intermediate quaternion.
	
	Quaternion<T> ret = q1*(sin((1-lambda)*_omega)/(sin(_omega))) + q2*(sin(lambda*_omega)/sin(_omega));
	assert (!(isnan(ret.w) || isnan(ret.x) || isnan(ret.y) || isnan(ret.z)));
	return ret;
}


template <class T>
inline Transformation3<T> Transformation3<T>::identity(){
  Transformation3<T> m;
  m.rotationQuaternion=Quaternion<T>();
  m.translationVector(0.,0.,0.);
  return m;
}



template <class T>
inline Transformation3<T>::Transformation3 (const T& x, const T& y, const T& z, const T& roll, const T& pitch, const T& yaw){
  rotationQuaternion=Quaternion<T>(roll,pitch,yaw);
  translationVector=Vector3<T>(x,y,z);
}

template <class T>
inline Transformation3<T>::Transformation3 (const Pose3<T>& v){
  rotationQuaternion=Quaternion<T>(v.roll(),v.pitch(),v.yaw());
  translationVector=Vector3<T>(v.x(),v.y(),v.z());

}

template <class T>
inline Vector3<T> Transformation3<T>::translation() const {
  return translationVector;
}
  
template <class T>
inline Quaternion<T> Transformation3<T>::rotation()    const {
  return rotationQuaternion;
}
  
template <class T>
inline Pose3<T> Transformation3<T>::toPoseType()   const {
  Vector3<T> t=translation();
  Vector3<T> r=rotationQuaternion.toAngles();
  Pose3<T> rv(t.x(), t.y(), t.z(), r.roll(), r.pitch(), r.yaw() );
  return rv;
}
  
template <class T>
inline void Transformation3<T>::setTranslation(const Vector3<T>& t){
  translationVector=t;
}

template <class T>
inline void Transformation3<T>::setRotation(const Quaternion<T>& q){
  rotationQuaternion=q.normalized();
}

template <class T>
inline void Transformation3<T>::setRotation(const Vector3<T>& r){
  setRotation(r.roll(),r.pitch(), r.yaw());
}

template <class T>
inline void Transformation3<T>::setRotation(const T& roll_phi, const T& pitch_theta, const T& yaw_psi){
  rotationQuaternion=Quaternion<T>(roll_phi, pitch_theta, yaw_psi);
}

template <class T>
inline void Transformation3<T>::setTranslation(const T& x, const T& y, const T& z){
  translationVector=Vector3<T>(x,y,z);
}


template <class T>
inline Transformation3<T> Transformation3<T>::inv() const {

  Transformation3<T> rv(*this);
  rv.rotationQuaternion=rotationQuaternion.inverse().normalized();
  rv.translationVector=rv.rotationQuaternion.rotatePoint(translationVector*-1.);
  return rv;
}

template <class T>
inline Vector3<T> operator * (const Transformation3<T>& m, const Vector3<T>& v){
  return m.translationVector+m.rotationQuaternion.rotatePoint(v);
}

template <class T>
inline Transformation3<T> operator * (const Transformation3<T>& m1, const Transformation3<T>& m2){
  Transformation3<T> rv;
  rv.translationVector=m1.rotationQuaternion.rotatePoint(m2.translationVector)+m1.translationVector;
  rv.rotationQuaternion=(m1.rotationQuaternion*m2.rotationQuaternion).normalized();
  return rv;
}

} // namespace AISNavigation

