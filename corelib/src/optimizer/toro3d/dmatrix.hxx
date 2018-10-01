
template <class X> DVector<X>::DVector(int n) {
  if (n<1) n=1;
  size=n;
  elems=new X[size];
  for (int i=0;i<size; i++) 
    elems[i]=X(0);
  shares=new int;
  (*shares)=1;
}

template <class X> DVector<X>::~DVector() {
  if (--(*shares)) return;
  delete [] elems;
  delete shares;
}

template <class X> DVector<X>::DVector(const DVector<X>& m) {
  shares=m.shares;
  elems=m.elems;
  size=m.size;
  (*shares)++;
}

template <class X> DVector<X>& DVector<X>::operator=(const DVector<X>& m) {
  if (shares==m.shares)
    return *this;
  if (!--(*shares)) {
    delete [] elems;
    delete shares;
  }
  shares=m.shares;
  elems=m.elems;
  size=m.size;
  (*shares)++;
  return *this;
}

template <class X> X DVector<X>::operator*(const DVector<X>& v) const{
  if (size!=v.size) throw DIncompatibleMatrixException();
  X p=X(0);
  for (int i=0; i<size; i++)
    p+=elems[i]*v.elems[i];
  return p;
}

template <class X> void DVector<X>::detach() {
  DVector<X> aux(size);
  for (int i=0;i<size;i++) aux.elems[i]=elems[i];
  operator=(aux);
}

template <class X> DVector<X> DVector<X>::operator+(const DVector<X>& v) const{
  if (size!=v.size) throw DIncompatibleMatrixException();
  DVector<X> r(size);
  for (int i=0; i<size; i++){
    r.elems[i]=elems[i]+v.elems[i];
  }
  return r;
}

template <class X> DVector<X> DVector<X>::operator-(const DVector<X>& v) const{
  if (size!=v.size) throw DIncompatibleMatrixException();
  DVector<X> r(size);
  for (int i=0; i<size; i++){
    r.elems[i]=elems[i]-v.elems[i];
  }
  return r;
}

template <class X> DVector<X> DVector<X>::operator*(const X& d) const{
  DVector<X> r(size);
  for (int i=0; i<size; i++){
    r.elems[i]=elems[i]*d;
  }
  return r;
}



template <class X> DMatrix<X>::DMatrix(int n,int m) {
  if (n<1) n=1;
  if (m<1) m=1;
  nrows=n;
  ncols=m;
  elems=new X[nrows*ncols];
  mrows=new X* [nrows];
  for (int i=0;i<nrows;i++) mrows[i]=elems+ncols*i;
  for (int i=0;i<nrows*ncols;i++) elems[i]=X(0);
  shares=new int;
  (*shares)=1;
}

template <class X> DMatrix<X>::~DMatrix() {
  if (--(*shares)) return;
  delete [] elems;
  delete [] mrows;
  delete shares;
}

template <class X> DMatrix<X>::DMatrix(const DMatrix& m) {
  shares=m.shares;
  elems=m.elems;
  nrows=m.nrows;
  ncols=m.ncols;
  mrows=m.mrows;
  (*shares)++;
}

template <class X> DMatrix<X>& DMatrix<X>::operator=(const DMatrix& m) {
  if (shares==m.shares)
    return *this;
  if (!--(*shares)) {
    delete [] elems;
    delete [] mrows;
    delete shares;
  }
  shares=m.shares;
  elems=m.elems;
  nrows=m.nrows;
  ncols=m.ncols;
  mrows=m.mrows;
  (*shares)++;
  return *this;
}

template <class X> DMatrix<X> DMatrix<X>::inv() const {
  if (nrows!=ncols) throw DNotInvertibleMatrixException();
  DMatrix<X> aux1(*this),aux2(I(nrows));
  aux1.detach();
  for (int i=0;i<nrows;i++) {
    int k=i;
    for (;k<nrows&&aux1.mrows[k][i]==X(0);k++){};
    if (k>=nrows) throw DNotInvertibleMatrixException();
    X val=aux1.mrows[k][i];
    for (int j=0;j<nrows;j++) {
      aux1.mrows[k][j]=aux1.mrows[k][j]/val;
      aux2.mrows[k][j]=aux2.mrows[k][j]/val;
    }
    if (k!=i) {
      for (int j=0;j<nrows;j++) {
	X tmp=aux1.mrows[k][j];
	aux1.mrows[k][j]=aux1.mrows[i][j];
	aux1.mrows[i][j]=tmp;
	tmp=aux2.mrows[k][j];
	aux2.mrows[k][j]=aux2.mrows[i][j];
	aux2.mrows[i][j]=tmp;
      }
    }
    for (int j=0;j<nrows;j++)
      if (j!=i) {
	X tmp=aux1.mrows[j][i];
	for (int l=0;l<nrows;l++) {
	  aux1.mrows[j][l]=aux1.mrows[j][l]-tmp*aux1.mrows[i][l];
	  aux2.mrows[j][l]=aux2.mrows[j][l]-tmp*aux2.mrows[i][l];
	}
      }
  }
  return aux2;
}

template <class X> const X DMatrix<X>::det() const {
  if (nrows!=ncols) throw DNotSquareMatrixException();
  DMatrix<X> aux(*this);
  X d=X(1);
  aux.detach();
  for (int i=0;i<nrows;i++) {
    int k=i;
    for (;k<nrows&&aux.mrows[k][i]==X(0);k++){};
    if (k>=nrows) return X(0);
    X val=aux.mrows[k][i];
    for (int j=0;j<nrows;j++) {
      aux.mrows[k][j]/=val;
    }
    d=d*val;
    if (k!=i) {
      for (int j=0;j<nrows;j++) {
	X tmp=aux.mrows[k][j];
	aux.mrows[k][j]=aux.mrows[i][j];
	aux.mrows[i][j]=tmp;
      }
      d=-d;	
    }
    for (int j=i+1;j<nrows;j++){
      X tmp=aux.mrows[j][i];
      if (!(tmp==X(0)) ){
	for (int l=0;l<nrows;l++) {
	  aux.mrows[j][l]=aux.mrows[j][l]-tmp*aux.mrows[i][l];
	}
	//d=d*tmp;
      }		
		}
  }
  return d;
}

template <class X> DMatrix<X> DMatrix<X>::transpose() const {
  DMatrix<X> aux(ncols, nrows);
  for (int i=0; i<nrows; i++)
    for (int j=0; j<ncols; j++)
      aux[j][i]=mrows[i][j];
  return aux;
}

template <class X> DMatrix<X> DMatrix<X>::operator*(const DMatrix<X>& m) const {
  if (ncols!=m.nrows) throw DIncompatibleMatrixException();
  DMatrix<X> aux(nrows,m.ncols);
  for (int i=0;i<nrows;i++)
    for (int j=0;j<m.ncols;j++){
      X a=0;
      for (int k=0;k<ncols;k++)
	a+=mrows[i][k]*m.mrows[k][j];
      aux.mrows[i][j]=a;
    }
  return aux;
}

template <class X> DMatrix<X> DMatrix<X>::operator+(const DMatrix<X>& m) const {
  if (ncols!=m.ncols||nrows!=m.nrows) throw DIncompatibleMatrixException();
  DMatrix<X> aux(nrows,ncols);
  for (int i=0;i<nrows*ncols;i++) aux.elems[i]=elems[i]+m.elems[i];
  return aux;
}

template <class X> DMatrix<X> DMatrix<X>::operator-(const DMatrix<X>& m) const {
  if (ncols!=m.ncols||nrows!=m.nrows) throw DIncompatibleMatrixException();
  DMatrix<X> aux(nrows,ncols);
  for (int i=0;i<nrows*ncols;i++) aux.elems[i]=elems[i]-m.elems[i];
  return aux;
}

template <class X> DMatrix<X> DMatrix<X>::operator*(const X& e) const {
  DMatrix<X> aux(nrows,ncols);
  for (int i=0;i<nrows*ncols;i++) aux.elems[i]=elems[i]*e;
  return aux;
}

template <class X> void DMatrix<X>::detach() {
  DMatrix<X> aux(nrows,ncols);
  for (int i=0;i<nrows*ncols;i++) aux.elems[i]=elems[i];
  operator=(aux);
}

template <class X> DMatrix<X> DMatrix<X>::I(int n) {
  DMatrix<X> aux(n,n);
  for (int i=0;i<n;i++) aux[i][i]=X(1);
  return aux;
}

template <class X> std::ostream& operator<<(std::ostream& os, const DMatrix<X> &m) {
  os << "{";
  for (int i=0;i<m.rows();i++) {
    if (i>0) os << ",";
    os << "{";
    for (int j=0;j<m.columns();j++) {
      if (j>0) os << ",";
      os << m[i][j];
    }
    os << "}";
  }
  return os << "}";
}


template <class X> DVector<X> operator * (const DMatrix<X> m, const DVector<X> v){
    if (v.dim()!=m.columns()) throw DIncompatibleMatrixException();
    DVector<X> r(m.rows());
    for (int i=0; i<m.rows(); i++){
      X a=X(0);
      for (int j=0; j<m.columns(); j++){
	a+=m[i][j]*v[j];
      }
      r[i]=a;
    }
    return r;
}

template <class X> DVector<X> operator * (const DVector<X> v, const DMatrix<X> m){
    if (v.dim()!=m.rows()) throw DIncompatibleMatrixException();
    DVector<X> r(m.columns());
    for (int i=0; i<m.columns(); i++){
      X a=X(0);
      for (int j=0; j<m.rows(); j++){
	a+=m[j][i]*v[j];
      }
      r[i]=a;
    }
    return r;
}
