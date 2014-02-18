#ifndef __SYMMMATRIX_2X2_HH__
#define __SYMMMATRIX_2X2_HH__

#include "transformation2.hh"


  template <typename R>
  struct SMatrix2{
    typedef R Real;
    typedef Vector2<R> VectorType;

    R a11, a12, a22;

    SMatrix2 (const R& a11, const R& a12, const R& a22);
    SMatrix2 ();
    SMatrix2(const VectorType& v1);
    SMatrix2<R> operator+(const SMatrix2<R>& m);
    SMatrix2<R>& operator+=(const SMatrix2<R>& m);

    SMatrix2<R> operator*(const R&);
    SMatrix2<R>& operator*=(const R&);

    SMatrix2 inv() const;

    R det() const;

   void computeEigenvalues(R& l1, R& l2, VectorType& v1, VectorType& v2, const R& epsilon=R(0));
  };

  template <typename R>
  typename SMatrix2<R>::VectorType operator * (const SMatrix2<R>& m, const typename SMatrix2<R>::VectorType& v){
    return typename SMatrix2<R>::VectorType(m.a11*v.x()+m.a12*v.y(), m.a12*v.x()+m.a22*v.y());
  }

  
  template <typename R>
  SMatrix2<R>::SMatrix2(const R& _a11, const R& _a12, const R& _a22){
    a11=_a11;
    a12=_a12;
    a22=_a22;
  }

  template <typename R>
  SMatrix2<R>::SMatrix2 (){
    a11=R(0);
    a12=R(0);
    a22=R(0);
  }

  template <typename R>
  SMatrix2<R>::SMatrix2(const typename SMatrix2<R>::VectorType& v1){
    a11=v1.x()*v1.x();
    a12=v1.x()*v1.y();
    a22=v1.y()*v1.y();
  }
  
  template <typename R>
  SMatrix2<R> SMatrix2<R>::operator+(const SMatrix2<R>& m){
    return SMatrix2<R>(a11+m.a11, a12+m.a12, a22+m.a22);
  }
  
  template <typename R>
  SMatrix2<R>& SMatrix2<R>::operator+=(const SMatrix2<R>& m){
    a11+=m.a11;
    a12+=m.a12;
    a22+=m.a22;
    return *this;
  }
  
  template <typename R>
  SMatrix2<R> SMatrix2<R>::operator*(const R& a){
    return SMatrix2<R>(a11*a, a12*a, a22*a);
  }

  template <typename R>
  SMatrix2<R>& SMatrix2<R>::operator*=(const R& a){
    a11*=a;
    a12*=a;
    a22*=a;
    return *this;
  }
  
  template <typename R>
  SMatrix2<R> SMatrix2<R>::inv() const{
    R d=a11*a22-a12*a12;
    d=R(1)/d;
    return SMatrix2<R>(a22*d, -a12*d, a11*d);
  }

  template <typename R>
  R SMatrix2<R>::det() const{
    return a11*a22-a12*a12;
  }

  template <typename R>
  void SMatrix2<R>::computeEigenvalues(R& l1, R& l2, typename SMatrix2<R>::VectorType& v1, typename SMatrix2<R>::VectorType& v2, const R& epsilon){
    R b=R(0.5)*(a11+a22),
      c=a11*a22-a12*a12;

    R det=b*b-c;
    if (det<=R(0)){
      det=R(0);
    }

    det=sqrt(det);
    l1=b+det;
    l2=b-det;
    
    R x1=R(1), x2=R(0);
 
    if (l1-l2>epsilon){
      R b11=a11-l1, b22=a22-l1, b12=a12;
      if (b11*b11 > b12*b12){
	x1=-b12/b11;
	x2=1;
      } else {
	x1=-b22/b12;
	x2=1;
      }
    }
    v1=VectorType(x1,x2);
    v2=VectorType(-x2,x1);

    R m1=v1*v1;
    if (m1>0)
      v1=v1*(1./sqrt(m1));

    R m2=v2*v2;
    if (m2>0)
      v2=v2*(1./sqrt(m2));
  }

  typedef SMatrix2<double>  DSMatrix2;



#endif
