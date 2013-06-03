#ifndef DUALQUATERNION_H
#define DUALQUATERNION_H

#include <Eigen/Dense>
#include <iostream>

#include "QuaternionMapping.h"

namespace camodocal
{

template<typename T>
class DualQuaternion;

typedef DualQuaternion<float> DualQuaternionf;
typedef DualQuaternion<double> DualQuaterniond;

template<typename T>
DualQuaternion<T>
operator+(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2);

template<typename T>
DualQuaternion<T>
operator-(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2);

template<typename T>
DualQuaternion<T>
operator*(T scale, const DualQuaternion<T>& dq);

template<typename T>
std::ostream &
operator << (std::ostream &, const DualQuaternion<T> & ); 

template<typename T>
class DualQuaternion
{
public:
    DualQuaternion();
    DualQuaternion(const Eigen::Quaternion<T>& r,
                   const Eigen::Quaternion<T>& d);
    DualQuaternion(const Eigen::Quaternion<T>& r,
                   const Eigen::Matrix<T, 3, 1>& t);

    DualQuaternion<T> conjugate(void) const;
    Eigen::Quaternion<T> dual(void) const;
    DualQuaternion<T> exp(void) const;
    void fromScrew(T theta, T d,
                   const Eigen::Matrix<T, 3, 1>& l,
                   const Eigen::Matrix<T, 3, 1>& m);
    static DualQuaternion<T> identity(void);
    DualQuaternion<T> inverse(void) const;
    DualQuaternion<T> log(void) const;
    void norm(T& real, T& dual) const;
    void normalize(void);
    DualQuaternion<T> normalized(void) const;
    Eigen::Matrix<T, 3, 1> transformPoint(const Eigen::Matrix<T, 3, 1>& point) const;
    Eigen::Matrix<T, 3, 1> transformVector(const Eigen::Matrix<T, 3, 1>& vector) const;
    Eigen::Quaternion<T> real(void) const;
    Eigen::Quaternion<T> rotation(void) const;
    Eigen::Matrix<T, 3, 1> translation(void) const;
    Eigen::Quaternion<T> translationQuaternion(void) const;
    Eigen::Matrix<T, 4, 4> toMatrix(void) const;
    static DualQuaternion<T> zeros(void);

    DualQuaternion<T> operator*(T scale) const;
    DualQuaternion<T> operator*(const DualQuaternion<T>& other) const;
    friend DualQuaternion<T> operator+<>(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2);
    friend DualQuaternion<T> operator-<>(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2);
//    friend DualQuaternion<T> operator*<>(T scale, const DualQuaternion<T>& dq);
    
    friend std::ostream& operator << <> (std::ostream &, const DualQuaternion<T> &); 

private:
    Eigen::Quaternion<T> m_real; // real part
    Eigen::Quaternion<T> m_dual; // dual part
};

template<typename T>
DualQuaternion<T>::DualQuaternion()
{
    m_real = Eigen::Quaternion<T>(1, 0, 0, 0);
    m_dual = Eigen::Quaternion<T>(0, 0, 0, 0);
}

template<typename T>
DualQuaternion<T>::DualQuaternion(const Eigen::Quaternion<T>& r,
                                  const Eigen::Quaternion<T>& d)
{
    m_real = r;
    m_dual = d;
}

template<typename T>
DualQuaternion<T>::DualQuaternion(const Eigen::Quaternion<T>& r,
                                  const Eigen::Matrix<T, 3, 1>& t)
{
    m_real = r.normalized();
    m_dual = Eigen::Quaternion<T>(T(0.5) * (Eigen::Quaternion<T>(T(0), t(0), t(1), t(2)) * m_real).coeffs());
}

template<typename T>
DualQuaternion<T>
DualQuaternion<T>::conjugate(void) const
{
    return DualQuaternion<T>(m_real.conjugate(), m_dual.conjugate());
}

template<typename T>
Eigen::Quaternion<T>
DualQuaternion<T>::dual(void) const
{
    return m_dual;
}

template<typename T>
DualQuaternion<T>
DualQuaternion<T>::exp(void) const
{
    Eigen::Quaternion<T> real = expq(m_real);
    Eigen::Quaternion<T> dual = real * m_dual;

    return DualQuaternion<T>(real, dual);
}

template<typename T>
void
DualQuaternion<T>::fromScrew(T theta, T d,
                             const Eigen::Matrix<T, 3, 1>& l,
                             const Eigen::Matrix<T, 3, 1>& m)
{
    m_real = Eigen::AngleAxis<T>(theta, l);
    m_dual.w() = -d / 2.0 * sin(theta / 2.0);
    m_dual.vec() = sin(theta / 2.0) * m + d / 2.0 * cos(theta / 2.0) * l;
}

template<typename T>
DualQuaternion<T>
DualQuaternion<T>::identity(void)
{
    return DualQuaternion<T>(Eigen::Quaternion<T>::Identity(),
                             Eigen::Quaternion<T>(0, 0, 0, 0));
}

template<typename T>
DualQuaternion<T>
DualQuaternion<T>::inverse(void) const
{
    T sqrLen0 = m_real.squaredNorm();
    T sqrLenE = 2.0 * (m_real.coeffs().dot(m_dual.coeffs()));

    if (sqrLen0 > 0.0)
    {
        T invSqrLen0 = 1.0 / sqrLen0;
        T invSqrLenE = -sqrLenE / (sqrLen0 * sqrLen0);

        DualQuaternion<T> conj = conjugate();
        conj.m_real.coeffs() = invSqrLen0 * conj.m_real.coeffs();
        conj.m_dual.coeffs() = invSqrLen0 * conj.m_dual.coeffs() + invSqrLenE * conj.m_real.coeffs();

        return conj;
    }
    else
    {
        return DualQuaternion<T>::zeros();
    }
}

template<typename T>
DualQuaternion<T>
DualQuaternion<T>::log(void) const
{
    Eigen::Quaternion<T> real = logq(m_real);
    Eigen::Quaternion<T> dual = m_real.conjugate() * m_dual;
    T scale = T(1) / m_real.squaredNorm(); 
    dual.coeffs() *= scale; 

    return DualQuaternion<T>(real, dual);
}

template<typename T>
void
DualQuaternion<T>::norm(T& real, T& dual) const
{
    real = m_real.norm();
    dual = m_real.coeffs().dot(m_dual.coeffs()) / real;
}

template<typename T>
void
DualQuaternion<T>::normalize(void)
{
    T length = m_real.norm();
    T lengthSqr = m_real.squaredNorm();

    // real part is of unit length
    m_real.coeffs() /= length;

    // real and dual parts are orthogonal
    m_dual.coeffs() /= length;
    m_dual.coeffs() -= (m_real.coeffs().dot(m_dual.coeffs()) * lengthSqr) * m_real.coeffs();
}

template<typename T>
DualQuaternion<T>
DualQuaternion<T>::normalized(void) const
{
    DualQuaternion<T> dq = *this;
    dq.normalize();

    return dq;
}

template<typename T>
Eigen::Matrix<T, 3, 1>
DualQuaternion<T>::transformPoint(const Eigen::Matrix<T, 3, 1>& point) const
{
    DualQuaternion<T> dq = (*this)
                           * DualQuaternion<T>(Eigen::Quaternion<T>(1, 0, 0, 0),
                                               Eigen::Quaternion<T>(0, point(0,0), point(1,0), point(2,0)))
                           * conjugate();

    Eigen::Matrix<T, 3, 1> p(dq.m_dual.x(), dq.m_dual.y(), dq.m_dual.z());

    // translation
    p += 2.0 * (m_real.w() * m_dual.vec() - m_dual.w() * m_real.vec() + m_real.vec().cross(m_dual.vec()));

    return p;
}

template<typename T>
Eigen::Matrix<T, 3, 1>
DualQuaternion<T>::transformVector(const Eigen::Matrix<T, 3, 1>& vector) const
{
    DualQuaternion<T> dq = (*this)
                           * DualQuaternion<T>(Eigen::Quaternion<T>(1, 0, 0, 0),
                                               Eigen::Quaternion<T>(0, vector(0,0), vector(1,0), vector(2,0)))
                           * conjugate();

    return Eigen::Matrix<T, 3, 1>(dq.m_dual.x(), dq.m_dual.y(), dq.m_dual.z());
}

template<typename T>
Eigen::Quaternion<T>
DualQuaternion<T>::real(void) const
{
    return m_real;
}

template<typename T>
Eigen::Quaternion<T>
DualQuaternion<T>::rotation(void) const
{
    return m_real;
}

template<typename T>
Eigen::Matrix<T, 3, 1>
DualQuaternion<T>::translation(void) const
{
    Eigen::Quaternion<T> t(2.0 * (m_dual * m_real.conjugate()).coeffs());

    Eigen::Matrix<T, 3, 1> tvec;
    tvec << t.x(), t.y(), t.z();

    return tvec;
}

template<typename T>
Eigen::Quaternion<T>
DualQuaternion<T>::translationQuaternion(void) const
{
    Eigen::Quaternion<T> t(2.0 * (m_dual * m_real.conjugate()).coeffs());

    return t;
}

template<typename T>
Eigen::Matrix<T, 4, 4>
DualQuaternion<T>::toMatrix(void) const
{
    Eigen::Matrix<T, 4, 4> H = Eigen::Matrix<T, 4, 4>::Identity();

    H.block(0, 0, 3, 3)= m_real.toRotationMatrix();

    Eigen::Quaternion<T> t(2.0 * (m_dual * m_real.conjugate()).coeffs());
    H(0,3) = t.x();
    H(1,3) = t.y();
    H(2,3) = t.z();

    return H;
}

template<typename T>
DualQuaternion<T>
DualQuaternion<T>::zeros(void)
{
    return DualQuaternion<T>(Eigen::Quaternion<T>(T(0), T(0), T(0), T(0)),
                             Eigen::Quaternion<T>(T(0), T(0), T(0), T(0)));
}

template<typename T>
DualQuaternion<T>
DualQuaternion<T>::operator*(T scale) const
{
    return DualQuaternion<T>(Eigen::Quaternion<T>(scale * m_real.coeffs()),
                             Eigen::Quaternion<T>(scale * m_dual.coeffs()));
}

template<typename T>
DualQuaternion<T>
DualQuaternion<T>::operator*(const DualQuaternion<T>& other) const
{
    return DualQuaternion<T>(m_real * other.m_real,
                             Eigen::Quaternion<T>((m_real * other.m_dual).coeffs() +
                                                  (m_dual * other.m_real).coeffs()));
}

template<typename T>
DualQuaternion<T>
operator+(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2)
{
    return DualQuaternion<T>(Eigen::Quaternion<T>(dq1.m_real.coeffs() + dq2.m_real.coeffs()),
                             Eigen::Quaternion<T>(dq1.m_dual.coeffs() + dq2.m_dual.coeffs()));
}

template<typename T>
DualQuaternion<T>
operator-(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2)
{
    return DualQuaternion<T>(Eigen::Quaternion<T>(dq1.m_real.coeffs() - dq2.m_real.coeffs()),
                             Eigen::Quaternion<T>(dq1.m_dual.coeffs() - dq2.m_dual.coeffs()));
}

template<typename T>
DualQuaternion<T>
operator*(T scale, const DualQuaternion<T>& dq)
{
    return DualQuaternion<T>(Eigen::Quaternion<T>(scale * dq.real().coeffs()),
                             Eigen::Quaternion<T>(scale * dq.dual().coeffs()));
}


template<typename T>
std::ostream& operator << (std::ostream & out, const DualQuaternion<T> & dq)
{
    out << dq.m_real.w() << " " << dq.m_real.x() << " " << dq.m_real.y() << " " << dq.m_real.z() << " "
        << dq.m_dual.w() << " " << dq.m_dual.x() << " " << dq.m_dual.y() << " " << dq.m_dual.z() << std::endl; 
    return out; 
}

template<typename T>
DualQuaternion<T>
//exp(Eigen::Quaternion<T> _real, Eigen::Quaternion<T> _dual);
expdq(const std::pair<Eigen::Quaternion<T>, Eigen::Quaternion<T> > & v8x1 )
{
    Eigen::Quaternion<T> real = expq(v8x1.first);
    Eigen::Quaternion<T> dual = real * v8x1.second;
    return DualQuaternion<T>(real, dual);
}

template<typename T>
std::pair<Eigen::Quaternion<T>, Eigen::Quaternion<T> >
logdq(const DualQuaternion<T> & dq)
{
    Eigen::Quaternion<T> real = logq(dq.real());
    Eigen::Quaternion<T> dual = dq.real().inverse() * dq.dual();
    return std::make_pair(real, dual);
}

}



#endif
