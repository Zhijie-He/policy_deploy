#pragma once

#include <cassert>
#include <cstdio>
#include <iostream>
#include <memory>
#include <vector>
#include <cstring>

template<typename T>
class CubicInterp {
public:
  CubicInterp() {

    a.resize(DIM_, 0.);
    b.resize(DIM_, 0.);
    c.resize(DIM_, 0.);
    d.resize(DIM_, 0.);
    _xf.resize(DIM_, 0.);
    _x0.resize(DIM_, 0.);
    _vf.resize(DIM_, 0.);
    _v0.resize(DIM_, 0.);
  }

  ~CubicInterp() = default;

  bool setDimension(int dim) {
    DIM_ = dim;

    a.resize(DIM_, 0.);
    b.resize(DIM_, 0.);
    c.resize(DIM_, 0.);
    d.resize(DIM_, 0.);
    _xf.resize(DIM_, 0.);
    _x0.resize(DIM_, 0.);
    _vf.resize(DIM_, 0.);
    _v0.resize(DIM_, 0.);

    return true;
  }

  bool SetParam(T *x0, T *v0, T *xf, T *vf, T fin_time) {
    _tf = fin_time;
    memcpy(_x0.data(), x0, DIM_ * sizeof(T));
    memcpy(_v0.data(), v0, DIM_ * sizeof(T));
    memcpy(_xf.data(), xf, DIM_ * sizeof(T));
    memcpy(_vf.data(), vf, DIM_ * sizeof(T));

    memcpy(d.data(), x0, DIM_ * sizeof(T));
    memcpy(c.data(), v0, DIM_ * sizeof(T));

    T tf2 = _tf * _tf;
    T tf3 = tf2 * _tf;
    for (int i(0); i < DIM_; i++) {
      T dv = vf[i] - v0[i];
      T dx = xf[i] - x0[i];
      b[i] = 3 * dx / tf2 - 3 * v0[i] / _tf - dv / _tf;
      a[i] = dv / tf2 + 2 * v0[i] / tf2 - 2 * dx / tf3;

//        std::cout << "x = " << a[i] << "x3+" << b[i] << "x2+" << c[i] << "x+" << d[i] <<"\n";
    }
    return true;
  }

  bool getCurvePoint(T t, T *ret) {
    T t2 = t * t;
    T t3 = t2 * t;

    for (int i(0); i < DIM_; i++)
      ret[i] = a[i] * t3 + b[i] * t2 + c[i] * t + d[i];

    return true;
  }

  bool getCurveDerPoint(T t, T *ret) {
    T t2 = t * t;

    for (int i(0); i < DIM_; i++)
      ret[i] = 3 * a[i] * t2 + 2 * b[i] * t + c[i];

    return true;
  }

private:

  // x(t) = a*t^3 + b*t^2 + c*t +d
  // v(t) = dx(t)/dt = 3a*t^2 + 2b*t + c*t
  // xf = x(tf), x0 = x(0)
  // vf = v(tf), v0 = v(0)
  std::vector<T> a;
  std::vector<T> b;
  std::vector<T> c;
  std::vector<T> d;
  std::vector<T> _x0;
  std::vector<T> _xf;
  std::vector<T> _vf;
  std::vector<T> _v0;

  T _tf = 0;
  int DIM_ = 1;
};

