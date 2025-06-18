/*! @file MathUtilities.h
 *  @brief Utility functions for math
 *
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <fstream>


/*!
 * Square a number
 */
template <typename T>
T square(T a) {
  return a * a;
}

template <typename T>
T clip(T a, T max, T min) {
  return a>max?max:(a<min?min:a);
}

/*!
 * Are two eigen matrices almost equal?
 */
template <typename T, typename T2>
bool almostEqual(const Eigen::MatrixBase<T>& a, const Eigen::MatrixBase<T>& b,
                 T2 tol) {
  long x = T::RowsAtCompileTime;
  long y = T::ColsAtCompileTime;

  if (T::RowsAtCompileTime == Eigen::Dynamic ||
      T::ColsAtCompileTime == Eigen::Dynamic) {
    assert(a.rows() == b.rows());
    assert(a.cols() == b.cols());
    x = a.rows();
    y = a.cols();
  }

  for (long i = 0; i < x; i++) {
    for (long j = 0; j < y; j++) {
      T2 error = std::abs(a(i, j) - b(i, j));
      if (error >= tol) return false;
    }
  }
  return true;
}


static Eigen::MatrixXd readCSV(const std::string& file, int rows, int cols, int skiprows=0) {
  // std::cout << "Going to load csv with " << rows << " rows and " << cols << " columns." << std::endl;
  std::ifstream in;
  in.open(file.c_str());
  std::string line;
  int row = 0;
  int col;

  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(rows, cols);
  if (in.is_open()) {
    for (int i=0;i<skiprows;i++){
      std::getline(in, line);
    }
    while (std::getline(in, line)) {
      char *ptr = (char *) line.c_str();
      int len = line.length();
      col = 0;
      char *start = ptr;
      for (int i = 0; i < len; i++)
        if (ptr[i] == ',') {
          res(row, col++) = atof(start);
          start = ptr + i + 1;
          if(col==cols) break;
        }
      // std::cout<<row<<", "<<col<<", "<< start<<std::endl;
      if(col<cols) res(row, col) = atof(start);
      row++;
      if(row==rows) break;
    }
    in.close();
  }
  return res;
}

