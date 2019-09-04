#ifndef __TENSOR_H__
#define __TENSOR_H__
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
class Tensor3333
{
public:
	Tensor3333();
	Tensor3333(const Tensor3333& other);
	Tensor3333& operator=(const Tensor3333& other);

	void SetIdentity();
	void SetZero();
	Tensor3333 Transpose();

	Tensor3333 operator+() const;
	Tensor3333 operator-() const;
	Tensor3333 operator+(const Tensor3333& B) const;
	Tensor3333 operator-(const Tensor3333& B) const;
	Tensor3333 operator*(const Eigen::Matrix3d& m) const;
	Tensor3333 operator*(double a) const;

	Eigen::Matrix3d& operator()(int i, int j);

public:
	Eigen::Matrix3d A[3][3];
};

Tensor3333 operator*(double a,const Tensor3333& B);
Tensor3333 operator*(const Eigen::Matrix3d& m,const Tensor3333& B);
#endif