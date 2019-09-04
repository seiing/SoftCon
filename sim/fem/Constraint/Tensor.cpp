#include "Tensor.h"
Tensor3333::
Tensor3333()
{

}
Tensor3333::
Tensor3333(const Tensor3333& other)
{
	A[0][0] = other.A[0][0],A[0][1] = other.A[0][1],A[0][2] = other.A[0][2];
	A[1][0] = other.A[1][0],A[1][1] = other.A[1][1],A[1][2] = other.A[1][2];
	A[2][0] = other.A[2][0],A[2][1] = other.A[2][1],A[2][2] = other.A[2][2];
}
Tensor3333&
Tensor3333::
operator=(const Tensor3333& other)
{
	A[0][0] = other.A[0][0],A[0][1] = other.A[0][1],A[0][2] = other.A[0][2];
	A[1][0] = other.A[1][0],A[1][1] = other.A[1][1],A[1][2] = other.A[1][2];
	A[2][0] = other.A[2][0],A[2][1] = other.A[2][1],A[2][2] = other.A[2][2];
}
Tensor3333
Tensor3333::
operator+() const
{
	Tensor3333 ret = *this;
	return ret;
}
Tensor3333
Tensor3333::
operator-() const
{
	Tensor3333 ret;
	ret.A[0][0] = -A[0][0],ret.A[0][1] = -A[0][1],ret.A[0][2] = -A[0][2];
	ret.A[1][0] = -A[1][0],ret.A[1][1] = -A[1][1],ret.A[1][2] = -A[1][2];
	ret.A[2][0] = -A[2][0],ret.A[2][1] = -A[2][1],ret.A[2][2] = -A[2][2];
	return ret;
}
Tensor3333
Tensor3333::
operator+(const Tensor3333& B) const
{
	Tensor3333 ret;
	ret.A[0][0] = A[0][0] + B.A[0][0],ret.A[0][1] = A[0][1] + B.A[0][1],ret.A[0][2] = A[0][2] + B.A[0][2];
	ret.A[1][0] = A[1][0] + B.A[1][0],ret.A[1][1] = A[1][1] + B.A[1][1],ret.A[1][2] = A[1][2] + B.A[1][2];
	ret.A[2][0] = A[2][0] + B.A[2][0],ret.A[2][1] = A[2][1] + B.A[2][1],ret.A[2][2] = A[2][2] + B.A[2][2];
	return ret;
}
Tensor3333
Tensor3333::
operator-(const Tensor3333& B) const
{
	Tensor3333 ret;
	ret.A[0][0] = A[0][0] - B.A[0][0],ret.A[0][1] = A[0][1] - B.A[0][1],ret.A[0][2] = A[0][2] - B.A[0][2];
	ret.A[1][0] = A[1][0] - B.A[1][0],ret.A[1][1] = A[1][1] - B.A[1][1],ret.A[1][2] = A[1][2] - B.A[1][2];
	ret.A[2][0] = A[2][0] - B.A[2][0],ret.A[2][1] = A[2][1] - B.A[2][1],ret.A[2][2] = A[2][2] - B.A[2][2];
	return ret;
}
Tensor3333
Tensor3333::
operator*(const Eigen::Matrix3d& m) const
{
	Tensor3333 ret;
	ret.A[0][0] = A[0][0]*m(0,0) + A[0][1]*m(1,0) + A[0][2]*m(2,0);
	ret.A[0][1] = A[0][0]*m(0,1) + A[0][1]*m(1,1) + A[0][2]*m(2,1);
	ret.A[0][2] = A[0][0]*m(0,2) + A[0][1]*m(1,2) + A[0][2]*m(2,2);

	ret.A[1][0] = A[1][0]*m(0,0) + A[1][1]*m(1,0) + A[1][2]*m(2,0);
	ret.A[1][1] = A[1][0]*m(0,1) + A[1][1]*m(1,1) + A[1][2]*m(2,1);
	ret.A[1][2] = A[1][0]*m(0,2) + A[1][1]*m(1,2) + A[1][2]*m(2,2);

	ret.A[2][0] = A[2][0]*m(0,0) + A[2][1]*m(1,0) + A[2][2]*m(2,0);
	ret.A[2][1] = A[2][0]*m(0,1) + A[2][1]*m(1,1) + A[2][2]*m(2,1);
	ret.A[2][2] = A[2][0]*m(0,2) + A[2][1]*m(1,2) + A[2][2]*m(2,2);
	return ret;
}
Tensor3333
Tensor3333::
operator*(double a) const
{
	Tensor3333 ret;

	ret.A[0][0] = a*A[0][0],ret.A[0][1] = a*A[0][1],ret.A[0][2] = a*A[0][2];
	ret.A[1][0] = a*A[1][0],ret.A[1][1] = a*A[1][1],ret.A[1][2] = a*A[1][2];
	ret.A[2][0] = a*A[2][0],ret.A[2][1] = a*A[2][1],ret.A[2][2] = a*A[2][2];

	return ret;
}
Eigen::Matrix3d&
Tensor3333::
operator()(int i, int j)
{
	return A[i][j];
}
void
Tensor3333::
SetIdentity()
{
	A[0][0] = Eigen::Matrix3d::Zero(),A[0][1] = Eigen::Matrix3d::Zero(),A[0][2] = Eigen::Matrix3d::Zero();
	A[1][0] = Eigen::Matrix3d::Zero(),A[1][1] = Eigen::Matrix3d::Zero(),A[1][2] = Eigen::Matrix3d::Zero();
	A[2][0] = Eigen::Matrix3d::Zero(),A[2][1] = Eigen::Matrix3d::Zero(),A[2][2] = Eigen::Matrix3d::Zero();

	A[0][0](0,0) = 1.0,A[0][1](0,1) = 1.0,A[0][2](0,2) = 1.0;
	A[1][0](1,0) = 1.0,A[1][1](1,1) = 1.0,A[1][2](1,2) = 1.0;
	A[2][0](2,0) = 1.0,A[2][1](2,1) = 1.0,A[2][2](2,2) = 1.0;	
}
void
Tensor3333::
SetZero()
{
	A[0][0] = Eigen::Matrix3d::Zero(),A[0][1] = Eigen::Matrix3d::Zero(),A[0][2] = Eigen::Matrix3d::Zero();
	A[1][0] = Eigen::Matrix3d::Zero(),A[1][1] = Eigen::Matrix3d::Zero(),A[1][2] = Eigen::Matrix3d::Zero();
	A[2][0] = Eigen::Matrix3d::Zero(),A[2][1] = Eigen::Matrix3d::Zero(),A[2][2] = Eigen::Matrix3d::Zero();
}
Tensor3333
Tensor3333::
Transpose()
{
	Tensor3333 ret;

	ret.A[0][0] = A[0][0],ret.A[0][1] = A[1][0],ret.A[0][2] = A[2][0];
	ret.A[1][0] = A[0][1],ret.A[1][1] = A[1][1],ret.A[1][2] = A[2][1];
	ret.A[2][0] = A[0][2],ret.A[2][1] = A[1][2],ret.A[2][2] = A[2][2];

	A[0][0] = ret.A[0][0],A[0][1] = ret.A[0][1],A[0][2] = ret.A[0][2];
	A[1][0] = ret.A[1][0],A[1][1] = ret.A[1][1],A[1][2] = ret.A[1][2];
	A[2][0] = ret.A[2][0],A[2][1] = ret.A[2][1],A[2][2] = ret.A[2][2];

	return ret;
}
Tensor3333
operator*(double a,const Tensor3333& B)
{
	Tensor3333 ret;

	ret.A[0][0] = a*B.A[0][0],ret.A[0][1] = a*B.A[0][1],ret.A[0][2] = a*B.A[0][2];
	ret.A[1][0] = a*B.A[1][0],ret.A[1][1] = a*B.A[1][1],ret.A[1][2] = a*B.A[1][2];
	ret.A[2][0] = a*B.A[2][0],ret.A[2][1] = a*B.A[2][1],ret.A[2][2] = a*B.A[2][2];

	return ret;
}
Tensor3333
operator*(const Eigen::Matrix3d& m,const Tensor3333& B)
{
	Tensor3333 ret;

	ret.A[0][0] = m(0,0)*B.A[0][0] + m(0,1)*B.A[1][0] + m(0,2)*B.A[2][0];
	ret.A[0][1] = m(0,0)*B.A[0][1] + m(0,1)*B.A[1][1] + m(0,2)*B.A[2][1];
	ret.A[0][2] = m(0,0)*B.A[0][2] + m(0,1)*B.A[1][2] + m(0,2)*B.A[2][2];

	ret.A[1][0] = m(1,0)*B.A[0][0] + m(1,1)*B.A[1][0] + m(1,2)*B.A[2][0];
	ret.A[1][1] = m(1,0)*B.A[0][1] + m(1,1)*B.A[1][1] + m(1,2)*B.A[2][1];
	ret.A[1][2] = m(1,0)*B.A[0][2] + m(1,1)*B.A[1][2] + m(1,2)*B.A[2][2];

	ret.A[2][0] = m(2,0)*B.A[0][0] + m(2,1)*B.A[1][0] + m(2,2)*B.A[2][0];
	ret.A[2][1] = m(2,0)*B.A[0][1] + m(2,1)*B.A[1][1] + m(2,2)*B.A[2][1];
	ret.A[2][2] = m(2,0)*B.A[0][2] + m(2,1)*B.A[1][2] + m(2,2)*B.A[2][2];
	return ret;
}