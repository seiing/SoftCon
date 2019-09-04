#ifndef __CAMERA_H_
#define __CAMERA_H_
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <iostream>

class Camera
{
public: 
	Camera();
	void SetCamera(const Eigen::Vector3d& lookAt,const Eigen::Vector3d& eye,const Eigen::Vector3d& up);
	void Apply();

	void Pan(int x,int y,int prev_x,int prev_y);
	void Zoom(int x,int y,int prev_x,int prev_y);
	void Rotate(int x,int y,int prev_x,int prev_y);
	void Translate(int x,int y,int prev_x,int prev_y);

	void SetEye(Eigen::Vector3d eye);
	void SetLookAt(Eigen::Vector3d lookAt);
	void SetFocusAt(Eigen::Vector3d lookAt);
	void SetFovy(double fovy);
	Eigen::Vector3d GetWorldPosition(int x,int y);

	Eigen::Vector3d GetEye(){return eye;};

private:
	Eigen::Vector3d lookAt;
	Eigen::Vector3d eye;
	Eigen::Vector3d up;
	double fovy;

	Eigen::Vector3d Rotateq(const Eigen::Vector3d& target, const Eigen::Vector3d& rotateVector,double angle);
	Eigen::Vector3d GetTrackballPoint(int mouseX, int mouseY,int w,int h);
	Eigen::Vector3d UnProject(const Eigen::Vector3d& vec);
};
#endif