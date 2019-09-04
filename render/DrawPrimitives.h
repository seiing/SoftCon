#ifndef __DRAW_PRIMITIVES_H__
#define __DRAW_PRIMITIVES_H__
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace GUI
{
	void DrawTetrahedron(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,
		const Eigen::Vector4d& color = Eigen::Vector4d(0.8,0.8,0.8,0.2),const bool& draw_line=false);
	void DrawTriangle(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,
		const Eigen::Vector3d& n0,const Eigen::Vector3d& n1,const Eigen::Vector3d& n2,
		const Eigen::Vector4d& color = Eigen::Vector4d(0.8,0.8,0.8,0.2),const bool& draw_line=false);
	void DrawLine(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector4d& color = Eigen::Vector4d(0.8,0.8,0.8,0.2),const double& line_width=1.0);
	void DrawPoint(const Eigen::Vector3d& p0,const double& size,const Eigen::Vector4d& color = Eigen::Vector4d(0.8,0.8,0.8,0.2));
	void DrawCylinder(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const double& line_width=1.0,const Eigen::Vector4d& color = Eigen::Vector4d(0.8,0.8,0.8,0.2));
	void DrawCylinder(double radius, double height, int slices, int stacks);
	void DrawTorus(const Eigen::Vector3d& center,const double& r,const Eigen::Vector3d& dir,const Eigen::Vector4d& color = Eigen::Vector4d(0.8,0.8,0.8,0.2));
	void DrawArrow3D(const Eigen::Vector3d& _pt, const Eigen::Vector3d& _dir,
                 const double _thickness,const Eigen::Vector3d& color = Eigen::Vector3d(0.8,0.8,0.8),
                 const double _arrowThickness = -1);
};
#endif