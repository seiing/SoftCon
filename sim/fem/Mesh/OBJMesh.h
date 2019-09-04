#ifndef __OBJ_MESH_H__
#define __OBJ_MESH_H__
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace FEM
{
class Mesh;
class OBJMesh : public Mesh
{
public:
	OBJMesh(const std::string& obj_file,const Eigen::Affine3d& T = Eigen::Affine3d::Identity());	
};
};
#endif