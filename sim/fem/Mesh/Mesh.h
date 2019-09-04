#ifndef __MESH_H__
#define __MESH_H__
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <memory>
namespace FEM
{
class Mesh
{
public:
	Mesh(){};
	virtual const std::vector<Eigen::Vector3d>& GetVertices(){return mVertices;};
	virtual const std::vector<Eigen::Vector3i>& GetTriangles(){return mTriangles;};
	virtual const std::vector<Eigen::Vector4i>& GetTetrahedrons(){return mTetrahedrons;};
	virtual const std::vector<Eigen::Vector3d>& GetVertexNormal() {return mVerticesNormal;};
	virtual const std::vector<Eigen::Vector2d>& GetTextureCoord() {return mTextureCoord;};
	virtual const std::vector<Eigen::Vector3i>& GetFaceNormal() {return mFacesNormal;};
	virtual const std::vector<Eigen::Vector3i>& GetFaceTexture() {return mFacesTexture;};
	virtual void Clear() {mVertices.clear(); mTetrahedrons.clear();};
	
protected:
	std::vector<Eigen::Vector3d> mVertices;
	std::vector<Eigen::Vector3i> mTriangles;
	std::vector<Eigen::Vector4i> mTetrahedrons;
	std::vector<Eigen::Vector3d> mVerticesNormal;
	std::vector<Eigen::Vector2d> mTextureCoord;
	std::vector<Eigen::Vector3i> mFacesNormal;
	std::vector<Eigen::Vector3i> mFacesTexture;
};
};


#endif