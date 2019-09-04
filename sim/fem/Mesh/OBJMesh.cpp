#include "Mesh.h"
#include "OBJMesh.h"
#include <fstream>
#include <sstream>
#include <iostream>
using namespace FEM;
OBJMesh::
OBJMesh(const std::string& path,const Eigen::Affine3d& T)
	:Mesh()
{
	std::ifstream ifs(path);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		return;
	}
	std::string str;
	std::string index;
	std::stringstream ss;

	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);
		ss>>index;

		if(!index.compare("v"))
		{
			double x,y,z;
			ss>>x>>y>>z;
			mVertices.push_back(Eigen::Vector3d(x,y,z));
		}
		else if(!index.compare("vt"))
		{
			double x,y;
			ss>>x>>y;
			mTextureCoord.push_back(Eigen::Vector2d(x,y));
		}
		else if(!index.compare("vn"))
		{
			double x,y,z;
			ss>>x>>y>>z;
			mVerticesNormal.push_back(Eigen::Vector3d(x,y,z));
		}
		else if(!index.compare("f"))
		{
			int i0,i1,i2;
			int n0,n1,n2;
			int t0, t1, t2;
			Eigen::Vector3d position;
			Eigen::Vector3d normal;

			const char* chh=str.c_str();
			sscanf (chh, "f %d/%d/%d %d/%d/%d %d/%d/%dm",
				&i0,&t0,&n0,
				&i1,&t1,&n1,
				&i2,&t2,&n2);

			mTriangles.push_back(Eigen::Vector3i(i0,i1,i2));
			mFacesNormal.push_back(Eigen::Vector3i(n0,n1,n2));
			mFacesTexture.push_back(Eigen::Vector3i(t0,t1,t2));
		}
		else if(!index.compare("t"))
		{
			int i0,i1,i2,i3;
			ss>>i0>>i1>>i2>>i3;
			mTetrahedrons.push_back(Eigen::Vector4i(i0,i1,i2,i3));
		}

	}
	ifs.close();

	for(auto& v : mVertices)
		v = T*v;
}