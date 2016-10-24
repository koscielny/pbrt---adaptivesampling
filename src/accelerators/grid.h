
/*
    pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_ACCELERATORS_GRID_H
#define PBRT_ACCELERATORS_GRID_H

// accelerators/grid.h*
#include "pbrt.h"
#include "primitive.h"

// GridAccel Forward Declarations
struct Voxel;

// Voxel Declarations
struct Voxel {
    // Voxel Public Methods
	uint32_t size() const {  return primitives.size(); }
    Voxel() { }
    Voxel(Reference<Primitive> &op, int vid) {
        allCanIntersect = false;
        primitives.push_back(op);
		VoxelID = vid;
    }
    void AddPrimitive(Reference<Primitive> prim) {
        primitives.push_back(prim);
    }
    bool Intersect(const Ray &ray, Intersection *isect, RWMutexLock &lock);
    bool IntersectP(const Ray &ray, RWMutexLock &lock);
	
	//add by ly
	int VoxelID=-1;
	void FindClusters(Clustertree *);//using Clustertree::AddOneCluster()
private:
    vector<Reference<Primitive> > primitives;
    bool allCanIntersect;
};


// GridAccel Declarations
class GridAccel : public Aggregate {
public:
    // GridAccel Public Methods
	GridAccel(const vector<Reference<Primitive> > &p, bool refineImmediately);
	GridAccel(const vector<Reference<Primitive> > &p, bool refineImmediately, Clustertree *);
    BBox WorldBound() const;
    bool CanIntersect() const { return true; }
    ~GridAccel();
    bool Intersect(const Ray &ray, Intersection *isect) const;
    bool IntersectP(const Ray &ray) const;
private:
    // GridAccel Private Methods
    int posToVoxel(const Point &P, int axis) const {
        int v = Float2Int((P[axis] - bounds.pMin[axis]) *
                          invWidth[axis]);
        return Clamp(v, 0, nVoxels[axis]-1);
    }
    float voxelToPos(int p, int axis) const {
        return bounds.pMin[axis] + p * width[axis];
    }
    inline int offset(int x, int y, int z) const {
        return z*nVoxels[0]*nVoxels[1] + y*nVoxels[0] + x;
    }

    // GridAccel Private Data
    vector<Reference<Primitive> > primitives;
    int nVoxels[3];
    BBox bounds;
    Vector width, invWidth;
    Voxel **voxels;
    MemoryArena voxelArena;
    mutable RWMutex *rwMutex;
};

//敏捷开发：结构和算法放一边，先线性遍历再考虑树结构加速。
//此算法是希望生成更好的采样密度分配，速度是第二考虑；所以也要实现生成采样密度图像的积分器。
//以均匀网格形式，几何、材质相似的面片聚类，聚类的包围盒为自适应采样提供一次性的metric
struct gridCluster{
	//体素ID or 体素指针？
	//grid的信息需要吗？
	//包围盒

	int voxelID;
	BBox boundbox;
	vector<Reference<Primitive> > primitives;

//	gridCluster():voxelID(-1),boundbox(),primitives(),representive_p(){}
	gridCluster() {}

	gridCluster(vector<Reference<Primitive> >& p, BBox bb, int vID)
		:primitives(p), boundbox(bb), voxelID(vID)
	{}
	gridCluster(Reference<Primitive>& p, int vID)//初始一个聚类
		:voxelID(vID)
	{
		primitives.push_back(p);
		boundbox = p->WorldBound();

		representive_p = p;
	}


//	bool IsSimilarNormal(Reference<Primitive> &p);
	bool IsSameMaterail(Reference<Primitive> &p);

	bool IsInCluster(Reference<Primitive> &p){
		return representive_p->isInCluster(p);//ly: 敏捷开发，只实现trianglemesh类型的法向比较
	}
	void addIntoCluster(Reference<Primitive> &p){
		primitives.push_back(p);
		boundbox = Union(this->boundbox, p->WorldBound());
	}
	void setIndex(int index){
		//可能多遍赋值，如果面片存在多个聚类内，它保存的聚类index就是最后一次赋值
		for (uint32_t i = 0; i < primitives.size(); ++i){
			if (primitives[i]->clusterId==-1)//ly:one touch
				primitives[i]->clusterId = index;
			primitives[i]->ctimes++;
		}
	}

	float ComputeSamplingValue(Ray cast_ray){//投射光线与该聚类相交
//axis-aligned boundbox
		Vector _dir = Normalize(cast_ray.d);
		float cos_XoY = abs(_dir.z);
		float cos_XoZ = abs(_dir.y);
		float cos_YoZ = abs(_dir.x);
//		float cos_XoY = sqrt(_dir.x*_dir.x + _dir.y*_dir.y);
//		float cos_XoZ = sqrt(_dir.x*_dir.x + _dir.z*_dir.z);
//		float cos_YoZ = sqrt(_dir.y*_dir.y + _dir.z*_dir.z);
		
		Vector _l = boundbox.pMax - boundbox.pMin;
		float S_XoY = (_l.x*_l.y);
		float S_XoZ = (_l.x*_l.z);
		float S_YoZ = (_l.y*_l.z);
		//float V_box = _l.x*_l.y*_l.z;//包围盒体积

		float S = S_XoY*cos_XoY + S_XoZ*cos_XoZ + S_YoZ*cos_YoZ;//包围盒相对视点\投射光线正对面积//三面相加
		float t = cast_ray.maxt;

		Normal *rn = new Normal();
		representive_p->getNormal(rn);
		Vector _n(*rn);
		_n = Normalize(_n);
		float v = abs(Dot(_dir, -_n));
		////Warning("%f",v);
		if (t == INFINITY || S < 0.00000001)
			return 0;
		else
			return 1 / (v*S);// 映射f((t^2 )*tanU*tanV*Spixel /S)，这里是线性映射f(t^2/S)的情形
	}

	//data
//	Normal representive_normal;
//	Reference<Material> representive_material;
	//or
	Reference<Primitive> representive_p;
};

class Clustertree{
public:
	//怎样表示一个聚类的层次结构？
	//用一个什么树连接所有聚类struct？你要作投影或求交操作
	//trivial的做法：所有cluster放进一个vector，线性遍历or Z-Buffer
	//Clustertree(const vector<Reference<Primitive> > &prims);
//	Clustertree():ClusterList(NULL){}
	Clustertree() {}
	void Constructor(Voxel** voxels, int nVoxel){
		for (int i = 0; i < nVoxel; i++){
			//int size = voxels[i]->size();
			
#ifdef DEBUG
			Warning("the %dth time parsing a Voxel of all %d. Voxel ID: %d, Voxel Size: %d", i, nVoxel,voxels[i]->VoxelID,voxels[i]->size());
#endif // !DEBUG
			if (voxels[i])
				voxels[i]->FindClusters(this);//voxel之间聚类互斥，不共享//FindClusters(this,i);
		}
	}

	void AddOneCluster(gridCluster c){
		int index = ClusterList.size();
		c.setIndex(index);
		ClusterList.push_back(c);
	}

	//计算采样率的方法
//	bool intersect();//与聚类包围盒求交，被getSamplingRate()调用
//	void Zbuffer();//包围盒投影到视平面

	//private:
	vector<gridCluster> ClusterList;
};

GridAccel *CreateGridAccelerator(const vector<Reference<Primitive> > &prims,
	const ParamSet &ps, Clustertree *);


void makeGrid( vector<Reference<Primitive> > &p,
	bool refineImmediately, Clustertree *CTree);

#endif // PBRT_ACCELERATORS_GRID_H
