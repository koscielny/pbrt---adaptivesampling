
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

#ifndef PBRT_CORE_PRIMITIVE_H
#define PBRT_CORE_PRIMITIVE_H

// core/primitive.h*
#include "pbrt.h"
#include "shape.h"
#include "material.h"
//bool IsSimilarNormal(Normal* n1, Normal* n2){
//	return (Dot(*n1, *n2) < 0.8660254037844386);//cos30度
//}
static uint32_t k1 = 0;
static uint32_t k2 = 0;
static uint32_t k3 = 0;
static uint32_t k4 = 0;
static uint32_t k5 = 0;
static uint32_t k = 0;
// Primitive Declarations
class Primitive : public ReferenceCounted {
public:
    // Primitive Interface
    Primitive() : primitiveId(nextprimitiveId++) { }
    virtual ~Primitive();
    virtual BBox WorldBound() const = 0;
    virtual bool CanIntersect() const;
    virtual bool Intersect(const Ray &r, Intersection *in) const = 0;
    virtual bool IntersectP(const Ray &r) const = 0;
    virtual void Refine(vector<Reference<Primitive> > &refined) const;
    void FullyRefine(vector<Reference<Primitive> > &refined) const;
    virtual const AreaLight *GetAreaLight() const = 0;
    virtual BSDF *GetBSDF(const DifferentialGeometry &dg,
        const Transform &ObjectToWorld, MemoryArena &arena) const = 0;
    virtual BSSRDF *GetBSSRDF(const DifferentialGeometry &dg,
        const Transform &ObjectToWorld, MemoryArena &arena) const = 0;
	//ly:在primitive 内部实现相似面片判定函数；
	//接口
	virtual bool isInCluster(Material &m, Normal &n){ return false; }//两种
	bool isInCluster(Reference<Primitive> &p){

		Normal* n1 = new Normal();
		Normal* n2 = new Normal();
		p->getNormal(n1);
		getNormal(n2);
		Normal n3 = Normalize(*n1);
		Normal n4 = Normalize(*n2);

		//*n1 = Normalize(*n1);
		//*n2 = Normalize(*n2);
		

		int m1 = getMaterial()->MaterialId;
		int m2 = p->getMaterial()->MaterialId;
		
		//Warning("material compare: %d, %d", m1,m2);
		//最开始错误在dot<0.866，应该是dot>0.866聚类
		//后来错误是没有normalize，n1,n2 过【小】，基本小于0.86或0.98，所以基本都大于0。866，都返回ture
		if (getMaterial()->MaterialId == p->getMaterial()->MaterialId&&getMaterial()!=NULL&&p->getMaterial()!=NULL){
			float d= Dot(n3, n4);
			//if (d < 0){  d = -d; }
			k++;
			//Assert (d <= 1);
			//if ( n2->Length()> 1.1) k1++;
			//if (n4.Length()> 1.1) k2++;
			//if (d > 1.1) k3++;
			//if (d > 0.98480775301221) k4++;
			//if (d < 0.98480775301221) k5++;
			//Warning("wow1_%d, wow2_%d, wowdot_%d, bigger_%d, smaller_%d, all_%d", k1, k2,k3,k4,k5,k);
			if (d >  0.98480775301221)//if (IsSimilarNormal(n1, n2))> 0.8660254037844386//> 0.98480775301221//> 0.99619469809175
				return true;
		}

		return false;
	}

	virtual void getNormal(Normal*){
#ifndef Debug
		Warning("Normal_base");
#endif // !Debug
		return; 
	}//明天换一种实现：将指针初始化后作为参数void getNormal(Normal *n);
	virtual Reference<Material> getMaterial() {
#ifdef DEBUG
		Warning("Material_base");
#endif // DEBUG
		return NULL; 
	}

	virtual vector<Reference<Primitive> >& getAggrePrims(){ return vector < Reference<Primitive> > {NULL}; }
    // Primitive Public Data
    const uint32_t primitiveId;

	//ly:面片定位其聚类的坐标，暂时放在基类里，如不是面片，或错误情况，仍然为初始值-1
	uint32_t clusterId = -1;
	uint32_t ctimes = 0;
protected:
    // Primitive Protected Data
    static uint32_t nextprimitiveId;
};



// GeometricPrimitive Declarations
class GeometricPrimitive : public Primitive {
public:
    // GeometricPrimitive Public Methods
    bool CanIntersect() const;
    void Refine(vector<Reference<Primitive> > &refined) const;
    virtual BBox WorldBound() const;
    virtual bool Intersect(const Ray &r, Intersection *isect) const;
    virtual bool IntersectP(const Ray &r) const;
    GeometricPrimitive(const Reference<Shape> &s,
                       const Reference<Material> &m, AreaLight *a);
    const AreaLight *GetAreaLight() const;
    BSDF *GetBSDF(const DifferentialGeometry &dg,
                  const Transform &ObjectToWorld, MemoryArena &arena) const;
    BSSRDF *GetBSSRDF(const DifferentialGeometry &dg,
                      const Transform &ObjectToWorld, MemoryArena &arena) const;

//ly:敏捷开发

	void getNormal(Normal* n){
#ifdef DEBUG
		Warning("Normal_ch1");
#endif // DEBUG
		shape->getNormal(n);
	}

	Reference<Material> getMaterial(){
#ifdef DEBUG
		Warning("Material_ch1");
#endif // DEBUG
		return material;
	}
private:
    // GeometricPrimitive Private Data
    Reference<Shape> shape;
    Reference<Material> material;
    AreaLight *areaLight;
};



// TransformedPrimitive Declarations
class TransformedPrimitive : public Primitive {
public:
    // TransformedPrimitive Public Methods
    TransformedPrimitive(Reference<Primitive> &prim,
                         const AnimatedTransform &w2p)
        : primitive(prim), WorldToPrimitive(w2p) { }
    bool Intersect(const Ray &r, Intersection *in) const;
    bool IntersectP(const Ray &r) const;
    const AreaLight *GetAreaLight() const { return NULL; }
    BSDF *GetBSDF(const DifferentialGeometry &dg,
                  const Transform &ObjectToWorld, MemoryArena &arena) const {
        return NULL;
    }
    BSSRDF *GetBSSRDF(const DifferentialGeometry &dg,
                  const Transform &ObjectToWorld, MemoryArena &arena) const {
        return NULL;
    }
    BBox WorldBound() const {
        return WorldToPrimitive.MotionBounds(primitive->WorldBound(), true);
    }


	void getNormal(Normal*){
#ifdef DEBUG
		Warning("Normal_ch2");
#endif // DEBUG
		return;
	}//{ Warning("Normal_子类2");  return primitive->getNormal(); }
	Reference<Material> getMaterial(){
#ifdef DEBUG
		Warning("Material_ch2");
#endif // DEBUG
		return primitive->getMaterial();
	}

private:
    // TransformedPrimitive Private Data
    Reference<Primitive> primitive;
    const AnimatedTransform WorldToPrimitive;
};



// Aggregate Declarations
class Aggregate : public Primitive {
public:
    // Aggregate Public Methods
    const AreaLight *GetAreaLight() const;
    BSDF *GetBSDF(const DifferentialGeometry &dg,
                  const Transform &, MemoryArena &) const;
    BSSRDF *GetBSSRDF(const DifferentialGeometry &dg,
                  const Transform &, MemoryArena &) const;

	void getNormal(Normal*){
#ifdef DEBUG
		Warning("Normal_ch3");
#endif // DEBUG
		return;
	}
	Reference<Material> getMaterial(){
#ifdef DEBUG
		Warning("Material_ch3");
#endif // DEBUG
		return NULL;
	}

};



#endif // PBRT_CORE_PRIMITIVE_H
