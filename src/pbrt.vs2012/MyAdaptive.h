
#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SAMPLERS_MYADAPTIVE_H
#define PBRT_SAMPLERS_MYADAPTIVE_H

// samplers/MyAdaptive.h*
#include "pbrt.h"
#include "sampler.h"

/****
*****使用myadaptive要设置bool MyAdaptiveSampler
****/

// AdaptiveSampler Declarations
class MYAdaptiveSampler : public Sampler {
public:
	// MyAdaptiveSampler Public Methods
	MYAdaptiveSampler(int xstart, int xend, int ystart, int yend,
		int minSamples, int maxSamples, 
		float sopen, float sclose, int xResolution, int yResolution,int BX);
	Sampler *GetSubSampler(int num, int count);
	~MYAdaptiveSampler();
	int RoundSize(int size) const {
		return RoundUpPow2(size);
	}
	int MaximumSampleCount() { return maxSamples; }
	int GetMoreSamples(Sample *sample, RNG &rng, float** vt = NULL, float tv = 0);
	bool ReportResults(Sample *samples, const RayDifferential *rays,
		const Spectrum *Ls, const Intersection *isects, int count);

	int getxPos(){//其实是得到下一点的pos
		if ((xPos+1) == xPixelEnd) {
			return xPixelStart;
		}
		else return (xPos + 1);
	}

	int getyPos(){ 
		if ((xPos+1) == xPixelEnd) {
			return (yPos + 1);
		}
		return yPos;
	}

	bool ifxyPos(){ return true; }
	
//	static float **VTable;
//	static float TotalV;
	/*
	void initVTable(float **t){
		VTable = t;
		TotalV = 0;
	}

	void updateVTable(float v)
	{
		if (VTable[xPos][yPos] == -1)
			VTable[xPos][yPos] = v;//注意：表项任然肯能为空，如果所有cast_sample都没体素/聚类相交
		else if (v>VTable[xPos][yPos]){
			TotalV -= VTable[xPos][yPos];//挑选同一像素内的高采样率
			VTable[xPos][yPos] = v;
			TotalV += v;//完成任务？计算采样率交给AdaptiveSampler
		}
	}

	void deleteVTable(int xRes, int yRes)
	{
		for (int i = 0; i < xRes; i++)
			delete VTable[i];
		delete VTable;
	}
	*/

	

private:
	// AdaptiveSampler Private Methods
//	bool needsSupersampling(Sample *samples, const RayDifferential *rays,
//		const Spectrum *Ls, const Intersection *isects, int count);

	// AdaptiveSampler Private Data
	int xPos, yPos;
	int minSamples, maxSamples;
	float *sampleBuf;
	int xRes, yRes;
//	enum AdaptiveTest {
//		ADAPTIVE_COMPARE_SHAPE_ID,
//		ADAPTIVE_CONTRAST_THRESHOLD
//	};
//	AdaptiveTest method;
//	bool supersamplePixel;
	int BucketX;
};


MYAdaptiveSampler *CreateMyAdaptiveSampler(const ParamSet &params, const Film *film,
	const Camera *camera);

#endif // PBRT_SAMPLERS_MYADAPTIVE_H