
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


// renderers/samplerrenderer.cpp*
#include "stdafx.h"
#include "renderers/samplerrenderer.h"
#include "scene.h"
#include "film.h"
#include "volume.h"
#include "sampler.h"
#include "integrator.h"
#include "progressreporter.h"
#include "camera.h"
#include "intersection.h"
#include "samplers/lowdiscrepancy.h"
#include "accelerators/grid.h"
//ly:控制可视化面片聚类

static float **VTable;
static float TotalV=0;

static uint32_t **cidTable;

#define SamplerRenderShowGrids false
#define SamplerRenderShow_Cluster false//nVoxels[axis] = Clamp(nVoxels[axis], 1, 64);
#define SamplerRenderShow_SamplingRate true
static	float xxs = 0;


#include <iostream>
#include <fstream>

//using namespace std;

static uint32_t hash(char *key, uint32_t len)
{
    uint32_t hash = 0, i;
    for (hash=0, i=0; i<len; ++i) {
        hash += key[i];
        hash += (hash << 10);
        hash ^= (hash >> 6);
    }
    hash += (hash << 3);
    hash ^= (hash >> 11);
    hash += (hash << 15);
    return hash;
} 

// SamplerRendererTask Definitions
void SamplerRendererTask::Run() {
    PBRT_STARTED_RENDERTASK(taskNum);
    // Get sub-_Sampler_ for _SamplerRendererTask_
    Sampler *sampler = mainSampler->GetSubSampler(taskNum, taskCount);
    if (!sampler)
    {
        reporter.Update();
        PBRT_FINISHED_RENDERTASK(taskNum);
        return;
    }

    // Declare local variables used for rendering loop
    MemoryArena arena;
    RNG rng(taskNum);

    // Allocate space for samples and intersections
    int maxSamples = sampler->MaximumSampleCount();//每像素最多的采样数，为下面临时变量数组空间大小作最宽容的考虑
    Sample *samples = origSample->Duplicate(maxSamples);
    RayDifferential *rays = new RayDifferential[maxSamples];
    Spectrum *Ls = new Spectrum[maxSamples];
    Spectrum *Ts = new Spectrum[maxSamples];
    Intersection *isects = new Intersection[maxSamples];
	char logname[] = "cidTableLOG";
	std::ofstream LOG(logname, std::ios::app);

	//	if (MyAdaptiveSampler&&SamplerRenderShow_SamplingRate&&!SamplerRenderShow_Cluster&&!SamplerRenderShowGrids&&!visualizeObjectIds)
//	{
//		Warning("AdaptiveSampling Correct!");
//	}//
//	else Warning("AdaptiveSampling Wrong!");


//	if (!IsInCasting){
//		Warning("mainStart :%d,%d. mainEnd: %d,%d", mainSampler->xPixelStart, mainSampler->yPixelStart, mainSampler->xPixelEnd, mainSampler->yPixelEnd);
//		Warning("Start :%d,%d. End:%d,%d", sampler->xPixelStart, sampler->yPixelStart,sampler->xPixelEnd, sampler->yPixelEnd);
//	}


    // Get samples from _Sampler_ and update image
    int sampleCount;
//	if (sampler->getyPos()<30)	
//		Warning("is VTable alive? %f:%d,%d", VTable[sampler->getxPos()][sampler->getyPos()], sampler->getxPos(), sampler->getyPos());
    while ((sampleCount = sampler->GetMoreSamples(samples, rng,VTable,TotalV)) > 0) {//亟待解决的问题：如何cast_ray获得采样率的计算因子//一般在getMoreSamples里移动像素，一次一个像素
		//设置一个全局表格，在这个while循环之前作一遍拟渲染
        // Generate camera rays and compute radiance along raysx
//		if (!IsInCasting)
//			Warning("SamplingRate%d",sampleCount);
		if (!IsInCasting)
			xxs += sampleCount;

		for (int i = 0; i < sampleCount; ++i) {
			// Find camera ray for _sample[i]_
			PBRT_STARTED_GENERATING_CAMERA_RAY(&samples[i]);
			float rayWeight = camera->GenerateRayDifferential(samples[i], &rays[i]);
			rays[i].ScaleDifferentials(1.f / sqrtf(sampler->samplesPerPixel));
			PBRT_FINISHED_GENERATING_CAMERA_RAY(&samples[i], &rays[i], rayWeight);

			// Evaluate radiance along camera ray
			PBRT_STARTED_CAMERA_RAY_INTEGRATION(&rays[i], &samples[i]);

			if (IsInCasting){
				if (rayWeight > 0.f && scene->Intersect(rays[i], &isects[i])) {
					// random shading based on shape id...
					uint32_t cid = isects[i].ClusterId;
					float v = scene->clusterStruct->ClusterList[cid].ComputeSamplingValue(rays[i]);//tanU*tanV和像素块绝对面积同场景恒定//debug: 打印包围盒pMinpMax看正确与否
//					if (!sampler->ifxyPos())
//						Warning("get wrong xyPos");
					//samples[i].imageX;
					//samples[i].imageY;
					//Warning("%d,%d", xpos, ypos);
//					sampler->updateVTable(v);
					int xpos =samples[i].imageX;//int xpos = sampler->getxPos();
					int ypos =samples[i].imageY; //int ypos = sampler->getyPos();
					//LOG << "\"" << xpos << "\t" << ypos << "\"" << "\t" << VTable[xpos][ypos] << "\t" << TotalV << std::endl;//Warning("%f", TotalV);

					if (VTable[xpos][ypos] == -1.0f){
						VTable[xpos][ypos] = v;//注意：表项任然肯能为空，如果所有cast_sample都没体素/聚类相交
						TotalV += v;
						cidTable[xpos][ypos] = cid;
					}
					else
					{
						if (v > VTable[xpos][ypos]){
							TotalV -= VTable[xpos][ypos];//挑选同一像素内的高采样率
							VTable[xpos][ypos] = v;
							TotalV += v;//完成任务？计算采样率交给AdaptiveSampler
							cidTable[xpos][ypos] = cid;
						}
					}
					LOG << xpos << "\t" << ypos << "\t" << cidTable[xpos][ypos] << std::endl;
					/*
					if(xpos<60)
					{
						Warning("ready:%d,%d,%f", xpos, ypos, VTable[xpos][ypos]);
					}
					*/
					//					uint32_t ids[2] = { isects[i].shapeId, isects[i].primitiveId };
					//					uint32_t h = hash((char *)ids, sizeof(ids));
					//					float rgb[3] = { float(h & 0xff), float((h >> 8) & 0xff),
					//						float((h >> 16) & 0xff) };
					//					Ls[i] = Spectrum::FromRGB(rgb);
					//					Ls[i] /= 255.f;
				}
				//				else
				//					Ls[i] = 0.f;
			}
			else {
				if (visualizeObjectIds) {
					if (rayWeight > 0.f && scene->Intersect(rays[i], &isects[i])) {
						// random shading based on shape id...
						uint32_t ids[2] = { isects[i].shapeId, isects[i].primitiveId };
						uint32_t h = hash((char *)ids, sizeof(ids));
						float rgb[3] = { float(h & 0xff), float((h >> 8) & 0xff),
							float((h >> 16) & 0xff) };
						Ls[i] = Spectrum::FromRGB(rgb);
						Ls[i] /= 255.f;
					}
					else
						Ls[i] = 0.f;
				}
				else if (SamplerRenderShow_Cluster) {
					if (rayWeight > 0.f && scene->Intersect(rays[i], &isects[i])) {
						// random shading based on cluster id...
						uint32_t ids[2] = { isects[i].ClusterId, isects[i].ClusterId };
						//Warning("is %d", ids);
						uint32_t h = hash((char *)ids, sizeof(ids));
						float rgb[3] = { float(h & 0xff), float((h >> 8) & 0xff),
							float((h >> 16) & 0xff) };
						Ls[i] = Spectrum::FromRGB(rgb);
						Ls[i] /= 255.f;
					}
					else
						Ls[i] = 0.f;
				}
				else if (SamplerRenderShowGrids){
					if (rayWeight > 0.f && scene->Intersect(rays[i], &isects[i])) {
						// random shading based on Voxel id...
						uint32_t ids[2] = { isects[i].VoxelId, isects[i].VoxelId };
						//Warning("is %d",ids);
						uint32_t h = hash((char *)ids, sizeof(ids));//(char *)char占两个uint32_t类型，即64位
						float rgb[3] = { float(h & 0xff), float((h >> 8) & 0xff),
							float((h >> 16) & 0xff) };
						Ls[i] = Spectrum::FromRGB(rgb);
						Ls[i] /= 255.f;
					}
					else
						Ls[i] = 0.f;
				}
				else{
					if (rayWeight > 0.f)
						Ls[i] = rayWeight * renderer->Li(scene, rays[i], &samples[i], rng,
						arena, &isects[i], &Ts[i]);
					else {
						Ls[i] = 0.f;
						Ts[i] = 1.f;
					}

					// Issue warning if unexpected radiance value returned
					if (Ls[i].HasNaNs()) {
						Error("Not-a-number radiance value returned "
							"for image sample.  Setting to black.");
						Ls[i] = Spectrum(0.f);
					}
					else if (Ls[i].y() < -1e-5) {
						Error("Negative luminance value, %f, returned "
							"for image sample.  Setting to black.", Ls[i].y());
						Ls[i] = Spectrum(0.f);
					}
					else if (isinf(Ls[i].y())) {
						Error("Infinite luminance value returned "
							"for image sample.  Setting to black.");
						Ls[i] = Spectrum(0.f);
					}
				}
				PBRT_FINISHED_CAMERA_RAY_INTEGRATION(&rays[i], &samples[i], &Ls[i]);
			}
		}

        // Report sample results to _Sampler_, add contributions to image
        if (sampler->ReportResults(samples, rays, Ls, isects, sampleCount))
        {
			if (!IsInCasting){
	            for (int i = 0; i < sampleCount; ++i)
		        {
			        PBRT_STARTED_ADDING_IMAGE_SAMPLE(&samples[i], &rays[i], &Ls[i], &Ts[i]);
					camera->film->AddSample(samples[i], Ls[i], cidTable, UsingMyAdaptiveSampler);
					PBRT_FINISHED_ADDING_IMAGE_SAMPLE();
				}
			}
			else{

			}

        }

        // Free _MemoryArena_ memory from computing image sample values
        arena.FreeAll();
    }

    // Clean up after _SamplerRendererTask_ is done with its image region
    camera->film->UpdateDisplay(sampler->xPixelStart,
        sampler->yPixelStart, sampler->xPixelEnd+1, sampler->yPixelEnd+1);
    delete sampler;
    delete[] samples;
    delete[] rays;
    delete[] Ls;
    delete[] Ts;
    delete[] isects;
    reporter.Update();
    PBRT_FINISHED_RENDERTASK(taskNum);
}



// SamplerRenderer Method Definitions
SamplerRenderer::SamplerRenderer(Sampler *s, Camera *c,
                                 SurfaceIntegrator *si, VolumeIntegrator *vi,
								 bool visIds, bool myAdaSampler) {
    sampler = s;
    camera = c;
    surfaceIntegrator = si;
    volumeIntegrator = vi;
    visualizeObjectIds = visIds;
	UseMyAdaptiveSampler = myAdaSampler;
}


SamplerRenderer::~SamplerRenderer() {
    delete sampler;
    delete camera;
    delete surfaceIntegrator;
    delete volumeIntegrator;
}


void SamplerRenderer::Render(const Scene *scene) {
    PBRT_FINISHED_PARSING();
    // Allow integrators to do preprocessing for the scene
    PBRT_STARTED_PREPROCESSING();
    surfaceIntegrator->Preprocess(scene, camera, this);
    volumeIntegrator->Preprocess(scene, camera, this);
    PBRT_FINISHED_PREPROCESSING();
//ly:begin cast_ray过程
	if (UseMyAdaptiveSampler) {//使用myadaptive要设置bool MyAdaptiveSampler！！
		Warning("ParamSetCorrectly!!");
		int x0, x1, y0, y1;
		camera->film->GetPixelExtent(&x0, &x1, &y0, &y1);
		float t0 = camera->shutterOpen, t1 = camera->shutterClose;
//		Distribution1D *lightDistribution = ComputeLightSamplingCDF(scene);
		int xResolution = camera->film->xResolution;//全局变量
		int yResolution = camera->film->yResolution;//全局变量
		//Warning("xResolution: %d. y:%d",xResolution,yResolution);
		VTable = new float*[xResolution+1];//全局表
		cidTable = new uint32_t*[xResolution + 1];//全局表
		for (int i = 0; i < xResolution + 1; i++){
			VTable[i] = new float[yResolution+1];
			for (int j = 0; j < yResolution + 1; j++)
				VTable[i][j] = -1.0f;

			cidTable[i] = new uint32_t[yResolution + 1];
			for (int j = 0; j < yResolution + 1; j++)
				cidTable[i][j] = -1;
		}
		//sampler->initVTable(VT);

		LDSampler lsampler(x0, x1, y0, y1, Cast_nPixelSamples, t0, t1);//cast过程固定了采样速率。如何对场景自适应地调整这个采样速率？
		Sample *sample = new Sample(&lsampler, NULL, NULL, scene);
		vector<Task *> castTasks;
		int nCastTasks = max(32 * NumSystemCores(),
			(xResolution * yResolution) / (16 * 16));
		nCastTasks = RoundUpPow2(nCastTasks);
		ProgressReporter CastProgress(nCastTasks, "Computing Sampling Rate per pixel");
		for (int i = 0; i < nCastTasks; ++i)
			castTasks.push_back(new SamplerRendererTask(scene, this, camera, CastProgress,
			&lsampler, sample, false, i, nCastTasks, true, UseMyAdaptiveSampler));//ly:add bool cast_ray ,set "true" here, set "false" otherwise
		std::reverse(castTasks.begin(), castTasks.end());
		EnqueueTasks(castTasks);
		WaitForAllTasks();
		for (uint32_t i = 0; i < castTasks.size(); ++i)
			delete castTasks[i];
		delete sample;
		CastProgress.Done();
	}
//ly:end

    PBRT_STARTED_RENDERING();
    // Allocate and initialize _sample_
    Sample *sample = new Sample(sampler, surfaceIntegrator,
                                volumeIntegrator, scene);
	Warning("totalv %f",TotalV);
    // Create and launch _SamplerRendererTask_s for rendering image

    // Compute number of _SamplerRendererTask_s to create for rendering
	    int nPixels = camera->film->xResolution * camera->film->yResolution;
    int nTasks = max(32 * NumSystemCores(), nPixels / (16*16));
    nTasks = RoundUpPow2(nTasks);
    ProgressReporter reporter(nTasks, "Rendering");
    vector<Task *> renderTasks;
    for (int i = 0; i < nTasks; ++i)
        renderTasks.push_back(new SamplerRendererTask(scene, this, camera,
                                                      reporter, sampler, sample, 
                                                      visualizeObjectIds, 
													  nTasks - 1 - i, nTasks, false, UseMyAdaptiveSampler));
    EnqueueTasks(renderTasks);
    WaitForAllTasks();
    for (uint32_t i = 0; i < renderTasks.size(); ++i)
        delete renderTasks[i];
    reporter.Done();
    PBRT_FINISHED_RENDERING();
    // Clean up after rendering and store final image
    delete sample;

//	sampler->deleteVTable(xResolution,yResolution);
	if (UseMyAdaptiveSampler){
		for (int i = 0; i < camera->film->xResolution + 1; i++){
		delete VTable[i];		
		delete cidTable[i];
	}
	delete VTable;
	delete cidTable;
	
	}
	std::ofstream LOGs("spp", std::ios::app);
	LOGs << "总采样数" << "\t" << xxs << "采样率" << "\t" << xxs / (camera->film->xResolution*camera->film->yResolution)<<std::endl;
	if (!SamplerRenderShow_SamplingRate)
	    camera->film->WriteImage();
	else {  camera->film->WriteImage_Accumulate(); camera->film->WriteImage();}//
}


Spectrum SamplerRenderer::Li(const Scene *scene,
        const RayDifferential &ray, const Sample *sample, RNG &rng,
        MemoryArena &arena, Intersection *isect, Spectrum *T) const {
    Assert(ray.time == sample->time);
    Assert(!ray.HasNaNs());
    // Allocate local variables for _isect_ and _T_ if n	eeded
    Spectrum localT;
    if (!T) T = &localT;
    Intersection localIsect;
    if (!isect) isect = &localIsect;
    Spectrum Li = 0.f;
    if (scene->Intersect(ray, isect))
        Li = surfaceIntegrator->Li(scene, this, ray, *isect, sample,
                                   rng, arena);
    else {
        // Handle ray that doesn't intersect any geometry
        for (uint32_t i = 0; i < scene->lights.size(); ++i)
           Li += scene->lights[i]->Le(ray);
    }
    Spectrum Lvi = volumeIntegrator->Li(scene, this, ray, sample, rng,
                                        T, arena);
    return *T * Li + Lvi;
}


Spectrum SamplerRenderer::Transmittance(const Scene *scene,
        const RayDifferential &ray, const Sample *sample, RNG &rng,
        MemoryArena &arena) const {
    return volumeIntegrator->Transmittance(scene, this, ray, sample,
                                           rng, arena);
}


