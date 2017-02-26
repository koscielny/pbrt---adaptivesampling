An Adaptive Sampling Method Based on Spatial Clustering
==========
----

The method based on Monte Carlo integration is the main technology to generate the high quality, physical realistic image, and the first important key in it is called pixel sampling, means how to get the proper positions in every pixel of the image scene to compute the global illumination effect and then give the generated image. This paper give a new sampling algorithm, spatial clustering based adaptive sampling, to overcome the usual more noise points in image generated by random sampling algorithm.

Illuminations are corresponding to material, surface as well as other geometry situation of the surface. On the geometry-complex area where illuminations change rapidly, large amount of noises arise due to the deficiency of samples. While on the flat area, only a limited number of samples are needed to reconstruct. If geometry and material information is acquired and predict the complex scene areas, we may attribute more samples to these areas and converge the image to become visibly stratifying faster with same sampling rate.

Based on this fact, we have deep study of adaptive sampling algorithms. Aimed at existing algorithms that lacks the exploit of 3D information in the scene, we proposed a new method that firstly clustering meshes with same material, similar normal and similar spatial position. Bounding box are used to quickly detect complexity and helped adaptive sampling of Monte Carlo direct lighting.

This new method of sampling makes some pixels of an image converge faster than the others. This means that they only need fewer samples to 
get precise result than the others. In usual way, all of the pixel take a uniform number of pixels, consequently when the total number of samples is small the image has more random noise，and the number is big lead to computing waste. The new algorithm detect the spatial scale of cluster of the aiming mesh as the complexity criterion and distribute more samples to more complex pixels, so that it can get more reasonable distribution of samples in image when the total computation is fixed．

-----

Basing on PBRT, this is the implementation of my bechlor thesis, which proposed an adaptive sampling algorithm.

See my algorithms in files src/samplers/MyAdaptive.h, MyAdaptive.cpp

My thesis is also provided： [paper/Thesis.pdf](https://github.com/koscielny/pbrt-adaptivesampling/raw/master/paper/Thesis.pdf)
