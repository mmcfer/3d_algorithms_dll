#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <iostream>


#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/thread/thread.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include "stdafx.h"

extern "C" {

	__declspec(dllexport) int calculate_icp(float x1[], float y1[], float z1[], int width1, int height1, float x2[], float y2[], float z2[], int width2, int height2, float maxDistance, int maxIter, float transEpsilon, float euclideanDistEpsi, float *R[], float *T[]);

	__declspec(dllexport) int normal_calculation(float x[], float y[], float z[], int width, int height, int ne_Method, int rect_width, int rect_height, float normal_smoothing_size, float max_depth_change_factor, int k_search, int *nWidth, int *nHeight, float *Nx[], float *Ny[], float *Nz[], float *Cur[]);

	__declspec(dllexport) int read_ply(char * path, float *X[], float *Y[], float *Z[]);

	__declspec(dllexport) int remove_outliers(float x[], float y[], float z[], int width, int height, int vecinos, float desviacion, int *oHeight, int *oWidth, float *oX[], float *oY[], float *oZ[]);

	__declspec(dllexport) int viewer_3D(float x[], float y[], float z[], int width, int height, int r[], int g[], int b[]);

	
}


