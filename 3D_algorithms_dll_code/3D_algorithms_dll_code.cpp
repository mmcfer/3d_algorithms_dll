// icp_calculation_dll.cpp: define las funciones exportadas de la aplicación DLL.
//

#include "3D_algorithms_dll_code.h"

int user_data;


int calculate_icp(float x1[], float y1[], float z1[], int width1, int height1, float x2[], float y2[], float z2[], int width2, int height2, float maxDistance, int maxIter, float transEpsilon, float euclideanDistEpsi, float *R[], float *T[])
{


	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);


	Eigen::Matrix4f final_transformation;
	//float Out_transformation;

	for (int i = 0; i <= width1*height1; ++i)
	{
		pcl::PointXYZ point;
		point.x = x1[i];
		point.y = y1[i];
		point.z = z1[i];
		cloud_Source->points.push_back(point);
	}

	cloud_Source->width = width1;
	cloud_Source->height = height1;


	for (int i = 0; i <= width2*height2; ++i)
	{
		pcl::PointXYZ point;
		point.x = x2[i];
		point.y = y2[i];
		point.z = z2[i];
		cloud_Target->points.push_back(point);
	}

	cloud_Target->width = width2;
	cloud_Target->height = height2;

	
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	
	// Set the max correspondence distance to X  (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(maxDistance);
	// Set the maximum number of iterations 
	icp.setMaximumIterations(maxIter);
	// Set the transformation epsilon 
     icp.setTransformationEpsilon(transEpsilon);
	// Set the euclidean distance difference epsilon
	icp.setEuclideanFitnessEpsilon(euclideanDistEpsi);
	
	icp.setInputSource(cloud_Source);
	icp.setInputTarget(cloud_Target);
	icp.align(*Final);


	final_transformation = icp.getFinalTransformation();

	CvMat r = cvMat(4, 4, CV_32F, R);
	CvMat t = cvMat(1, 4, CV_32F, T);


	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++){
			r.data.fl[j + i * 3] = final_transformation(i, j);

		}

	for (int j = 0; j < 3; j++)
		t.data.fl[j] = final_transformation(j, 3);


	return 0;


	return (0);
}


int normal_calculation(float x[], float y[], float z[], int width, int height, int ne_Method, int rect_width, int rect_height, float normal_smoothing_size, float max_depth_change_factor, int k_search, int *nWidth, int *nHeight, float *Nx[], float *Ny[], float *Nz[], float *Cur[])
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		for (int i = 0; i <= width*height; ++i)
		{
			pcl::PointXYZ point;
			point.x = x[i];
			point.y = y[i];
			point.z = z[i];
			cloud->points.push_back(point);
		}

		cloud->width = width;
		cloud->height = height;


		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

		switch (ne_Method) {
		case 0:
			ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
			break;
		case 1:
			ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
			break;
		case 2:
			ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
			break;
		case 3:
			ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
			break;
		default: break;
		}

		ne.setRectSize(rect_width, rect_height);
		ne.setNormalSmoothingSize(normal_smoothing_size);
		ne.setMaxDepthChangeFactor(max_depth_change_factor);
		//ne.setKSearch(k_search);
		ne.setInputCloud(cloud);
		ne.compute(*normals);



		CvMat nx = cvMat(normals->height, normals->width, CV_32F, Nx);
		CvMat ny = cvMat(normals->height, normals->width, CV_32F, Ny);
		CvMat nz = cvMat(normals->height, normals->width, CV_32F, Nz);
		CvMat cur = cvMat(normals->height, normals->width, CV_32F, Cur);

		*nWidth = normals->width;
		*nHeight = normals->height;
		

		for (size_t i = 0; i < normals->height; ++i)
		{
			for (size_t j = 0; j < normals->width; ++j)
			{
				nx.data.fl[j + i*normals->width] = normals->points[j + i*normals->width].normal_x;
				ny.data.fl[j + i*normals->width] = normals->points[j + i*normals->width].normal_y;
				nz.data.fl[j + i*normals->width] = normals->points[j + i*normals->width].normal_z;
				cur.data.fl[j + i*normals->width] = normals->points[j + i*normals->width].curvature;
			
			}
		}

		return 0;
	}


int read_ply(char * path, float *X[], float *Y[], float *Z[])
{
			
	ofstream myfile;
	myfile.open("read_ply.txt");
	myfile << path;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	

	//int length = strlen((char*)path);
	char pathfile[73];
	

	strncpy(pathfile, (char*)path, sizeof(pathfile));
	myfile << "\n buena \n";
	myfile << pathfile;
	myfile << "\n Writing this to a file.\n";
	myfile.close();
	

	if (pcl::io::loadPLYFile(std::string((const char *)path), *cloud_out) < 0)
	{
		return (-1);
	}
	
		
	CvMat x = cvMat(cloud_out->height, cloud_out->width, CV_32F, X);
	CvMat y = cvMat(cloud_out->height, cloud_out->width, CV_32F, Y);
	CvMat z = cvMat(cloud_out->height, cloud_out->width, CV_32F, Z);
	
	ofstream myfile2;

	myfile2.open("read_ply.txt");

	for (size_t i = 0; i < cloud_out->height; i++)
	{
		for (size_t j = 0; j < cloud_out->width; j++)
		{
			x.data.fl[j + i*cloud_out->width] = cloud_out->points[j + i*cloud_out->width].x;
			y.data.fl[j + i*cloud_out->width] = cloud_out->points[j + i*cloud_out->width].y;
			z.data.fl[j + i*cloud_out->width] = cloud_out->points[j + i*cloud_out->width].z;

			myfile2 << "\n marc \n";
		}
	}


	myfile2 << "Writing this to a file.\n";
	myfile2.close();
	

	return (0);
}


int remove_outliers(float x[], float y[], float z[], int width, int height, int vecinos, float desviacion, int *oHeight, int *oWidth, float *oX[], float *oY[], float *oZ[])
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i = 0; i <= width*height; ++i)
	{
		pcl::PointXYZ point;
		point.x = x[i];
		point.y = y[i];
		point.z = z[i];
		cloud->points.push_back(point);
	}

	cloud->width = width;
	cloud->height = height;


	// Filtrado
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter(true);
	filter.setInputCloud(cloud);

	//20 vecinos 
	filter.setMeanK(vecinos);
	filter.setStddevMulThresh(desviacion);
	filter.setKeepOrganized(true);
	filter.filter(*filteredCloud);

	pcl::IndicesConstPtr indices_ptr = filter.getRemovedIndices();

	CvMat ox = cvMat(filteredCloud->height, filteredCloud->width, CV_32F, oX);
	CvMat oy = cvMat(filteredCloud->height, filteredCloud->width, CV_32F, oY);
	CvMat oz = cvMat(filteredCloud->height, filteredCloud->width, CV_32F, oZ);

	*oHeight = filteredCloud->height;
	*oWidth = filteredCloud->width;

	for (size_t i = 0; i < filteredCloud->height; ++i)
	{
		for (size_t j = 0; j < filteredCloud->width; ++j)
		{

			ox.data.fl[j + i*filteredCloud->width] = filteredCloud->points[j + i*filteredCloud->width].x;
			oy.data.fl[j + i*filteredCloud->width] = filteredCloud->points[j + i*filteredCloud->width].y;
			oz.data.fl[j + i*filteredCloud->width] = filteredCloud->points[j + i*filteredCloud->width].z;
		}
	}

	return 1;
}



static void
viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0, 0, 0);
	// show coordinate system 
	//viewer.addCoordinateSystem(1.0, "coord");
	// remove coordinate system 
	//viewer.removeCoordinateSystem("coord");
	//viewer.removeCoordinateSystem();
	viewer.initCameraParameters();

	std::cout << "i only run once" << std::endl;

}

static void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}

int viewer_3D(float x[], float y[], float z[], int width, int height, int r[], int g[], int b[])
{

	
	

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	int rgb;
	for (int i = 0; i <= width*height; i++)
	{
		pcl::PointXYZRGBA point;
		rgb = ((int)r[i]) << 16 | ((int)g[i]) << 8 | ((int)b[i]);
		point.x = x[i];
		point.y = y[i];
		point.z = z[i];
		point.rgb = ((int)r[i]) << 16 | ((int)g[i]) << 8 | ((int)b[i]);
		cloud->points.push_back(point);
	}



	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//blocks until the cloud is actually rendered
	viewer.showCloud(cloud);


	//This will only get called once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//This will get called once per visualization iteration
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//you can also do cool processing here
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
		user_data++;
	}
	

	return 1;

}





