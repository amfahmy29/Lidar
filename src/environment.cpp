/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "quiz/cluster/kdtree.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	//std::unordered_set<int> currentInliers;
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while(maxIterations--)
	{
		std::unordered_set<int> currentInliers;
		while(currentInliers.size()< 2)
		currentInliers.insert(rand()%cloud->points.size());

		float x1, x2, y1, y2, A, B,C,distance;
		auto index  = currentInliers.begin();
		x1 = cloud->points[*index].x;
		y1 = cloud->points[*index].y;
		index++;
		x2 = cloud->points[*index].x;
		y2 = cloud->points[*index].y;

		A = y2-y1;
		B = x2-x1;
		C = (x1*y2-x2*y1);

		 
		for(int i =0; i<cloud->points.size();i++)
		{
			if(currentInliers.count(i)>0)
				continue;

			float x3,y3;

			pcl::PointXYZ point = cloud->points[i];

			x3 =point.x;
			y3 = point.y;

			distance = (fabs(A*x3+B*y3+C))/(sqrt(A*A+B*B));

			if(distance <= distanceTol)
				currentInliers.insert(i);

			
		}

		if(currentInliers.size()>inliersResult.size())	
				inliersResult = currentInliers;

	}
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	//std::unordered_set<int> currentInliers;
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while(maxIterations--)
	{
		std::unordered_set<int> currentInliers;
		while(currentInliers.size()< 3)
			currentInliers.insert(rand()%cloud->points.size());

		float x1, x2,x3, y1, y2, y3,z1,z2,z3, A, B,C,D,   distance;
		auto index  = currentInliers.begin();
		x1 = cloud->points[*index].x;
		y1 = cloud->points[*index].y;
		z1 = cloud->points[*index].z;
		index++;
		x2 = cloud->points[*index].x;
		y2 = cloud->points[*index].y;
		z2 = cloud->points[*index].z;

		index++;
		x3 = cloud->points[*index].x;
		y3 = cloud->points[*index].y;
		z3 = cloud->points[*index].z;

		A = ((y2-y1)*(z3 -z1))-((z2-z1)*(y3-y1));
		B = ((z2-z1)*(x3 -x1))-((x2-x1)*(z3-z1));
		C = ((x2-x1)*(y3 -y1))-((y2-y1)*(x3-x1));
		D = -(A*x1+B*y1+C*z1);

		 
		for(int i =0; i<cloud->points.size();i++)
		{
			if(currentInliers.count(i)>0)
				continue;

			float x4,y4,z4;

			pcl::PointXYZ point = cloud->points[i];

			x4 = point.x;
			y4 = point.y;
			z4 = point.z;
     // D = -(A*x4+B*y4+C*z4);
			

			distance = (fabs(A*x4+B*y4+C*z4+D))/(sqrt(A*A+B*B+C*C));

			if(distance <= distanceTol)
				currentInliers.insert(i);

			
		}

		if(currentInliers.size()>inliersResult.size())	
				inliersResult = currentInliers;



	}
	
	return inliersResult;

}

void clusterHelper(int indice, const std::vector<std::vector<float>> points,std::vector<int>& cluster,bool *processedFlag,KdTree *tree,float distanceTol)
{
	processedFlag[indice] = true;

	cluster.push_back(indice);
	std::vector<int> nearest = tree->search(points[indice],distanceTol);

	for(int id : nearest)
	{
		

		if(processedFlag[id] == false)
		{
			
			clusterHelper(id,points,cluster,processedFlag,tree,distanceTol);
		
		}
	}



}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	bool processed [points.size()];


	for(int i=0;i<points.size();i++)
	{
		processed[i] = false;
	}
	
	int index;

	for(index =0;index<points.size();index++)
	{
		if(processed[index] == true)
		{
			continue;
		}
		std::vector<int> cluster;
		
		clusterHelper(index,points,cluster,processed,tree,distanceTol);
		
		clusters.push_back(cluster);
	}

 
	return clusters;

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZ> pointProcessorI,  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3 , Eigen::Vector4f (-10, -6.2, -2, 1), Eigen::Vector4f ( 25, 7, 10, 1));
  //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane (inputCloud, 25, 0.3);
  std::unordered_set<int> inliners = RansacPlane(inputCloud, 100,0.2);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr  obstacleCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < inputCloud->points.size(); index++)
	{
		pcl::PointXYZ point = inputCloud->points[index];
		if(inliners.count(index))
			planeCloud->points.push_back(point);
		else
			obstacleCloud->points.push_back(point);
	}

  renderPointCloud(viewer,planeCloud,"Plane", Color(0,1,0)); 
  //renderPointCloud(viewer,obstacleCloud,"Obstacle",Color(1,0,0));
    
  
  KdTree* tree = new KdTree;

  std::vector<std::vector<float>> points;

  for(int i=0;i<obstacleCloud->points.size();i++)
  {
    std::vector<float> currentPoint;
    currentPoint.push_back(obstacleCloud->points[i].x);
    currentPoint.push_back(obstacleCloud->points[i].y);
    currentPoint.push_back(obstacleCloud->points[i].z);
  
    points.push_back(currentPoint);

  }
      for (int i=0; i<points.size(); i++) 
    	  tree->insert(points[i],i);

    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 0.4);
    std::cout << clusters.size() << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterCloud ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentCluster (new pcl::PointCloud<pcl::PointXYZ>());
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering (obstacleCloud, 0.53,10, 500);
  	int clusterId = 0;
    std::vector<Color> colors ={Color(1,0,0),Color(1,1,0),Color(0,0,1)};


for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  		clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
		  //colors[clusterId%3]
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),Color(1,0,0));
      Box box = pointProcessorI.BoundingBox(clusterCloud);
      renderBox(viewer,box, clusterId);
  		++clusterId;
  	}
  	/*
    for(std::vector<int> cluster : clusters)
  	{
  	
  		for(int indice: cluster)
      {

  		  currentCluster->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
      }
      clusterCloud.push_back(currentCluster);
  		renderPointCloud(viewer, currentCluster,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
      //Box box = pointProcessorI.BoundingBox(currentCluster);
      //renderBox(viewer,box, clusterId);
  		++clusterId;
  	}
    
    clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster1: clusterCloud)
    {
        pointProcessorI.numPoints(cluster1);
        renderPointCloud(viewer,cluster1,"Obstacle"+std::to_string(clusterId),colors[clusterId %colors.size()]);
        Box box = pointProcessorI.BoundingBox(cluster1);
        renderBox(viewer,box, clusterId);
        ++clusterId;
    }
    */

    //renderPointCloud(viewer,inputCloud,"filterCloud");
}
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclInput = lidar->scan();
    //renderRays(viewer,lidar->position,pclInput);
    
     // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> * processPointClouds = new ProcessPointClouds<pcl::PointXYZ>();


    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processPointClouds->SegmentPlane (pclInput, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"Obstacle",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"Plane",Color(0,1,0)); 
    /*cluster*/
    int clusterId =0;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPointClouds->Clustering (segmentCloud.first, 1.0,3, 30);
    std::vector<Color> colors ={Color(1,0,0),Color(1,1,0),Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters)
    {
        processPointClouds->numPoints(cluster);
        renderPointCloud(viewer,cluster,"Obstacle"+std::to_string(clusterId),colors[clusterId %colors.size()]);
        Box box = processPointClouds->BoundingBox(cluster);
        renderBox(viewer,box, clusterId);
        ++clusterId;
    }


}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}





int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
  //cityBlock(viewer);
  ProcessPointClouds<pcl::PointXYZ> pointProcessorI ;
std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
auto streamIterator = stream.begin();
pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudI;

while (!viewer->wasStopped ())
{

  // Clear viewer
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();

  // Load pcd and run obstacle detection process
  inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
  cityBlock(viewer, pointProcessorI, inputCloudI);

  streamIterator++;
  if(streamIterator == stream.end())
    streamIterator = stream.begin();

  viewer->spinOnce ();
}
}