// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
//======Author: Vimal kumar V, LERNER'S MODIFICATION STARTS
//(Code for Node and KdTree taken from src/quiz/cluster/kdtree.h)
//An additional struct clust implemented with functions ===========
#include <unordered_set>
#include<math.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	void insertHelper(Node** node,uint depth,std::vector<float>point,int id)
	{
		//Tree is empty
		if(*node == NULL)
			*node = new Node(point,id);
		else
		{
			//Calculate current dimension
			uint cd = depth % 2;

			if(point[cd]<((*node)->point[cd]))
				insertHelper(&((*node)->left),depth+1,point,id);
			else
				insertHelper(&((*node)->right),depth+1,point,id);
		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id);
	}

	void searchHelper(std::vector<float>target,Node* node,int depth,float distanceTol,std::vector<int> &ids)
	{
		if(node!=NULL)
		{
			if((node->point[0]>=(target[0]-distanceTol)&&node->point[0]<=(target[0]+distanceTol))&&(node->point[1]>=(target[1]-distanceTol)&&node->point[1]<=(target[1]+distanceTol)))
			{
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
				if(distance<=distanceTol)
					ids.push_back(node->id);
			}

			//Check across boundary
			if((target[depth%2]-distanceTol)<node->point[depth%2])
				searchHelper(target,node->left,depth+1,distanceTol,ids);
			if((target[depth%2]+distanceTol)>node->point[depth%2])
				searchHelper(target,node->right,depth+1,distanceTol,ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}
	

};
//Learner implemented struct with functions from
//src/quiz/cluster/cluster.cpp===========
struct Clust
{
  
void clusterHelper(int indice,const std::vector<std::vector<float>>points,std::vector<int>& cluster,std::vector<bool> &processed,KdTree* tree,float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);
	std::vector<int> nearest = tree->search(points[indice],distanceTol);

	for(int id:nearest)
	{
		if(!processed[id])
		clusterHelper(id,points,cluster,processed,tree,distanceTol);
	}
}
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(),false);

	int i =0;
	while(i<points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i,points,cluster,processed,tree,distanceTol);
		clusters.push_back(cluster);
		i++;
	}
 
	return clusters;
}

//Modified clusterHelper(clusterHelper2) and polymorphic euclideanCluster to enable clustering as per min and maxSize
  
void clusterHelper2(int indice,const std::vector<std::vector<float>>points,std::vector<int>& cluster,std::vector<bool> &processed,KdTree* tree,float distanceTol,int maxSize)
{
	processed[indice] = true;
	cluster.push_back(indice);
    
	std::vector<int> nearest = tree->search(points[indice],distanceTol);

	for(int id:nearest)
	{
        if(cluster.size()>maxSize)
        {
          for(int j=0;j<cluster.size();j++)
          {
            processed[cluster[j]]=false;
          }
          cluster.clear();
          clusterHelper2(indice,points,cluster,processed,tree,distanceTol*0.8,maxSize);
          cluster.clear();
          return;
        }
		if(!processed[id])
		clusterHelper2(id,points,cluster,processed,tree,distanceTol,maxSize);
	}
}
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,int minSize,int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(),false);

	int i =0;
	while(i<points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper2(i,points,cluster,processed,tree,distanceTol,maxSize);
      if(cluster.size()<minSize)
      {
        i++;
        continue;
      }
		clusters.push_back(cluster);
		i++;
	}
 
	return clusters;
}


};
//========LERNER'S MODIFICATION ENDS=========
template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

};

#endif /* PROCESSPOINTCLOUDS_H_ */