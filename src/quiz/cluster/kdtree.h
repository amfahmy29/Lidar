/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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


	void insertHelper(Node ** node,uint depth,std::vector<float> point,int id)
	{
		uint cd; 
		if(*node == NULL)
		{
			*node = new Node(point,id);

		}
		else
		{
			cd = depth % 3;
			

			if(point[cd] < ((*node)->point[cd]))
			{
				insertHelper((&((*node)->left)),depth+1,point, id);
				
			}
			else
			{
				/* code */
				insertHelper((&((*node)->right)),depth+1,point, id);
			}
			

			
		}
		
	}

	bool isInsideBox(std::vector<float> point, std::vector<float> target, float distanceTol)
	{
		for(int dimension = 0;dimension < point.size();++dimension)
		{
			float minN = target[dimension] - distanceTol;
			float maxN = target[dimension] + distanceTol;
			if(point[dimension] > maxN || point[dimension] < minN)
			{
				return false;
			}
		}
		return true;
	}



	float computeDist(std::vector<float> point1, std::vector<float> point2)
	{
		float retDist = 0;
		for(int dimension = 0;dimension < point1.size();++dimension)
		{
			float subN = point1[dimension] - point2[dimension];
			retDist += subN * subN;
		}

		retDist = sqrt(retDist);
		return retDist;
	}

	void searchHelper(Node * node, std::vector<float> target,int depth, float distanceTol,std::vector<int>& ids)
	{
		int cd = depth%3;
		if(node != NULL)
		{

			if((node->point[0]>=(target[0]-distanceTol)&&(node->point[0]<=(target[0]+distanceTol))) && (node->point[1]>=(target[1]-distanceTol)&&node->point[1]<=(target[1]+distanceTol))&&(node->point[2]>=(target[2]-distanceTol)&&(node->point[2]<=(target[2]+distanceTol))))
			//if(isInsideBox(node->point, target, distanceTol))
			{
					
				float distance =  sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1])+(node->point[2]-target[2])*(node->point[2]-target[2]));
				
				// 
				if(distance <= distanceTol)
				{

					ids.push_back(node->id);
				}

				
			
			}


			if((target[cd]-distanceTol) <= (node->point[cd]))
			{
				searchHelper(node->left,target,depth+1, distanceTol,ids);	
			}

			if((target[cd]+distanceTol) >= (node->point[cd]))
			{
				searchHelper(node->right,target,depth+1, distanceTol,ids);	
			}
		}

	}






	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		 searchHelper(root,target, 0,distanceTol,ids);

		return ids;
	}
	

};




