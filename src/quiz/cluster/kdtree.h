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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node **node, std::vector<float> point, int id, bool isY)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else if (point[int(isY)] < (*node)->point[int(isY)] )
		{
			insertHelper(&((*node)->left), point, id, !isY);
		}
		else
		{
			insertHelper(&((*node)->right), point, id, !isY);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, point, id, false);  
	}

	void searchHelper(std::vector<float> target, float distanceTol, std::vector<int> *ids, Node *node, bool isY)
	{
		float max_val_x = target[0] + distanceTol;
		float min_val_x = target[0] - distanceTol;
		float max_val_y = target[1] + distanceTol;
		float min_val_y = target[1] - distanceTol;
		float pt_x, pt_y, dist_x, dist_y, distance;
		bool in_range_x, in_range_y;

		if (node != NULL)
		{
			pt_x = node->point[0];
			pt_y = node->point[1];
			in_range_x = (min_val_x <= pt_x) && (pt_x <= max_val_x);
			in_range_y = (min_val_y <= pt_y) && (pt_y <= max_val_y);
			// Check the node is located in the boundary of target
			if (in_range_x && in_range_y)
			{
				dist_x = target[0] - pt_x;
				dist_y = target[1] - pt_y;
				distance = sqrt(dist_x * dist_x + dist_y * dist_y);

				if (distance < distanceTol)
				{
					ids->push_back (node->id); // Then add id to ids
				}
			}
			
			if (target[int(isY)]-distanceTol < node->point[int(isY)])
			{
				searchHelper(target, distanceTol, ids, node->left, !isY);
			}

			if (target[int(isY)]+distanceTol > node->point[int(isY)])
			{
				searchHelper(target, distanceTol, ids, node->right, !isY);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, &ids, root, false);
		return ids;
	}
};




