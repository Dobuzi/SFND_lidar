#ifndef KDTREE_H_
#define KDTREE_H_

#include <vector>
#include <cmath>
#include <unordered_set>

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

	void insertHelper(Node **node, std::vector<float> point, int id, int depth)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else if (point[depth%3] < (*node)->point[depth%3])
		{
			insertHelper(&((*node)->left), point, id, depth+1);
		}
		else
		{
			insertHelper(&((*node)->right), point, id, depth+1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, point, id, 0);  
	}

	void searchHelper(std::vector<float> target, float distanceTol, std::vector<int> *ids, Node *node, int depth)
	{
		float max_val_x = target[0] + distanceTol;
		float min_val_x = target[0] - distanceTol;
		float max_val_y = target[1] + distanceTol;
		float min_val_y = target[1] - distanceTol;
        float max_val_z = target[2] + distanceTol;
		float min_val_z = target[2] - distanceTol;
		float pt_x, pt_y, pt_z, dist_x, dist_y, dist_z, distance;
		bool in_range_x, in_range_y, in_range_z;

		if (node != NULL)
		{
			pt_x = node->point[0];
			pt_y = node->point[1];
            pt_z = node->point[2];
			in_range_x = (min_val_x <= pt_x) && (pt_x <= max_val_x);
			in_range_y = (min_val_y <= pt_y) && (pt_y <= max_val_y);
            in_range_z = (min_val_z <= pt_z) && (pt_z <= max_val_z);

			// Check the node is located in the boundary of target
			if (in_range_x && in_range_y && in_range_z)
			{
				dist_x = target[0] - pt_x;
				dist_y = target[1] - pt_y;
                dist_z = target[2] - pt_z;
				distance = sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);

				if (distance < distanceTol)
				{
					ids->push_back (node->id); // Then add id to ids
				}
			}

			if (target[depth%3]-distanceTol < node->point[depth%3])
			{
				searchHelper(target, distanceTol, ids, node->left, depth+1);
			}

			if (target[depth%3]+distanceTol > node->point[depth%3])
			{
				searchHelper(target, distanceTol, ids, node->right, depth+1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, distanceTol, &ids, root, 0);
		return ids;
	}
};

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize);

#endif // KDTREE_H_
