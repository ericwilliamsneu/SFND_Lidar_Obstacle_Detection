/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


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

	void insertHelper(Node** node, std::vector<float> point, int id, int depth)
	{
		if (*node == NULL) *node = new Node(point, id);
		else
		{
			int dimension = depth % 2;
			if (point[dimension] < (*node)->point[dimension]) insertHelper(&(*node)->left, point, id, depth + 1);
			else insertHelper(&(*node)->right, point, id, depth + 1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, point, id, 0);

	}

	void searchHelper(Node* node, std::vector<float> target, float distanceTol, int depth, std::vector<int>& ids)
	{
		int dimension = depth % 2;
		if (node == NULL)
		{
#ifdef PRINT_DEBUG
			std::cout << "NULL node found, stopping" << std::endl;
#endif
		}
		else
		{
#ifdef PRINT_DEBUG
			std::cout << "Investigating Node with ID: " << node->id << std::endl;
#endif
			//If within box, check distance and add
			int dcheck = 0;
			for (int d = 0; d < target.size(); d++)
			{
				if (node->point[d] <= (target[d] + distanceTol) && node->point[d] >= (target[d] - distanceTol)) dcheck++;
			}
			if (dcheck == target.size())	//All dimensions match
			{
#ifdef PRINT_DEBUG
				std::cout << "Node is within box, checking distance..." << std::endl;
#endif
				float dsum = 0.0;
				for (int d = 0; d < target.size(); d++)
				{
					float ddif = target[d] - node->point[d];
					dsum += ddif * ddif;
				}
				dsum = sqrt(dsum);
				if (dsum <= distanceTol)
				{
#ifdef PRINT_DEBUG
					std::cout << "Node is nearby to target!" << std::endl;
#endif
					ids.push_back(node->id);
				}
			}
			if ((target[dimension] + distanceTol) > node->point[dimension])
			{
#ifdef PRINT_DEBUG
				std::cout << "Target is to the right... searching right" << std::endl;
#endif
				searchHelper(node->right, target, distanceTol, depth + 1, ids);
			}
			if ((target[dimension] - distanceTol) < node->point[dimension])
			{
#ifdef PRINT_DEBUG
				std::cout << "Target is to the left... searching left" << std::endl;
#endif
				searchHelper(node->left, target, distanceTol, depth + 1, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}
};




