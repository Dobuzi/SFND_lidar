#include "kdtree.h"

void proximity(const std::vector<std::vector<float>>& points, int i, std::vector<int> *cluster, KdTree* tree, float distanceTol, std::vector<bool> *processed)
{
	(*processed)[i] = true; // mark point as processed
	cluster->push_back (i); // add point to cluster
	
	std::vector<int> nearby = tree->search(points[i], distanceTol); // nearby points = tree(point)

	for (int j = 0; j < nearby.size(); j++) // Iterate through each nearby point
	{
		if (!(*processed)[nearby[j]]) // If point has not been processed
		{
			proximity(points, nearby[j], cluster, tree, distanceTol, processed); // Proximity(cluster)
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters; // list of clusters
	std::vector<bool> processed (points.size(), false); // processing marker 
	std::vector<int> cluster; // Define cluster

	for (int i = 0; i < points.size (); i++) // Iterate through each point
	{	
		if (!processed[i]) // If point has not been processed
		{
			cluster.clear(); // Clear cluster (== Create cluster)
			proximity(points, i, &cluster, tree, distanceTol, &processed); // Proximity(point, cluster)
            if (minSize < cluster.size() && cluster.size() < maxSize) // Check cluster size
            {
                clusters.push_back (cluster); // cluster add clusters
            }
		}
	}
	
	return clusters;
}
