#include "BVH.hpp"
#include <algorithm>
#include <cassert>

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p)) {
	time_t start, stop;
	time(&start);
	if(primitives.empty())
		return;

	root = recursiveBuild(primitives);

	time(&stop);
	double diff = difftime(stop, start);
	int hrs     = (int) diff / 3600;
	int mins    = ((int) diff / 60) - (hrs * 60);
	int secs    = (int) diff - (hrs * 3600) - (mins * 60);

	printf(
	        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
	        hrs, mins, secs);
}

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects) {
	BVHBuildNode *node = new BVHBuildNode();
	// Compute bounds of all primitives in BVH node
	Bounds3 bounds;
	for(auto &object: objects)
		bounds = Union(bounds, object->getBounds());
	switch(splitMethod) {
		case SplitMethod::NAIVE: {
			// Compute bounds of all primitives in BVH node
			if(objects.size() == 1) {
				// Create leaf _BVHBuildNode_
				node->bounds = objects[0]->getBounds();
				node->object = objects[0];
				node->left   = nullptr;
				node->right  = nullptr;
				return node;
			} else if(objects.size() == 2) {
				node->left  = recursiveBuild(std::vector{objects[0]});
				node->right = recursiveBuild(std::vector{objects[1]});

				node->bounds = Union(node->left->bounds, node->right->bounds);
				return node;
			} else {
				Bounds3 centroidBounds;
				for(int i = 0; i < objects.size(); ++i)
					centroidBounds =
					        Union(centroidBounds, objects[i]->getBounds().Centroid());
				int dim = centroidBounds.maxExtent();
				switch(dim) {
					case 0:
						std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
							return f1->getBounds().Centroid().x <
							       f2->getBounds().Centroid().x;
						});
						break;
					case 1:
						std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
							return f1->getBounds().Centroid().y <
							       f2->getBounds().Centroid().y;
						});
						break;
					case 2:
						std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
							return f1->getBounds().Centroid().z <
							       f2->getBounds().Centroid().z;
						});
						break;
				}

				auto beginning = objects.begin();
				auto middling  = objects.begin() + (objects.size() / 2);
				auto ending    = objects.end();

				auto leftshapes  = std::vector<Object *>(beginning, middling);
				auto rightshapes = std::vector<Object *>(middling, ending);

				assert(objects.size() == (leftshapes.size() + rightshapes.size()));

				node->left  = recursiveBuild(leftshapes);
				node->right = recursiveBuild(rightshapes);

				node->bounds = Union(node->left->bounds, node->right->bounds);
			}
			break;
		};
		case SplitMethod::SAH: {
			if(objects.size() <= maxPrimsInNode) {
				// Create leaf node: If the number of primitives is less than or equal to maxPrimsInNode, create a leaf node
				if(objects.size() == 1) {
					node->bounds = objects[0]->getBounds();
					node->object = objects[0];
					node->left   = nullptr;
					node->right  = nullptr;
				} else {
					node->bounds = bounds;
					node->left   = recursiveBuild(
                            std::vector<Object *>(objects.begin(), objects.begin() + objects.size() / 2));
					node->right = recursiveBuild(
					        std::vector<Object *>(objects.begin() + objects.size() / 2, objects.end()));
				}
				return node;
			}

			// Compute bounds of primitive centroids
			Bounds3 centroidBounds;
			for(int i = 0; i < objects.size(); ++i)
				centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());

			int dim      = centroidBounds.maxExtent();
			int nBuckets = 12;// You can adjust the number of buckets
			struct BucketInfo {
				int count = 0;
				Bounds3 bounds;
			};
			BucketInfo buckets[nBuckets];///< Array of buckets (BucketInfo buckets[nBuckets]) to store counts and bounds of primitives.

			// Initialize variables for SAH
			float minCost          = std::numeric_limits<float>::infinity();///< the minimum cost
			int minCostSplitBucket = -1;                                    ///< the best bucket to split (minCostSplitBucket)
			int minCostAxis        = -1;                                    ///< the best axis to split

			for(int axis = 0; axis < 3; ++axis) {
				// Reset buckets
				for(int i = 0; i < nBuckets; ++i) {
					buckets[i].count  = 0;
					buckets[i].bounds = Bounds3();
				}

				// Compute min and max centroid bounds
				Vector3f a        = centroidBounds.pMin;
				float minCentroid = centroidBounds.pMin[axis];
				float maxCentroid = centroidBounds.pMax[axis];
				if(minCentroid == maxCentroid)
					continue;// Can't split along this axis

				// Place primitives into buckets
				for(int i = 0; i < objects.size(); ++i) {
					float centroid = objects[i]->getBounds().Centroid()[axis];
					// Calculate the index of the bucket for each primitive based on its centroid along the current axis.
					int bucketIndex = nBuckets * ((centroid - minCentroid) / (maxCentroid - minCentroid));
					if(bucketIndex == nBuckets)
						bucketIndex = nBuckets - 1;
					// cUpdate the bucket's count and bounds.
					buckets[bucketIndex].count++;
					buckets[bucketIndex].bounds = Union(buckets[bucketIndex].bounds, objects[i]->getBounds());
				}

				// Compute costs for splitting after each bucket
				float cost[nBuckets - 1];
				for(int i = 0; i < nBuckets - 1; ++i) {
					Bounds3 b0, b1;
					int count0 = 0, count1 = 0;
					for(int j = 0; j <= i; ++j) {
						b0 = Union(b0, buckets[j].bounds);
						count0 += buckets[j].count;
					}
					for(int j = i + 1; j < nBuckets; ++j) {
						b1 = Union(b1, buckets[j].bounds);
						count1 += buckets[j].count;
					}
					// For each possible split between buckets, compute the cost using the SAH formula
					cost[i] = 0.125f + (count0 * b0.SurfaceArea() + count1 * b1.SurfaceArea()) / bounds.SurfaceArea();
				}

				// Find minimum cost split
				for(int i = 0; i < nBuckets - 1; ++i) {
					if(cost[i] < minCost) {
						// Update minCost, minCostAxis, and minCostSplitBucket if a lower cost is found
						minCost            = cost[i];
						minCostSplitBucket = i;
						minCostAxis        = axis;
					}
				}
			}

			// Decide whether to split or create a leaf node
			if(minCost < objects.size()) {
				// Partition primitives into left and right based on the best split
				std::vector<Object *> leftShapes;
				std::vector<Object *> rightShapes;
				float minCentroid = centroidBounds.pMin[minCostAxis];
				float maxCentroid = centroidBounds.pMax[minCostAxis];

				for(int i = 0; i < objects.size(); ++i) {
					float centroid  = objects[i]->getBounds().Centroid()[minCostAxis];
					int bucketIndex = nBuckets * ((centroid - minCentroid) / (maxCentroid - minCentroid));
					if(bucketIndex == nBuckets)
						bucketIndex = nBuckets - 1;

					if(bucketIndex <= minCostSplitBucket) {
						// If the minimum cost found is less than the cost of creating a leaf node,
						// partition the primitives into left and right subsets based on the best split.
						leftShapes.push_back(objects[i]);
					} else
						rightShapes.push_back(objects[i]);
				}

				if(leftShapes.empty() || rightShapes.empty()) {
					// Split primitives evenly if SAH partition failed
					auto mid    = objects.size() / 2;
					leftShapes  = std::vector<Object *>(objects.begin(), objects.begin() + mid);
					rightShapes = std::vector<Object *>(objects.begin() + mid, objects.end());
				}

				assert(objects.size() == (leftShapes.size() + rightShapes.size()));
				// Recursively build the left and right child nodes.
				node->left   = recursiveBuild(leftShapes);
				node->right  = recursiveBuild(rightShapes);
				node->bounds = Union(node->left->bounds, node->right->bounds);
			} else {
				// If the SAH partitioning fails (e.g., all primitives end up on one side),
				// fall back to splitting the primitives evenly.
				// Create leaf node
				if(objects.size() == 1) {
					node->bounds = objects[0]->getBounds();
					node->object = objects[0];
					node->left   = nullptr;
					node->right  = nullptr;
				} else {
					node->bounds = bounds;
					node->left   = recursiveBuild(
                            std::vector<Object *>(objects.begin(), objects.begin() + objects.size() / 2));
					node->right = recursiveBuild(
					        std::vector<Object *>(objects.begin() + objects.size() / 2, objects.end()));
				}
			}
			break;
		};
		default: {
			fprintf(stderr, "BVH Split Method unknown\n");
			break;
		}
	}

	return node;
}

Intersection BVHAccel::Intersect(const Ray &ray) const {
	Intersection isect;
	if(!root)
		return isect;
	isect = BVHAccel::getIntersection(root, ray);
	return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const {
	// Traverse the BVH to find intersection
	Intersection isect;
	if(!node || !node->bounds.IntersectP(ray, ray.direction_inv, {ray.direction.x < 0, ray.direction.y < 0, ray.direction.z < 0}))
		return isect;

	if(node->left == nullptr && node->right == nullptr) {
		return node->object->getIntersection(ray);
	}

	Intersection leftIsect  = getIntersection(node->left, ray);
	Intersection rightIsect = getIntersection(node->right, ray);

	if(leftIsect.happened && rightIsect.happened) {
		return leftIsect.distance < rightIsect.distance ? leftIsect : rightIsect;
	} else if(leftIsect.happened) {
		return leftIsect;
	} else if(rightIsect.happened) {
		return rightIsect;
	} else {
		return {};
	}
}