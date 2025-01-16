#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

void BVH::build_recursive(int left_index, int right_index, BVHNode* node) {
	//PUT YOUR CODE HERE


	 //right_index, left_index and split_index refer to the indices in the objects vector
	// do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	 // node.index can have a index of objects vector or a index of nodes vector

	if (right_index - left_index <= Threshold) {
		node->makeLeaf(left_index, right_index - left_index);
	}
	else {
		// find axis with largest range
		float x_range = node->getAABB().max.x - node->getAABB().min.x;
		float y_range = node->getAABB().max.y - node->getAABB().min.y;
		float z_range = node->getAABB().max.z - node->getAABB().min.z;
		float split_axis, largest_range;

		if (x_range > y_range) {
			split_axis = 0;
			largest_range = x_range;
		}
		else {
			split_axis = 1;
			largest_range = y_range;
		}

		if (z_range > largest_range) {
			split_axis = 2;
			largest_range = z_range;
		}

		// sort objects
		Comparator cmp = Comparator();
		cmp.dimension = split_axis;
		std::sort(objects.begin() + left_index, objects.begin() + right_index, cmp);

		float mid_point = node->getAABB().min.getAxisValue(split_axis) + (largest_range / 2);
		int split_index;

		// check if one of the nodes is empty and calculate split index
		if (objects.at(left_index)->getCentroid().getAxisValue(split_axis) > mid_point || objects.at(right_index - 1)->getCentroid().getAxisValue(split_axis) <= mid_point) {
			split_index = (left_index + right_index) / 2;
		}
		else {
			for (int i = left_index; i < right_index; i++) {
				if (objects.at(i)->getCentroid().getAxisValue(split_axis) > mid_point) {
					split_index = i;
					break;
				}
			}
		}

		// calculate bounding boxes of left and right nodes
		AABB left_node_bbox = AABB(Vector(FLT_MAX, FLT_MAX, FLT_MAX), Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX));
		AABB right_node_bbox = AABB(Vector(FLT_MAX, FLT_MAX, FLT_MAX), Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX));

		for (int i = left_index; i < split_index; i++) {
			AABB bbox = objects.at(i)->GetBoundingBox();
			left_node_bbox.extend(bbox);
		}

		for (int i = split_index; i < right_index; i++) {
			AABB bbox = objects.at(i)->GetBoundingBox();
			right_node_bbox.extend(bbox);
		}

		// create new nodes and assign bounding boxes
		BVHNode* left_node = new BVHNode();
		BVHNode* right_node = new BVHNode();
		left_node->setAABB(left_node_bbox);
		right_node->setAABB(right_node_bbox);

		// save index of left node
		node->makeNode(nodes.size());

		// push new nodes into node vector
		nodes.push_back(left_node);
		nodes.push_back(right_node);

		build_recursive(left_index, split_index, left_node);
		build_recursive(split_index, right_index, right_node);
	}
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
	float tmp;
	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;

	BVHNode* currentNode = nodes[0];

	if (!currentNode->getAABB().intercepts(ray, tmp)) {
		return false;
	}

	while (true) {
		if (!currentNode->isLeaf()) {
			BVHNode* left_node = nodes[currentNode->getIndex()];
			BVHNode* right_node = nodes[currentNode->getIndex() + 1];

			float t_left, t_right;
			bool hit_left = left_node->getAABB().intercepts(ray, t_left);
			bool hit_right = right_node->getAABB().intercepts(ray, t_right);

			if (hit_left && hit_right) {
				StackItem* s;
				if (t_left < t_right) {
					s = new StackItem(right_node, t_right);
					currentNode = left_node;
				}
				else {
					s = new StackItem(left_node, t_left);
					currentNode = right_node;
				}
				hit_stack.push(*s);
				continue;
			}
			else if (hit_left) {
				currentNode = left_node;
				continue;
			}
			else if (hit_right) {
				currentNode = right_node;
				continue;
			}
		}
		else {
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				if (objects.at(i)->intercepts(ray, tmp) && tmp < tmin) {
					tmin = tmp;
					*hit_obj = objects.at(i);
					hit = true;
				}
			}
		}

		while (true) {
			if (hit_stack.empty()) {
				hit_point = ray.origin + ray.direction * tmin;
				return hit;
			}
			else {
				StackItem s = hit_stack.top();
				hit_stack.pop();

				if (s.t < tmin) {
					currentNode = s.ptr;
					break;
				}
			}
		}
	}
}
bool BVH::Traverse(Ray& ray) {  //shadow ray with length
	float tmp;

	double length = ray.direction.length(); //distance between light and intersection point
	ray.direction.normalize();

	BVHNode* currentNode = nodes[0];

	if (!currentNode->getAABB().intercepts(ray, tmp)) {
		return false;
	}

	while (true) {
		if (!currentNode->isLeaf()) {
			BVHNode* left_node = nodes[currentNode->getIndex()];
			BVHNode* right_node = nodes[currentNode->getIndex() + 1];

			float t_left, t_right;
			bool hit_left = left_node->getAABB().intercepts(ray, t_left);
			bool hit_right = right_node->getAABB().intercepts(ray, t_right);

			if (hit_left && hit_right) {
				StackItem* s = new StackItem(right_node, t_right);
				hit_stack.push(*s);
				currentNode = left_node;
				continue;
			}
			else if (hit_left) {
				currentNode = left_node;
				continue;
			}
			else if (hit_right) {
				currentNode = right_node;
				continue;
			}
		}
		else {
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				if (objects.at(i)->intercepts(ray, tmp) && tmp < length) {
					return true;
				}
			}
		}

		if (hit_stack.empty()) {
			return false;
		}
		else {
			currentNode = hit_stack.top().ptr;
			hit_stack.pop();
		}
	}
}
