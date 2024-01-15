// Houses the code for a K-d tree (K-dimensional tree)
// data structure. This data structure is used to make
// searching in Euclidean space (e.g 3D space) more
// efficient. If you're familiar with binary trees then
// you can conceptualise a K-d tree as a binary tree in
// K dimensions instead of 2.
#pragma once

#include "objRender.h"

// a K-d tree node
struct Node {
    std::vector<float> point; // the coordinates of the node's point in K-dimensional space
    int id;
    Node *left; // the left child node
    Node *right; // the right child node

    Node(
        std::vector<float> arr, 
        int setId
    ) : point(arr), 
        id(setId), 
        left(NULL), 
        right(NULL) {}
};

// The K-d tree data structure
struct KdTree {
    Node *rootNode;

    KdTree() : rootNode(NULL) {}

    // Inserts a new point into the K-d tree
    // param: **node - a pointer to a pointer to a node which is to be inserted
    //                 into the tree.
    // param: depth - tracks the depth at which the node is inserted at
    // param: point - the coordinates of the node to be inserted (can be used 
    //                instead of **node)
    // param: id - the id of the node to be inserted (can be used instead of
    //             **node)
    void insertHelper(
        Node **node, 
        unsigned int depth, 
        std::vector<float> point, 
        int id
    ) {
        if(*node == NULL) *node = new Node(point, id);
        else {
            unsigned int cd = depth % 3;

            if(point[cd] < ((*node)->point[cd])) insertHelper(&((*node)->left), depth + 1, point, id);
            else insertHelper(&((*node)->right), depth + 1, point, id);
        }
    }
    
    // Inserts a new point into the K-d tree
    // param: point - a vector of coordinates which describe the point's location
    // param: id - the point's ID
    void insert(
        std::vector<float> point, 
        int id
    ) {
        insertHelper(&rootNode, 0, point, id);
    }

    // Recursive function to search for the nodes which are within a specified  
    // distance (distanceTolerance) from that target location. Only works for
    // 2D locations.
    // param: target - the target location defined by x, y coordinates
    // param: *node - a pointer to the node at which the tree traversal begins
    // param: depth - tracks the depth of the tree traversal
    // param: distanceTolerance - the distance threshold at which points are 
    //                            determined to be within the desired distance
    //                            from the target or not
    // param: &ids - a reference to a vector which is populated with the IDs of
    //               nodes which are within the distanceTolerance from the target
    //               location.
    void searchHelper(
        std::vector<float> target, 
        Node* node, 
        int depth, 
        float distanceTolerance, 
        std::vector<int> &ids
    ) {
        if(node != NULL) {
            if((node->point[0] >= (target[0] - distanceTolerance) && node->point[0] <= (target[0] + distanceTolerance)) && (node->point[1] >= (target[1] - distanceTolerance) && node->point[1] <= (target[1] + distanceTolerance))) {
                float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]));
                if(distance <= distanceTolerance) ids.push_back(node->id);
            }
            if((target[depth % 3] - distanceTolerance) < node->point[depth % 3]) {
                searchHelper(target, node->left, depth + 1, distanceTolerance, ids);
            }
            if((target[depth % 3] + distanceTolerance) > node->point[depth % 3]) {
                searchHelper(target, node->right, depth + 1, distanceTolerance, ids);
            }
        }
    }

    // Recursive function to search for the nodes which are within a specified  
    // distance (distanceTolerance) from that target location. Only works for
    // 3D locations.
    // param: target - the target location defined by x, y, z coordinates
    // param: *node - a pointer to the node at which the tree traversal begins
    // param: depth - tracks the depth of the tree traversal
    // param: distanceTolerance - the distance threshold at which points are 
    //                            determined to be within the desired distance
    //                            from the target or not
    // param: &ids - a reference to a vector which is populated with the IDs of
    //               nodes which are within the distanceTolerance from the target
    //               location.
    void searchHelper3D(
        std::vector<float> target, // target location defined by x,y,z coordinates
        Node *node, 
        int depth, 
        float distanceTolerance, 
        std::vector<int> &ids
    ) {
        if(node != NULL) {
            if((node->point[0] >= (target[0] - distanceTolerance) && (node->point[0] >= (target[0] + distanceTolerance))
            && (node->point[1] >= (target[1] - distanceTolerance)) && (node->point[1] >= (target[1] + distanceTolerance))
            && (node->point[2] >= (target[2] - distanceTolerance)) && (node->point[2] >= (target[2] + distanceTolerance)))) {
                float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]) + (node->point[2] - target[2])*(node->point[2] - target[2]));
                if(distance <= distanceTolerance) ids.push_back(node->id);
                if((target[depth % 3] - distanceTolerance) < node->point[depth % 3]) searchHelper3D(target, node->left, depth + 1, distanceTolerance, ids);
                if((target[depth % 3] + distanceTolerance) > node->point[depth % 3]) searchHelper3D(target, node->right, depth + 1, distanceTolerance, ids);
            }
        }
    }

    // A utility function which calls searchHelper with some
    // default values. Searches for the nodes which are within
    // a distance of distanceTolerance from a target location
    // param: target - the target location defined by x, y, z coordinates
    // param: distanceTolerance - the distance threshold at which points are 
    //                            determined to be within the desired distance
    //                            from the target or not
    // returns a vector of IDs of the identified nodes
    std::vector<int> search(
        std::vector<float> target, 
        float distanceTolerance
    ) {
        std::vector<int> ids;
        searchHelper(target, rootNode, 0, distanceTolerance, ids);
        return ids;
    }
};