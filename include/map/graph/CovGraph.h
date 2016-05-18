#ifndef COV_GRAPH_H
#define COV_GRAPH_H

#include <set>
#include <map>
#include <vector>
#include "Frame.h"

using namespace std;

namespace lqlslam {

typedef multiset<pair<int, int> > Links;

class Node {
private:
    /*
     * links, set of pair<int, int> first is -weight, second is nodeId of linked node
     *     -weight is sorted by asc order, so weight is sorted by desc order
     */
    Links links;
    /*
     * map from nodeId to -weight
     */
    map<int, int> id2w;
    /*
     * update
     * @param int nodeId, id of neighbor node
     * @param int minusWeight, -weight
     * function: update weight of link
     */
    void update(int nodeId, int minusWeight);
public:
    /*
     * data
     */
    void* data;
    /*
     * id of current node
     */
    int nodeId;
    /*
     * Node
     * @param void* data, data
     * @param int nodeId, id of node
     * function: constructor
     */
    Node(void *data, int nodeId);
    /*
     * clear
     * function: clear all connections
     */
    void clear();
    /*
     * link
     * @param Node* node, a node
     * @param int weight, weight of edge
     * function: link a node with current node
     */
    void link(Node* node, int weight);
    /*
     * unLink
     * @param Node* node, a node
     * function: unlink a node with current node
     */
    void unLink(Node* node);
    /*
     * getNeighbors
     * @param vector<int>& neigh, store id of neighbor nodes
     * @param int maxNum, max num of results
     * @param int minWeight, min weight of neighbor in results
     * function: get neighbors, satisfy minWeight and maxNum
     *     if maxNum equals -1, means no num limit
     */
    void getNeighbors(vector<int>& neigh, int maxNum, int minWeight);
};

class CovGraph {
private:
    /*
     * map from nodeId to node, only for keyFrame
     */
    map<int, Node*> nodes;
    /*
     * newNode
     * @param Frame* frame, current frame
     * function: get a new Node, if current frame is keyframe,
     *     nodeId as keyFrameId, nodes will be updated
     *     if current frame isn't key frame, nodeId will be assigned -1
     *     as tmp node, nodes will not be updated
     */
    Node* newNode(Frame* frame);
    /*
     * updateNode
     * @param Node* node, current node to update links
     * @param bool uniDirection, whether to update links in uniDirection
     * function: update links of node
     */
    void updateNode(Node* node, bool uniDirection);
    /*
     * getNeighborKF
     * @param Node* node, current node of frame
     * @param vector<Frame*>& neigh, store neighbor key frames
     * @param int maxNum, max num of results
     * @param int minWeight, min weight of neighbor in results
     * function: get neighbor keyframes of current node, store in neigh
     *     sort by num of common mappoints in desc order
     */
    void getNeighborKF(Node* node, vector<Frame*>& neigh, int maxNum, int minWeight);
public:
    /*
     * CovGraph
     * function: constructor
     */
    CovGraph();
    /*
     * addKeyFrame
     * @param Frame* frame, new key frame to add
     * function: add a new keyframe to covisibility graph
     */
    void addKeyFrame(Frame* frame);
    /*
     * updateKeyFrame
     * @param Frame* frame, a key frame in covisibility graph
     * function: update links of keyframe in covisibility graph
     */
    void updateKeyFrame(Frame* frame);
    /*
     * getNeighbors
     * @param Frame* frame, current frame
     * @param vector<Frame*>& neigh, store neighbor key frames
     * @param int maxNum, max num of results
     * @param int minWeight, min weight of neighbor in results
     * function: get neighbors, satisfy minWeight and maxNum
     *     if maxNum equals -1, means no num limit
     */
    void getNeighbors(Frame* frame, vector<Frame*>& neigh, int maxNum, int minWeight);
};

}

#endif
