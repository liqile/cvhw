#include "CovGraph.h"

namespace lqlslam {

void Node::update(int nodeId, int minusWeight) {
    int w = id2w[nodeId];
    links.erase(pair<int, int>(w, nodeId));
    links.insert(pair<int, int>(minusWeight, nodeId));
    id2w[nodeId] = minusWeight;
}

Node::Node(void* data, int nodeId) {
    this->nodeId = nodeId;
    this->data = data;
    clear();
}

void Node::clear() {
    links.clear();
    id2w.clear();
}

void Node::link(Node *node, int weight) {
    weight = -weight;
    if (id2w.find(node->nodeId) != id2w.end()) {
        update(node->nodeId, weight);
    } else {
        links.insert(pair<int, int>(weight, node->nodeId));
        id2w[node->nodeId] = weight;
    }
}

void Node::unLink(Node* node) {
    if (id2w.find(node->nodeId) != id2w.end()) {
        int w = id2w[node->nodeId];
        links.erase(pair<int, int>(w, node->nodeId));
        id2w.erase(node->nodeId);
    }
}

void Node::getNeighbors(vector<int> &neigh, int maxNum, int minWeight) {
    neigh.clear();
    if (maxNum == -1) {
        maxNum = links.size();
    }
    Links::const_iterator itr = links.begin();
    while (itr != links.end()) {
        if (neigh.size() >= maxNum) {
            return;
        }
        if (-itr->first < minWeight) {
            return;
        }
        neigh.push_back(itr->second);
        itr ++;
    }
}

Node* CovGraph::newNode(Frame *frame) {
    if (frame->isKeyFrame) {
        Node* node = new Node((void*) frame, frame->keyFrameId);
        nodes[frame->keyFrameId] = node;
        return node;
    } else {
        return new Node((void*) frame, -1);
    }
}

void CovGraph::updateNode(Node *node, bool uniDirection) {
    node->clear();
    Frame* frame = (Frame*)(node->data);
    map<Frame*, int> counter;
    cout << "[local map] frameid: " << frame->frameId << " enter cov count" << endl;
    frame->covCount(counter);
    cout << "[local map] frameid: " << frame->frameId << " leave cov count" << endl;
    map<Frame*, int>::iterator itr = counter.begin();
    while (itr != counter.end()) {
       // if (frame->frameId == 94) {
            cout <<"[localmap] enter itr"<<endl;
       // }
        Frame* f = itr->first;
       // if (frame->frameId == 94) {
            cout <<"[localmap] leave itr"<<endl;
       // }
        if (f->frameId != frame->frameId) {
            cout << "[local map] before link, frameid: " << f->frameId<< endl;
            assert(nodes.count(f->keyFrameId));
            Node* n = nodes[f->keyFrameId];
            if (n == NULL) {
                cout << "NULL" << endl;
            } else {
                cout << "NOT NULL" << endl;
            }
            node->link(n, itr->second);
            cout << "[local map] after link, frameId: " << f->frameId << endl;
            if (!uniDirection) {
                n->link(node, itr->second);
            }
        }
        itr ++;
    }
    cout << "[local map] update finish, frameid: " << frame->frameId << endl;
}

void CovGraph::getNeighborKF(Node *node, vector<Frame*>& neigh, int maxNum, int minWeight) {
    vector<int> id;
    id.clear();
    cout << "[local map] try to get node neighbors, frameid: " << ((Frame*)(node->data))->frameId << endl;
    node->getNeighbors(id, maxNum, minWeight);
    cout << "[local map] after getting node neighbors, frameid: " << ((Frame*)(node->data))->frameId << endl;
    neigh.clear();
    for (int i = 0; i < id.size(); i++) {
        cout << "[local map], id of neighbor node: " << id[i] << endl;
        Node* n = nodes[id[i]];
        Frame* f = (Frame*)(n->data);
        neigh.push_back(f);
    }
    cout << "[local map] get neighbor kf of node finished, frameid: " << ((Frame*)(node->data))->frameId << endl;
}

CovGraph::CovGraph() {
    nodes.clear();
}

void CovGraph::addKeyFrame(Frame *frame) {
    Node* node = newNode(frame);
    updateNode(node, false);
}

void CovGraph::delKeyFrame(Frame *frame) {
    Node* node = nodes[frame->keyFrameId];
    vector<int> id;
    node->getNeighbors(id, -1, -1);
    for (int i = 0; i < id.size(); i++) {
        Node* n = nodes[id[i]];
        n->unLink(node);
    }
    nodes.erase(frame->keyFrameId);
    delete node;
}

void CovGraph::updateKeyFrame(Frame *frame) {
    Node* node = nodes[frame->keyFrameId];
    updateNode(node, false);
}

void CovGraph::getNeighbors(Frame *frame, vector<Frame *> &neigh, int maxNum, int minWeight) {
    if (frame->isKeyFrame) {
        Node* node = nodes[frame->keyFrameId];
        getNeighborKF(node, neigh, maxNum, minWeight);
    } else {
        Node* node = newNode(frame);
        updateNode(node, true);
        getNeighborKF(node, neigh, maxNum, minWeight);
        cout << "[local map] try to delete node, frameid : " << frame->frameId << endl;
        delete node;
        cout << "[local map] success to delete node, frameid : " << frame->frameId << endl;
    }
}

}
