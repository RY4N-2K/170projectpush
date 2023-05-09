#pragma once
#include "node.h"
#include "graph.h"
#include <unordered_set>
#include <queue>


class AStarED {
public:
    //to compare Node objects
    struct CompareNode {
        bool operator()(const Node& lhs, const Node& rhs) const {
            return lhs.f > rhs.f;
        }
    };

    //Constructor
    AStarED();

    //A* search with the Euclidean Distance heuristic
    std::vector<Node> search(const Node& start, const Node& goal);

private:
    //calculate the Euclidean Distance hueristic
    int EDHeuristic(const Node& node, const Node& goal);

    //creates path from the inputted node to the start node
    std::vector<Node> generatePath(const Node& node);
};
