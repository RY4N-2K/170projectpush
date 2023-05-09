#include "astared.h"
#include <unordered_map>


AStarED::AStarED() {}


//search implementation for astar w/ eudiclidean search
std::vector<Node> AStarED::search(const Node& start, const Node& goal) {


    //a map in order to store visited nodes with respective f-values
    std::unordered_map<Node, int, std::hash<Node>> visited;


    //to compare f-values in order to set priority queue
    auto comp = [](const Node& lhs, const Node& rhs) {
        return lhs.f > rhs.f;
    };


    //priority queue (stores nodes to be expanded)
    std::priority_queue<Node, std::vector<Node>, CompareNode> frontier;


    //pushes the first node into the frontier
    frontier.push(start);


    while (!frontier.empty()) {
        //retrieves node with the lowest f-value from fronteirr
        Node curr = frontier.top();
        frontier.pop();


        //a check to see if the current node is the goal
        if (curr == goal) {
            // Generate and return the path from the goal node to the start node
            return path(curr);
        }


        //adds current node w f-value to visited map
        visited[curr] = curr.f;


        //generates the neighbors of the current node
        std::vector<Node> neighbors = Graph().get_neighbors(curr);


        for (Node& neighbor : neighbors) {
            //calculates temp g-value for neighbor
            int tempG = curr.g + 1;


            //checks if neighbor already visited or in frontier w/ lower g-value
            auto visitedNeighbor = visited.find(neighbor);
            if (visitedNeighbor == visited.end() || visitedNeighbor->second > tempG + neighbor.f) {
                // Update the g-value and parent of the neighbor
                neighbor.g = tempG;
                neighbor.parent = std::make_shared<Node>(curr);


                //calculates neighbor hueristic value
                int h = calculateHeuristic(neighbor, goal);


                //then calculates the f-value
                int f = neighbor.g + h;


                //set neighbor's f-value
                neighbor.f = f;


                // pushes neighbor to frontier
                frontier.push(neighbor);
            }
        }
    }


    //if no solution found return empty path
    return std::vector<Node>();
}




int AStarED::EDHeuristic(const Node& node, const Node& goal) {
    int heuristic = 0;


    for (int i = 0; i < node.tiles.size(); ++i) {
        int tile = node.tiles[i];
        if (tile != 0) {
            int x1 = i % 3;
            int y1 = i / 3;
            int goalIndex = -1;
            for (int j = 0; j < goal.tiles.size(); ++j) {
                if (goal.tiles[j] == tile) {
                    goalIndex = j;
                    break;
                }
            }
            int x2 = goalIndex % 3;
            int y2 = goalIndex / 3;


            int dx = x1 - x2;
            int dy = y1 - y2;


            heuristic += dx * dx + dy * dy; //the euclidean distance
        }
    }


    return heuristic;
}




std::vector<Node> AStarED::path(const Node& goal) {
    std::vector<Node> p;
    const Node* curr = &goal;


    while (curr != nullptr) {
        p.push_back(*curr);
        curr = curr->parent.get();
    }


    std::reverse(p.begin(), p.end());
    return p;
}