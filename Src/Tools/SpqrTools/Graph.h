#include "Tools/Math/Eigen.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Geometry.h"
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <cassert>
#include <algorithm>
#include <fstream> 
#include <sstream>

namespace spqr {

    class Node {
    private:
        unsigned id;
        Vector2f position;
        std::vector<Node*> neighbors;

    protected:
        /*
        All non const methods should be protected and be accessed only by friend classes.
        The nodes should be created/modified only from a graph to avoid defining nodes with the same Id.
        */
        Node(unsigned id, float x, float y) : id(id), position(Vector2f(x,y)) {}

        Node(const Node& other) : id(other.id), position(other.position), neighbors(other.neighbors) {}

        void addLink(Node* n) {
            neighbors.push_back(n);
        }

        void removeLink(Node* n){
            neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), n), neighbors.end());
        }

    public:
        Vector2f getPosition() const {
            return position;
        }

        const std::vector<Node*>& getNeighbors() const {
            return neighbors;
        }

        unsigned getId() const {
            return id;
        }

        friend class UndirectedGraph;
    };

    class UndirectedGraph {
    private:
        std::vector<Node *> nodes;
        std::unordered_map<unsigned, Node*> nodeMap;

    public:
        UndirectedGraph() = default;
        
        const std::vector<Node *> getNodes() const{
            return nodes;
        }
        
        const std::unordered_map<unsigned, Node*> getNodeMap() const{
            return nodeMap;
        }
        
        bool containsNode(unsigned id) const{
            auto it = nodeMap.find(id);
            return it != nodeMap.end();
        }

        void addNode(float x, float y) {
            // auto-incremental id
            unsigned id = nodes.size();
            addNode(id, x, y);
        }

        void addNode(unsigned id, float x, float y) {
            Node * n = new Node(id,x,y);
            addNode(n);
        }
        
        void addNode(const Node * n) {
            assert(!containsNode(n->getId()) && "Duplicated Node ID in addNode");
            nodes.push_back((Node *) n);
            nodeMap[n->getId()] = (Node *) n;
        }

        Node* getNode(const unsigned id) const{
            assert(containsNode(id) && "Missing Node ID in getNode");
            return nodeMap.find(id)->second;
        }

        void addLink(const unsigned id1, const unsigned id2) {
            Node* n1 = getNode(id1);
            Node* n2 = getNode(id2);
            addLink(n1, n2);
        }

        void addLink(Node* n1, Node* n2) {
            n1->addLink(n2);
            n2->addLink(n1);
        }

        void removeLink(const unsigned id1, const unsigned id2) {
            Node* n1 = getNode(id1);
            Node* n2 = getNode(id2);
            removeLink(n1, n2);   
        }

        void removeLink(Node* n1, Node* n2) {
            n1->removeLink(n2);
            n2->removeLink(n1);
        }
        
        void loadFromFile(const std::string& filename) {
            /*
            The file is divided in three sections: a line specifying the number of nodes,
            n lines specifying the nodes and m lines specifying the edges.

            NUM_NODES
            NODE1_ID NODE1_X NODE1_Y
            NODE2_ID NODE2_X NODE2_Y
            ...
            NODE1_ID NODE2_ID
            NODE1_ID NODE3_ID
            ...

            e.g.
            4            // Number of nodes
            0 0.0 0.0    // Node details (id, x, y)
            1 1.0 1.0
            2 2.0 2.0
            3 3.0 3.0
            0 1          // Edge (id1, id2)
            1 2
            2 3
            1 3
            */
            std::ifstream file(filename);
            if (!file.is_open()) {
                throw std::runtime_error("Could not open file");
            }

            std::string line;
            std::getline(file, line);
            int numNodes = std::stoi(line);

	        // Adding nodes 
            for (int i = 0; i < numNodes; ++i) {
                std::getline(file, line);
                std::istringstream iss(line);
                unsigned id;
                float x, y;
                iss >> id >> x >> y;
                addNode(id, x, y);
            }

	        // Adding edges
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                unsigned id1, id2;
                iss >> id1 >> id2;
                addLink(id1, id2);
            }

            file.close();
        }
        
        void mergeGraph(UndirectedGraph g){
            std::vector<Node *> incomingNodes = g.getNodes();
            
            for(const Node * n : incomingNodes)
            	addNode(new Node(*n));
        }

        const Node * getClosestNode(Vector2f position) const{
            float best_distance = INFINITY;
            Node * best;

            for(Node * n : nodes){
                Vector2f diff = n->getPosition() - position;
                float distance = diff.norm();

                if(distance < best_distance){
                    best_distance = distance;
                    best = n;
                } 
            }

            return best;
        }

        const bool checkPointInsideGraphRectangle(Vector2f position) const {
            float min_x_graph = INFINITY;
            float min_y_graph = INFINITY;
            float max_x_graph = -INFINITY;
            float max_y_graph = -INFINITY;
            for(const spqr::Node* node : nodes){
                Vector2f node_position = node->getPosition();
                min_x_graph = std::min(node_position.x(), min_x_graph);
                min_y_graph = std::min(node_position.y(), min_y_graph);
                max_x_graph = std::max(node_position.x(), max_x_graph);
                max_y_graph = std::max(node_position.y(), max_y_graph);
            }
            
            //* This fix is needed to avoid the jolly and libero too close to the opponent kick-in
            if(max_y_graph == 2200) max_y_graph = 3100;
            if(min_y_graph == -2200) min_y_graph = -3100;


            Vector2f bottom_left = Vector2f(min_x_graph, max_y_graph);
            Vector2f top_right = Vector2f(max_x_graph, min_y_graph);
            return Geometry::isPointInsideRectangle(bottom_left, top_right, position);
        }

        void draw() const{
            DECLARE_DEBUG_DRAWING3D("module:UndirectedGraph", "field");

            for(Node * node : nodes){
                Vector2f nodePosition = node->getPosition();
                POINT3D("module:UndirectedGraph", nodePosition[0], nodePosition[1], 0.f, 15.f, ColorRGBA::blue);

                for(Node * neighbor : node->getNeighbors()){
                    Vector2f neighborPosition = neighbor->getPosition();
                    LINE3D("module:UndirectedGraph", nodePosition[0], nodePosition[1], 0.f, neighborPosition[0], neighborPosition[1], 0.f, 5.f, ColorRGBA::red);
                }
                
            }

        };
    };

}
