#include <cstddef>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>
#include <list>
#include <map>

// A -std=c++20 as flaget használtam a forrásfájl fordításához,
// mivel void bfs(int, auto) const; függvényt csak ebben a verzióban támogatja a fordító

// Graph abstract class
class Graph {
    protected:
        size_t size;

    public:
        Graph(int n) : size(n) {}
        virtual ~Graph() noexcept {}
        size_t getSize() const noexcept { return size; }
        virtual auto getEdges() const noexcept -> std::map<int, std::vector<int>> = 0;
        virtual void addEdge(int, int) = 0;
        virtual void addVertex() = 0;
        virtual void removeEdge(int, int) = 0;
        virtual void removeVertex(int) = 0;
        void bfs(int, auto) const;
        void dfs(int, std::function<void(int, std::map<int, std::vector<int>>&)>, std::shared_ptr<std::map<int, bool>> );
};

void Graph::bfs(int start, auto functor) const{
    std::map<int, std::vector<int>> edges = this->getEdges();

    std::vector<bool> visited;
    visited.resize(this->getSize(), false);

    std::list<int> queue;

    visited[start] = true;
    queue.push_back(start);

    while (!queue.empty()) {
        start = queue.front();
        queue.pop_front();

        functor(start, edges);

        for (auto node : edges[start]) {
            if (!visited[node]) {
                visited[node] = true;
                queue.push_back(node);
            }
        }
    }
}
void Graph::dfs(int start, std::function<void(int, std::map<int, std::vector<int>>&)> functor, std::shared_ptr<std::map<int, bool>> visited_ptr = std::make_shared<std::map<int, bool>>() ) {

    std::map<int, bool>& visited = *visited_ptr;


    if (visited.empty()) {
        for (int i = 0; i < this->getSize(); i++) {
            visited.insert(std::make_pair(i, false));
        }
    }
    
    visited[start] = true;

    std::map<int, std::vector<int>> edges = this->getEdges();


    functor(start, edges);
 

    for (std::vector<int>::iterator i = edges[start].begin(); i != edges[start].end(); ++i){
        if (!visited[*i]){
            this->dfs(*i, functor , visited_ptr);
        }
    }

}

// ListGraph class
class ListGraph : public Graph {
    private:
        // vertices maped to their edges
        std::map<int, std::vector<int>> edges;

    public:

        ListGraph(int n) noexcept : Graph(n) {}

        //copy 
        ListGraph(const ListGraph& other) noexcept : Graph(other.getSize()){
            edges = other.edges;
        }

        //move constructor
        ListGraph(ListGraph&& other) noexcept : Graph(other.getSize()){
            edges = std::move(other.edges);
        }

        //copy assignment
        ListGraph& operator=(const ListGraph& other) noexcept{
            ListGraph tmp(other.getSize());
            tmp.edges = other.edges;

            return *this = std::move(tmp);
        }

        //move assignment
        ListGraph& operator=(ListGraph&& other) noexcept{
            std::swap(edges, other.edges);
            return *this;
        }

        auto getEdges() const noexcept -> std::map<int, std::vector<int>> override final{
            return edges;
        }

        void addEdge(int i, int j) override final{
            if (i >= getSize() || j >= getSize())
                throw std::out_of_range("vertex out of bounds");

            edges[i].push_back(j);
            edges[j].push_back(i);
        }

        void addVertex() override final{
            edges.insert(std::make_pair(size++, std::vector<int>()));
        }

        void removeEdge(int i, int j) override final{
            if (i >= getSize() || j >= getSize())
                throw std::out_of_range("vertex out of bounds");

            edges[i].erase(std::remove(edges[i].begin(), edges[i].end(), j), edges[i].end());
            edges[j].erase(std::remove(edges[j].begin(), edges[j].end(), i), edges[j].end());
        }

        void removeVertex(int i) override final{
            if (i >= getSize())
                throw std::out_of_range("vertex out of bounds");

            edges.erase(i);
            for (auto& node : edges) {
                node.second.erase(std::remove(node.second.begin(), node.second.end(), i), node.second.end());
            }
        }

        ~ListGraph() {}
};

// MatrixGraph class
class MatrixGraph : public Graph {
    private:
        //vertices
        std::vector<int> vertices;
    public:
        MatrixGraph(int n) noexcept : Graph(n) {
            vertices.resize(n*n, 0);
        }

        //copy constructor
        MatrixGraph(const MatrixGraph& other) noexcept : Graph(other.getSize()){
            vertices = other.vertices;
        }

        //move constructor
        MatrixGraph(MatrixGraph&& other) noexcept : Graph(other.getSize()){
            vertices = std::move(other.vertices);
        }

        //copy assignment
        MatrixGraph& operator=(const MatrixGraph& other) noexcept{
            MatrixGraph tmp(other);
            std::swap(vertices, tmp.vertices);
            return *this;
        }

        //move assignment
        MatrixGraph& operator=(MatrixGraph&& other) noexcept{
            std::swap(vertices, other.vertices);
            return *this;
        }

        auto getEdges() const noexcept -> std::map<int ,std::vector<int>> override final{
            std::map<int, std::vector<int>> e;

            for (int i = 0; i < getSize(); i++) {
                std::vector<int> edge;

                for (int j = 0; j < getSize(); j++) {
                    if (vertices[i*getSize() + j] == 1) {
                        edge.push_back(j);
                    }
                }

                e.insert(std::make_pair(i, edge));
            }
            return e;
        }

        void addEdge(int i, int j) override final{
            if (i >= getSize() || j >= getSize())
                throw std::out_of_range("vertex out of bounds");

            vertices[i*getSize() + j] = 1;
        }

        void addVertex() override final{
            vertices.resize((getSize()+1)*(getSize()+1), 0);
        }

        void removeEdge(int i, int j) override final{
            if (i >= getSize() || j >= getSize())
                throw std::out_of_range("vertex out of bounds");

            vertices[i*getSize() + j] = 0;
        }

        void removeVertex(int i) override final{
            if (i >= getSize())
                throw std::out_of_range("vertex out of bounds");

            std::vector<int> newVertices;
            newVertices.reserve((getSize()-1)*(getSize()-1));

            int newIdx = 0;

            for (int k = 0; k < getSize(); k++) {
                if (k == i) continue;

                for (int l = 0; l < getSize(); l++) {
                    if (l != i) {
                        newVertices[newIdx++] = vertices[k * getSize() + l];
                    }
                }
            }

            vertices = std::move(newVertices);

            this->size--;
        }

        ~MatrixGraph() = default;
};

int main(){


    auto graph_out = 
R"(
               0    
              / \   
             /   \  
            /     2 
           1        
            \       
             \      
              3     )";
                 
    std::cout << "========== The Graph =========" << std::endl << graph_out << std::endl << std::endl;

    std::cout << "============= BFS ============" << std::endl;

    ListGraph g(4);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 3);
    g.bfs(0, [](int i, std::map<int, std::vector<int>> edges) { std::cout << i << " "; });
    std::cout << std::endl;
    g.removeEdge(0, 2);
    g.bfs(0, [](int i, std::map<int, std::vector<int>>& edges) { std::cout << i << " "; });
    std::cout << std::endl;
    g.removeVertex(1);
    g.bfs(0, [](int i, std::map<int, std::vector<int>>& edges) { std::cout << i << " "; });
    std::cout << std::endl;

    std::cout << "============= DFS ============" << std::endl;

    MatrixGraph g2(4);
    g2.addEdge(0, 1);
    g2.addEdge(0, 2);
    g2.addEdge(1, 3);
    g2.dfs(0, [](int i, std::map<int, std::vector<int>>& edges) { std::cout << i << " "; });
    std::cout << std::endl;

    g2.removeEdge(0, 2);

    g2.dfs(0, [](int i, std::map<int, std::vector<int>>& edges) { std::cout << i << " "; });
    std::cout << std::endl;
    g2.removeVertex(1);

    g2.dfs(0, [](int i, std::map<int, std::vector<int>>& edges) { std::cout << i << " "; });
    std::cout << std::endl;

    return 0;
}
