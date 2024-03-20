#include <vector>
#include <unordered_map>
#include <queue>
#include <climits>
#include <cfloat>
#include <algorithm>


using namespace std;

// Structure to represent movie attributes
struct Movie {
    string title;
    string genre;
    string director;

    // Default constructor
    Movie() = default;

    // Constructor with arguments
    Movie(const string& t, const string& g, const string& d) : title(t), genre(g), director(d) {}
};

// Structure to represent graph vertices and edges 
class MovieGraph {
private:
    unordered_map<string, Movie> movies;
    unordered_map<string, vector<pair<string, double>>> edges;

public:
    // Function to add a movie to the graph
    void addMovie(const string& title, const string& genre, const string& director) {
        movies[title] = Movie(title, genre, director);
    }

    // Function to add a similarity edge between two movies
    void addSimilarity(const string& movie1, const string& movie2, double similarity) {
        edges[movie1].push_back({movie2, similarity});
        edges[movie2].push_back({movie1, similarity});
    }

    // Function to find the shortest path between two movies 
    vector<string> shortestPath(const string& source, const string& destination) {
        unordered_map<string, double> dist;
        unordered_map<string, string> parent;
        priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> pq;

        for (const auto& [movie, _] : movies) {
            dist[movie] = DBL_MAX;
        }

        dist[source] = 0;
        pq.push({0, source});

        while (!pq.empty()) {
            auto [d, node] = pq.top();
            pq.pop();

            if (node == destination) {
                break;
            }

            if (d > dist[node]) {
                continue;
            }

            for (const auto& [neighbour, weight] : edges[node]) {
                if (dist[node] + weight < dist[neighbour]) {
                    dist[neighbour] = dist[node] + weight;
                    parent[neighbour] = node;
                    pq.push({dist[neighbour], neighbour});
                }
            }
        }

        vector<string> path;
        string current = destination;
        while (current != source) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(source);
        reverse(path.begin(), path.end());

        return path;
    }

    // Function to find the minimum spanning tree of the graph
    vector<pair<string, string>> minSpanTree() {
        vector<pair<string, string>> result;
        unordered_map<string, bool> visited;
        priority_queue<pair<double, pair<string, string>>, vector<pair<double, pair<string, string>>>, greater<pair<double, pair<string, string>>>> pq;

        for (const auto& [movie, _] : movies) {
            visited[movie] = false;
        }

        visited.begin()->second = true;

        for (const auto& [movie, neighbours] : edges) {
            if (visited[movie]) {
                for (const auto& [neighbour, weight] : neighbours) {
                    pq.push({weight, {movie, neighbour}});
                }
            }
        }

        while (!pq.empty()) {
            auto [w, edge] = pq.top();
            pq.pop();
            auto [source, destination] = edge;

            if (visited[destination]) {
                continue;
            }

            visited[destination] = true;
            result.push_back({source, destination});

            for (const auto& [neighbour, weight] : edges[destination]) {
                if (!visited[neighbour]) {
                    pq.push({weight, {destination, neighbour}});
                }
            }
        }

        return result;
    }
};

int main() {
    // Test the movie graph functionality
    MovieGraph movieGraph;

    // Add movies
    movieGraph.addMovie("Almost Famous", "Drama", "Cameron Crowe");
    movieGraph.addMovie("Dazed and Confused", "Comedy", "Richard Linklater");
    movieGraph.addMovie("Pan's Labyrinth", "Fantasy", "Guillermo del Toro");

    // Add similarities between movies
    movieGraph.addSimilarity("Almost Famous", "Dazed and Confused", 0.8); // High similarity
    movieGraph.addSimilarity("Almost Famous", "Pan's Labyrinth", 0.4);     // Moderate similarity
    movieGraph.addSimilarity("Dazed and Confused", "Pan's Labyrinth", 0.6); // Moderate similarity

    // Find shortest path between movies
    vector<string> path = movieGraph.shortestPath("Almost Famous", "Pan's Labyrinth");
    cout << "Shortest Path from Almost Famous to Pan's Labyrinth: ";
    for (const string& movie : path) {
        cout << movie << " -> ";
    }
    cout << endl;

    // Find minimum spanning tree
    vector<pair<string, string>> mst = movieGraph.minSpanTree();
    cout << "Minimum Spanning Tree:" << endl;
    for (const auto& [source, destination] : mst) {
        cout << source << " - " << destination << endl;
    }

    return 0;
}
