#include <iostream>
#include <cstdio>
#include <random>
#include <SFML/Graphics.hpp>
#include "geometry.h"

using namespace std;

const int WIDTH = 800;
const int HEIGHT = 600;
const int RADIUS = 10;
const double GOAL_SAMPLING_PROB = 0.05;
const double INF = 1e18;

const double JUMP_SIZE = 30.0;
const double DISK_SIZE = JUMP_SIZE; // Ball radius around which nearby points are found

Point start, stop;
int obstacle_cnt = 1;

vector<Point> nodes;
vector<int> currentPathNodes;
vector<int> parent, nearby;
vector<double> cost, jumps;
int nodeCnt = 0, goalIndex = -1;

vector<sf::ConvexShape> polygons;
sf::CircleShape startingPoint, endingPoint;
bool pathFound = false;

Track track;
Point startDirection = {1.0, 0.0}; // Facing right initially
vector<Point> nodeDirection;       // Stores direction of each node

void initializeTrack() {
    const int centerX = WIDTH / 2;
    const int centerY = HEIGHT / 2;
    const int trackWidth = 40;
    const int radius = 150;
    const int numPoints = 100;

    vector<Point> leftBoundary;
    vector<Point> rightBoundary;

    // outer boundary example
    for (int i = 0; i < numPoints; ++i) {
        double angle = 2 * M_PI * i / numPoints;
        int x = centerX + (radius + trackWidth / 2) * cos(angle);
        int y = centerY + (radius + trackWidth / 2) * sin(angle);
        leftBoundary.emplace_back(x, y);
    }

    // inner boundary example
    for (int i = 0; i < numPoints; ++i) {
        double angle = 2 * M_PI * i / numPoints;
        int x = centerX + (radius - trackWidth / 2) * cos(angle);
        int y = centerY + (radius - trackWidth / 2) * sin(angle);
        rightBoundary.emplace_back(x, y);
    }

    // Add points to the track
    for (const auto& p : leftBoundary) track.addLeftPoint(p);
    for (const auto& p : rightBoundary) track.addRightPoint(p);

    // Start and stop inside the track
    start = {centerX, centerY - radius};
    stop = {centerX, (centerY + radius - 10)};
}

void prepareInput() {
    // Make starting and ending point circles ready
    startingPoint.setRadius(RADIUS);
    endingPoint.setRadius(RADIUS);
    startingPoint.setFillColor(sf::Color(208, 0, 240));
    endingPoint.setFillColor(sf::Color::Blue);
    startingPoint.setPosition(start.x, start.y);
    endingPoint.setPosition(stop.x, stop.y);
    startingPoint.setOrigin(RADIUS/2, RADIUS/2);
    endingPoint.setOrigin(RADIUS/2, RADIUS/2);
}

void draw(sf::RenderWindow& window) {
    sf::Vertex line[2];
    sf::CircleShape nodeCircle;

    // Draw track boundaries
    sf::VertexArray leftBoundary(sf::LineStrip, track.leftBoundary.size() + 1);
    for (int i = 0; i < track.leftBoundary.size(); i++) {
        leftBoundary[i].position = sf::Vector2f(track.leftBoundary[i].x, track.leftBoundary[i].y);
        leftBoundary[i].color = sf::Color::Green;
    }
    leftBoundary[track.leftBoundary.size()] = leftBoundary[0]; // close loop
    window.draw(leftBoundary);

    sf::VertexArray rightBoundary(sf::LineStrip, track.rightBoundary.size() + 1);
    for(int i = 0; i < track.rightBoundary.size(); i++) {
        rightBoundary[i].position = sf::Vector2f(track.rightBoundary[i].x, track.rightBoundary[i].y);
        rightBoundary[i].color = sf::Color::Green;
    }
    rightBoundary[track.rightBoundary.size()] = rightBoundary[0]; // close loop
    window.draw(rightBoundary);

    // Draw edges between nodes
    for(int i = (int)nodes.size() - 1; i; i--) {
        Point par = nodes[parent[i]];
        line[0] = sf::Vertex(sf::Vector2f(par.x, par.y));
        line[1] = sf::Vertex(sf::Vector2f(nodes[i].x, nodes[i].y));
        window.draw(line, 2, sf::Lines);
    }

    window.draw(startingPoint);
    window.draw(endingPoint);

    // If destination is reached then path is retraced and drawn
    if(pathFound) {
        int node = goalIndex;
        while(parent[node] != node) {
            int par = parent[node];
            line[0] = sf::Vertex(sf::Vector2f(nodes[par].x, nodes[par].y));
            line[1] = sf::Vertex(sf::Vector2f(nodes[node].x, nodes[node].y));
            line[0].color = line[1].color = sf::Color::Red;
            window.draw(line, 2, sf::Lines);
            node = par;
        }
    }
}

template <typename T> // Returns a random number in [low, high]
T randomCoordinate(T low, T high){
    random_device random_device;
    mt19937 engine{random_device()};
    uniform_real_distribution<double> dist(low, high);
    return dist(engine);
}

bool isEdgeObstacleFree(Point a, Point b) {
    // Check against left boundary
    for (int i = 1; i < track.leftBoundary.size(); i++) {
        if (check_intersection(a, b, track.leftBoundary[i - 1], track.leftBoundary[i]))
            return false;
    }
    // Close the loop
    if (check_intersection(a, b, track.leftBoundary.back(), track.leftBoundary.front()))
        return false;

    // Check against right boundary
    for (int i = 1; i < track.rightBoundary.size(); i++) {
        if (check_intersection(a, b, track.rightBoundary[i - 1], track.rightBoundary[i]))
            return false;
    }
    // Close the loop
    if (check_intersection(a, b, track.rightBoundary.back(), track.rightBoundary.front()))
        return false;

    return true;
}

void extractCurrentPath() {
    currentPathNodes.clear();
    int idx = goalIndex;
    while (true) {
        currentPathNodes.push_back(idx);
        if (idx == parent[idx]) break;
        idx = parent[idx];
    }
}

Point pickRandomPoint() {
    double random_sample = randomCoordinate(0.0, 1.0);
    if((random_sample - GOAL_SAMPLING_PROB) <= EPS && !pathFound)
        return stop + Point(RADIUS, RADIUS);
    return {static_cast<double>(randomCoordinate(0, WIDTH)), static_cast<double>(randomCoordinate(0, HEIGHT))};
}

void checkDestinationReached() {
    sf::Vector2f position = endingPoint.getPosition();
    if(checkCollision(nodes[parent[nodeCnt - 1]], nodes.back(), Point(position.x, position.y), RADIUS)) {
        pathFound = true;
        goalIndex = nodeCnt - 1;
        cout << "Found first path with a distance of " << cost.back() << " units. " << endl << endl;
    }
}

void insertNodesInPath(int rootIndex, Point& q) {
    Point p = nodes[rootIndex];
    if(!isEdgeObstacleFree(p, q)) return;
    while(!(p == q)) {
        Point nxt = p.steer(q, JUMP_SIZE);
        nodes.push_back(nxt);
        parent.push_back(rootIndex);
        cost.push_back(cost[rootIndex] + distance(p, nxt));
        rootIndex = nodeCnt++;
        p = nxt;
    }
}

double directionScore(Point from, Point to, Point direction) {
    Point movementVec = to - from;
    double dot = movementVec.dot(direction);
    double magProduct = movementVec.magnitude() * direction.magnitude();
    if (magProduct < EPS) return -1.0; // Bad movement

    return dot / magProduct;  // Cosine of the angle: 1 = perfect alignment, 0 = 90 deg, -1 = backwards
}

void rewire() {
    int lastInserted = nodeCnt - 1;
    for (auto nodeIndex : nearby) {
        int par = lastInserted;
        int cur = nodeIndex;

        double newCost = cost[par] + distance(nodes[par], nodes[cur]);

        // Check edge and cost improvement
        if (newCost + EPS < cost[cur] && isEdgeObstacleFree(nodes[par], nodes[cur])) {
            parent[cur] = par;
            cost[cur] = newCost;
            nodeDirection[cur] = (nodes[cur] - nodes[par]).normalized();
        }
    }
}

int randomIndex(int min, int max) {
    static std::random_device rd;
    static std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist(min, max);
    return dist(rng);
}

void RRT() {
    Point newPoint, nearestPoint, nextPoint;
    bool updated = false;
    int nearestIndex;
    double minCost;
    nearby.clear();
    jumps.resize(nodeCnt);

    while (!updated) {
        // Select a random point depending on pathFound state
        if (!pathFound) {
            newPoint = pickRandomPoint();
        } else {
            // Sample near current path nodes to try improving
            int baseNode = currentPathNodes[randomIndex(0, currentPathNodes.size() - 1)];
            Point base = nodes[baseNode];
            newPoint = base + Point(
                randomCoordinate(-JUMP_SIZE, JUMP_SIZE),
                randomCoordinate(-JUMP_SIZE, JUMP_SIZE)
            );
        }

        // Find nearest point to newPoint
        nearestPoint = *nodes.begin();
        nearestIndex = 0;
        for (int i = 0; i < nodeCnt; i++) {
            if (pathFound && randomCoordinate(0.0, 1.0) < 0.25)
                cost[i] = cost[parent[i]] + distance(nodes[parent[i]], nodes[i]);

            jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE;
            auto pnt = nodes[i];

            if ((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS &&
                isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps[i]))) {
                nearestPoint = pnt;
                nearestIndex = i;
            }
        }

        nextPoint = stepNear(nearestPoint, newPoint, jumps[nearestIndex]);

        if (!isEdgeObstacleFree(nearestPoint, nextPoint))
            continue;

        // Direction biasing
        double dirScore = directionScore(nearestPoint, nextPoint, nodeDirection[nearestIndex]);
        if (dirScore < 0.2) continue; // no going backwards and no super abrupt turns
        double random = randomCoordinate(0.0, 1.0) - 0.2;
        if (random > dirScore) {
            continue;
        }

        // RRT* rewiring path refinement
        for (int i = 0; i < nodeCnt; i++) {
            if ((nodes[i].distance(nextPoint) - DISK_SIZE) <= EPS &&
                isEdgeObstacleFree(nodes[i], nextPoint)) {
                nearby.push_back(i);
            }
        }

        int par = nearestIndex;
        minCost = cost[par] + distance(nodes[par], nextPoint);
        for (auto nodeIndex : nearby) {
            double c = cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint);
            if ((c - minCost) <= EPS) {
                minCost = c;
                par = nodeIndex;
            }
        }

        parent.push_back(par);
        cost.push_back(minCost);
        nodes.push_back(nextPoint);
        nodeDirection.push_back((nextPoint - nodes[par]).normalized());
        nodeCnt++;
        updated = true;

        if (!pathFound) {
            checkDestinationReached();
            if (pathFound) {
                extractCurrentPath(); // Build current path
            } 
        }

        rewire(); // Will use nearby
    }
}

int main() {
    initializeTrack();
    prepareInput();

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "RRT* Path Planning");

    nodeCnt = 1;
    nodes.push_back(start);
    nodeDirection.push_back(startDirection);
    int iterations = 0;
    parent.push_back(0);
    cost.push_back(0);

    cout << endl << "Welcome to RRT* Path Planning" << endl << endl;

    cout << endl << "Starting node is in Pink and Destination node is in Blue" << endl << endl;
    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
                return 0;
            }
        }

        RRT();
        iterations++;

        if(iterations % 200 == 0) {
            cout << "Iterations: " << iterations << endl;
            if(!pathFound) cout << "Not reached yet :( " << endl;
            else cout << "Shortest distance till now: " << cost[goalIndex] << " units." << endl;
            cout << endl;
        }

        window.clear();
        draw(window);
        window.display();
    }
}