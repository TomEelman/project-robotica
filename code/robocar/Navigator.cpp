#include "Navigator.h"

#include <cmath>
#include <iostream>
#include <queue>
#include <algorithm>
#include <climits>

// ════════════════════════════════════════════════════════════
//  Node
// ════════════════════════════════════════════════════════════

Node::Node(int _x, int _y)
    : x(_x), y(_y), f(0), g(0), h(0)
{
}

bool Node::operator>(const Node& other) const {
    return f > other.f;
}

bool Node::operator==(const Node& other) const {
    return x == other.x && y == other.y;
}

// ════════════════════════════════════════════════════════════
//  A* algoritme
// ════════════════════════════════════════════════════════════

std::vector<Node> Navigator::FindPath(const std::vector<std::vector<int>>& grid,
                                      const Node& start,
                                      const Node& goal)
{
    const int dirX[] = {-1, 0, 1, 0};
    const int dirY[] = { 0, 1, 0,-1};

    int rows = static_cast<int>(grid.size());
    int cols = static_cast<int>(grid[0].size());

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::vector<std::vector<bool>> closedList(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<int>>  gScore    (rows, std::vector<int> (cols, INT_MAX));
    std::vector<std::vector<Node>> parent    (rows, std::vector<Node>(cols));

    gScore[start.x][start.y] = 0;
    openList.push(start);

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        if (current == goal) {
            std::vector<Node> pad;
            while (!(current == start)) {
                pad.push_back(current);
                current = parent[current.x][current.y];
            }
            pad.push_back(start);
            std::reverse(pad.begin(), pad.end());
            return pad;
        }

        closedList[current.x][current.y] = true;

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dirX[i];
            int ny = current.y + dirY[i];

            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols) continue;
            if (grid[nx][ny] != 0)                             continue;
            if (closedList[nx][ny])                            continue;

            int newG = gScore[current.x][current.y] + 1;
            if (newG < gScore[nx][ny]) {
                gScore[nx][ny] = newG;

                Node neighbor(nx, ny);
                neighbor.g = newG;
                neighbor.h = std::abs(nx - goal.x) + std::abs(ny - goal.y);
                neighbor.f = neighbor.g + neighbor.h;

                parent[nx][ny] = current;
                openList.push(neighbor);
            }
        }
    }
    return {};
}

// ════════════════════════════════════════════════════════════
//  Constructor
// ════════════════════════════════════════════════════════════

Navigator::Navigator()
    : path(nullptr, 0)
    , currentTarget(0.0f, 0.0f, 0.0f)
    , isUpdated(false)
    , saved(false)
{
}

// ════════════════════════════════════════════════════════════
//  PlanPath
// ════════════════════════════════════════════════════════════

bool Navigator::PlanPath(const GridMap& map, Position start, Position goal) {
    const auto& grid = map.GetGrid();

    // Gebruik getters want x en y zijn private in Position
    Node startNode(static_cast<int>(start.GetX()), static_cast<int>(start.GetY()));
    Node goalNode (static_cast<int>(goal.GetX()),  static_cast<int>(goal.GetY()));

    std::vector<Node> result = FindPath(grid, startNode, goalNode);

    if (result.empty()) {
        std::cerr << "Navigator: geen pad gevonden van ("
                  << start.GetX() << "," << start.GetY() << ") naar ("
                  << goal.GetX()  << "," << goal.GetY()  << ")\n";
        return false;
    }

    // Converteer Node vector naar Position array voor Path constructor
    int size = static_cast<int>(result.size());
    std::vector<Position> wpVec;
    wpVec.reserve(static_cast<size_t>(size));
    for (const Node& node : result) {
        wpVec.emplace_back(static_cast<float>(node.x),
                           static_cast<float>(node.y),
                           0.0f);
    }

    path          = Path(wpVec.data(), size);
    currentTarget = path.GetNextPoint();
    isUpdated     = true;
    saved         = false;

    std::cout << "Navigator: pad gevonden met " << size << " waypoints\n";
    return true;
}

// ════════════════════════════════════════════════════════════
//  Update
// ════════════════════════════════════════════════════════════

void Navigator::Update(Position current) {
    if (path.IsEmpty()) return;

    if (ReachedPoint(current)) {
        path.Advance();

        if (!path.IsEmpty()) {
            currentTarget = path.GetNextPoint();
            std::cout << "Navigator: volgend waypoint ("
                      << currentTarget.GetX() << ", "
                      << currentTarget.GetY() << ")\n";
        } else {
            std::cout << "Navigator: doel bereikt!\n";
        }
    }

    isUpdated = true;
}

// ════════════════════════════════════════════════════════════
//  GetNextCommand
// ════════════════════════════════════════════════════════════

DriveCommand Navigator::GetNextCommand(Position current) {
    if (path.IsEmpty()) {
        return DriveCommand(0.0f, 0.0f);
    }

    float distance = CalculateDistance(current, currentTarget);
    float angle    = CalculateAngle   (current, currentTarget);

    if (distance < REACHED_THRESHOLD_MM) {
        return DriveCommand(0.0f, 0.0f);
    }

    float angular = ANGULAR_GAIN * angle;
    float linear  = LINEAR_SPEED;

    // Bij grote hoekfout eerst draaien, dan rijden
    if (fabsf(angle) > 0.5f) {
        linear = 0.0f;
    }

    return DriveCommand(linear, angular);
}

// ════════════════════════════════════════════════════════════
//  ReachedPoint
// ════════════════════════════════════════════════════════════

bool Navigator::ReachedPoint(Position current) {
    return CalculateDistance(current, currentTarget) < REACHED_THRESHOLD_MM;
}

// ════════════════════════════════════════════════════════════
//  SaveWaypoints
// ════════════════════════════════════════════════════════════

bool Navigator::SaveWaypoints() {
    saved = true;
    return saved;
}

// ════════════════════════════════════════════════════════════
//  IsUpdated
// ════════════════════════════════════════════════════════════

bool Navigator::IsUpdated() const {
    return isUpdated;
}

// ════════════════════════════════════════════════════════════
//  Hulpfuncties
// ════════════════════════════════════════════════════════════

float Navigator::CalculateDistance(Position current, Position target) {
    float dx = target.GetX() - current.GetX();
    float dy = target.GetY() - current.GetY();
    return sqrtf(dx * dx + dy * dy);
}

float Navigator::CalculateAngle(Position current, Position target) {
    float dx = target.GetX() - current.GetX();
    float dy = target.GetY() - current.GetY();

    float targetAngle  = atan2f(dy, dx);
    float currentAngle = current.GetTheta();

    float error = targetAngle - currentAngle;

    // Normaliseer naar [-π, π]
    while (error >  static_cast<float>(M_PI)) error -= 2.0f * static_cast<float>(M_PI);
    while (error < -static_cast<float>(M_PI)) error += 2.0f * static_cast<float>(M_PI);

    return error;
}