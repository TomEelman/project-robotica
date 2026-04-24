#include "Navigator.h"

#include <cmath>
#include <iostream>

// ════════════════════════════════════════════════════════════
//  Constructor
// ════════════════════════════════════════════════════════════

Navigator::Navigator()
    : path({}, 0)
    , currentTarget(0.0f, 0.0f)
    , isUpdated(false)
    , saved(false)
{
}

// ════════════════════════════════════════════════════════════
//  PlanPath: Gebruik A* om een pad te berekenen
// ════════════════════════════════════════════════════════════

bool Navigator::PlanPath(const GridMap& map, Position start, Position goal) {
    // Zet de GridMap om naar het 2D grid dat Pathfinder verwacht
    // 0 = vrij, 1 = geblokkeerd
    const auto& grid = map.GetGrid();

    Node startNode(static_cast<int>(start.x), static_cast<int>(start.y));
    Node goalNode (static_cast<int>(goal.x),  static_cast<int>(goal.y));

    std::vector<Node> result = FindPath(grid, startNode, goalNode);

    if (result.empty()) {
        std::cerr << "Navigator: geen pad gevonden van ("
                  << start.x << "," << start.y << ") naar ("
                  << goal.x  << "," << goal.y  << ")\n";
        return false;
    }

    // Converteer Node vector naar Position vector
    std::vector<Position> waypoints;
    for (const Node& node : result) {
        waypoints.emplace_back(static_cast<float>(node.x),
                               static_cast<float>(node.y));
    }

    path      = Path(waypoints, static_cast<int>(waypoints.size()));
    isUpdated = true;
    saved     = false;

    std::cout << "Navigator: pad gevonden met " << waypoints.size() << " waypoints\n";
    return true;
}

// ════════════════════════════════════════════════════════════
//  Update: stel het volgende doelwaypoint in
// ════════════════════════════════════════════════════════════

void Navigator::Update(Position current) {
    if (path.IsEmpty()) return;

    // Ga naar het volgende waypoint als het huidige bereikt is
    if (ReachedPoint(current)) {
        path.Advance();  // Stap naar volgend punt

        if (!path.IsEmpty()) {
            currentTarget = path.GetNextPoint();
            std::cout << "Navigator: volgend waypoint ("
                      << currentTarget.x << ", " << currentTarget.y << ")\n";
        } else {
            std::cout << "Navigator: doel bereikt!\n";
        }
    }

    isUpdated = true;
}

// ════════════════════════════════════════════════════════════
//  GetNextCommand: bereken rijcommando richting currentTarget
// ════════════════════════════════════════════════════════════

DriveCommand Navigator::GetNextCommand(Position current) {
    if (path.IsEmpty()) {
        return DriveCommand(0.0f, 0.0f);  // Stilstaan
    }

    float distance = CalculateDistance(current, currentTarget);
    float angle    = CalculateAngle   (current, currentTarget);

    if (distance < REACHED_THRESHOLD_MM) {
        return DriveCommand(0.0f, 0.0f);  // Waypoint bereikt, wacht op Update()
    }

    // Proportioneel bijsturen op hoekfout
    // angle in radialen: positief = links, negatief = rechts
    float angular  = ANGULAR_GAIN * angle;
    float linear   = LINEAR_SPEED;

    // Bij grote hoekfout: eerst draaien, dan rijden
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
    // TODO: schrijf waypoints naar bestand als dat later nodig is
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
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    return sqrtf(dx * dx + dy * dy);
}

// Geeft de hoekfout in radialen tussen rijrichting en doel
// Vereist dat Position ook een 'theta' (rijrichting) veld heeft
// Als dat niet bestaat, geef dan alleen de absolute hoek terug
float Navigator::CalculateAngle(Position current, Position target) {
    float dx = target.x - current.x;
    float dy = target.y - current.y;

    float targetAngle  = atan2f(dy, dx);       // hoek naar doel
    float currentAngle = current.theta;         // huidige rijrichting in radialen

    float error = targetAngle - currentAngle;

    // Normaliseer naar [-π, π]
    while (error >  M_PI) error -= 2.0f * static_cast<float>(M_PI);
    while (error < -M_PI) error += 2.0f * static_cast<float>(M_PI);

    return error;
}