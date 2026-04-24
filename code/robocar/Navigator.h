#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "Path.h"
#include "Position.h"
#include "DriveCommand.h"
#include "GridMap.h"

#include <vector>

// ════════════════════════════════════════════════════════════
//  Node - intern gebruikt door het A* algoritme
// ════════════════════════════════════════════════════════════

struct Node {
    int x, y;   // Gridcoordinaten
    int f, g, h; // A* kosten: f = g + h

    Node(int _x = 0, int _y = 0);
    bool operator>(const Node& other) const;
    bool operator==(const Node& other) const;
};

// ════════════════════════════════════════════════════════════
//  Navigator
// ════════════════════════════════════════════════════════════

class Navigator {
public:
    Navigator();

    // Bereken een pad via A* van start naar doel op de gegeven gridmap
    bool PlanPath(const GridMap& map, Position start, Position goal);

    // Update de navigator met de huidige positie
    void Update(Position current);

    // Geeft het volgende rijcommando richting het huidige waypoint
    DriveCommand GetNextCommand(Position current);

    // True als het huidige waypoint bereikt is
    bool ReachedPoint(Position current);

    // Sla de waypoints op (voor logging/debug)
    bool SaveWaypoints();

    // True als het pad bijgewerkt is
    bool IsUpdated() const;

private:
    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     saved;

    static constexpr float REACHED_THRESHOLD_MM = 50.0f;  // binnen 50mm = waypoint bereikt
    static constexpr float LINEAR_SPEED         = 200.0f; // mm/s vooruit
    static constexpr float ANGULAR_GAIN         = 2.0f;   // hoe hard bijsturen op hoekfout

    // ── A* algoritme (intern) ────────────────────────────────
    std::vector<Node> FindPath(const std::vector<std::vector<int>>& grid,
                               const Node& start,
                               const Node& goal);

    // ── Hulpfuncties ─────────────────────────────────────────
    float CalculateDistance(Position current, Position target);
    float CalculateAngle   (Position current, Position target);
};

#endif