#ifndef BOUNDARY_CONDITIONS_H
#define BOUNDARY_CONDITIONS_H

enum class BoundaryConditionType {
    Pressure,
    FlowRate,
    Temperature
};

struct BoundaryCondition {
    int nodeId;
    BoundaryConditionType type;
    double value;

    BoundaryCondition(int id, BoundaryConditionType bcType, double bcValue)
        : nodeId(id), type(bcType), value(bcValue) {}
};

#endif // BOUNDARY_CONDITIONS_H