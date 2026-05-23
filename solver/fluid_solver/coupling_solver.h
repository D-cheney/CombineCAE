#ifndef COUPLING_SOLVER_H
#define COUPLING_SOLVER_H

#include "solver_context.h"
#include "utils/coupling/coupling_config.h"

#include <vector>

struct CouplingIterationResult {
    int outerIteration;
    double interfaceResidual;
    double massConservationError;
    bool converged;

    CouplingIterationResult()
        : outerIteration(0), interfaceResidual(0.0),
          massConservationError(0.0), converged(false) {}
};

struct CouplingState {
    std::vector<double> interfacePressures;
    std::vector<double> interfaceFlows;
    std::vector<double> interfaceTemperatures;

    void clear() {
        interfacePressures.clear();
        interfaceFlows.clear();
        interfaceTemperatures.clear();
    }
};

class CouplingSolver {
private:
    CouplingConfig config;
    CouplingState currentState;
    CouplingState previousState;
    std::vector<CouplingIterationResult> iterationHistory;

public:
    CouplingSolver();
    explicit CouplingSolver(const CouplingConfig& cfg);

    void setConfig(const CouplingConfig& cfg);
    const CouplingConfig& getConfig() const { return config; }

    // 主耦合求解函数
    bool solve(SolverContext& context);

    // 单次外迭代
    CouplingIterationResult solveOuterIteration(SolverContext& context);

    // 界面数据交换
    void exchangeInterfaceData(SolverContext& context);

    // 收敛判断
    bool checkConvergence(const CouplingIterationResult& result);

    // 松弛处理
    void applyUnderRelaxation(SolverContext& context, double factor);

    // 状态管理
    const CouplingState& getCurrentState() const { return currentState; }
    const std::vector<CouplingIterationResult>& getIterationHistory() const {
        return iterationHistory;
    }

private:
    // 提取界面数据
    void extractInterfaceData(SolverContext& context, CouplingState& state);

    // 应用界面数据
    void applyInterfaceData(SolverContext& context, const CouplingState& state);

    // 计算界面残差
    double calculateInterfaceResidual(const CouplingState& state1,
                                      const CouplingState& state2);

    // 计算质量守恒误差
    double calculateMassConservationError(SolverContext& context);
};

#endif // COUPLING_SOLVER_H
