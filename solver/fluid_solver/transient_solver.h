#ifndef TRANSIENT_SOLVER_H
#define TRANSIENT_SOLVER_H

#include "solver_context.h"

#include <vector>

enum class TimeIntegrationMethod {
    ExplicitEuler,      // 显式Euler（快速但条件稳定）
    ImplicitEuler,      // 隐式Euler（无条件稳定）
    CrankNicolson       // Crank-Nicolson（二阶精度，平衡稳定性与准确性）
};

struct TransientConfig {
    TimeIntegrationMethod method;
    double timeStep;
    double totalTime;
    double startTime;
    bool adaptiveTimeStep;
    double cflNumber;           // CFL数目标值
    double conservationTolerance; // 守恒检查容差
    int maxSubSteps;            // 自适应时间步最大子步数

    TransientConfig()
        : method(TimeIntegrationMethod::CrankNicolson),
          timeStep(0.1),
          totalTime(1.0),
          startTime(0.0),
          adaptiveTimeStep(true),
          cflNumber(0.8),
          conservationTolerance(1e-6),
          maxSubSteps(10) {}
};

struct TimeStepResult {
    bool success;
    double actualTimeStep;
    double residualNorm;
    double massConservationError;
    double energyConservationError;
    int iterations;

    TimeStepResult()
        : success(false), actualTimeStep(0.0), residualNorm(0.0),
          massConservationError(0.0), energyConservationError(0.0),
          iterations(0) {}
};

class TransientSolver {
private:
    TransientConfig config;
    std::vector<double> previousState;
    std::vector<double> currentState;

public:
    TransientSolver();
    explicit TransientSolver(const TransientConfig& cfg);

    void setConfig(const TransientConfig& cfg);
    const TransientConfig& getConfig() const { return config; }

    // 主求解函数
    bool solve(SolverContext& context, double totalTime);

    // 单时间步求解
    TimeStepResult solveTimeStep(SolverContext& context, double dt,
                                 const std::vector<double>& prevState);

    // 时间积分方法
    TimeStepResult explicitEulerStep(SolverContext& context, double dt,
                                     const std::vector<double>& prevState);
    TimeStepResult implicitEulerStep(SolverContext& context, double dt,
                                     const std::vector<double>& prevState);
    TimeStepResult crankNicolsonStep(SolverContext& context, double dt,
                                     const std::vector<double>& prevState);

    // 自适应时间步控制
    double calculateAdaptiveTimeStep(SolverContext& context, double currentDt,
                                     const TimeStepResult& lastResult);

    // 守恒检查
    bool checkConservation(SolverContext& context, double dt,
                           const std::vector<double>& prevState,
                           double& massError, double& energyError);

    // 状态管理
    void saveState(SolverContext& context);
    void restoreState(SolverContext& context, const std::vector<double>& state);
    std::vector<double> extractState(const SolverContext& context) const;

private:
    // 残差计算
    double calculateResidualNorm(SolverContext& context,
                                 const std::vector<double>& state);

    // 线性化求解
    bool solveLinearizedSystem(SolverContext& context,
                               const std::vector<double>&prevState,
                               std::vector<double>& newState,
                               double theta); // theta: 0=explicit, 0.5=CN, 1=implicit
};

#endif // TRANSIENT_SOLVER_H
