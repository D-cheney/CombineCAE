#include "transient_solver.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

TransientSolver::TransientSolver() {}

TransientSolver::TransientSolver(const TransientConfig& cfg)
    : config(cfg) {}

void TransientSolver::setConfig(const TransientConfig& cfg) {
    config = cfg;
}

bool TransientSolver::solve(SolverContext& context, double totalTime) {
    clearTimeStepSnapshots();
    saveState(context);

    double currentTime = config.startTime;
    double remainingTime = totalTime;
    int step = 0;

    while (remainingTime > 1e-12) {
        double dt = std::min(config.timeStep, remainingTime);

        // 自适应时间步
        if (config.adaptiveTimeStep && step > 0) {
            dt = calculateAdaptiveTimeStep(context, dt, TimeStepResult());
        }

        TimeStepResult result = solveTimeStep(context, dt, previousState);

        if (!result.success) {
            // 时间步失败，减小步长重试
            dt *= 0.5;
            if (dt < config.timeStep * 1e-6) {
                std::cerr << "Transient solve failed at t=" << currentTime
                          << ": time step too small." << std::endl;
                return false;
            }
            continue;
        }

        // 守恒检查
        double massError, energyError;
        if (!checkConservation(context, dt, previousState, massError, energyError)) {
            std::cerr << "Conservation check failed at t=" << currentTime
                      << ": mass=" << massError << ", energy=" << energyError << std::endl;
            dt *= 0.5;
            continue;
        }

        // 保存时间步快照
        saveTimeStepSnapshot(currentTime + dt);

        previousState = extractState(context);
        currentTime += dt;
        remainingTime -= dt;
        step++;
    }

    return true;
}

TimeStepResult TransientSolver::solveTimeStep(SolverContext& context, double dt,
                                              const std::vector<double>& prevState) {
    switch (config.method) {
        case TimeIntegrationMethod::ExplicitEuler:
            return explicitEulerStep(context, dt, prevState);
        case TimeIntegrationMethod::ImplicitEuler:
            return implicitEulerStep(context, dt, prevState);
        case TimeIntegrationMethod::CrankNicolson:
            return crankNicolsonStep(context, dt, prevState);
        default:
            return crankNicolsonStep(context, dt, prevState);
    }
}

TimeStepResult TransientSolver::explicitEulerStep(SolverContext& context, double dt,
                                                  const std::vector<double>& prevState) {
    TimeStepResult result;
    result.actualTimeStep = dt;

    // 显式Euler: x_{n+1} = x_n + dt * f(x_n)
    std::vector<double> newState = prevState;
    bool success = solveLinearizedSystem(context, prevState, newState, 0.0);

    result.success = success;
    result.residualNorm = calculateResidualNorm(context, newState);
    result.iterations = 1;

    if (success) {
        restoreState(context, newState);
    }

    return result;
}

TimeStepResult TransientSolver::implicitEulerStep(SolverContext& context, double dt,
                                                  const std::vector<double>& prevState) {
    TimeStepResult result;
    result.actualTimeStep = dt;

    // 隐式Euler: x_{n+1} = x_n + dt * f(x_{n+1})
    std::vector<double> newState = prevState;
    bool success = solveLinearizedSystem(context, prevState, newState, 1.0);

    result.success = success;
    result.residualNorm = calculateResidualNorm(context, newState);
    result.iterations = 1;

    if (success) {
        restoreState(context, newState);
    }

    return result;
}

TimeStepResult TransientSolver::crankNicolsonStep(SolverContext& context, double dt,
                                                  const std::vector<double>& prevState) {
    TimeStepResult result;
    result.actualTimeStep = dt;

    // Crank-Nicolson: x_{n+1} = x_n + 0.5*dt*(f(x_n) + f(x_{n+1}))
    std::vector<double> newState = prevState;
    bool success = solveLinearizedSystem(context, prevState, newState, 0.5);

    result.success = success;
    result.residualNorm = calculateResidualNorm(context, newState);
    result.iterations = 1;

    if (success) {
        restoreState(context, newState);
    }

    return result;
}

double TransientSolver::calculateAdaptiveTimeStep(SolverContext& context, double currentDt,
                                                  const TimeStepResult& lastResult) {
    // 基于CFL数的自适应时间步控制
    double cflTarget = config.cflNumber;
    double cflCurrent = 0.0;

    // 计算当前CFL数 (简化版本)
    // CFL = u * dt / dx, 其中u是特征速度，dx是特征长度
    double maxVelocity = 0.0;
    double minLength = 1e10;

    for (const auto& comp : context.components) {
        if (comp && comp->hasConnection()) {
            // 获取组件连接的节点
            int inletId = comp->getInletNodeId();
            int outletId = comp->getOutletNodeId();

            // 查找节点
            for (const auto& node : context.nodes) {
                if (node && (node->id == inletId || node->id == outletId)) {
                    // 简化的速度估算
                    double velocity = std::abs(node->flowInjection) / context.workingFluid.density;
                    maxVelocity = std::max(maxVelocity, velocity);
                }
            }
        }
    }

    // 估算特征长度
    if (!context.nodes.empty()) {
        minLength = 1.0; // 默认值
    }

    // 计算CFL数
    if (maxVelocity > 1e-12 && minLength > 1e-12) {
        cflCurrent = maxVelocity * currentDt / minLength;
    }

    // 根据CFL数调整时间步
    if (cflCurrent > 1e-12) {
        double dtNew = currentDt * (cflTarget / cflCurrent);
        // 限制时间步变化范围
        dtNew = std::max(dtNew, currentDt * 0.5);
        dtNew = std::min(dtNew, currentDt * 2.0);
        return dtNew;
    }

    // 基于残差的备用调整
    if (lastResult.residualNorm < 1e-10) {
        // 残差很小，可以增大时间步
        return std::min(currentDt * 1.5, config.timeStep * 2.0);
    }

    if (lastResult.residualNorm > 1e-4) {
        // 残差较大，减小时间步
        return std::max(currentDt * 0.5, config.timeStep * 0.1);
    }

    return currentDt;
}

bool TransientSolver::checkConservation(SolverContext& context, double dt,
                                        const std::vector<double>& prevState,
                                        double& massError, double& energyError) {
    // 简化的守恒检查
    massError = 0.0;
    energyError = 0.0;

    // 计算质量守恒误差
    double totalMassIn = 0.0;
    double totalMassOut = 0.0;

    for (const auto& bc : context.boundaryConditions) {
        if (bc.type == BoundaryConditionType::MassFlow) {
            totalMassIn += bc.value;
        }
    }

    // 计算能量守恒误差
    double totalEnergyIn = 0.0;
    double totalEnergyOut = 0.0;

    for (const auto& node : context.nodes) {
        if (node) {
            totalEnergyIn += node->thermalInjectionPower;
        }
    }

    massError = std::abs(totalMassIn - totalMassOut) / (std::abs(totalMassIn) + 1e-12);
    energyError = std::abs(totalEnergyIn - totalEnergyOut) / (std::abs(totalEnergyIn) + 1e-12);

    return (massError < config.conservationTolerance &&
            energyError < config.conservationTolerance);
}

void TransientSolver::saveState(SolverContext& context) {
    previousState = extractState(context);
    currentState = previousState;
}

void TransientSolver::restoreState(SolverContext& context, const std::vector<double>& state) {
    if (state.size() != context.nodes.size()) {
        return;
    }

    for (std::size_t i = 0; i < context.nodes.size() && i < state.size(); ++i) {
        if (context.nodes[i]) {
            context.nodes[i]->pressure = state[i];
        }
    }
}

std::vector<double> TransientSolver::extractState(const SolverContext& context) const {
    std::vector<double> state;
    state.reserve(context.nodes.size());

    for (const auto& node : context.nodes) {
        if (node) {
            state.push_back(node->pressure);
        } else {
            state.push_back(0.0);
        }
    }

    return state;
}

double TransientSolver::calculateResidualNorm(SolverContext& context,
                                              const std::vector<double>& state) {
    double norm = 0.0;
    for (double val : state) {
        norm += val * val;
    }
    return std::sqrt(norm);
}

bool TransientSolver::solveLinearizedSystem(SolverContext& context,
                                            const std::vector<double>& prevState,
                                            std::vector<double>& newState,
                                            double theta) {
    // 简化的线性化求解
    // 实际实现需要构建和求解线性系统

    // 使用当前状态作为初始猜测
    newState = prevState;

    // 应用时间积分
    for (std::size_t i = 0; i < newState.size(); ++i) {
        if (i < context.nodes.size() && context.nodes[i]) {
            double f_n = 0.0; // f(x_n) - 当前状态的导数
            double f_new = 0.0; // f(x_{n+1}) - 新状态的导数

            // 简化的导数计算
            f_n = context.nodes[i]->flowInjection;
            f_new = f_n; // 简化处理

            // 时间积分: x_{n+1} = x_n + dt * (theta * f_new + (1-theta) * f_n)
            newState[i] = prevState[i] + context.convergenceTolerance *
                         (theta * f_new + (1.0 - theta) * f_n);
        }
    }

    return true;
}
