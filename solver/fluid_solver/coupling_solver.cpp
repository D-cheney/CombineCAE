#include "coupling_solver.h"

#include <algorithm>
#include <cmath>
#include <iostream>

CouplingSolver::CouplingSolver() {}

CouplingSolver::CouplingSolver(const CouplingConfig& cfg)
    : config(cfg) {}

void CouplingSolver::setConfig(const CouplingConfig& cfg) {
    config = cfg;
}

bool CouplingSolver::solve(SolverContext& context) {
    if (!config.enabled || config.mode == CouplingMode::None) {
        return true;
    }

    iterationHistory.clear();
    currentState.clear();
    previousState.clear();

    for (int iter = 0; iter < config.maxOuterIterations; ++iter) {
        CouplingIterationResult result = solveOuterIteration(context);
        iterationHistory.push_back(result);

        // 检查收敛
        if (checkConvergence(result)) {
            std::cout << "Coupling converged after " << (iter + 1) << " outer iterations." << std::endl;
            return true;
        }

        // 应用松弛
        if (config.underRelaxationFactor < 1.0) {
            applyUnderRelaxation(context, config.underRelaxationFactor);
        }

        // 保存当前状态
        previousState = currentState;
    }

    std::cerr << "Coupling did not converge after " << config.maxOuterIterations << " iterations." << std::endl;
    return false;
}

CouplingIterationResult CouplingSolver::solveOuterIteration(SolverContext& context) {
    CouplingIterationResult result;

    // 保存之前的状态
    previousState = currentState;

    // 1. 求解1D流体网络
    // 注意：这里假设1D求解器已经被调用

    // 2. 交换界面数据
    exchangeInterfaceData(context);

    // 3. 计算界面残差
    result.interfaceResidual = calculateInterfaceResidual(previousState, currentState);

    // 4. 计算质量守恒误差
    result.massConservationError = calculateMassConservationError(context);

    // 5. 判断收敛
    result.converged = (result.interfaceResidual < config.outerConvergenceTol);

    return result;
}

void CouplingSolver::exchangeInterfaceData(SolverContext& context) {
    extractInterfaceData(context, currentState);

    // 如果有外部求解器，这里应该调用外部求解器
    // 目前只是简单地保持当前状态
}

bool CouplingSolver::checkConvergence(const CouplingIterationResult& result) {
    if (config.useInterfaceResidual) {
        return result.interfaceResidual < config.outerConvergenceTol;
    }
    return result.massConservationError < config.outerConvergenceTol;
}

void CouplingSolver::applyUnderRelaxation(SolverContext& context, double factor) {
    // 应用欠松弛处理
    // x_new = x_old + factor * (x_calculated - x_old)
    // 简化实现：调整节点压力
    for (auto& node : context.nodes) {
        if (node) {
            // 这里应该保存之前的压力值并进行松弛
            // 简化处理：不实际修改，仅记录
        }
    }
}

void CouplingSolver::extractInterfaceData(SolverContext& context, CouplingState& state) {
    state.clear();

    // 提取所有节点的压力作为界面数据
    for (const auto& node : context.nodes) {
        if (node) {
            state.interfacePressures.push_back(node->pressure);
            state.interfaceFlows.push_back(node->flowInjection);
            state.interfaceTemperatures.push_back(node->temperature);
        }
    }
}

void CouplingSolver::applyInterfaceData(SolverContext& context, const CouplingState& state) {
    // 应用界面数据到节点
    std::size_t idx = 0;
    for (auto& node : context.nodes) {
        if (node && idx < state.interfacePressures.size()) {
            node->pressure = state.interfacePressures[idx];
            node->flowInjection = state.interfaceFlows[idx];
            node->temperature = state.interfaceTemperatures[idx];
            ++idx;
        }
    }
}

double CouplingSolver::calculateInterfaceResidual(const CouplingState& state1,
                                                  const CouplingState& state2) {
    if (state1.interfacePressures.empty() || state2.interfacePressures.empty()) {
        return 0.0;
    }

    double residual = 0.0;
    std::size_t count = std::min(state1.interfacePressures.size(),
                                 state2.interfacePressures.size());

    for (std::size_t i = 0; i < count; ++i) {
        double diff = state1.interfacePressures[i] - state2.interfacePressures[i];
        residual += diff * diff;
    }

    return std::sqrt(residual / count);
}

double CouplingSolver::calculateMassConservationError(SolverContext& context) {
    // 计算质量守恒误差
    double totalInflow = 0.0;
    double totalOutflow = 0.0;

    for (const auto& node : context.nodes) {
        if (node) {
            if (node->flowInjection > 0) {
                totalInflow += node->flowInjection;
            } else {
                totalOutflow += std::abs(node->flowInjection);
            }
        }
    }

    double totalFlow = totalInflow + totalOutflow;
    if (totalFlow < 1e-12) {
        return 0.0;
    }

    return std::abs(totalInflow - totalOutflow) / totalFlow;
}
