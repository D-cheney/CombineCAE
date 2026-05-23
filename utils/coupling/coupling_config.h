#ifndef COUPLING_CONFIG_H
#define COUPLING_CONFIG_H

#include <string>

enum class CouplingMode {
    None,           // 无耦合
    Weak,           // 弱耦合（交替推进）
    Strong          // 强耦合（联合求解）
};

struct CouplingConfig {
    bool enabled;
    CouplingMode mode;
    std::string inputFile;
    std::string outputFile;
    bool readEveryIteration;
    bool writeEveryIteration;

    // 弱耦合参数
    int maxOuterIterations;        // 最大外迭代次数
    double outerConvergenceTol;    // 外迭代收敛容差
    double underRelaxationFactor;  // 松弛因子 (0.5-0.9)
    bool useInterfaceResidual;     // 使用界面残差判断收敛

    // 强耦合参数（预留）
    bool buildJointJacobian;       // 构建联合Jacobian矩阵
    bool solveSimultaneously;      // 同时求解1D-2D系统

    CouplingConfig()
        : enabled(false),
          mode(CouplingMode::None),
          inputFile("coupling_input.json"),
          outputFile("coupling_output.json"),
          readEveryIteration(true),
          writeEveryIteration(true),
          maxOuterIterations(50),
          outerConvergenceTol(1e-6),
          underRelaxationFactor(0.7),
          useInterfaceResidual(true),
          buildJointJacobian(false),
          solveSimultaneously(false) {}
};

#endif // COUPLING_CONFIG_H
