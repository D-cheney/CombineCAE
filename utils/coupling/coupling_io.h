#ifndef COUPLING_IO_H
#define COUPLING_IO_H

#include "solver/fluid_solver/solver_context.h"
#include "coupling_config.h"

namespace CouplingIO {

bool applyCouplingInputs(SolverContext& context,
                         const CouplingConfig& config,
                         double time,
                         int iteration);

bool writeCouplingOutputs(const SolverContext& context,
                          const CouplingConfig& config,
                          double time,
                          int iteration);

} // namespace CouplingIO

#endif // COUPLING_IO_H
