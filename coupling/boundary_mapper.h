/**
 * @file boundary_mapper.h
 * @brief Maps boundary values between 1D components and 2D boundaries.
 *
 * Responsibilities:
 * - Map 1D component boundary values to 2D boundary conditions
 * - Map 2D boundary results back to 1D component parameters
 * - Validate mapping consistency
 * - Report mapping statistics
 */

#ifndef BOUNDARY_MAPPER_H
#define BOUNDARY_MAPPER_H

#include "coupling/coupling_types.h"

#include <string>
#include <vector>
#include <map>
#include <functional>

// ============================================================================
// MappedBoundaryValue: A single mapped value
// ============================================================================
struct MappedBoundaryValue {
    std::string sourceName;       // Source name (1D component field)
    std::string targetName;       // Target name (2D boundary field)
    double value;                 // Mapped value
    std::string unit;             // Unit string
    bool valid;                   // Whether mapping is valid

    MappedBoundaryValue()
        : value(0.0), valid(false) {}
};

// ============================================================================
// BoundaryMapper
// ============================================================================
class BoundaryMapper {
public:
    // Callback type for reading 1D values
    using Read1DValueFunc = std::function<double(const std::string& componentName, const std::string& fieldName)>;

    // Callback type for writing 1D values
    using Write1DValueFunc = std::function<void(const std::string& componentName, const std::string& fieldName, double value)>;

private:
    Read1DValueFunc read1D_;
    Write1DValueFunc write1D_;
    mutable std::vector<MappedBoundaryValue> lastMapping_;
    bool verbose_;

public:
    BoundaryMapper()
        : verbose_(false) {}

    // ========================================================================
    // Configuration
    // ========================================================================

    void setRead1DFunction(Read1DValueFunc func) { read1D_ = func; }
    void setWrite1DFunction(Write1DValueFunc func) { write1D_ = func; }
    void setVerbose(bool verbose) { verbose_ = verbose; }

    // ========================================================================
    // 1D -> 2D Mapping
    // ========================================================================

    /**
     * Map 1D values to 2D boundary conditions for a coupling interface.
     * @param iface Coupling interface definition
     * @param case2D Target 2D case
     * @return Map of boundary tag -> boundary value
     */
    std::map<int, Boundary2D> mapTo2D(const CouplingInterface& iface,
                                       const TwoDCase& case2D) const {
        std::map<int, Boundary2D> result;

        if (!read1D_) {
            if (verbose_) {
                printf("BoundaryMapper: No 1D read function set\n");
            }
            return result;
        }

        // Find target boundary in 2D case
        const Boundary2D* targetBoundary = case2D.getBoundary(iface.twodBoundaryTag);
        if (!targetBoundary) {
            if (verbose_) {
                printf("BoundaryMapper: Boundary tag %d not found in case %s\n",
                       iface.twodBoundaryTag, case2D.id.c_str());
            }
            return result;
        }

        Boundary2D mappedBoundary = *targetBoundary;
        MappedBoundaryValue mv;
        mv.sourceName = iface.onedComponent + "." + iface.transferTo2D.sourceTemperature;
        mv.targetName = case2D.id + ".boundary." + std::to_string(iface.twodBoundaryTag);

        // Apply transfer type
        if (iface.transferTo2D.type == "dirichlet_temperature") {
            // Read temperature from 1D
            double T = read1D_(iface.onedComponent, iface.transferTo2D.sourceTemperature);
            mappedBoundary.type = "dirichlet";
            mappedBoundary.value = T;
            mv.value = T;
            mv.unit = "K";
            mv.valid = true;
        }
        else if (iface.transferTo2D.type == "robin_temperature") {
            // Read fluid temperature and h coefficient from 1D
            double TFluid = read1D_(iface.onedComponent, iface.transferTo2D.sourceTemperature);
            double h = read1D_(iface.onedComponent, iface.transferTo2D.sourceH);
            mappedBoundary.type = "robin";
            mappedBoundary.value = h;
            mappedBoundary.referenceValue = TFluid;
            mv.value = h;
            mv.unit = "W/(m2*K)";
            mv.valid = true;

            if (verbose_) {
                printf("BoundaryMapper: Robin T_f=%.2f K, h=%.2f W/(m2*K)\n", TFluid, h);
            }
        }
        else if (iface.transferTo2D.type == "dirichlet_pressure") {
            // Read pressure from 1D
            double p = read1D_(iface.onedComponent, iface.transferTo2D.sourcePressure);
            mappedBoundary.type = "dirichlet";
            mappedBoundary.value = p;
            mv.value = p;
            mv.unit = "Pa";
            mv.valid = true;
        }
        else if (iface.transferTo2D.type == "neumann_heatflow") {
            // Read heat flow from 1D
            double q = read1D_(iface.onedComponent, "heat_flow");
            mappedBoundary.type = "neumann";
            mappedBoundary.value = q;
            mv.value = q;
            mv.unit = "W/m2";
            mv.valid = true;
        }

        if (mv.valid) {
            result[iface.twodBoundaryTag] = mappedBoundary;
            lastMapping_.push_back(mv);
        }

        return result;
    }

    // ========================================================================
    // 2D -> 1D Feedback
    // ========================================================================

    /**
     * Extract feedback values from 2D results and write to 1D.
     * @param iface Coupling interface definition
     * @param result2D 2D simulation results
     */
    void mapFrom2D(const CouplingInterface& iface,
                   const TwoDResult& result2D) {
        if (!write1D_) {
            if (verbose_) {
                printf("BoundaryMapper: No 1D write function set\n");
            }
            return;
        }

        // Feedback heat flow
        if (!iface.feedbackTo1D.heatflow.empty()) {
            double qTotal = result2D.getScalar("total_heat_flow", 0.0);
            // Also check boundary-specific heat flow
            auto it = result2D.boundaryHeatFlow.find(iface.twodBoundaryTag);
            if (it != result2D.boundaryHeatFlow.end()) {
                qTotal = it->second;
            }
            write1D_(iface.onedComponent, iface.feedbackTo1D.heatflow, qTotal);
        }

        // Feedback average wall temperature
        if (!iface.feedbackTo1D.wallTemperatureAvg.empty()) {
            double Tavg = result2D.getScalar("avg_temperature", 0.0);
            auto it = result2D.boundaryAvgTemperature.find(iface.twodBoundaryTag);
            if (it != result2D.boundaryAvgTemperature.end()) {
                Tavg = it->second;
            }
            write1D_(iface.onedComponent, iface.feedbackTo1D.wallTemperatureAvg, Tavg);
        }

        // Feedback max wall temperature
        if (!iface.feedbackTo1D.wallTemperatureMax.empty()) {
            double Tmax = result2D.getScalar("max_temperature", 0.0);
            auto it = result2D.boundaryMaxTemperature.find(iface.twodBoundaryTag);
            if (it != result2D.boundaryMaxTemperature.end()) {
                Tmax = it->second;
            }
            write1D_(iface.onedComponent, iface.feedbackTo1D.wallTemperatureMax, Tmax);
        }

        // Feedback pressure drop
        if (!iface.feedbackTo1D.pressureDrop.empty()) {
            double dp = result2D.getScalar("pressure_drop", 0.0);
            auto it = result2D.boundaryPressureDrop.find(iface.twodBoundaryTag);
            if (it != result2D.boundaryPressureDrop.end()) {
                dp = it->second;
            }
            write1D_(iface.onedComponent, iface.feedbackTo1D.pressureDrop, dp);
        }

        // Feedback resistance coefficient
        if (!iface.feedbackTo1D.resistanceCoeff.empty()) {
            double K = result2D.getScalar("resistance_coefficient", 0.0);
            write1D_(iface.onedComponent, iface.feedbackTo1D.resistanceCoeff, K);
        }
    }

    // ========================================================================
    // Validation
    // ========================================================================

    /**
     * Validate that all coupling interfaces have valid mappings.
     */
    std::vector<std::string> validateMappings(
        const std::vector<CouplingInterface>& interfaces,
        const std::vector<TwoDCase>& cases) const {

        std::vector<std::string> errors;

        for (const auto& iface : interfaces) {
            if (!iface.enabled) continue;

            // Check 2D case exists
            bool caseFound = false;
            for (const auto& c : cases) {
                if (c.id == iface.twodCase) {
                    caseFound = true;
                    // Check boundary tag exists
                    if (!c.getBoundary(iface.twodBoundaryTag)) {
                        errors.push_back("Interface '" + iface.name +
                            "': boundary tag " + std::to_string(iface.twodBoundaryTag) +
                            " not found in case '" + iface.twodCase + "'");
                    }
                    break;
                }
            }
            if (!caseFound) {
                errors.push_back("Interface '" + iface.name +
                    "': 2D case '" + iface.twodCase + "' not found");
            }
        }

        return errors;
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    const std::vector<MappedBoundaryValue>& getLastMapping() const {
        return lastMapping_;
    }

    void clearLastMapping() {
        lastMapping_.clear();
    }

    /**
     * Print mapping report.
     */
    void printMappingReport() const {
        printf("\n=== Boundary Mapping Report ===\n");
        printf("Total mappings: %zu\n", lastMapping_.size());
        for (const auto& m : lastMapping_) {
            printf("  %s -> %s: %.6g %s [%s]\n",
                   m.sourceName.c_str(), m.targetName.c_str(),
                   m.value, m.unit.c_str(),
                   m.valid ? "OK" : "INVALID");
        }
        printf("===============================\n");
    }
};

#endif // BOUNDARY_MAPPER_H
