/**
 * @file relaxation_controller.h
 * @brief Relaxation controller for 1D-2D coupling iterations.
 *
 * Supports:
 * - Fixed relaxation: x_new = (1-alpha) * x_old + alpha * x_calc
 * - Aitken acceleration (optional)
 * - Per-variable relaxation factors
 */

#ifndef RELAXATION_CONTROLLER_H
#define RELAXATION_CONTROLLER_H

#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>

// ============================================================================
// RelaxationController
// ============================================================================
class RelaxationController {
public:
    enum class Mode {
        Fixed,          // Fixed relaxation factor
        Aitken          // Aitken dynamic relaxation
    };

private:
    Mode mode_;
    double fixedAlpha_;         // Fixed relaxation factor
    double minAlpha_;           // Minimum relaxation factor
    double maxAlpha_;           // Maximum relaxation factor
    bool enabled_;

    // Aitken state
    std::unordered_map<std::string, double> previousValues_;
    std::unordered_map<std::string, double> previousDeltas_;
    std::unordered_map<std::string, double> aitkenAlpha_;

public:
    RelaxationController()
        : mode_(Mode::Fixed), fixedAlpha_(0.5), minAlpha_(0.1), maxAlpha_(0.9),
          enabled_(true) {}

    // ========================================================================
    // Configuration
    // ========================================================================

    void setMode(Mode mode) { mode_ = mode; }
    void setFixedAlpha(double alpha) {
        fixedAlpha_ = std::max(0.0, std::min(1.0, alpha));
    }
    void setAlphaRange(double minAlpha, double maxAlpha) {
        minAlpha_ = std::max(0.0, minAlpha);
        maxAlpha_ = std::min(1.0, maxAlpha);
    }
    void setEnabled(bool enabled) { enabled_ = enabled; }

    Mode getMode() const { return mode_; }
    double getFixedAlpha() const { return fixedAlpha_; }
    bool isEnabled() const { return enabled_; }

    // ========================================================================
    // Reset state
    // ========================================================================

    void reset() {
        previousValues_.clear();
        previousDeltas_.clear();
        aitkenAlpha_.clear();
    }

    void resetVariable(const std::string& name) {
        previousValues_.erase(name);
        previousDeltas_.erase(name);
        aitkenAlpha_.erase(name);
    }

    // ========================================================================
    // Apply relaxation
    // ========================================================================

    /**
     * Apply relaxation to a scalar value.
     * @param name Variable name (for per-variable state)
     * @param oldValue Previous iteration value
     * @param calculatedValue Newly calculated value
     * @return Relaxed value
     */
    double relax(const std::string& name, double oldValue, double calculatedValue) {
        if (!enabled_) return calculatedValue;

        switch (mode_) {
            case Mode::Fixed:
                return relaxFixed(oldValue, calculatedValue);
            case Mode::Aitken:
                return relaxAitken(name, oldValue, calculatedValue);
            default:
                return relaxFixed(oldValue, calculatedValue);
        }
    }

    /**
     * Apply relaxation to a vector of values.
     */
    std::vector<double> relaxVector(const std::string& name,
                                     const std::vector<double>& oldValues,
                                     const std::vector<double>& calcValues) {
        std::vector<double> result;
        result.reserve(oldValues.size());
        for (size_t i = 0; i < oldValues.size(); ++i) {
            std::string varName = name + "_" + std::to_string(i);
            result.push_back(relax(varName, oldValues[i], calcValues[i]));
        }
        return result;
    }

    // ========================================================================
    // Convergence check
    // ========================================================================

    /**
     * Check if change is within tolerance.
     */
    static bool checkConvergence(double oldValue, double newValue, double tolerance) {
        double change = std::abs(newValue - oldValue);
        double refValue = std::max(std::abs(oldValue), 1e-12);
        return (change / refValue) < tolerance;
    }

    /**
     * Check absolute change.
     */
    static bool checkAbsoluteConvergence(double oldValue, double newValue, double tolerance) {
        return std::abs(newValue - oldValue) < tolerance;
    }

private:
    double relaxFixed(double oldValue, double calculatedValue) const {
        return (1.0 - fixedAlpha_) * oldValue + fixedAlpha_ * calculatedValue;
    }

    double relaxAitken(const std::string& name, double oldValue, double calculatedValue) {
        double delta = calculatedValue - oldValue;

        // First iteration: use fixed alpha
        if (previousValues_.find(name) == previousValues_.end()) {
            previousValues_[name] = calculatedValue;
            previousDeltas_[name] = delta;
            aitkenAlpha_[name] = fixedAlpha_;
            return (1.0 - fixedAlpha_) * oldValue + fixedAlpha_ * calculatedValue;
        }

        double prevDelta = previousDeltas_[name];
        double deltaDelta = delta - prevDelta;

        // Compute Aitken alpha
        double alpha = fixedAlpha_;
        if (std::abs(deltaDelta) > 1e-15) {
            alpha = -prevDelta * prevDelta / deltaDelta;
            alpha = std::max(minAlpha_, std::min(maxAlpha_, alpha));
        }

        // Store state for next iteration
        previousDeltas_[name] = delta;
        previousValues_[name] = calculatedValue;
        aitkenAlpha_[name] = alpha;

        return (1.0 - alpha) * oldValue + alpha * calculatedValue;
    }
};

#endif // RELAXATION_CONTROLLER_H
