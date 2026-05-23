#include "coupling/twod_solver_interface.h"
#include "coupling/coupling_types.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>

// ============================================================================
// AnalyticalHeat2DSolver Implementation
// ============================================================================

AnalyticalHeat2DSolver::AnalyticalHeat2DSolver()
    : solved_(false), avgTemperature_(0.0), maxTemperature_(0.0) {}

bool AnalyticalHeat2DSolver::initialize(const TwoDCase& caseConfig) {
    caseConfig_ = caseConfig;
    solved_ = false;
    avgTemperature_ = 0.0;
    maxTemperature_ = 0.0;
    boundaryHeatFlows_.clear();
    boundaryAvgTemps_.clear();
    boundaryMaxTemps_.clear();
    return true;
}

void AnalyticalHeat2DSolver::applyBoundaries(const std::map<int, Boundary2D>& boundaries) {
    boundaries_ = boundaries;
}

bool AnalyticalHeat2DSolver::solve() {
    if (boundaries_.empty()) {
        std::cerr << "AnalyticalHeat2DSolver: No boundaries applied" << std::endl;
        return false;
    }

    // Simple analytical solution: linear interpolation between boundary temperatures
    double minT = 1e30, maxT = -1e30;
    double sumT = 0.0;
    int count = 0;

    for (const auto& pair : boundaries_) {
        const Boundary2D& b = pair.second;
        if (b.type == "dirichlet") {
            minT = std::min(minT, b.value);
            maxT = std::max(maxT, b.value);
            sumT += b.value;
            count++;

            // Approximate: avg = mean of boundary temps
            boundaryAvgTemps_[b.tag] = b.value;
            boundaryMaxTemps_[b.tag] = b.value;

            // Approximate heat flow: k * (T_hot - T_cold) / L * Area
            double k = 1.0;
            const Material2D* mat = caseConfig_.getMaterial("solid");
            if (mat) k = mat->thermalConductivity;

            double L = caseConfig_.geometry.width;
            if (L < 1e-12) L = 1.0;
            double area = caseConfig_.geometry.height * caseConfig_.geometry.thickness;
            if (area < 1e-12) area = 1.0;

            // Simple estimate
            double qEstimate = k * std::abs(b.value - (minT + maxT) / 2.0) / L * area;
            boundaryHeatFlows_[b.tag] = qEstimate;
        }
        else if (b.type == "robin") {
            double h = b.value;
            double TFluid = b.referenceValue;
            // Estimate wall temperature
            double k = 1.0;
            const Material2D* mat = caseConfig_.getMaterial("solid");
            if (mat) k = mat->thermalConductivity;

            double L = caseConfig_.geometry.width / 2.0;
            if (L < 1e-12) L = 1.0;

            // T_wall ~ T_fluid / (1 + h*L/k)
            double Twall = TFluid / (1.0 + h * L / k);
            boundaryAvgTemps_[b.tag] = Twall;
            boundaryMaxTemps_[b.tag] = Twall;
            boundaryHeatFlows_[b.tag] = h * (TFluid - Twall) * caseConfig_.geometry.height;

            minT = std::min(minT, Twall);
            maxT = std::max(maxT, Twall);
            sumT += Twall;
            count++;
        }
    }

    if (count > 0) {
        avgTemperature_ = sumT / count;
    }
    maxTemperature_ = maxT;
    solved_ = true;
    return true;
}

TwoDResult AnalyticalHeat2DSolver::extractResults() const {
    TwoDResult result;
    result.caseId = caseConfig_.id;
    result.success = solved_;
    result.physics = "heat";

    if (solved_) {
        result.setScalar("avg_temperature", avgTemperature_);
        result.setScalar("max_temperature", maxTemperature_);
        result.setScalar("min_temperature", avgTemperature_ - (maxTemperature_ - avgTemperature_));

        double totalQ = 0.0;
        for (const auto& pair : boundaryHeatFlows_) {
            totalQ += pair.second;
        }
        result.setScalar("total_heat_flow", totalQ);

        result.boundaryHeatFlow.clear();
        for (const auto& p : boundaryHeatFlows_) result.boundaryHeatFlow[p.first] = p.second;
        result.boundaryAvgTemperature.clear();
        for (const auto& p : boundaryAvgTemps_) result.boundaryAvgTemperature[p.first] = p.second;
        result.boundaryMaxTemperature.clear();
        for (const auto& p : boundaryMaxTemps_) result.boundaryMaxTemperature[p.first] = p.second;
    }

    return result;
}

double AnalyticalHeat2DSolver::getTemperature(double x, double y) const {
    (void)x; (void)y;
    return avgTemperature_;
}

double AnalyticalHeat2DSolver::getMaxTemperature() const {
    return maxTemperature_;
}

double AnalyticalHeat2DSolver::getAvgTemperature() const {
    return avgTemperature_;
}

double AnalyticalHeat2DSolver::getBoundaryHeatFlow(int boundaryTag) const {
    auto it = boundaryHeatFlows_.find(boundaryTag);
    return (it != boundaryHeatFlows_.end()) ? it->second : 0.0;
}

double AnalyticalHeat2DSolver::getBoundaryAvgTemperature(int boundaryTag) const {
    auto it = boundaryAvgTemps_.find(boundaryTag);
    return (it != boundaryAvgTemps_.end()) ? it->second : 0.0;
}

double AnalyticalHeat2DSolver::getBoundaryMaxTemperature(int boundaryTag) const {
    auto it = boundaryMaxTemps_.find(boundaryTag);
    return (it != boundaryMaxTemps_.end()) ? it->second : 0.0;
}

bool AnalyticalHeat2DSolver::exportField(const std::string& filePath) const {
    std::ofstream out(filePath.c_str());
    if (!out.is_open()) return false;

    out << "# Analytical 2D Heat Result\n";
    out << "# Case: " << caseConfig_.id << "\n";
    out << "# Avg T: " << avgTemperature_ << " K\n";
    out << "# Max T: " << maxTemperature_ << " K\n";
    out << "# Boundaries:\n";
    for (const auto& pair : boundaryHeatFlows_) {
        out << "#   Tag " << pair.first << ": Q=" << pair.second << " W\n";
    }
    return true;
}

void AnalyticalHeat2DSolver::reset() {
    solved_ = false;
    avgTemperature_ = 0.0;
    maxTemperature_ = 0.0;
    boundaries_.clear();
    boundaryHeatFlows_.clear();
    boundaryAvgTemps_.clear();
    boundaryMaxTemps_.clear();
}

// ============================================================================
// AnalyticalPressure2DSolver Implementation
// ============================================================================

AnalyticalPressure2DSolver::AnalyticalPressure2DSolver()
    : solved_(false), avgPressure_(0.0) {}

bool AnalyticalPressure2DSolver::initialize(const TwoDCase& caseConfig) {
    caseConfig_ = caseConfig;
    solved_ = false;
    avgPressure_ = 0.0;
    boundaryAvgPressures_.clear();
    return true;
}

void AnalyticalPressure2DSolver::applyBoundaries(const std::map<int, Boundary2D>& boundaries) {
    boundaries_ = boundaries;
}

bool AnalyticalPressure2DSolver::solve() {
    if (boundaries_.empty()) {
        std::cerr << "AnalyticalPressure2DSolver: No boundaries applied" << std::endl;
        return false;
    }

    double sumP = 0.0;
    int count = 0;

    for (const auto& pair : boundaries_) {
        const Boundary2D& b = pair.second;
        if (b.type == "dirichlet") {
            boundaryAvgPressures_[b.tag] = b.value;
            sumP += b.value;
            count++;
        }
    }

    if (count > 0) {
        avgPressure_ = sumP / count;
    }
    solved_ = true;
    return true;
}

TwoDResult AnalyticalPressure2DSolver::extractResults() const {
    TwoDResult result;
    result.caseId = caseConfig_.id;
    result.success = solved_;
    result.physics = "pressure";

    if (solved_) {
        result.setScalar("avg_pressure", avgPressure_);

        // Find max and min pressure boundaries for drop estimate
        double maxP = -1e30, minP = 1e30;
        for (const auto& pair : boundaryAvgPressures_) {
            maxP = std::max(maxP, pair.second);
            minP = std::min(minP, pair.second);
        }
        result.setScalar("pressure_drop", maxP - minP);
        result.boundaryAvgPressure.clear();
        for (const auto& p : boundaryAvgPressures_) result.boundaryAvgPressure[p.first] = p.second;
    }

    return result;
}

double AnalyticalPressure2DSolver::getPressure(double x, double y) const {
    (void)x; (void)y;
    return avgPressure_;
}

double AnalyticalPressure2DSolver::getAvgPressure() const {
    return avgPressure_;
}

double AnalyticalPressure2DSolver::getBoundaryAvgPressure(int boundaryTag) const {
    auto it = boundaryAvgPressures_.find(boundaryTag);
    return (it != boundaryAvgPressures_.end()) ? it->second : 0.0;
}

double AnalyticalPressure2DSolver::getPressureDrop(int inletTag, int outletTag) const {
    double pIn = getBoundaryAvgPressure(inletTag);
    double pOut = getBoundaryAvgPressure(outletTag);
    return pIn - pOut;
}

bool AnalyticalPressure2DSolver::exportField(const std::string& filePath) const {
    std::ofstream out(filePath.c_str());
    if (!out.is_open()) return false;

    out << "# Analytical 2D Pressure Result\n";
    out << "# Case: " << caseConfig_.id << "\n";
    out << "# Avg P: " << avgPressure_ << " Pa\n";
    return true;
}

void AnalyticalPressure2DSolver::reset() {
    solved_ = false;
    avgPressure_ = 0.0;
    boundaries_.clear();
    boundaryAvgPressures_.clear();
}

// ============================================================================
// SolverFactory Implementation
// ============================================================================

std::unique_ptr<Heat2DSolver> SolverFactory::createHeatSolver(BackendType type) {
    (void)type; // For now, always use analytical
    return std::unique_ptr<Heat2DSolver>(new AnalyticalHeat2DSolver());
}

std::unique_ptr<Pressure2DSolver> SolverFactory::createPressureSolver(BackendType type) {
    (void)type; // For now, always use analytical
    return std::unique_ptr<Pressure2DSolver>(new AnalyticalPressure2DSolver());
}
