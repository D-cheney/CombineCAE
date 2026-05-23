#include <cassert>
#include <fstream>
#include <sstream>
#include <iostream>

#include "core/material_library.h"
#include "core/material_properties.h"

// Simple file reading helper
static std::string readFileContent(const std::string &path)
{
    std::ifstream file(path);
    if (!file.is_open())
        return "";
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

bool testMaterialLibraryBasic()
{
    std::cout << "[TEST] Material Library Basic Operations" << std::endl;

    MaterialLibrary lib;

    // Test 1: Add and retrieve a single material
    MaterialProperties water;
    water.density = 998.2;
    water.dynamicViscosity = 1.002e-3;
    bool added = lib.addMaterial("water", water);
    assert(added && "Failed to add material");
    assert(lib.hasMaterial("water") && "Material not found");

    MaterialProperties retrieved = lib.getMaterial("water");
    assert(retrieved.density == 998.2 && "Material density mismatch");
    std::cout << "  ✓ Single material add/retrieve" << std::endl;

    // Test 2: Case-insensitive lookup
    assert(lib.hasMaterial("WATER") && "Case-insensitive lookup failed");
    std::cout << "  ✓ Case-insensitive lookup" << std::endl;

    // Test 3: List materials
    auto names = lib.listMaterialNames();
    assert(names.size() == 1 && "List size mismatch");
    std::cout << "  ✓ List material names" << std::endl;

    return true;
}

bool testMaterialLibraryFromJson()
{
    std::cout << "[TEST] Material Library JSON Load" << std::endl;

    MaterialLibrary lib;

    // Read the water.json file
    std::string waterJson = readFileContent("fluids/water.json");
    if (waterJson.empty())
    {
        std::cerr << "  ✗ Could not read fluids/water.json" << std::endl;
        return false;
    }

    // Load materials from JSON
    bool loaded = lib.loadFromJsonString(waterJson);
    assert(loaded && "Failed to load materials from JSON");
    std::cout << "  ✓ Load materials from JSON" << std::endl;

    // Test 2: Verify materials exist
    assert(lib.hasMaterial("water_standard") && "water_standard not found");
    assert(lib.hasMaterial("glycol_blend") && "glycol_blend not found");
    std::cout << "  ✓ Materials loaded from JSON" << std::endl;

    // Test 3: Retrieve and check interpolated material
    MaterialProperties water20 = lib.getMaterial("water_standard", 20.0);
    assert(water20.density > 0 && "Material not loaded properly");
    std::cout << "  ✓ Retrieve materials with temperature" << std::endl;

    // Test 4: List loaded materials
    auto names = lib.listMaterialNames();
    assert(names.size() == 2 && "Expected 2 materials");
    std::cout << "  ✓ List loaded materials" << std::endl;

    return true;
}

bool testMaterialInterpolation()
{
    std::cout << "[TEST] Material Temperature Interpolation" << std::endl;

    std::vector<double> temps = {20.0, 40.0, 60.0};
    std::vector<MaterialProperties> props;

    for (int i = 0; i < 3; i++)
    {
        MaterialProperties mp;
        mp.density = 1000.0 - i * 10.0;
        mp.dynamicViscosity = 0.001 - i * 0.0002;
        props.push_back(mp);
    }

    MaterialLibrary lib;
    bool added = lib.addInterpolatedMaterial("test_fluid", temps, props);
    assert(added && "Failed to add interpolated material");
    std::cout << "  ✓ Add interpolated material" << std::endl;

    // Retrieve at exact temperature
    MaterialProperties at20 = lib.getMaterial("test_fluid", 20.0);
    assert(at20.density == 1000.0 && "Interpolation at exact point failed");
    std::cout << "  ✓ Retrieve at exact temperature" << std::endl;

    // Retrieve at interpolated temperature
    MaterialProperties at30 = lib.getMaterial("test_fluid", 30.0);
    assert(at30.density > 990.0 && at30.density < 1000.0 && "Interpolation out of range");
    std::cout << "  ✓ Retrieve at interpolated temperature" << std::endl;

    return true;
}

int main()
{
    std::cout << "=== Material Library Unit Tests ===" << std::endl;
    std::cout << std::endl;

    int tests_passed = 0;
    int tests_failed = 0;

    try
    {
        if (testMaterialLibraryBasic())
            tests_passed++;
        else
            tests_failed++;
    }
    catch (const std::exception &e)
    {
        std::cerr << "  ✗ Exception: " << e.what() << std::endl;
        tests_failed++;
    }

    std::cout << std::endl;

    try
    {
        if (testMaterialLibraryFromJson())
            tests_passed++;
        else
            tests_failed++;
    }
    catch (const std::exception &e)
    {
        std::cerr << "  ✗ Exception: " << e.what() << std::endl;
        tests_failed++;
    }

    std::cout << std::endl;

    try
    {
        if (testMaterialInterpolation())
            tests_passed++;
        else
            tests_failed++;
    }
    catch (const std::exception &e)
    {
        std::cerr << "  ✗ Exception: " << e.what() << std::endl;
        tests_failed++;
    }

    std::cout << std::endl;
    std::cout << "=== Results ===" << std::endl;
    std::cout << "Passed: " << tests_passed << std::endl;
    std::cout << "Failed: " << tests_failed << std::endl;

    return tests_failed == 0 ? 0 : 1;
}
