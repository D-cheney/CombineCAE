#ifndef JSON_SCHEMA_VALIDATOR_H
#define JSON_SCHEMA_VALIDATOR_H

#include <string>
#include <vector>
#include <map>
#include <set>

namespace NetworkIO
{

    /**
     * @brief Validation error information
     */
    struct ValidationError
    {
        std::string path;          ///< JSON path where error occurred (e.g., "components[0].type")
        std::string message;       ///< Human-readable error description
        std::string expectedValue; ///< Expected value/format (if applicable)
        std::string actualValue;   ///< Actual value found

        ValidationError(const std::string &p, const std::string &msg,
                        const std::string &expected = "", const std::string &actual = "")
            : path(p), message(msg), expectedValue(expected), actualValue(actual) {}

        std::string toString() const
        {
            std::string result = "At '" + path + "': " + message;
            if (!expectedValue.empty())
            {
                result += " (expected: " + expectedValue + ")";
            }
            if (!actualValue.empty())
            {
                result += " (got: " + actualValue + ")";
            }
            return result;
        }
    };

    /**
     * @brief JSON Schema validator for network configuration files
     *
     * Provides basic validation of network simulation config files according to schema rules.
     * Supports:
     * - Required/optional field checking
     * - Enum validation (allowed values)
     * - Type validation (string, number, integer)
     * - Array element validation
     */
    class JsonSchemaValidator
    {
    public:
        JsonSchemaValidator() = default;

        /**
         * @brief Validate a JSON configuration object string
         *
         * @param jsonString The JSON content to validate
         * @param errors Output vector for validation errors
         * @return true if validation succeeds, false if validation fails
         */
        bool validate(const std::string &jsonString, std::vector<ValidationError> &errors);

        /**
         * @brief Get list of valid component types
         * @return Vector of valid component type strings
         */
        static std::vector<std::string> getValidComponentTypes();

        /**
         * @brief Get list of valid solver methods
         * @return Vector of valid solver method names
         */
        static std::vector<std::string> getValidSolverMethods();

        /**
         * @brief Check if a value is in the allowed set
         *
         * @param value The value to check
         * @param allowed Set of allowed values
         * @return true if value is in allowed set
         */
        static bool isEnumValid(const std::string &value, const std::set<std::string> &allowed);

    private:
        // Internal validation helpers
        bool validateTopLevel(const std::string &jsonString, std::vector<ValidationError> &errors);
        bool validateComponentArray(const std::string &componentSection, std::vector<ValidationError> &errors);
        bool validateConnectionArray(const std::string &connectionSection, std::vector<ValidationError> &errors);
    };

    /**
     * @brief Format validation errors for user display
     *
     * @param errors Vector of validation errors
     * @return Formatted error string with all errors listed
     */
    std::string formatValidationErrors(const std::vector<ValidationError> &errors);

} // namespace NetworkIO

#endif // JSON_SCHEMA_VALIDATOR_H
