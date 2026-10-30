#include "config.hpp"

#include <Preferences.h>
#include <cstdlib>
#include <cmath>
#include <cstring>

// ============================================================
// Internal state
// ============================================================

namespace {

constexpr char NVS_NAMESPACE[] = "uwb-anchor";

AnchorConfig config;

bool configLoaded = false;

// Serial command buffer.
constexpr size_t COMMAND_BUFFER_SIZE = 128;
char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;
bool commandOverflow = false;

// ============================================================
// Validation
// ============================================================

bool validAnchorId(unsigned long id) {
    // Reserve 0 and 65535 for special/invalid addressing.
    return id >= 1 && id <= 65534;
}

bool validCoordinate(float value) {
    // Reject NaN, infinity, and unreasonable coordinates.
    return std::isfinite(value) && std::fabs(value) <= 100000.0f;
}

// Parse a floating-point number and reject trailing characters.
bool parseFloatToken(const char* token, float& result) {
    if (token == nullptr || *token == '\0') {
        return false;
    }

    char* end = nullptr;
    float value = std::strtof(token, &end);

    if (end == token || *end != '\0') {
        return false;
    }

    if (!validCoordinate(value)) {
        return false;
    }

    result = value;
    return true;
}

// Parse an unsigned integer and reject signs or trailing characters.
bool parseIdToken(const char* token, unsigned long& result) {
    if (token == nullptr || *token == '\0') {
        return false;
    }

    for (const char* p = token; *p != '\0'; ++p) {
        if (*p < '0' || *p > '9') {
            return false;
        }
    }

    char* end = nullptr;
    unsigned long value = std::strtoul(token, &end, 10);

    if (end == token || *end != '\0') {
        return false;
    }

    if (!validAnchorId(value)) {
        return false;
    }

    result = value;
    return true;
}

// ============================================================
// Persistent storage
// ============================================================

bool persistConfig(const AnchorConfig& candidate) {
    Preferences preferences;

    if (!preferences.begin(NVS_NAMESPACE, false)) {
        Serial.println("[CONFIG] ERROR: Could not open NVS.");
        return false;
    }

    bool success = true;

    success &= preferences.putUShort("id", candidate.id) > 0;
    success &= preferences.putFloat("x", candidate.x) > 0;
    success &= preferences.putFloat("y", candidate.y) > 0;
    success &= preferences.putFloat("z", candidate.z) > 0;

    success &= preferences.putBool(
        "id_set",
        candidate.idConfigured
    ) > 0;

    success &= preferences.putBool(
        "pos_set",
        candidate.positionConfigured
    ) > 0;

    preferences.end();

    if (!success) {
        Serial.println("[CONFIG] ERROR: Failed to save configuration.");
        return false;
    }

    return true;
}

// Save a proposed configuration and commit it to RAM only on success.
bool commitConfig(const AnchorConfig& candidate) {
    if (!persistConfig(candidate)) {
        return false;
    }

    config = candidate;
    configLoaded = true;

    return true;
}

// ============================================================
// Command processing
// ============================================================

void processCommand(char* line) {
    // Tokenize the command by whitespace.
    constexpr size_t MAX_TOKENS = 6;

    char* tokens[MAX_TOKENS];
    size_t count = 0;

    char* token = std::strtok(line, " \t");

    while (token != nullptr && count < MAX_TOKENS) {
        tokens[count++] = token;
        token = std::strtok(nullptr, " \t");
    }

    if (count == 0) {
        return;
    }

    // Support both:
    //   set id 1
    //   id 1
    //   set pos 0 0 2.5
    //   pos 0 0 2.5
    const char* operation = tokens[0];
    size_t argumentIndex = 1;

    if (std::strcmp(tokens[0], "set") == 0) {
        if (count < 2) {
            Serial.println("[CONFIG] Usage: set id <id>");
            Serial.println("         set pos <x> <y> <z>");
            return;
        }

        operation = tokens[1];
        argumentIndex = 2;
    }

    // --------------------------------------------------------
    // Set anchor ID
    // --------------------------------------------------------

    if (std::strcmp(operation, "id") == 0) {
        if (count != argumentIndex + 1) {
            Serial.println("[CONFIG] Usage: set id <id>");
            return;
        }

        unsigned long id = 0;

        if (!parseIdToken(tokens[argumentIndex], id)) {
            Serial.println(
                "[CONFIG] Invalid ID. Use an integer from 1 to 65534."
            );
            return;
        }

        AnchorConfig candidate = config;
        candidate.id = static_cast<uint16_t>(id);
        candidate.idConfigured = true;

        if (!commitConfig(candidate)) {
            Serial.println("[CONFIG] ID was not changed.");
            return;
        }

        Serial.printf(
            "[CONFIG] Anchor ID saved: %u\n",
            static_cast<unsigned int>(config.id)
        );

        return;
    }

    // --------------------------------------------------------
    // Set anchor coordinates
    // --------------------------------------------------------

    if (std::strcmp(operation, "pos") == 0) {
        if (count != argumentIndex + 3) {
            Serial.println("[CONFIG] Usage: set pos <x> <y> <z>");
            return;
        }

        float x, y, z;

        if (!parseFloatToken(tokens[argumentIndex], x) ||
            !parseFloatToken(tokens[argumentIndex + 1], y) ||
            !parseFloatToken(tokens[argumentIndex + 2], z)) {
            Serial.println(
                "[CONFIG] Invalid coordinates. Use finite numbers in meters."
            );
            return;
        }

        AnchorConfig candidate = config;

        candidate.x = x;
        candidate.y = y;
        candidate.z = z;
        candidate.positionConfigured = true;

        if (!commitConfig(candidate)) {
            Serial.println("[CONFIG] Position was not changed.");
            return;
        }

        Serial.printf(
            "[CONFIG] Position saved: X=%.3f m, Y=%.3f m, Z=%.3f m\n",
            config.x,
            config.y,
            config.z
        );

        return;
    }

    // --------------------------------------------------------
    // Show configuration
    // --------------------------------------------------------

    if (std::strcmp(operation, "show") == 0 && count == 1) {
        printConfig();
        return;
    }

    // --------------------------------------------------------
    // Help
    // --------------------------------------------------------

    if (std::strcmp(operation, "help") == 0 && count == 1) {
        printConfigHelp();
        return;
    }

    Serial.println("[CONFIG] Unknown command. Enter 'help'.");
}

} // namespace

// ============================================================
// Public API
// ============================================================

bool loadConfig() {
    Preferences preferences;

    // Open in read-write mode so a new namespace can be created.
    if (!preferences.begin(NVS_NAMESPACE, false)) {
        Serial.println("[CONFIG] ERROR: Could not open NVS.");
        configLoaded = false;
        return false;
    }

    AnchorConfig candidate;

    candidate.idConfigured = preferences.getBool("id_set", false);
    candidate.positionConfigured = preferences.getBool("pos_set", false);

    candidate.id = preferences.getUShort("id", 0);
    candidate.x = preferences.getFloat("x", 0.0f);
    candidate.y = preferences.getFloat("y", 0.0f);
    candidate.z = preferences.getFloat("z", 0.0f);

    preferences.end();

    // Validate stored data.
    if (candidate.idConfigured &&
        !validAnchorId(candidate.id)) {
        candidate.idConfigured = false;
    }

    if (candidate.positionConfigured &&
        (!validCoordinate(candidate.x) ||
         !validCoordinate(candidate.y) ||
         !validCoordinate(candidate.z))) {
        candidate.positionConfigured = false;
    }

    config = candidate;
    configLoaded = true;

    Serial.println("[CONFIG] Configuration loaded from NVS.");
    printConfig();

    return true;
}

bool saveConfig() {
    if (!configLoaded) {
        Serial.println(
            "[CONFIG] ERROR: Load configuration before saving."
        );
        return false;
    }

    return persistConfig(config);
}

void processSerialCommands() {
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        // CR and LF terminate a command.
        if (c == '\r' || c == '\n') {
            if (commandOverflow) {
                Serial.println(
                    "[CONFIG] ERROR: Command too long. Try again."
                );
            } else if (commandLength > 0) {
                commandBuffer[commandLength] = '\0';
                processCommand(commandBuffer);
            }

            commandLength = 0;
            commandOverflow = false;
            continue;
        }

        if (commandOverflow) {
            continue;
        }

        if (commandLength < COMMAND_BUFFER_SIZE - 1) {
            commandBuffer[commandLength++] = c;
        } else {
            commandOverflow = true;
        }
    }
}

const AnchorConfig& getConfig() {
    return config;
}

bool isConfigured() {
    return configLoaded &&
           config.idConfigured &&
           config.positionConfigured;
}

void printConfig() {
    Serial.println();
    Serial.println("========== UWB ANCHOR CONFIG ==========");

    if (config.idConfigured) {
        Serial.printf(
            "Anchor ID : %u\n",
            static_cast<unsigned int>(config.id)
        );
    } else {
        Serial.println("Anchor ID : NOT CONFIGURED");
    }

    if (config.positionConfigured) {
        Serial.printf(
            "Position  : X=%.3f, Y=%.3f, Z=%.3f m\n",
            config.x,
            config.y,
            config.z
        );
    } else {
        Serial.println("Position  : NOT CONFIGURED");
    }

    Serial.printf(
        "Ready     : %s\n",
        isConfigured() ? "YES" : "NO"
    );

    Serial.println("=======================================");
    Serial.println();
}

void printConfigHelp() {
    Serial.println();
    Serial.println("========== UWB CONFIG COMMANDS ==========");
    Serial.println("set id <id>           Set anchor ID");
    Serial.println("set pos <x> <y> <z>   Set coordinates in meters");
    Serial.println("show                  Display current configuration");
    Serial.println("help                  Show this help");
    Serial.println();
    Serial.println("Short forms:");
    Serial.println("id <id>");
    Serial.println("pos <x> <y> <z>");
    Serial.println("=========================================");
    Serial.println();
}