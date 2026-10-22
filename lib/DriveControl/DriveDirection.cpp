#include <DriveDirection.h>
#include <string.h>

bool parseDriveDirection(const char* name, DriveDirection& out) {
    for (const DriveDirectionName& entry : kDriveDirectionNames) {
        if (strcmp(name, entry.name) == 0) {
            out = entry.direction;
            return true;
        }
    }
    return false;
}
