#include <Adafruit_VL53L0X.h>
#include <DistanceSensor.h>

DistanceSensor::DistanceSensor(int address, uint8_t shtPin) : address{ address }, shtPin{ shtPin } {
    this->sensor = new Adafruit_VL53L0X();
}

bool DistanceSensor::setAddress() {
    digitalWrite(shtPin, HIGH);

    if (!sensor->begin(address)) {
        Serial.printf("Failed to boot VL53L0X at address %#x (XSHUT pin %u), marking offline\n", address, shtPin);
        // Hold the dead sensor in reset so it doesn't answer on the default address
        // while the remaining sensors are being assigned theirs.
        digitalWrite(shtPin, LOW);
        online = false;
        return false;
    }
    online = true;
    return true;
}

uint16_t DistanceSensor::measure() {
    if (!online) {
        return OUT_OF_RANGE_MM;
    }
    VL53L0X_RangingMeasurementData_t measure = {};
    VL53L0X_Error status = sensor->rangingTest(&measure, false);
    if (status != VL53L0X_ERROR_NONE) {
        return OUT_OF_RANGE_MM;
    }
    if (measure.RangeStatus != 4) {  // if not out of range
        return measure.RangeMilliMeter;
    } else {
        return OUT_OF_RANGE_MM;
    }
}
