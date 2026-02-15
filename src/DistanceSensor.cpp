#include <Adafruit_VL53L0X.h>
#include <DistanceSensor.h>

DistanceSensor::DistanceSensor(int address, uint8_t shtPin) : address{ address }, shtPin{ shtPin } {
    this->sensor = new Adafruit_VL53L0X();
}

void DistanceSensor::setAddress() {
    digitalWrite(shtPin, HIGH);

    if (!sensor->begin(address)) {
        Serial.printf("Failed to boot %s VL53L0X\n", address);
        while (1) {
        }
    }
}

uint16_t DistanceSensor::measure() {
    VL53L0X_RangingMeasurementData_t measure;
    sensor->rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {  // if not out of range
        return measure.RangeMilliMeter;
    } else {
        return 1000;
    }
}
