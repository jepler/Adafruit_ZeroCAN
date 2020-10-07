#include <Arduino.h>

#include <Adafruit_ZeroCAN.h>

Adafruit_ZeroCAN can{};

void setup() {
    Serial.begin(9600); // open the serial port at 9600 bps:

    digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
    can.begin(1000000, true, false);
}

Adafruit_ZeroCAN::Message m{0xaa, false, false, 8, {'Z', 'e', 'r', 'o', 'C', 'A', 'N', 0}};

void loop() {
    auto state = can.busState();
    Serial.print("bus state ");    
    Serial.println((int)state);

    if(state == Adafruit_ZeroCAN::BUS_OFF) {
        can.restart();
    }

    m.data[7] ++;
    can.send(m);
    delay(250);
}
