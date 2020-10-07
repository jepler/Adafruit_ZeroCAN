/*!
 * @file Adafruit_ZeroCAN.h
 *
 * This is a library for the CAN peripheral on SAME51 devices
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Jeff Epler for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#pragma once

#include <Arduino.h>

struct _adafruit_ZeroCAN_state;

class Adafruit_ZeroCAN {
public:
    enum BusState {
        ERROR_ACTIVE, ERROR_WARNING, ERROR_PASSIVE, BUS_OFF
    };

    struct Message {
        uint32_t id;
        bool extended;
        bool rtr;
        uint8_t size;
        uint8_t data[8];
    };

    struct Match {
        uint32_t id;
        uint32_t mask;
        bool extended;
    };

    class Listener {
    public:
        Listener();
        bool begin(Adafruit_ZeroCAN &can, Match *matches, size_t nmatch);
        ~Listener();
        bool in_waiting() const;
        bool read(Message &);
    };

    Adafruit_ZeroCAN(uint8_t TX_PIN, uint8_t RX_PIN);
    Adafruit_ZeroCAN();
    ~Adafruit_ZeroCAN() {}
    
    bool begin(int baudrate, bool loopback, bool silent);

    int transmitErrorCount() const;
    int receiveErrorCount() const;
    BusState busState() const;

    bool send(const Message &m);
    void restart();

private:
    struct State;
    int8_t _tx, _rx;
    void *_hw;
    _adafruit_ZeroCAN_state *state;
};

