#ifndef DATETIME_H
#define DATETIME_H

#include "pico/stdlib.h"

class DateTime {

private:
    uint64_t timestamp;  // microseconden sinds opstart (via Pico timer)

public:
    DateTime() : timestamp(0) {}

    // Sla het huidige moment op
    void Now() {
        timestamp = time_us_64();
    }

    // Geef timestamp terug in microseconden
    uint64_t GetMicros() const {
        return timestamp;
    }

    // Geef timestamp terug in milliseconden
    uint64_t GetMillis() const {
        return timestamp / 1000;
    }

    // Verschil in milliseconden met een andere DateTime
    uint64_t ElapsedMillis(const DateTime& other) const {
        return (timestamp - other.timestamp) / 1000;
    }
};

#endif