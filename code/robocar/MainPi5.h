#pragma once

#include "Pi5UARTHandler.h"
#include <atomic>
#include <mutex>
#include <thread>

extern std::atomic<bool> running;

// ─────────────────────────────────────────────────────────────────
//  CommandKeepAlive
//
//  Achtergrond-thread die het actieve DriveCommand elke 100ms
//  opnieuw naar de Pico stuurt. De Pico-watchdog time-out is 500ms,
//  dus zo blijft de robot rijden zonder dat de hoofdlus elke iteratie
//  zelf hoeft te sturen.
// ─────────────────────────────────────────────────────────────────
class CommandKeepAlive {
public:
    explicit CommandKeepAlive(Pi5UARTHandler& uart);

    void SetCommand(float linearMmS, float angularDegS);
    void Stop();
    void Start();
    void Shutdown();

    bool  IsActief() const;
    float GetLin()   const;
    float GetAng()   const;

private:
    Pi5UARTHandler&   uart;
    std::mutex        mtx;
    float             lin;
    float             ang;
    std::atomic<bool> actief;
    std::atomic<bool> threadActief{false};
    std::thread       worker;
};