#include <Arduino.h>
#include "MSP.h"
#include "ARA_ESP.h"
#include "ProgramHightLevel.h"

ARA_ESP esp = ARA_ESP();


// в бегине надо сделать curr_heigh = -1

void ProgramHightLevel::TakeOff(float heigh)
{
    if (heigh < 0 || heigh > 1)
        return;
    
    curr_heigh = heigh;

    float recalc_height = Map(curr_heigh, 0, 1, 0, 100);

    delay(20);
    esp.nav_mode(2);
    delay(20);
    // esp.throttle(Map(1500, 1000, 1800, 0, 100));
    esp.throttle(recalc_height);
    // delay(heigh / 0.75);
    delay( (curr_heigh / 1.5) + 1000 );
    // esp.throttle(0.5);
}

/// @brief
/// @param angle в градусах
void ProgramHightLevel::TurnRight(float angle)
{
    // 1500 - 0 гр/с
    // 2000 - 400 гр/с
    // 1550 - ??

    float recalc = angle / Map(1550, 1500, 2000, 0, 400);
    esp.yaw(
        Map(1550, 1200, 1800, -100, 100)
        );
    delay(recalc);
    esp.yaw(
        Map(1500, 1200, 1800, -100, 100)
        );
}

/// @brief
/// @param angle в градусах
void ProgramHightLevel::TurnLeft(float angle)
{
    float recalc = angle / Map(1450, 1500, 2000, 0, 400);
    esp.yaw(
        Map(1450, 1200, 1800, -100, 100)
        );
    delay(recalc);
    esp.yaw(
        Map(1500, 1200, 1800, -100, 100)
        );
}

void ProgramHightLevel::Landing()
{
    // скорость подьема 1.5 м/c
    // curr_heigh
    
    esp.throttle(0);
    esp.throttle(recalc_height);

    float delta = curr_heigh / 10;
    for(int i = 0; i < 10; i++)
    {
        curr_heigh -= delta;
        float recalc_height = Map(curr_heigh, 0, 1, 0, 100);

        esp.throttle(recalc_height);
        delay( (recalc_height / 1.5) );
    }
    
    curr_heigh = -1;
    esp.throttle(0);
}