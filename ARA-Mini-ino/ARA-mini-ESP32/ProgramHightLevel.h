#pragma once

// #include "ARA_ESP.h"
// #include "ProgramHightLevel.h"


class ProgramHightLevel
{
  public:
    void TakeOff(float heigh);
    void Rise(float heigh);
    void TurnRight(float time_turn);
    void TurnLeft(float time_turn);
    void Landing();

  private:
    float curr_heigh;
};

extern ARA_ESP esp;
