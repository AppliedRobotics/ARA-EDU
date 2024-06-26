// #pragma once

// #include "ARA_ESP.h"
#include "ProgramHightLevel.h"


class ProgramHightLevel
{
  public:
    void Rise(float heigh);
    void TurnRight(float time_turn);
    void TurnLeft();

  private:
    float curr_heigh;
};

extern ARA_ESP esp;
