#pragma once

enum State {
  POWER_ON,
  SYSTEM_CHECK,
  READY_TO_LAUNCH,
  LAUNCHING_COUNTDOWN,
  LAUNCH,
  APOGEE,
  ERROR
};

extern State currentState;