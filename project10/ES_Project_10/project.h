typedef enum {
   DISTANCE_FRONT = 0,
   DISTANCE_RIGHT = 1
}DistanceSensor;

typedef enum {
  FORWARD = 0,
  BACKWARD = 1
} lateralDirection;

typedef enum {
   STATE_START,
   STATE_SET_ORIENTATION,
   STATE_RAMP_LIFT,
   STATE_FORWARD,
   STATE_STOPPING,
   STATE_TURN_LEFT,
   STATE_TURN_RIGHT,
   STATE_TURN_180,
   STATE_PREPARE_PHASE_2,
   STATE_STOPPED
} State;

typedef enum {
  FORWARD_PATH,
  A_TO_D,
  B_TO_C,
} phase2Path;
