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
   STATE_TURN,
   STATE_STOPPED
} State;
