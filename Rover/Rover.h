#define LEFT_INTERRUPT 0
#define RIGHT_INTERRUPT 1

const double DISTANCE_PER_TICK = 0.010205;
const double RADIANS_PER_TICK = 0.085756303;
const double TRACK_WIDTH = 0.119;

static unsigned long timer = 0;

double x;
double y;
double heading;

signed long leftTicks;
signed long rightTicks;

signed long lastLeftTicks;
signed long lastRightTicks;
