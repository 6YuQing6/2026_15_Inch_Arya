#include "robot-config.h"

// Pneumatics
digital_out MatchLoader = digital_out(Brain.ThreeWirePort.A);
digital_out Expansion = digital_out(Brain.ThreeWirePort.H);
digital_out ColorSort = digital_out(Brain.ThreeWirePort.G);
digital_out Stopper = digital_out(Brain.ThreeWirePort.C);
digital_out Aligner = digital_out(Brain.ThreeWirePort.B);