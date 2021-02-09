// Functions that are called from LabVIEW

// initialize all controllers before the first laser firing
// in: 1) total number of separate paths in a layer 2) filename for top-level config file
// out: success (0) or not (<0)
extern "C" _declspec(dllexport) int _stdcall initControllers(unsigned int totalpartNum, const char* cfgFileName);

// get power to input to laser, based on the latest output (1D signal)
// in: 1) index of a current path (to distinguish between different parts) 2) output signal
// out: power (int), -1 if any error or dummy controller
extern "C" _declspec(dllexport) int _stdcall nextPower(unsigned int curPartNum, double in);

// cleanup after everything is done: call the destructor of the controller dispatching mechanism
extern "C" _declspec(dllexport) int _stdcall cleanupControllers();
