// Functions that are called from LabVIEW

// initialize all controllers before the first laser firing
// in: 1) total number of separate paths in a layer 2) filename for top-level config file 3) log on / off
// out: success (0) or not (<0)
extern "C" _declspec(dllexport) int _stdcall initControllers(unsigned int totalpartNum, const char* cfgFileName, int enableLog);

// get power to input to laser, based on the latest output (1D signal)
// in: 1) index of a current path (to distinguish between different parts) 2) output signal
// out: power (int), -1 if any error or dummy controller
extern "C" _declspec(dllexport) int _stdcall nextPower(unsigned int curPartNum, double in);

// cleanup after everything is done: call the destructor of the controller dispatching mechanism
extern "C" _declspec(dllexport) int _stdcall cleanupControllers();

// specific actions before a particular scan starts
extern "C" _declspec(dllexport) int _stdcall prologue(unsigned int layerNum, unsigned int curPartNum);

// specific actions in the aftermath of a particular scan
extern "C" _declspec(dllexport) int _stdcall epilogue(unsigned int layerNum, unsigned int curPartNum);


