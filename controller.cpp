#include "pch.h"
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <map>

const std::string logFileName = "C:/gateway/control.log";
std::ofstream logFile;
bool logOpened = false;

std::string current_time_and_date()
{
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);
	tm t;
	localtime_s(&t, &in_time_t);
	std::stringstream ss;
	ss << std::put_time(&t, "%Y-%m-%d %X");
	return ss.str();
}

static void writeLog(bool enabled, std::ofstream& os, const std::string& str)
{
	if (enabled) {
		os << current_time_and_date() << " : " << str << std::endl;
		os.flush();
	}
}

static std::map<std::string, std::string> parseCfgFile(std::ifstream &f)
{
	std::map<std::string, std::string> ret;
	std::string line;
	while (std::getline(f, line)) {
		std::istringstream ss(line);
		std::string key, val;
		std::getline(ss, key, '=');
		std::getline(ss, val);
		ret[key] = val;
	}
	writeLog(logOpened, logFile, "Config file reading done");
	return ret;
}

// read in a profile from a file overwrites the input vector (in)
// generally, profile can be int (e.g. power) or double (output reference)
static int loadVectorFromFile(std::vector<double>& in, const std::string& fileName)
{
	writeLog(logOpened, logFile, "Profile file:" + fileName);
	std::ifstream pp(fileName);
	if (!pp.is_open()) {
		writeLog(logOpened, logFile, "Failed to read profile file");
		return -1;
	}
	in = std::vector<double>(); // empty vector of zeros to begin with
	// start filling up the contents
	int j = 0;
	std::string line;
	while (std::getline(pp, line)) {
		double p = 0;
		try {
			p = std::stod(line);
		} catch (...) {
			writeLog(logOpened, logFile, "Failed to parse a number:[" + line + "]");
			return -1;
		}
		in.push_back(p);
	}
	writeLog(logOpened, logFile, "Number of lines in the file:" + std::to_string(in.size()));
	return 0;
}

// generic controller decides what's the power output based on last available measurement & its state (specific)
class Controller
{
public:
	virtual int nextPower(double in) = 0; // all controllers are SISO
	virtual void reset() = 0; // reset the controller state at the end of the layer
	virtual ~Controller() {}
};

// plug for a case when nothing should be done (open-loop)
class DummyController : public Controller
{
	DummyController() {}
public:
	// doesn't do anything, as name suggests
	~DummyController() {}
	static Controller* fromFile(std::ifstream&) { 
		writeLog(logOpened, logFile, "DummyController initialized");
		return new DummyController;  
	}
	virtual int nextPower(double in) { return -1; }
	virtual void reset() {}
};

// command a specified power profile in a feedforward manner
class FeedforwardController : public Controller
{
	unsigned int _curPosition;
	std::vector<int> _powerProfile; // power must be an integer, as Scanlab function only accepts int
	FeedforwardController(const std::vector<double>& pp) { 
		_curPosition = 0;
		_powerProfile = std::vector<int>(pp.begin(), pp.end()); // implicit conversion from double to int
	}
public:
	~FeedforwardController() {} // nothing specific to do
	static Controller* fromFile(std::ifstream& f) {
		auto params = parseCfgFile(f);
		std::string pfile = params["Power"];
		if (pfile.empty()) {
			std::stringstream ss;
			ss << __FUNCTION__ << " : " << "Power profile not stated in cfg file";
			writeLog(logOpened, logFile, ss.str());
			return nullptr;
		}
		std::vector<double> pp;
		if (loadVectorFromFile(pp, pfile) < 0) {
			writeLog(logOpened, logFile, "Failed to read in power profile");
			return nullptr;
		}
		writeLog(logOpened, logFile, "FF controller initialized: " + std::to_string(pp.size()) + " values");
		return new FeedforwardController(pp);
	}
	virtual int nextPower(double in) {
		//writeLog(logOpened, logFile, __FUNCTION__);
		// don't care about nothin, just move along the profile
		if (_curPosition == _powerProfile.size()) {
			writeLog(logOpened, logFile, "Reached the end of power profile");
			return -1; 
			// this behavior is different from previous version: here we revert to a value prescribed by scan file
			// don't hold the last FF value as before
		}
		std::stringstream ss;
		ss << __FUNCTION__ << " : cur=" << _curPosition << " p=" << _powerProfile[_curPosition];
		writeLog(logOpened, logFile, ss.str());
		return _powerProfile[_curPosition++];
	}
	virtual void reset() {}
};
// copy file routine
//std::ifstream  src("from.ogv", std::ios::binary);
//std::ofstream  dst("to.ogv",   std::ios::binary);
//dst << src.rdbuf();

/*L2LController
int epilogue() {
	signal = ... smth acquired during all prev calls 
	p_profile_new = calc(p_profile_cur, signal); // core calculation
	p_profile_cur = p_profile_new; // reset the profile
	write(p_file,p_profile_new); // save layer X profile for later use 
}
*/

// respond to output changes to track a reference
class FeedbackController : public Controller
{
	// static parameters
	std::vector<double> _reference; // time-varying reference. if shorter than total scan time, last value is held
	unsigned int _initialDelay; // wait for N timestamps in the beginning, don't jump into control instantly
	double _Kp, _Ki, _Kd; // gains
	unsigned int _Pmin, _Pmax; // safety limits
	unsigned int _P0; // baseline power

	// controller state
	unsigned int _prev_power; // usually p[k] = c*p[k-1] + smth
	double _prev_err; // TODO i should prob store N past powers and M past error for generic controller
	
	// counters
	unsigned int _curPosition;
	unsigned int _delayCounter; 

	FeedbackController() {} // members are set in the fromFile
	const double nextReferenceValue() {
		double r = _reference[_curPosition];
		if (_curPosition == _reference.size() - 1) {
			writeLog(logOpened, logFile, "Reached the end of commanded reference");
		}
		else {
			_curPosition++;
		}
		return r;
	}
public:
	~FeedbackController() {} // nothing specific to do
	static Controller* fromFile(std::ifstream& f) {
		auto params = parseCfgFile(f);

		std::string rfile = params["Reference"];
		if (rfile.empty()) {
			std::stringstream ss;
			ss << __FUNCTION__ << " : " << "Reference file not stated in cfg file";
			writeLog(logOpened, logFile, ss.str());
			return nullptr;
		}
		std::vector<double> r;
		if (loadVectorFromFile(r, rfile) < 0) {
			writeLog(logOpened, logFile, "Failed to read in reference profile");
			return nullptr;
		}

		std::string kp = params["Kp"];
		std::string ki = params["Ki"];
		std::string kd = params["Kd"];
		std::string p0 = params["P0"];
		std::string pmin = params["Pmin"];
		std::string pmax = params["Pmax"];
		std::string nd = params["Delay"];

		if (kp.empty() || ki.empty() || kd.empty()) {
			writeLog(logOpened, logFile, "Check gains, Kp/Ki/Kd is missing");
			return nullptr;
		}

		if (p0.empty() || pmin.empty() || pmax.empty()) {
			writeLog(logOpened, logFile, "Check powers, baseline/safety limits are missing");
			return nullptr;
		}

		if (nd.empty()) {
			writeLog(logOpened, logFile, "Initial delay is missing");
			return nullptr;
		}

		double Kp, Ki, Kd;
		int P0, Pmin, Pmax, Nd;
		try {
			Kp = std::stod(kp);
			Ki = std::stod(ki);
			Kd = std::stod(kd);
			P0 = std::stoi(p0);
			Pmin = std::stoi(pmin);
			Pmax = std::stoi(pmax);
			Nd = std::stoi(nd);
		} catch (...) {
			writeLog(logOpened, logFile, "Non-numeric garbage in parameters");
			return nullptr;
		}
		auto fb = new FeedbackController();
		fb->_reference = r;
		fb->_initialDelay = Nd;
		fb->_Kp = Kp;
		fb->_Ki = Ki;
		fb->_Kd = Kd;
		fb->_Pmax = Pmax;
		fb->_Pmin = Pmin;
		fb->_P0 = P0;

		fb->_curPosition = 0;
		fb->_delayCounter = fb->_initialDelay;
		fb->_prev_err = 0;
		fb->_prev_power = P0;

		std::stringstream ss;
		ss << "Nd=" << Nd << " gains=[" << Kp << "," << Ki << "," << Kd << "] powers=[" << P0 << "," << Pmin << "-" << Pmax << "]";
		writeLog(logOpened, logFile, "FB controller initialized: " + ss.str());
		return fb;
	}
	virtual int nextPower(double in) {
		//writeLog(logOpened, logFile, __FUNCTION__);
		if (_delayCounter > 0) {
			_delayCounter--;
			return -1; // wait for initial transients to die
		}
		// get reference value for this moment in time
		const double rcur = this->nextReferenceValue();
		const double err = rcur - in;
		// calculate output power
		// discrete controllers are usually in the form of smth/z-1
		// so power[k] = Ki*power[k-1] + Kp*e[k] + Kd*e[k-1]
		// FIXME it's less than ideal, should prob rethink the parameters
		int p = _Ki * _prev_power + _Kp * err + _Kd * _prev_err;
		if (p < _Pmin)
			p = _Pmin;
		if (p > _Pmax)
			p = _Pmax;
		// update controller state
		_prev_power = p;
		_prev_err = err;

		std::stringstream ss;
		ss << __FUNCTION__ << " : r=" << rcur << " e=" << err << " p=" << p;
		writeLog(logOpened, logFile, ss.str());
		return p;
	}
	virtual void reset() {
		_prev_power = _P0; // reset integrator
		_prev_err = 0; // reset history of errors
		_delayCounter = _initialDelay; // reset delay
		_curPosition = 0; // reset position in reference vector
	}
};

// Control dispatching must persist in between DLL function calls
// so it has to be off the stack but also need to be cleaned up properly
// for clean up, create separate function that calls delete on a pointer, that 
// necessitates new for construction, thus the implementation below

class ControlDispatcher;
static ControlDispatcher* CD = nullptr;

class ControlDispatcher
{
	unsigned int _prevPathInd; // save previous path index so we can gage when the layer number changes (path index switches from N to 0)
	std::vector<Controller*> _ctrls;
	ControlDispatcher(const std::vector<Controller*>& cs) {
		_prevPathInd = 0;
		_ctrls = cs;
	}
public:
	// clean up, explcitly called through delete CD in an exposed C function
	~ControlDispatcher() {
		writeLog(logOpened, logFile, __FUNCTION__);
		for (unsigned int i = 0; i < _ctrls.size(); ++i) {
			delete _ctrls[i];
		}
	}
	// Controller factory, returns NULL or a pointer to a class based on success of file parsing
	static Controller* fromFile(std::ifstream& cfg) {
		std::string type;
		if (!std::getline(cfg, type)) {
			writeLog(logOpened, logFile, "Failed to read controller type");
			return nullptr;
		}
		// select based on type
		if (type == "NONE") {
			return DummyController::fromFile(cfg);
		}
		if (type == "FB") {
			return FeedbackController::fromFile(cfg);
		}
		else if (type == "FF") {
			return FeedforwardController::fromFile(cfg);
		// ... other possible options...
		} else {
			writeLog(logOpened, logFile, "Controller type [" + type + "] unrecognized");
			return nullptr;
		}
	}
	// main initialization routine: get list of controller cfg files, go one by one, 
	// call specific controller initializers based on a type stated in each file
	static int init(const std::vector<std::string>& cfgs) {
		auto ctrls = std::vector<Controller*>(cfgs.size(), nullptr); // container for all Controllers
		for (unsigned int i = 0; i < cfgs.size(); ++i) {
			std::ifstream cfg(cfgs[i]);
			if (!cfg.is_open()) {
				writeLog(logOpened, logFile, "Failed to open controller config file: [" + std::string(cfgs[i]) + "]");
				return -1;
			}
			writeLog(logOpened, logFile, "Initialize controller: " + cfgs[i]);
			// try to initialize specific controller based on this particular file
			if (!(ctrls[i] = ControlDispatcher::fromFile(cfg))) {
				writeLog(logOpened, logFile, "Initialization failed: " + cfgs[i]);
				return -1;
			}
		}
		// all files were fine, we have a list now
		CD = new ControlDispatcher(ctrls); // allocate memory for the dispatcher class
		writeLog(logOpened, logFile, "Initialization successful");
		return 0;
	}
	// main purpose of all this: what's the power output given controller state, current part number, and output signal
	int nextPower(unsigned int partNum, double in) {
		//writeLog(logOpened, logFile, __FUNCTION__);
		_prevPathInd = partNum;
		return _ctrls[partNum]->nextPower(in);
	}
	bool partChange(unsigned int partNum) const {
		return partNum != _prevPathInd;
	}
	void reset() {
		_ctrls[_prevPathInd]->reset();
	}
};

// interface exposed to Labview: controller initialization
extern "C" _declspec(dllexport) int _stdcall initControllers(unsigned int partNum, const char* cfgFileName)
{
	if (!logOpened) {
		logFile.open(logFileName, std::ios::app);
		logOpened = logFile.is_open();
	}
	if (!logOpened) {
		return -3;
	}
	// open high-level config file
	std::ifstream cfg(cfgFileName);
	if (!cfg.is_open()) {
		std::stringstream ss;
		ss << "Failed to open controller config file: [";
		ss << std::string(cfgFileName);
		ss << "]";
		writeLog(logOpened, logFile, ss.str());
		return -1;
	}
	// each line is a file name for a single controller
	std::string line;
	std::vector<std::string> ctrlCfgs;
	while (std::getline(cfg, line)) {
		ctrlCfgs.push_back(line);
	}
	// make sure that number of those files matches number of paths that exists in a layer
	// FIXME potential problem: what if number of scans per layer changes?
	if (partNum != ctrlCfgs.size()) {
		std::stringstream ss;
		ss << "Number of parts doesn't match number of controllers: [";
		ss << partNum << " vs " << ctrlCfgs.size() << "]";
		writeLog(logOpened, logFile, ss.str());
		return -2;
	}
	// go deeper into parsing of those files
	return ControlDispatcher::init(ctrlCfgs);
}

extern "C" _declspec(dllexport) int _stdcall nextPower(unsigned int partNum, double in)
{
	if (!CD) {
		writeLog(logOpened, logFile, "Dispatcher wasn't initialized!");
		return -1;
	}
	//writeLog(logOpened, logFile, __FUNCTION__);
	if (CD->partChange(partNum)) {
		// just started new part, so previous controller needs to be reset
		CD->reset();
	}
	return CD->nextPower(partNum, in);
}

extern "C" _declspec(dllexport) int _stdcall cleanupControllers()
{
	if (CD != nullptr) {
		writeLog(logOpened, logFile, "ControlDispatcher is being released");
		delete CD;
	} else {
		writeLog(logOpened, logFile, "ControlDispatcher is already released, nothing to do");
	}
	if (logOpened)
		logFile.close();
	return 0;
}
