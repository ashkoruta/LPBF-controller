#include "pch.h"
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cassert>
#include <map>
#include <functional>
#include <algorithm>
// MST codegen
#include "Adaptive_initialize.h"
#include "Adaptive_terminate.h"
#include "Adaptive.h"
// my interpolation codegen
#include "interp_xy_initialize.h"
#include "interp_xy_terminate.h"
#include "interp_xy.h"



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

typedef std::map<std::string, std::string> CfgMap;
static CfgMap parseCfgFile(std::ifstream &f)
{
	CfgMap ret;
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

static int dumpToFile(const std::vector<double>& in, const std::string& fileName)
{
	writeLog(logOpened, logFile, "Writing to file:" + fileName);
	std::ofstream out(fileName);
	if (!out.is_open()) {
		writeLog(logOpened, logFile, "Failed to open output file");
		return -1;
	}
	for (size_t i = 0; i < in.size(); ++i) {
		out << in[i] << std::endl;
	}
	out.close();
	return 0;
}

struct ScanData
{
	std::vector<double> _x;
	std::vector<double> _y;
	std::vector<double> _t;
	ScanData() : _x(), _y(), _t() {}
	ScanData(const std::vector<double>& x, const std::vector<double>& y) : _x(x), _y(y) {}
	const std::vector<double>& x() const {
		return _x;
	}
	const std::vector<double>& y() const {
		return _y;
	}
	const std::vector<double>& t() const {
		return _t;
	}
	int fillData(const std::string& fileName) {
		writeLog(logOpened, logFile, "Scan file:" + fileName);
		std::ifstream pp(fileName);
		if (!pp.is_open()) {
			writeLog(logOpened, logFile, "Failed to read profile file");
			return -1;
		}
		std::string line;
		double x = 0, y = 0, t = 0;
		while (std::getline(pp,line)) {
			sscanf_s(line.c_str(), "%lf,%lf,%lf", &x, &y, &t); 	// FIXME should prob handle errors better
			_x.push_back(x);
			_y.push_back(y);
			_t.push_back(t);
		}
		writeLog(logOpened, logFile, "Scan file parsed");
		return 0;
	}

};
// generic controller decides what's the power output based on last available measurement & its state (specific)
class Controller
{
public:
	virtual int nextPower(double in) = 0; // all controllers are SISO
	virtual int prologue(unsigned int layerNum) = 0; // before the scan of a part at a layer, specific actions
	virtual int epilogue(unsigned int layerNum) = 0; // at the end of the scan at a layer, specific actions
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
	virtual int nextPower(double in) { return -1; } // handled outside, in LabVIEW, as "nothing to do"
	virtual int prologue(unsigned int) { return 0; }
	virtual int epilogue(unsigned int) { return 0; }
};

// command a specified power profile in a feedforward manner
class FeedforwardController : public Controller
{
protected:
	unsigned int _curPosition;
	std::vector<int> _powerProfile; // power must be an integer, as Scanlab function only accepts int
	FeedforwardController() : _curPosition(0), _powerProfile() {}
	int fillOptions(CfgMap& params) {
		std::string pfile = params["Power"];
		if (pfile.empty()) {
			std::stringstream ss;
			ss << __FUNCTION__ << " : " << "Power profile not stated in cfg file";
			writeLog(logOpened, logFile, ss.str());
			return -1;
		}
		std::vector<double> pp;
		if (loadVectorFromFile(pp, pfile) < 0) {
			writeLog(logOpened, logFile, "Failed to read in power profile");
			return -1;
		}
		_powerProfile = std::vector<int>(pp.begin(), pp.end()); // conversion from double to int s.t. Scanlab accepts it
		writeLog(logOpened, logFile, "FF controller initialized: " + std::to_string(pp.size()) + " values");
		return 0;
	}
public:
	~FeedforwardController() {} // nothing specific to do
	static Controller* fromFile(std::ifstream& f) {
		auto params = parseCfgFile(f);
		auto ctrl = new FeedforwardController;
		int ret = ctrl->fillOptions(params);
		if (ret < 0) {
			delete ctrl;
			return nullptr;
		}
		return ctrl;
	}
	virtual int nextPower(double in) {
		//writeLog(logOpened, logFile, __FUNCTION__);
		// don't care about nothin, just move along the profile
		if (_curPosition == _powerProfile.size()) {
			writeLog(logOpened, logFile, "Reached the end of power profile");
			return -1; 
			// this behavior is different from previous version: here we revert to a value prescribed by scan file
			// don't hold the last FF value as before
			// FIXME do we tho? if -1, LV doesn't update power. so it stays as last commanded?
		}
		std::stringstream ss;
		ss << __FUNCTION__ << " : cur=" << _curPosition << " p=" << _powerProfile[_curPosition];
		writeLog(logOpened, logFile, ss.str());
		return _powerProfile[_curPosition++];
	}
	virtual int prologue(unsigned int) { return 0; } // generic FF doesn't do anything before or after the scan
	virtual int epilogue(unsigned int) { return 0; } // generic FF doesn't do anything before or after the scan
};

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
	virtual int prologue(unsigned int layerNum) { return 0; } // nothing to do
	virtual int epilogue(unsigned int layerNum) {
		// once a scan is done, next layer should be anew. so reset everything to the initial values
		_prev_power = _P0; // reset integrator
		_prev_err = 0; // reset history of errors
		_delayCounter = _initialDelay; // reset delay
		_curPosition = 0; // reset position in reference vector
		return 0;
	}
};

// layer-to-layer controller: feedforward controller that modifies its power profile from layer to layer
// read-in cfg slightly differently, and dump calculated profile on HDD
class L2LController : public FeedforwardController
{
	virtual void updatePowerProfile() {} // specific inhereted controllers must define it; can't make it purely virtual b/c want to instantiate the class
protected:
	std::string _namePrefix;
	unsigned int _curLayer;
	std::vector<double> _outputs;
	L2LController() : FeedforwardController(), _namePrefix(""), _curLayer(), _outputs() {}
	int fillOptions(CfgMap& params) {
		int ret = FeedforwardController::fillOptions(params);
		if (ret < 0) {
			return -1;
		}
		// now L2L specific things
		std::string n = params["OutputSize"];
		size_t n_out = 60000;
		if (n.empty()) { // use default value
			writeLog(logOpened, logFile, "Failed to read in size for output collection, using default 60,000");
		} else {
			try {
				n_out = std::stoi(n);
			} catch (...) {
				writeLog(logOpened, logFile, "Incorrect size for output collection, using default 60,000");
				n_out = 60000;
			}
		}
		_outputs = std::vector<double>(n_out);
		// file name prefix for HDD dumps
		std::string pre = params["SavingPrefix"];
		if (pre.empty()) {
			writeLog(logOpened, logFile, "Failed to read in prefix to save power profiles");
		}
		_namePrefix = pre;
		writeLog(logOpened, logFile, "L2L controller initialized: " + std::to_string(this->_powerProfile.size()) + " values");
		return 0;
	}
public:
	static Controller* fromFile(std::ifstream& f) {
		auto params = parseCfgFile(f);
		auto ctrl = new L2LController;
		int ret = ctrl->fillOptions(params);
		if (ret < 0) {
			delete ctrl;
			return nullptr;
		}
		return ctrl;
	}
	virtual int nextPower(double in) {
		// save the output value into the stash
		if (_curPosition >= _outputs.size()) {
			writeLog(logOpened, logFile, "Too many outputs, buffer ran out");
		} else {
			_outputs[_curPosition] = in;
		}
		// return the next power; positional counter is incremented within
		return FeedforwardController::nextPower(in);
	}
	virtual int prologue(unsigned int layerNum) {
		// don't need to do anything special, just note the layer number
		// bulk of work is done in the epilogue
		_curLayer = layerNum;
		return 0;
	}
	virtual int epilogue(unsigned int layerNum) {
		std::stringstream ss;
		ss << __FUNCTION__ << " : curLayer=" << this->_curLayer;
		writeLog(logOpened, logFile, ss.str());
		
		char conv[] = "000";
		sprintf_s(conv, "%.3d", this->_curLayer);
		std::string num(conv,3);

		// save current data to HDD
		// it is possible that buffer for outputs was larger than actual number of calls to nextPower. shrink
		// TODO what's the correct behavior? say we ran out of P. so P = -1 returned, which is equiv to last commanded power?
		_outputs.erase(_outputs.begin() + _curPosition, _outputs.end());
		dumpToFile(_outputs, this->_namePrefix + "_y" + num + ".txt"); // save layer X measurement for later use - sanity check mostly
		writeLog(logOpened, logFile, "Acquired output saved to disk"); 
		// TODO epilogue / prologue don't influence RT in-layer control. so logging doesn't ruin anything and should be enabled
		
		// calculate & set FF power profile for the next layer
		this->updatePowerProfile(); 

		// save the new profile to disk
		auto powerToDouble = std::vector<double>(this->_powerProfile.begin(), this->_powerProfile.end());
		dumpToFile(powerToDouble, this->_namePrefix + "_p" + num + ".txt");
		
		_curPosition = 0;
		return 0; 
	}
};

// iterative learning controller: layer to layer control p_k+1 = Q(p_k + L*e_k)
// read-in cfg slightly differently, and dump calculated profile on HDD
class ILCController : public L2LController
{
	virtual void updatePowerProfile() {
		std::stringstream ss;
		ss << __FUNCTION__;
		writeLog(logOpened, logFile, ss.str());
		assert(_powerProfile.size() == _outputs.size());
		// simple vector summation
		for (size_t i = 0; i < _powerProfile.size(); ++i) {
			_powerProfile[i] = double(_powerProfile[i]) + _L * (_R - _outputs[i]);
		}
		// TODO add robustness filter Q
	}
protected:
	double _L; // learning gain
	double _R; // reference value. TODO should probably be a vector for a general case
	// TODO should have some sort of filtering that I'm not concerned with currently
	ILCController() : L2LController(), _L(0.0), _R(0.0) {}
	int fillOptions(CfgMap& params) {
		int ret = L2LController::fillOptions(params);
		if (ret < 0)
			return ret;
		// additional stuff 
		// gain
		std::string Ls = params["Gain"];
		if (Ls.empty()) {
			writeLog(logOpened, logFile, "Gain is not defined in ILC controller cfg");
			return -1; // gain must be defined
		}
		try {
			_L = std::stod(Ls);
		} catch (...) {
			writeLog(logOpened, logFile, "Gain is not a number in ILC controller cfg");
			return -1; // gain must be defined
		}
		// reference level
		std::string Rs = params["Reference"];
		if (Rs.empty()) {
			writeLog(logOpened, logFile, "Reference is not defined in ILC controller cfg");
			return -1; // gain must be defined
		}
		try {
			_R = std::stod(Rs);
		}
		catch (...) {
			writeLog(logOpened, logFile, "Reference is not a number in ILC controller cfg");
			return -1; // gain must be defined
		}
		// TODO define Q filter somehow
		return 0;
	}
public:
	static Controller* fromFile(std::ifstream& f) {
		auto params = parseCfgFile(f);
		auto ctrl = new ILCController;
		int ret = ctrl->fillOptions(params); 
		// TODO all of these functions are very similar, maybe there is a way to reduce copy-paste even more
		if (ret < 0) {
			delete ctrl;
			return nullptr;
		}
		return ctrl;
	}
};

// everything MST wants, cropped and aligned
struct MSTData {
	ScanData scn;
	std::vector<double> power;
	std::vector<double> output;
};

// MS&T layer-to-layer controller: maintains and updates an estimate of y = Gp
class MSTController : public L2LController
{
	MSTData interp2cam() const {
		// essentially, alignXYTM equivalent.
		// crops the output data s.t. only actual frames are present,
		// then interpolates. it's all MATLAB codegen (by me)

		// allocate buffers for returned vectors
		MSTData ret;
		auto v = std::vector<double>(this->_scanLengthConst);
		ret.scn = ScanData(v,v);
		ret.power = v;
		ret.output = v;  // memeory allocation

		int x_sz = 0, y_sz = 0, out_sz = 0, p_sz = 0;
		double thr = this->_threshold; // conversion
		const std::vector<double> pint = std::vector<double>(this->_powerProfile.begin(), this->_powerProfile.end()); // implicit conversion

		// call the interpolator
		interp_xy(this->_outputs.data(), pint.data(),
			this->_scan.x().data(), this->_scan.y().data(), this->_scan.t().data(), thr,
			ret.scn._x.data(), &x_sz,
			ret.scn._y.data(), &y_sz,
			ret.output.data(), &out_sz,
			ret.power.data(), &p_sz);

		// trim data based on sizes returned from the interp_xy
		auto sz = x_sz;
		assert(x_sz == y_sz);
		assert(x_sz == p_sz);
		assert(x_sz == out_sz);
		ret.scn._x.erase(ret.scn._x.begin() + sz, ret.scn._x.end());
		ret.scn._y.erase(ret.scn._y.begin() + sz, ret.scn._y.end());
		ret.power.erase(ret.scn._x.begin() + sz, ret.power.end());
		ret.output.erase(ret.output.begin() + sz, ret.output.end());
		return ret;
	}

	virtual void updatePowerProfile() {
		std::stringstream ss;
		ss << __FUNCTION__;
		writeLog(logOpened, logFile, ss.str());
		// interpolate coordinates to camera timestamps
		MSTData interp = this->interp2cam();
		auto Gnew = _G;
		auto v = std::vector<double>(this->_scanLengthConst); // allocator
		auto Pnew = v;
		auto err = v;
		// calculate stuff
		Adaptive(interp.scn._x.data(), interp.scn._y.data(), interp.power.data(), interp.output.data(), _G.data(), Pnew.data(), Gnew.data(), err.data());
		// update internal structures
		this->_powerProfile = std::vector<int>(Pnew.begin(),Pnew.end()); // TODO I'm positive it will return junk; but sort it out later
		this->_G = Gnew;
	}
	void initG() {
		_G = std::vector<double>(_scanLengthConst * _scanLengthConst);
		for (size_t i = 0; i < _scanLengthConst; ++i) {
			_G[i*_scanLengthConst + i] = 1; // initialize diagonal to 1
		}
	}
protected:
	ScanData _scan; // x,y,t from G-code - never changes
	std::vector<double> _G; // gain matrix, flattened
	const unsigned int _threshold = 10;  // hardcoded threshold - leave for now; will recompile each time anyway to include MST script updates
	unsigned int _scanLengthConst = 1900; // ugly ass constant - length of all the stuff for MATLAB MST func 
	// IMPORTANT threshold is in signature of interest, NOT max as usual (don't have max here)
	// TODO other stuff related to hardcoded sizes of vectors
	MSTController() : L2LController() {}
	int fillOptions(CfgMap& params) {
		int ret = L2LController::fillOptions(params);
		if (ret < 0)
			return ret;
		// additional stuff 
		// read in scan file data
		std::string scn = params["Scan"];
		if (scn.empty()) {
			writeLog(logOpened, logFile, "Scan file name not provided");
			return -1;
		}
		if (_scan.fillData(scn) < 0) { // fill the scan file coordinates and times into structure
			writeLog(logOpened, logFile, "Scan file is bad");
			return -1;
		}
		// initialize MATLAB stuff
		Adaptive_initialize();
		interp_xy_initialize();
		// MST things
		std::string scnLen = params["ScanLengthConst"]; // ultimate constant for power profile length, output measurement length, etc.
		if (scnLen.empty()) {
			writeLog(logOpened, logFile, "Ultimate constant not provided");
			return -1;
		}
		try {
			_scanLengthConst = std::stod(scnLen);
		}
		catch (...) {
			writeLog(logOpened, logFile, "Ultimate constant poorely defined");
			return -1;
		} // TODO should prob write helper functions to parse numbers...
		// initialize gain matrix as eye(N)
		this->initG();
		// anything else?
		return 0;
	}
public:
	static Controller* fromFile(std::ifstream& f) {
		auto params = parseCfgFile(f);
		auto ctrl = new MSTController;
		int ret = ctrl->fillOptions(params);
		if (ret < 0) {
			delete ctrl;
			return nullptr;
		}
		return ctrl;
	}
	// FIXME it's pretty hardcoded safety check, need to do better
	virtual int nextPower(double in) {
		int p = FeedforwardController::nextPower(in);
		if (p >= 300) {
			writeLog(logOpened, logFile, "!!! Power level warning !!!");
			return 99;
		}
	}
	~MSTController() {
		Adaptive_terminate();
		interp_xy_terminate();
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
	unsigned int _prevPathInd;
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
		if (type == "FF") {
			return FeedforwardController::fromFile(cfg);
		}
		if (type == "ILC") {
			return ILCController::fromFile(cfg);
		}
		if (type == "MST") {
			return MSTController::fromFile(cfg);
		}
		// ... other possible options...
		writeLog(logOpened, logFile, "Controller type [" + type + "] unrecognized");
		return nullptr;
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
		return _ctrls[partNum]->nextPower(in); // TODO not sure if nextPower within any controller would need layer number?
	}
	int prologue(unsigned int layerNum, unsigned int partNum) {
		_prevPathInd = partNum;
		return _ctrls[_prevPathInd]->prologue(layerNum);

	}
	int epilogue(unsigned int layerNum, unsigned int partNum) {
		_prevPathInd = partNum;
		return _ctrls[_prevPathInd]->epilogue(layerNum);
	}
};

// interface exposed to Labview: controller initialization
extern "C" _declspec(dllexport) int _stdcall initControllers(unsigned int partNum, const char* cfgFileName, int enableLog)
{
	// TODO it could be good idea to put logging into control cfg but that's future work
	if (enableLog) {
		logFile.open(logFileName, std::ios::app);
		logOpened = logFile.is_open();
		if (!logOpened) {
			return -3;
		}
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
	return CD->nextPower(partNum, in);
}

// specific actions before a particular scan starts
extern "C" _declspec(dllexport) int _stdcall prologue(unsigned int layerNum, unsigned int curPartNum)
{
	if (!CD) {
		writeLog(logOpened, logFile, "Dispatcher wasn't initialized!");
		return -1;
	}
	return CD->prologue(layerNum, curPartNum);
}

// specific actions in the aftermath of a particular scan
extern "C" _declspec(dllexport) int _stdcall epilogue(unsigned int layerNum, unsigned int curPartNum)
{
	if (!CD) {
		writeLog(logOpened, logFile, "Dispatcher wasn't initialized!");
		return -1;
	}
	return CD->epilogue(layerNum, curPartNum);
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
