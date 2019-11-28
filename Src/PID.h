#pragma once

extern uint32_t getUs();

struct Limit {
	double lower;
	double upper;
	void applyTo(double & value) {
		if (value < lower)
			value = lower;
		if (value > upper)
			value = upper;
	}
};

struct Tunings {
	double kp;
	double ki;
	double kd;
};

class PID {
protected:
	double & input;
	double & output;
	double & setpoint;
	uint32_t sampleTime_us;
	double sampleTime;
	Limit outputLimit, integralLimit;
	Tunings & tunings;

	uint32_t lastCalc_us = 0;
	double lastError = 0;
	double integral;

public:
	PID(double & _in, double & _out, double & _set, Tunings & _tunings)
    : input(_in), output(_out), setpoint(_set),
    tunings(_tunings){
		input = _in;
		output = _out;
		setpoint = _set;
		SetOutputLimit(0, 255);
		SetSampleTime(1000);
	}

	void SetSampleTime(uint32_t _us) {
		if (_us < 100)
			_us = 100;
		sampleTime_us = _us;
		sampleTime = (double)_us / 1000000.0;
	}

	void Update() {
		uint32_t current_us = getUs();

		if (current_us < lastCalc_us + 100)
			return;
		lastCalc_us = current_us;

		double _error = setpoint - input;
		integral += tunings.ki * (_error + lastError) * sampleTime;
		double _derivative = tunings.kd * (_error - lastError) / sampleTime;

		output = (tunings.kp * _error) + integral + _derivative;

		integralLimit.applyTo(integral);
		outputLimit.applyTo(output);

//    char message[128];
//    snprintf(message, sizeof message, "%f, %f, %f, %f", _error, integral, _derivative, *output);
//    Serial.println(message);
		lastError = _error;
	}

	void SetOutputLimit(double _lower, double _upper) {
		outputLimit = { _lower, _upper };
		SetIntegralLimit(_lower, _upper);
	}

	void SetIntegralLimit(double _lower, double _upper) {
		integralLimit = { _lower, _upper };
	}

	void SetTunings(Tunings _tunings) {
		tunings = _tunings;
	}
};
