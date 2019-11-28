#pragma once

extern uint32_t getUs();

#define MAX_POSITION		9500
#define MIN_POSITION		-9500

class VelocityStepper {
protected:
	uint32_t stepPeriodUs = 0;
	bool clockwise = false;
	bool stop = false;
	double & setPosition;
	uint32_t lastMicros = 0;
  bool stopped = false;

	Limit positionLimit { MIN_POSITION, MAX_POSITION };
public:
	VelocityStepper(double & _setPosition) : setPosition(_setPosition) {
	}
	void SetVelocity(double _setVelocity) {
		if (_setVelocity < 0) {
			clockwise = false;
			_setVelocity = -_setVelocity;
		} else
			clockwise = true;

		if (_setVelocity > -1 && _setVelocity < 1){
			stepPeriodUs = 10000000;
      stopped = true;
    }
		else{
      stopped = false;
			stepPeriodUs = (uint32_t)((double)1000000 / _setVelocity);
    }
	}
	void Update() {
		uint32_t currentMicros = getUs();

		if (stopped || currentMicros < lastMicros + stepPeriodUs)
			return;
		/*Serial.print(stepPeriodUs);
		Serial.print(',');
		Serial.println(currentMicros - lastMicros);*/

		lastMicros = currentMicros;

		if (clockwise)
			setPosition = setPosition + 10;
		else
			setPosition = setPosition - 10;

    positionLimit.applyTo(setPosition);
	}
};

class Accel {
protected:
	double & setAccel;
	double currentVelocity = 0;
	VelocityStepper & velocityHandler;
	Limit velocityLimit {MIN_POSITION, MAX_POSITION};
	uint32_t lastCalc_us = 0;
public:
	Accel(VelocityStepper & _vHandler, double & _setAccel)
    : velocityHandler(_vHandler), setAccel(_setAccel) {
	}

	void Update() {
		uint32_t current_us = getUs();
		velocityHandler.Update();

		//update every 1000us
		if (current_us < lastCalc_us + 1000)
      return;

    lastCalc_us = current_us;

    //1000 times per second
    currentVelocity += (setAccel) / 1000;

    velocityLimit.applyTo(currentVelocity);
		velocityHandler.SetVelocity(currentVelocity);
	}

	double GetVelocity() {
		return currentVelocity;
	}

	double GetAccel() {
		return setAccel;
	}
};
