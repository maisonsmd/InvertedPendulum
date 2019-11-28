#pragma once

#define SCHEDULER_GUARD(__current__, __var__) if(__var__ > __current__) __var__ = __current__
#define SCHEDULER_CREATE(__name__) static uint32_t __name__

// this function already guard timekeeper variable
static bool onSchedule(const uint32_t current, uint32_t & var, const uint32_t interval) {
	SCHEDULER_GUARD(current, var);
	if (current < var + interval) return false;

	var = current;
	return true;
}