/*
 * thermistor.cpp
 * Jon Cruz
 *
 * Processes raw voltage output of thermistor voltage
 * divider into a temperature reading
 */

#include "lilred_management_system/thermistor.h"

/* Make a line object using two points */
line::line(float point1_x, float point1_y, float point2_x, float point2_y) {
	slope = (point2_y - point1_y) / (point2_x - point1_x);
	point_x = point2_x;
	point_y = point2_y;
}

/* For a given x, get the y value */
float line::getFuncValue(float x) {
	return slope * (x - point_x) + point_y;
}


/* Make a thermistor object from a resistance list and its length */
thermistor::thermistor(float resList[], std::size_t listLen) {
	resPullup = 0;
	vcc = 0;
	resValues = resList;
	resListLen = listLen;
}

/* Set pullup resistor value in ohms */
void thermistor::setResPullup(float value) {
	resPullup = value;
}

/* Set VCC value in volts */
void thermistor::setVcc(float value) {
	vcc = value;
}

/* Create a set of lines that interpolate between the resistance list's points */
/* Needs list of temperatures that correspond to the resistances               */
void thermistor::fillRtTable(float lowTemp, float highTemp, float increment) {
	float prevResNum = 0;
	float prevTemp = lowTemp;

	for (float i = 1, j = lowTemp + increment; i <= resListLen && j < highTemp; i++, j += increment) {
		line rtLine(resValues[(int)prevResNum], prevTemp, resValues[(int)i], j);
		rtLines.push_back(rtLine);

		prevResNum = i;
		prevTemp = j;
	}
}

/* Takes thermistor voltage divider reading and determines the temperature */
float thermistor::voltageToTemp(float voltage) {
	float resTherm = resPullup * (voltage / (vcc - voltage)); // thermistor resistance in ohms
	line* rtLine = rtLineSearch(&resTherm, resValues, resListLen);

	if (rtLine == 0)
		return ERROR_VALUE; // out of range
	else
		return rtLine->getFuncValue(resTherm);
}

/* Binary search through resistance list to find the appropriate line for a given resistance reading */
line* thermistor::rtLineSearch(float* key, float* base, std::size_t num) {
	std::size_t start = 0;
	std::size_t end = num;

	// out of range
	if (*key > *base || *key < *(base + end))
		return 0;

	// binary search
	while (start < end) {
		std::size_t mid = start + (end - start) / 2;
		float* toCompare = base + mid;

		if (*key > *toCompare) {
			if (*key < *(toCompare - 1))
				return &rtLines[mid-1];

			end = mid;
		}
		else if (*key < *toCompare) {
			if (*key > *(toCompare + 1))
				return &rtLines[mid];

			start = mid;
		}
		else
			return &rtLines[mid];
	}
}
