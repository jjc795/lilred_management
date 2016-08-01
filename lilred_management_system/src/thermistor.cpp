#include <ros/ros.h>
#include "lilred_management_system/thermistor.h"

line::line(float point1_x, float point1_y, float point2_x, float point2_y) {
	slope = (point2_y - point1_y) / (point2_x - point1_x);
	point_x = point2_x;
	point_y = point2_y;
}

float line::getFuncValue(float x) {
	return slope * (x - point_x) + point_y;
}


thermistor::thermistor(float resList[]) {
	resPullup = 0;
	vcc = 0;
	resValues = resList;
	//resListLen = sizeof(resList)/sizeof(resList[0]);
	resListLen = (std::size_t)72;
}

void thermistor::setResPullup(float value) {
	resPullup = value;
}

void thermistor::setVcc(float value) {
	vcc = value;
}

void thermistor::fillRtTable(float lowTemp, float highTemp, float increment) {
	//int listLen = sizeof(resValues) / sizeof(resValues[0]);

	float prevResNum = 0;
	float prevTemp = lowTemp;

	for (float i = 1, j = lowTemp + increment; i <= resListLen && j < highTemp; i++, j += increment) {
		line rtLine(resValues[(int)prevResNum], prevTemp, resValues[(int)i], j);
		rtLines.push_back(rtLine);

		prevResNum = i;
		prevTemp = j;
	}
}

float thermistor::voltageToTemp(float voltage) {
	float resTherm = resPullup * (voltage / (vcc - voltage)); // thermistor resistance in ohms
	line* rtLine = rtLineSearch(&resTherm, resValues, resListLen, sizeof(resValues[0]));

	if (rtLine == 0)
		return ERROR_VALUE;
	else
		return rtLine->getFuncValue(resTherm);
}

/* Used on resistance list to determine interval that matches with proper RT line */
line* thermistor::rtLineSearch(float* key, float* base, std::size_t num, std::size_t size) {
	std::size_t start = 0;
	std::size_t end = num;

	if (*key > *base || *key < *(base + end))
		return 0;

	while (start < end) {
		std::size_t mid = start + (end - start) / 2;
		float* toCompare = base + mid;
		ROS_INFO_STREAM(start << " " <<  end << " " << mid);

		if (*key > *toCompare) {
			if (*key < *(toCompare - 1)) {
				return &rtLines[mid-1];
			}
			end = mid;
		}
		else if (*key < *toCompare) {
			if (*key > *(toCompare + 1)) {
				return &rtLines[mid];
			}
			start = mid;
		}
		else {
			return &rtLines[mid];
		}
	}
}
