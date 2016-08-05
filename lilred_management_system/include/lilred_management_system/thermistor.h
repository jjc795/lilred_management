/*
 * thermistor.h
 * Jon Cruz
 *
 * Header for thermistor data processing
 */

#include <vector>

#define ERROR_VALUE 1000

class line {
public:
	line(float point1_x, float point1_y, float point2_x, float point2_y);
	float getFuncValue(float x);

private:
	float slope;
	float point_x;
	float point_y;
};

class thermistor {
public:
	thermistor(float resList[], std::size_t listLen);
	void setResPullup(float value);
	void setVcc(float value);
	void fillRtTable(float lowTemp, float highTemp, float increment);
	float voltageToTemp(float voltage);
	float cToF(float temp);
	float fToC(float temp);

private:
	float resPullup;
	float vcc;
	float* resValues;
	std::size_t resListLen;
	std::vector<line> rtLines;

	line* rtLineSearch(float* key, float* base, std::size_t num);
};
