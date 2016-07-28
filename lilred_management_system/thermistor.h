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
	thermistor(float* resList);
	void setResPullup(float value);
	void setVcc(float value);
	void fillRtTable(float lowTemp, float highTemp, float increment);
	float voltageToTemp(float voltage);

private:
	float resPullup;
	float vcc;
	float* resValues;
	std::vector<line> rtLines;

	void* rtLineSearch(void* key, void* base, size_t num, size_t size);
};
