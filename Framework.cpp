#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <cmath>

//progowanie dla wybranego zakresu 3 sk³adowych koloru
cv::Mat filterByColor(cv::Mat& I, short minR = -1, short maxR = 500, short minG = -1, short maxG = 500, short minB = -1, short maxB = 500) {
	int maxX = 0, maxY = 0;
	int minX = I.cols, minY = I.rows;
	cv::Mat  res(I.rows, I.cols, CV_8UC3);
	cv::Mat_<cv::Vec3b> _I = I;
	cv::Mat_<cv::Vec3b> _R = res;
	int r, g, b;
	for (int i = 0; i < I.rows; ++i) {
		for (int j = 0; j < I.cols; ++j) {
			if (_I(i, j)[0] >= minB && _I(i, j)[1] >= minG && _I(i, j)[2] >= minR
				&& _I(i, j)[0] <= maxB && _I(i, j)[1] <= maxG && _I(i, j)[2] <= maxR)
			{
				_R(i, j)[0] = 1;
				_R(i, j)[1] = 1;
				_R(i, j)[2] = 1;
			}
			else {
				_R(i, j)[0] = 0;
				_R(i, j)[1] = 0;
				_R(i, j)[2] = 0;
			}
		}
	}
	res = _R;
	return res;
}

// Zwraca je¿eli istnieje wspó³rzêdne s¹siaduj¹cego piksela równie¿ nale¿¹cego do obiektu w kolejnoœci zgodnej ze wskazówkami zegara
cv::Vec2i getPositiveNeighbor(cv::Mat_<cv::Vec3s>& I, int ix, int jx) {
	bool found = false;
	int directions[4][2] = { {0, 1},  {1, 0},  {0, -1}, {-1, 0} };
	int i_new, j_new;
	for (int i = 0; i < 4; ++i) {
		i_new = ix + directions[i][0];
		j_new = jx + directions[i][1];
		if (i_new >= 0 && j_new >= 0 && i_new < I.rows && j_new < I.cols && I(i_new, j_new)[0] != 0) {
			found = true;
			break;
		}
	}
	if (found) return cv::Vec2i(i_new, j_new);
	else return cv::Vec2i(ix, jx);
}

// Zwraca je¿eli istnieje wspó³rzêdne s¹siaduj¹cego piksela równie¿ nale¿¹cego do obiektu w kolejnoœci przeciwnej ze wskazówkami zegara
cv::Vec2i getPositiveNeighborInvert(cv::Mat_<cv::Vec3s>& I, int i_prev, int j_prev, int ix, int jx) {
	int directions[4][2] = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };
	int dir = 0;
	if (ix <= i_prev) {
		if (jx < j_prev)      dir = 2;
		else if (i_prev > ix) dir = 1;
	}
	else if (j_prev >= jx)    dir = 3;

	int i_new, j_new;
	dir = dir % 4;
	for (int i = 0; i < 4; ++i) {
		i_new = ix + directions[(dir + i) % 4][0];
		j_new = jx + directions[(dir + i) % 4][1];
		if (i_new >= 0 && j_new >= 0 && i_new < I.rows && j_new < I.cols && I(i_new, j_new)[0] != 0)
			break;
	}
	return cv::Vec2i(i_new, j_new);
}

// Zwraca maksymaln¹ d³ugoœæ konturu dla piksela znaduj¹cego siê wewn¹trz tego konturu
int findContourOfCurrentPosition(cv::Mat_<cv::Vec3s>& _I, int x, int y, int startNextI, int startNextJ, int contourNumber) {
	cv::Vec2i nextPoint;
	int i1 = x, j1 = y, i2 = startNextI, j2 = startNextJ;
	int length = 0;
	while (!(nextPoint[0] == x && nextPoint[1] == y && i1 == startNextI && j1 == startNextJ)) {
		if (j1 + 1 == _I.cols || _I(i1, j1 + 1)[0] > 0)
			_I(i1, j1) = cv::Vec3s(contourNumber, contourNumber, contourNumber);
		else
			_I(i1, j1) = cv::Vec3s(-contourNumber, -contourNumber, -contourNumber);
		nextPoint = getPositiveNeighborInvert(_I, i2, j2, i1, j1);
		if (nextPoint[0] == x && nextPoint[1] == y && i1 == startNextI && j1 == startNextJ)
			break;
		i2 = i1, j2 = j1;
		i1 = nextPoint[0], j1 = nextPoint[1];
		++length;
	}
	return length;
}

// Znajduje wszystkie kontury obiektów znajduj¹cych siê na obrazku
cv::Mat_<cv::Vec3s> findContours(cv::Mat_<cv::Vec3b>& I, int& contourElements, std::vector<int>& circuit, std::vector<float>& area) {
	cv::Mat_<cv::Vec3s> _I = I.clone();
	int contourNumberPrev = 0, contourNumber = 1;
	for (int i = 1; i < I.rows - 1; ++i) {
		contourNumberPrev = 0;
		int start = 0, end = 0;
		for (int j = 1; j < I.cols - 1; ++j) {
			if (_I(i, j)[0] == 1 && _I(i, j - 1)[0] == 0 && contourNumberPrev == 0) {
				++contourNumber;
				contourNumberPrev = contourNumber;
				cv::Vec2i point = getPositiveNeighbor(_I, i, j);
				if (point[0] != i || point[1] != j) {
					int length = findContourOfCurrentPosition(_I, i, j, point[0], point[1], contourNumber);
					circuit.push_back(length);
					area.push_back(0);
				}
				else {
					_I(i, j) = cv::Vec3s(-contourNumber, -contourNumber, -contourNumber);
					circuit.push_back(0);
					area.push_back(0);
				}
			}
			else if (_I(i, j)[0] > 1) {
				contourNumberPrev = _I(i, j)[0];
				start = j;
			}
			if (_I(i, j)[0] < 0 && contourNumberPrev != 0) {
				if (start != 0) {
					end = j;
					area[contourNumberPrev - 2] += end - start;
				}
				contourNumberPrev = 0, start = 0, end = 0;
			}
		}
	}
	contourElements = contourNumber - 1;
	return _I;
}

cv::Mat perform(cv::Mat I) {
	cv::Mat_<cv::Vec3b> result;
	cv::Mat_<cv::Vec3b> _I = I;
	cv::Mat_<cv::Vec3b> yellow = filterByColor(I, 185, 256, 140, 230, 0, 100); //zolty
	cv::imshow("Yello", yellow);
	int contourNumbers = 0;
	std::vector<int> circuit;
	std::vector<float> area;
	cv::Mat_<cv::Vec3s> contourMap = findContours(yellow, contourNumbers, circuit, area);
	std::vector<int> redArea;
	cv::Mat_<cv::Vec3b> red = filterByColor(I, 110, 220, 0, 135, 5, 70); //czerwony
	for (int i = 0; i < area.size(); ++i) {
		redArea.push_back(0);
	}
	for (int i = 0; i < I.rows; ++i) {
		int pixVal = 0;
		int pixValSave = 2;
		bool redBegin = false;
		for (int j = 0; j < I.cols; ++j) {
			int val = contourMap(i, j)[0];
			if (val > 1) {
				pixVal = val;
			}
			if (val < 0) {
				pixVal = 0;
			}
			int redVal = red(i, j)[0];
			if (redVal > 0 && pixVal > 0) {
				redArea[pixVal - 2]++;
				pixValSave = pixVal;
				redBegin = true;
			}
			else if (redVal > 0 && redBegin)
			{
				redArea[pixValSave - 2]++;
			}
			else if (redVal == 0)
			{
				redBegin = false;

			}
		}
	}
	std::vector<int> goodContours;
	for (int i = 0; i < contourNumbers; ++i) {
		if (area[i] > 100 && redArea[i] / area[i] > 0.45) {
			goodContours.push_back(i);
		}
	}
	result = _I.clone();
	for (int i = 0; i < I.rows; ++i) {
		int pixVal = 0;
		for (int j = 0; j < I.cols; ++j) {
			int val = contourMap(i, j)[0];
			if (val > 1 && find(goodContours.begin(), goodContours.end(), val - 2) != goodContours.end()) {
				pixVal = val;
			}
			if (val < 0) {
				pixVal = 0;
			}
			if (pixVal > 0) {
				result(i, j)[0] = 255;
				result(i, j)[1] = 204;
				result(i, j)[2] = 0;
			}
		}
	}
	for (int i = 0; i < area.size(); ++i) {
		std::cout << " i " << i << " ,redArea" << redArea[i] << std::endl;
	}
	return result;
}
int main(int, char* []) {

	cv::Mat image = cv::imread("metro1.jpg");
	cv::imshow("Oryginal", image);
	cv::Mat processedImage = perform(image);
	//processedImage = image;
	cv::imshow("Processed", processedImage);
	cv::imwrite("Metrooo.jpg", processedImage);

	cv::waitKey(-1);
	return 0;
}
