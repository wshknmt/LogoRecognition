#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <cmath>
#include <vector>
int pole[5] = { 0,0,0,0,0 };
int obwod[5] = { 0,0,0,0,0 };
double W3 = 0.0;

double obliczM1(cv::Mat& I, int minI, int maxI, int minJ, int maxJ);
double obliczM7(cv::Mat& I, int minI, int maxI, int minJ, int maxJ);
double obliczMmale(cv::Mat& I, int p, int q, int minI, int maxI, int minJ, int maxJ);
int max(int x, int y) {
	if (x > y) return x;
	else return y;
}
int min(int x, int y) {
	if (x > y) return y;
	else return x;
}
int zwrocJasnosc(int r, int g, int b) {
	return (r + g + b) / 3;
}

double zwrocW3(int k) {
	return (1.0 * obwod[k] / (2 * sqrt(3.141592653589793238463 * pole[k])) - 1.0);
}
int sumaWag(int tab[], int size) {
	int suma = 0;
	for (int i = 0; i < size; ++i) {
		suma += tab[i];
	}
	if (suma == 0) return 1;
    return suma;
}
cv::Mat perform(cv::Mat& I) {
  CV_Assert(I.depth() != sizeof(uchar));
  cv::Mat  res(I.rows, I.cols, CV_8UC3);
  cv::Mat_<cv::Vec3b> _I = I;
  cv::Mat_<cv::Vec3b> _R = res;
  int minI[5] = { I.rows , I.rows , I.rows , I.rows , I.rows};
  int maxI[5] = { 0 , 0 , 0 , 0 , 0 };
  int minJ[5] = { I.cols , I.cols , I.cols , I.cols , I.cols };
  int maxJ[5] = { 0 , 0 , 0 , 0 , 0 };
  int colorR[5] = { 0, 45, 90, 135, 180 };
  for (int i = 1; i < I.rows - 1; ++i) {
	  for (int j = 1; j < I.cols - 1; ++j) {
		  for (int k = 0; k < 5; ++k) {
			  if (_I(i, j)[2] == colorR[k]) {
				  if (minI[k] > i) minI[k] = i;
				  if (minJ[k] > j) minJ[k] = j;
				  if (maxI[k] < i) maxI[k] = i;
				  if (maxJ[k] < j) maxJ[k] = j;

				  //Obliczanie pola
				  pole[k]++;

				  //Obliczanie obwodu
				  if (_I(i - 1, j - 1)[0] == 255 || _I(i - 1, j)[0] == 255 || _I(i - 1, j + 1)[0] == 255 || _I(i, j - 1)[0] == 255 || _I(i, j + 1)[0] == 255 || _I(i + 1, j - 1)[0] == 255 || _I(i + 1, j)[0] == 255 || _I(i + 1, j + 1)[0] == 255) {
					  obwod[k]++;
				  }
			  }
		  }


		  
	  }
  }
  res = _R;
  
  for (int i = 0; i < 5; i++) {
	  double srodekCiezkosciY = obliczMmale(I, 1, 0, minI[i], maxI[i], minJ[i], maxJ[i]) / obliczMmale(I, 0, 0, minI[i], maxI[i], minJ[i], maxJ[i]);
	  double srodekCiezkosciX = obliczMmale(I, 0, 1, minI[i], maxI[i], minJ[i], maxJ[i]) / obliczMmale(I, 0, 0, minI[i], maxI[i], minJ[i], maxJ[i]);
	  double srodekObrazkaX = (minJ[i] + maxJ[i]) / 2;
	  double srodekObrazkaY = (minI[i] + maxI[i]) / 2;
	  //std::cout << "srodekCiezkosciX " << srodekCiezkosciX << " srodekCiezkosciY " << srodekCiezkosciY << " srodekObrazkaX " << srodekObrazkaX << " srodekObrazkaY " << srodekObrazkaY << std::endl;
	  double nachylenie = atan2(srodekObrazkaY - srodekCiezkosciY, srodekObrazkaX - srodekCiezkosciX) * 180 / 3.14;
	  std::cout << "Strzalka numer: " << i + 1 << " pole: " << pole[i] << " , obwod: " << obwod[i] << " , W3: " << zwrocW3(i) << " , M1: " << obliczM1(I, minI[i], maxI[i], minJ[i], maxJ[i]) << " , M7: " << obliczM7(I, minI[i], maxI[i], minJ[i], maxJ[i]) <<" nachylenie : "<< nachylenie << std::endl;
  }
  return res;
}

double obliczMmale(cv::Mat& I, int p, int q, int minI, int maxI, int minJ, int maxJ) {
	CV_Assert(I.depth() != sizeof(uchar));
	cv::Mat_<cv::Vec3b> _I = I;
	double suma = 0.0;
	for (int i = minI; i < maxI; ++i) {
		for (int j = minJ; j < maxJ; ++j) {
			if (_I(i, j)[0] == 0) {
				suma += pow(i, p) * pow(j, q);
			}
		}
	}
	
	return suma;
}

double obliczM20(cv::Mat& I, int minI, int maxI, int minJ, int maxJ) {
	return obliczMmale(I, 2, 0, minI, maxI, minJ, maxJ) - (pow(obliczMmale(I, 1, 0, minI, maxI, minJ, maxJ), 2) / obliczMmale(I, 0, 0, minI, maxI, minJ, maxJ));
}

double obliczM02(cv::Mat& I, int minI, int maxI, int minJ, int maxJ) {
	return obliczMmale(I, 0, 2, minI, maxI, minJ, maxJ) - (pow(obliczMmale(I, 0, 1, minI, maxI, minJ, maxJ), 2) / obliczMmale(I, 0, 0, minI, maxI, minJ, maxJ));
}

double obliczM11(cv::Mat& I, int minI, int maxI, int minJ, int maxJ) {
	return obliczMmale(I, 1, 1, minI, maxI, minJ, maxJ) - (obliczMmale(I, 1, 0, minI, maxI, minJ, maxJ) * obliczMmale(I, 0, 1, minI, maxI, minJ, maxJ) / obliczMmale(I, 0, 0, minI, maxI, minJ, maxJ));
}

double obliczM1(cv::Mat& I, int minI, int maxI, int minJ, int maxJ) {
	return (obliczM20(I, minI, maxI, minJ, maxJ) + obliczM02(I, minI, maxI, minJ, maxJ)) / pow(obliczMmale(I, 0, 0, minI, maxI, minJ, maxJ), 2);
}

double obliczM7(cv::Mat& I, int minI, int maxI, int minJ, int maxJ) {
	return (obliczM20(I, minI, maxI, minJ, maxJ) * obliczM02(I, minI, maxI, minJ, maxJ) - pow(obliczM11(I, minI, maxI, minJ, maxJ), 2)) / pow(obliczMmale(I, 0, 0, minI, maxI, minJ, maxJ), 4);
}

int main(int, char *[]) {
	cv::Mat elipsa = cv::imread("elipsa.dib");
    cv::imshow("Image processed", elipsa);
	/*std::cout << "Elipsa pole: " << pole << " , obwod: " << obwod << " , W3: " << zwrocW3() << " , M1: "<<obliczM1(elipsa)<<" , M7: "<<obliczM7(elipsa) << std::endl;
	pole = 0; obwod = 0;
	cv::waitKey(-1);

	cv::Mat elipsa1 = cv::imread("elipsa1.dib");
	cv::Mat elipsa1_res = perform(elipsa1);
	cv::imshow("Image processed", elipsa1_res);
	std::cout << "Elipsa 1 pole: " << pole << " , obwod: " << obwod << " , W3: "<< zwrocW3() << " , M1: " << obliczM1(elipsa1) << " , M7: " << obliczM7(elipsa1) << std::endl;
	pole = 0; obwod = 0;
	cv::waitKey(-1);

	cv::Mat kolo = cv::imread("kolo.dib");
	cv::Mat kolo_res = perform(kolo);
	cv::imshow("Image processed", kolo_res);
	std::cout << "Kolo pole: " << pole << " , obwod: " << obwod << " , W3: " << zwrocW3() << " , M1: " << obliczM1(kolo) << " , M7: " << obliczM7(kolo) << std::endl;
	pole = 0; obwod = 0;
	cv::waitKey(-1);

	cv::Mat prost = cv::imread("prost.dib");
	cv::Mat prost_res = perform(prost);
	cv::imshow("Image processed", prost_res);
	std::cout << "Prost pole: " << pole << " , obwod: " << obwod << " , W3: " << zwrocW3() << " , M1: " << obliczM1(prost) << " , M7: " << obliczM7(prost) << std::endl;
	pole = 0; obwod = 0;
	cv::waitKey(-1);

	cv::Mat troj = cv::imread("troj.dib");
	cv::Mat troj_res = perform(troj);
	cv::imshow("Image processed", troj_res);
	std::cout << "Troj pole: " << pole << " , obwod: " << obwod << " , W3: " << zwrocW3() << " , M1: " << obliczM1(troj) << " , M7: " << obliczM7(troj) << std::endl;
	pole = 0; obwod = 0;*/
	


	cv::Mat strzalki1 = cv::imread("strzalki_1.dib");
	cv::Mat strzalki1_res = perform(strzalki1);
	/*cv::imshow("Image processed", strzalki1_res);

	
	cv::waitKey(-1);*/
	for (int i = 0; i < 5; ++i) {
		pole[i] = 0;
		obwod[i] = 0;
	}
	std::cout << std::endl;
	cv::Mat strzalki2 = cv::imread("strzalki_2.dib");
	cv::Mat strzalki2_res = perform(strzalki2);

    cv::waitKey(-1);
    return 0;
}
