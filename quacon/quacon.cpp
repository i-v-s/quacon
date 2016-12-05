// quacon.cpp: определяет точку входа для консольного приложения.
//

#include "stdafx.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include "serial.h"

int out[4] = {};


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))
	{
		if (x < 400)
		{
			out[0] = x - 200;
			out[1] = -(y - 200);
			if (out[0] > 31) out[0] = 31;
			if (out[0] < -31) out[0] = -31;
			if (out[1] > 31) out[1] = 31;
			if (out[1] < -31) out[1] = -31;
		}
		else
		{
			out[2] = x - 600;
			out[3] = -(y - 200);
			if (out[2] > 31) out[2] = 31;
			if (out[2] < -31) out[2] = -31;
			if (out[3] > 31) out[3] = 31;
			if (out[3] < -31) out[3] = -31;
		}
		unsigned char o[4] = {
			(out[0] + 32) | 0,
			(out[1] + 32) | 0x40,
			(out[2] + 32) | 0x80,
			(out[3] + 32) | 0xC0
		};
		sendData(o, 4);
		//printf();
		//cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}

cv::Mat cam, hsv;
cv::Vec3b grcol(73, 238, 153), orcol(1, 255, 74);


void cbf(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		grcol = hsv.at<cv::Vec3b>(y, x);
	}

}

bool find2Pts(std::vector<std::vector<cv::Point>> &cts, cv::Point2d * pts)
{
	using namespace cv;
	using namespace std;
	if (cts.size() < 2) return false;
	std::sort(cts.begin(), cts.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
		return cv::contourArea(c1, false) > cv::contourArea(c2, false);
	});

	vector<Moments> mu(2);
	for (int i = 0; i < 2; i++)
		mu[i] = moments(cts[i], false);

	///  Get the mass centers:
	for (int i = 0; i < 2; i++)
	{
		pts[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	return true;
}

int main()
{
	bool ctlOn = false, maskOn = false;
	int count = 0;
	int img = -1;
	cv::VideoCapture cap(0);
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	cv::Mat ctl(400, 800, CV_8UC3);
	cv::Rect rect(0, 0, 800, 400);
	cv::namedWindow("Camera", CV_WINDOW_FULLSCREEN);
	cv::setMouseCallback("Camera", cbf, NULL);
	int hue = 10, sat = 80, val = 100;
	cv::createTrackbar("H", "Camera", &hue, 255);
	cv::createTrackbar("S", "Camera", &sat, 255);
	cv::createTrackbar("V", "Camera", &val, 255);

	while (true)
	{
		if (ctlOn) {
			cv::rectangle(ctl, rect, cv::Scalar::all(255), CV_FILLED);
			cv::circle(ctl, cv::Point(200 + out[0], 200 - out[1]), 20, cv::Scalar::all(0), -1);
			cv::circle(ctl, cv::Point(600 + out[2], 200 - out[3]), 20, cv::Scalar::all(0), -1);

			cv::imshow("Control", ctl);
		}

		if(img < 0)
			cap >> cam;
		cv::Mat bl;
		cv::blur(cam, bl, cv::Size(3, 3));
		cv::cvtColor(bl, hsv, CV_BGR2HSV);
		cv::Mat grm, orm;
		cv::Vec3b d(hue, sat, val);

		cv::inRange(hsv, grcol - d, grcol + d, grm);
		cv::inRange(hsv, orcol - d, orcol + d, orm);
		std::vector<std::vector<cv::Point> > ctGr, ctOr;
		cv::findContours(grm, ctGr, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		cv::findContours(orm, ctOr, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		//for(int x = 0; x < contours.size(); x++)
		cv::drawContours(bl, ctGr, -1, cv::Scalar(0, 255, 0, 255));
		cv::drawContours(bl, ctOr, -1, cv::Scalar(0, 0, 255, 255));
		cv::Point2d grp[2], orp[2];
		if (find2Pts(ctGr, grp) && find2Pts(ctOr, orp))
		{
			cv::circle(bl, grp[0], 5, cv::Scalar(0, 255, 0, 255), -1);
			cv::circle(bl, grp[1], 5, cv::Scalar(0, 255, 0, 255), -1);
			cv::circle(bl, orp[0], 5, cv::Scalar(0, 0, 255, 255), -1);
			cv::circle(bl, orp[1], 5, cv::Scalar(0, 0, 255, 255), -1);
		}

		cv::imshow("Camera", maskOn ? grm : bl);
		switch (int k = cv::waitKey(10))
		{
		case 's': 
			cv::imwrite("s" + std::to_string(count++) + ".png", cam);
			break;
		case 'm': maskOn = !maskOn; break;
		case 'c': if (ctlOn) {

				ctlOn = false;
			} else {
				connect();
				cv::namedWindow("Control", CV_WINDOW_FULLSCREEN);
				cv::setMouseCallback("Control", CallBackFunc, NULL);
				ctlOn = true;
			}
			break;
		case 27: return 0;
		case '0': img = -1; break;
		default: if (k >= '0' && k <= '9') {
				cam = cv::imread("s" + std::to_string(k - '0') + ".png");
				cv::Size sz = cam.size();
				if( sz.width && sz.height) img = k - '0';
				else img = -1;
				break;
			}
		}
	}
    return 0;
}

