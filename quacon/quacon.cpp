// quacon.cpp: определяет точку входа для консольного приложения.
//

#include "stdafx.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include "serial.h"

double ctlOut[4] = {};
int ctlHeight = 150;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))
	{
		if (x < ctlHeight)
		{
			ctlOut[0] = ((double)x / ctlHeight) * 2 - 1;
			ctlOut[1] = ((double)y / ctlHeight) * 2 - 1;
			/*if (out[0] > 31) out[0] = 31;
			if (out[0] < -31) out[0] = -31;
			if (out[1] > 31) out[1] = 31;
			if (out[1] < -31) out[1] = -31;*/
		}
		else
		{
			ctlOut[2] = ((double)x / ctlHeight) * 2 - 3;
			ctlOut[3] = ((double)y / ctlHeight) * 2 - 1;
			/*if (out[2] > 31) out[2] = 31;
			if (out[2] < -31) out[2] = -31;
			if (out[3] > 31) out[3] = 31;
			if (out[3] < -31) out[3] = -31;*/
		}
		/*unsigned char o[4] = {
			(out[0] + 32) | 0,
			(out[1] + 32) | 0x40,
			(out[2] + 32) | 0x80,
			(out[3] + 32) | 0xC0
		};*/
		//sendData(o, 4);
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

bool findTwoPoints(std::vector<std::vector<cv::Point>> &cts, cv::Point2d * pts)
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

struct Pos2D
{
	cv::Vec2d pos, dir;
	void arrow(cv::Mat &mat, const cv::Scalar &color) {
		cv::arrowedLine(mat, cv::Point2d(pos - dir), cv::Point2d(pos + dir), color);
	}
};

Pos2D setpoint = { cv::Vec2d(320, 240), cv::Vec2d(0, -70) };

bool getPos2D(cv::Point2d * green, cv::Point2d * orange, Pos2D &pos)
{
	using namespace cv;
	Vec2d gr = green[1] - green[0], go = orange[0] - green[0], or = orange[1] - orange[0];
	if (gr[0] * go[1] - gr[1] * go[0] < 0) std::swap(green[0], green[1]);
	if (or [0] * go[1] - or [1] * go[0] > 0) std::swap(orange[0], orange[1]);
	double gn = norm(gr), on = norm(or), gon = norm(orange[1] - green[0]), ogn = norm(green[1] - orange[0]);
	double med = (gn + on + gon + ogn) / 4;
	if (med < 1) return false;
	gn /= med; on /= med; gon /= med; ogn /= med;
	if (gn < 0.7 || gn > 1.5) return false;
	if (on < 0.7 || on > 1.5) return false;
	if (gon < 0.7 || gon > 1.5) return false;
	if (ogn < 0.7 || ogn > 1.5) return false;
	pos.pos = (green[0] + green[1] + orange[0] + orange[1]) * 0.25;
	pos.dir = (orange[0] + orange[1] - green[0] - green[1]) * 0.25;

	return true;
}

void calcErrors(const Pos2D &sp, const Pos2D &p, double *errors)
{
	double pl2 = p.dir[0] * p.dir[0] + p.dir[1] * p.dir[1];
	double spl = sqrt(sp.dir[0] * sp.dir[0] + sp.dir[1] * sp.dir[1]);
	double pl  = sqrt(pl2);
	double vp = sp.dir[0] * p.dir[1] - sp.dir[1] * p.dir[0];
	if (sp.dir[0] * p.dir[0] + sp.dir[1] * p.dir[1] > 0)
		errors[0] = vp / (spl * pl);
	else
		errors[0] = (vp > 0) ? 1 : -1;
	errors[1] = ((1 / pl) - (1 / spl)) * 100;
	cv::Vec2d ep = p.pos - sp.pos;
	double d = 1 / pl2;
	errors[3] = (ep[0] * p.dir[0] + ep[1] * p.dir[1]) * d;
	errors[2] = -(ep[0] * p.dir[1] - ep[1] * p.dir[0]) * d;
}

void calcOuts(const double *errors, double *outs)
{
	outs[0] = -errors[0];
	if (outs[0] >  1) outs[0] =  1;
	if (outs[0] < -1) outs[0] = -1;
	outs[1] = -errors[1];
	if (outs[1] >  1) outs[1] =  1;
	if (outs[1] < -1) outs[1] = -1;
	outs[2] = -errors[2];
	if (outs[2] >  1) outs[2] =  1;
	if (outs[2] < -1) outs[2] = -1;
	outs[3] = -errors[3];
	if (outs[3] >  1) outs[3] =  1;
	if (outs[3] < -1) outs[3] = -1;
}

int main()
{
	bool ctlOn = false, maskOn = false;
	int count = 0;
	int img = -1;
	cv::VideoCapture cap(0);
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	cv::Mat ctl(ctlHeight, ctlHeight * 2, CV_8UC3);
	cv::Rect rect(0, 0, ctlHeight * 2, ctlHeight);
	cv::namedWindow("Camera", CV_WINDOW_FULLSCREEN);
	cv::setMouseCallback("Camera", cbf, NULL);
	int hue = 10, sat = 80, val = 100;
	cv::createTrackbar("H", "Camera", &hue, 255);
	cv::createTrackbar("S", "Camera", &sat, 255);
	cv::createTrackbar("V", "Camera", &val, 255);

	cv::namedWindow("Control", CV_WINDOW_FULLSCREEN);
	cv::setMouseCallback("Control", CallBackFunc, NULL);

	while (true)
	{
		if(img < 0)
			cap >> cam;
		cv::Mat bl;
		cv::flip(cam, bl, 1);
		cv::blur(bl, bl, cv::Size(3, 3));
		setpoint.arrow(bl, cv::Scalar(255, 0, 0, 128));
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
		if (findTwoPoints(ctGr, grp) && findTwoPoints(ctOr, orp))
		{
			cv::circle(bl, grp[0], 5, cv::Scalar(0, 255, 0, 255), -1);
			cv::circle(bl, grp[1], 5, cv::Scalar(0, 255, 0, 255), -1);
			cv::circle(bl, orp[0], 5, cv::Scalar(0, 0, 255, 255), -1);
			cv::circle(bl, orp[1], 5, cv::Scalar(0, 0, 255, 255), -1);
			Pos2D pos;
			if (getPos2D(grp, orp, pos))
			{
				pos.arrow(bl, cv::Scalar(255, 255, 255, 255));
				double errors[4] = {};
				calcErrors(setpoint, pos, errors);
				calcOuts(errors, ctlOut);
			}
		}
		cv::imshow("Camera", maskOn ? grm : bl);


		cv::rectangle(ctl, rect, cv::Scalar::all(255), CV_FILLED);
		cv::circle(ctl, cv::Point2d((1 + ctlOut[0]) * ctlHeight / 2, (1 - ctlOut[1]) * ctlHeight / 2), 20, cv::Scalar::all(0), -1);
		cv::circle(ctl, cv::Point2d((3 + ctlOut[2]) * ctlHeight / 2, (1 - ctlOut[3]) * ctlHeight / 2), 20, cv::Scalar::all(0), -1);

		cv::imshow("Control", ctl);

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

