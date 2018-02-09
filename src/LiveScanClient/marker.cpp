//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#include "marker.h"
#include <opencv2/opencv.hpp>

using namespace std;

MarkerDetector::MarkerDetector() {
	min_size_ = 100;
	max_size_ = 1000000000;
	threshold_ = 120;
	approx_poly_coef_ = 0.12;

	marker_frame_ = 0.4;
	marker_corners_ = 5;

	draw_ = true;

	GetMarkerPointsForWarp(pts_);
}

bool MarkerDetector::get_marker(cv::Mat& img, MarkerInfo& marker) {
	vector<MarkerInfo> markers;
	cv::Mat img2, img3;
	cv::cvtColor(img, img2, CV_BGR2GRAY);
	cv::threshold(img2, img2, threshold_, 255, CV_THRESH_BINARY);

	img2.copyTo(img3);

	vector<vector<cv::Point>> contours;
	cv::findContours(img3, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	for (const auto& contour : contours) {
		vector<cv::Point> corners;
		const double area = cv::contourArea(contour);

		if (area < min_size_ || area > max_size_)
			continue;

		cv::approxPolyDP(contour, corners, sqrt(area) * approx_poly_coef_, true);

		vector<cv::Point2f> cornersFloat;
		cornersFloat.reserve(corners.size());
		for (auto& corner : corners) {
			cornersFloat.emplace_back(cv::Point2f(static_cast<float>(corner.x), static_cast<float>(corner.y)));
		}

		if (!cv::isContourConvex(corners) && corners.size() == marker_corners_ && order_corners(cornersFloat)) {

			int code = get_code(img2, pts_, cornersFloat);

			if (code < 0) {
				reverse(cornersFloat.begin() + 1, cornersFloat.end());
				code = get_code(img2, pts_, cornersFloat);

				if (code < 0)
					continue;
			}

			//I have commented this out as it crashed for some people, if you want additional accuracy in calibration try to uncomment it.
			//CornersSubPix(cornersFloat, contours[i], order);

			vector<Point2f> cornersFloat2(marker_corners_);
			vector<Point3f> points3D;

			for (int i = 0; i < marker_corners_; i++) {
				cornersFloat2[i] = Point2f(cornersFloat[i].x, cornersFloat[i].y);
			}

			GetMarkerPoints(points3D);

			markers.emplace_back(MarkerInfo(code, cornersFloat2, points3D));

			if (draw_) {
				for (unsigned int j = 0; j < corners.size(); j++) {
					cv::circle(img, cornersFloat[j], 2, cv::Scalar(0, 50 * j, 0), 1);
					cv::line(img, cornersFloat[j], cornersFloat[(j + 1) % cornersFloat.size()], cv::Scalar(0, 0, 255), 2);
				}
			}
		}
	}

	if (!markers.empty()) {
		double maxArea = 0;
		int maxInd = 0;

		for (unsigned int i = 0; i < markers.size(); i++) {
			if (get_marker_area(markers[i]) > maxArea) {
				maxInd = i;
				maxArea = get_marker_area(markers[i]);
			}
		}

		marker = markers[maxInd];
		if (draw_) {
			for (int j = 0; j < marker_corners_; j++) {
				const auto pt1 = cv::Point2f(marker.corners[j].X, marker.corners[j].Y);
				const auto pt2 = cv::Point2f(marker.corners[(j + 1) % marker_corners_].X,
											 marker.corners[(j + 1) % marker_corners_].Y);
				cv::line(img, pt1, pt2, cv::Scalar(0, 255, 0), 2);
			}
		}

		return true;
	}
	return false;
}

bool MarkerDetector::get_marker(RGB* img, int height, int width, MarkerInfo& marker) {
	cv::Mat cvImg(height, width, CV_8UC3);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			cvImg.at<cv::Vec3b>(i, j)[0] = img[j + width * i].rgbBlue;
			cvImg.at<cv::Vec3b>(i, j)[1] = img[j + width * i].rgbGreen;
			cvImg.at<cv::Vec3b>(i, j)[2] = img[j + width * i].rgbRed;
		}
	}

	const bool res = get_marker(cvImg, marker);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			img[j + width * i].rgbBlue = cvImg.at<cv::Vec3b>(i, j)[0];
			img[j + width * i].rgbGreen = cvImg.at<cv::Vec3b>(i, j)[1];
			img[j + width * i].rgbRed = cvImg.at<cv::Vec3b>(i, j)[2];
		}
	}

	return res;
}

bool MarkerDetector::order_corners(vector<cv::Point2f>& corners) {
	vector<int> hull;

	cv::convexHull(corners, hull);

	if (hull.size() != corners.size() - 1)
		return false;

	int index = -1;
	for (unsigned int i = 0; i < corners.size(); i++) {
		bool found = false;
		for (int j : hull) {
			if (j == i) {
				found = true;
				break;
			}
		}

		if (!found) {
			index = i;
			break;
		}
	}

	vector<cv::Point2f> corners2;
	for (unsigned int i = 0; i < corners.size(); i++) {
		corners2.push_back(corners[(index + i) % corners.size()]);
	}

	corners = corners2;
	return true;
}

int MarkerDetector::get_code(cv::Mat& img, vector<cv::Point2f> points, vector<cv::Point2f> corners) {
	cv::Mat H, img2;

	int minX = 0, minY = 0;

	double markerInterior = 2 - 2 * marker_frame_;

	for (auto& point : points) {
		point.x = static_cast<float>((point.x - marker_frame_ + 1) * 50);
		point.y = static_cast<float>((point.y - marker_frame_ + 1) * 50);
	}

	H = cv::findHomography(corners, points);
	cv::warpPerspective(img, img2, H, cv::Size((int)(50 * markerInterior), (int)(50 * markerInterior)));

	const int xdiff = img2.cols / 3;
	const int ydiff = img2.rows / 3;
	const int tot = xdiff * ydiff;
	int vals[9];

	cv::Mat integral;
	cv::integral(img2, integral);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			int temp = integral.at<int>((i + 1) * xdiff, (j + 1) * ydiff);
			temp += integral.at<int>(i * xdiff, j * ydiff);
			temp -= integral.at<int>((i + 1) * xdiff, j * ydiff);
			temp -= integral.at<int>(i * xdiff, (j + 1) * ydiff);

			temp = temp / tot;

			if (temp < 128)
				vals[j + i * 3] = 0;
			else if (temp >= 128)
				vals[j + i * 3] = 1;
		}
	}

	int ones = 0;
	int code = 0;
	for (int i = 0; i < 4; i++) {
		if (vals[i] == vals[i + 4])
			return -1;
		if (vals[i] == 1) {
			code += static_cast<int>(pow(2, (double)(3 - i)));
			ones++;
		}
	}

	if (ones / 2 == (float)ones / 2.0) {
		if (vals[8] == 0)
			return -1;
	}

	if (ones / 2 != ones / 2.0) {
		if (vals[8] == 1)
			return -1;
	}

	return code;
}

void MarkerDetector::CornersSubPix(vector<cv::Point2f>& corners, vector<cv::Point> contour, bool order) {
	int* indices = new int[corners.size()];

	for (unsigned int i = 0; i < corners.size(); i++) {
		for (unsigned int j = 0; j < contour.size(); j++) {
			if (corners[i].x == contour[j].x && corners[i].y == contour[j].y) {
				indices[i] = j;
				break;
			}
		}
	}

	vector<cv::Point>* pts = new vector<cv::Point>[corners.size()];

	for (unsigned int i = 0; i < corners.size(); i++) {
		int index1, index2;
		if (order) {
			index1 = indices[i];
			index2 = indices[(i + 1) % corners.size()];
		}
		else {
			index1 = indices[(i + 1) % corners.size()];
			index2 = indices[i];
		}

		if (index1 < index2) {
			pts[i].resize(index2 - index1);
			copy(contour.begin() + index1, contour.begin() + index2, pts[i].begin());
		}
		else {
			pts[i].resize(index2 + contour.size() - index1);
			copy(contour.begin() + index1, contour.end(), pts[i].begin());
			copy(contour.begin(), contour.begin() + index2, pts[i].end() - index2);
		}
	}

	auto lines = new cv::Vec4f[corners.size()];

	for (unsigned int i = 0; i < corners.size(); i++) {
		cv::fitLine(pts[i], lines[i], CV_DIST_L2, 0, 0.01, 0.01);
	}

	vector<cv::Point2f> corners2;
	for (unsigned int i = corners.size() - 1; i < 2 * corners.size() - 1; i++)
		corners2.push_back(GetIntersection(lines[(i + 1) % corners.size()], lines[i % corners.size()]));

	corners = corners2;
}

cv::Point2f MarkerDetector::GetIntersection(cv::Vec4f lin1, cv::Vec4f lin2) {
	float c1 = lin2[2] - lin1[2];
	float c2 = lin2[3] - lin1[3];
	float a1 = lin1[0];
	float a2 = lin1[1];
	float b1 = -lin2[0];
	float b2 = -lin2[1];

	cv::Mat A(2, 2, CV_32F);
	cv::Mat b(2, 1, CV_32F);
	cv::Mat dst(2, 1, CV_32F);

	A.at<float>(0, 0) = a1;
	A.at<float>(0, 1) = b1;
	A.at<float>(1, 0) = a2;
	A.at<float>(1, 1) = b2;
	b.at<float>(0, 0) = c1;
	b.at<float>(1, 0) = c2;

	//Gleichungssystem loesen
	cv::solve(A, b, dst);

	cv::Point2f res(dst.at<float>(0, 0) * lin1[0] + lin1[2], dst.at<float>(0, 0) * lin1[1] + lin1[3]);
	return res;
}

void MarkerDetector::GetMarkerPoints(vector<Point3f>& pts) {
	pts.emplace_back(Point3f(0.0f, -1.0f, 0.0f), Point3f(-1.0f, -1.6667f, 0.0f), Point3f(-1.0f, 1.0f, 0.0f),
	                 Point3f(1.0f, 1.0f, 0.0f), Point3f(1.0f, -1.6667f, 0.0f)
	);
}

void MarkerDetector::GetMarkerPointsForWarp(vector<cv::Point2f>& pts) {
	pts.emplace_back(cv::Point2f(0, 1), (cv::Point2f(-1, 1.6667f)), (cv::Point2f(-1, -1)), (cv::Point2f(1, -1)),
	                 (cv::Point2f(1, 1.6667f)));
}

double MarkerDetector::get_marker_area(MarkerInfo& marker) {
	cv::Mat hull;

	vector<cv::Point2f> cvCorners(marker_corners_);
	for (auto i = 0; i < marker_corners_; i++) {
		cvCorners[i] = cv::Point2f(marker.corners[i].X, marker.corners[i].Y);
	}

	cv::convexHull(cvCorners, hull);
	return cv::contourArea(hull);
}
