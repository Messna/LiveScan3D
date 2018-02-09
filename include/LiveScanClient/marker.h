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
#pragma once

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include "utils.h"
#include "iMarker.h"

using namespace std;

class MarkerDetector : public IMarker {
public:
	MarkerDetector();

	bool get_marker(RGB* img, int height, int width, MarkerInfo& marker) override;
private:
	int marker_corners_;
	vector<cv::Point2f> pts_;

	int min_size_;
	int max_size_;
	int threshold_;
	double approx_poly_coef_;
	double marker_frame_;
	bool draw_;

	bool get_marker(cv::Mat& img, MarkerInfo& marker);
	static bool order_corners(vector<cv::Point2f>& corners);
	int get_code(cv::Mat& img, vector<cv::Point2f> points, vector<cv::Point2f> corners);
	void CornersSubPix(vector<cv::Point2f>& corners, vector<cv::Point> contour, bool order);
	cv::Point2f GetIntersection(cv::Vec4f lin1, cv::Vec4f lin2);
	void GetMarkerPoints(vector<Point3f>& pts);
	void GetMarkerPointsForWarp(vector<cv::Point2f>& pts);
	double get_marker_area(MarkerInfo& marker);
};
