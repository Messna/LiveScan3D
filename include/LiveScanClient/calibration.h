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
#include "marker.h"
#include "utils.h"

vector<float> RotatePoint(vector<float>& point, std::vector<std::vector<float>>& R);
vector<float> InverseRotatePoint(vector<float>& point, std::vector<std::vector<float>>& R);

struct MarkerPose {
	int marker_id;
	float R[3][3];
	float t[3];
};

class calibration {
public:
	vector<float> world_t;
	vector<vector<float>> world_r;
	int used_marker_id{};

	// User defined markers
	vector<MarkerPose> marker_poses;

	bool calibrated;

	calibration();
	~calibration();

	bool calibrate(RGB* buffer, Point3f* camera_coordinates, int color_width, int color_height);
	bool load_calibration();
	void save_calibration();
private:
	IMarker* detector_;
	int sample_counter_;
	int required_samples_;

	vector<vector<Point3f>> marker3d_samples_;

	void procrustes(MarkerInfo& marker, vector<Point3f>& marker_in_world, vector<float>& world_to_marker_t, vector<vector<float>>& world_to_marker_r) const;
	static bool get_marker_corners_3d(vector<Point3f>& marker_3d, MarkerInfo& marker, Point3f* camera_coordinates, int color_width);
};
