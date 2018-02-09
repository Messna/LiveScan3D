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

#include "calibration.h"
#include "Kinect.h"
#include "opencv/cv.h"

#include <fstream>

calibration::calibration() {
	calibrated = false;
	sample_counter_ = 0;
	required_samples_ = 20;

	world_t = vector<float>(3, 0.0f);
	for (int i = 0; i < 3; i++) {
		world_r.emplace_back(vector<float>(3, 0.0f));
		world_r[i][i] = 1.0f;
	}

	detector_ = new MarkerDetector();
}

calibration::~calibration() {
	if (detector_ != nullptr) {
		delete detector_;
		detector_ = nullptr;
	}
}

bool calibration::calibrate(RGB* buffer, Point3f* camera_coordinates, int color_width, int color_height) {
	MarkerInfo marker;

	const bool markers_found = detector_->get_marker(buffer, color_height, color_width, marker);
	if (!markers_found) return false;

	int index_in_poses = -1;

	for (unsigned int j = 0; j < marker_poses.size(); j++) {
		if (marker.id == marker_poses[j].marker_id) {
			index_in_poses = j;
			break;
		}
	}
	if (index_in_poses == -1)
		return false;

	const auto marker_pose = marker_poses[index_in_poses];
	used_marker_id = marker_pose.marker_id;

	vector<Point3f> marker_in_world(marker.corners.size());
	const bool success = get_marker_corners_3d(marker_in_world, marker, camera_coordinates, color_width);

	if (!success)
		return false;

	marker3d_samples_.push_back(marker_in_world);
	sample_counter_++;

	if (sample_counter_ < required_samples_)
		return false;

	for (size_t i = 0; i < marker_in_world.size(); i++) {
		marker_in_world[i] = Point3f();
		for (int j = 0; j < required_samples_; j++) {
			marker_in_world[i].X += marker3d_samples_[j][i].X / static_cast<float>(required_samples_);
			marker_in_world[i].Y += marker3d_samples_[j][i].Y / static_cast<float>(required_samples_);
			marker_in_world[i].Z += marker3d_samples_[j][i].Z / static_cast<float>(required_samples_);
		}
	}

	procrustes(marker, marker_in_world, world_t, world_r);

	auto r_copy = world_r;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			world_r[i][j] = 0;

			for (int k = 0; k < 3; k++) {
				world_r[i][j] += marker_pose.R[i][k] * r_copy[k][j];
			}
		}
	}

	vector<float> translationIncr(3);
	translationIncr[0] = marker_pose.t[0];
	translationIncr[1] = marker_pose.t[1];
	translationIncr[2] = marker_pose.t[2];;

	translationIncr = InverseRotatePoint(translationIncr, world_r);

	world_t[0] += translationIncr[0];
	world_t[1] += translationIncr[1];
	world_t[2] += translationIncr[2];

	calibrated = true;

	marker3d_samples_.clear();
	sample_counter_ = 0;

	save_calibration();

	return true;
}

bool calibration::load_calibration() {
	ifstream file;
	file.open("calibration.txt");
	if (!file.is_open())
		return false;

	for (int i = 0; i < 3; i++)
		file >> world_t[i];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			file >> world_r[i][j];
	}
	file >> used_marker_id;
	file >> calibrated;

	return true;
}

void calibration::save_calibration() {
	ofstream file;
	file.open("calibration.txt");
	for (int i = 0; i < 3; i++)
		file << world_t[i] << " ";
	file << endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			file << world_r[i][j];
		file << endl;
	}
	file << used_marker_id << endl;
	file << calibrated << endl;

	file.close();
}

void calibration::procrustes(MarkerInfo& marker, vector<Point3f>& marker_in_world, vector<float>& world_to_marker_t, vector<vector<float>>& world_to_marker_r) const {
	const int vertices = marker.points.size();

	Point3f markerCenterInWorld;
	Point3f markerCenter;
	for (int i = 0; i < vertices; i++) {
		markerCenterInWorld.X += marker_in_world[i].X / vertices;
		markerCenterInWorld.Y += marker_in_world[i].Y / vertices;
		markerCenterInWorld.Z += marker_in_world[i].Z / vertices;

		markerCenter.X += marker.points[i].X / vertices;
		markerCenter.Y += marker.points[i].Y / vertices;
		markerCenter.Z += marker.points[i].Z / vertices;
	}

	world_to_marker_t.resize(3);
	world_to_marker_t[0] = -markerCenterInWorld.X;
	world_to_marker_t[1] = -markerCenterInWorld.Y;
	world_to_marker_t[2] = -markerCenterInWorld.Z;

	vector<Point3f> markerInWorldTranslated(vertices);
	vector<Point3f> markerTranslated(vertices);
	for (int i = 0; i < vertices; i++) {
		markerInWorldTranslated[i].X = marker_in_world[i].X + world_to_marker_t[0];
		markerInWorldTranslated[i].Y = marker_in_world[i].Y + world_to_marker_t[1];
		markerInWorldTranslated[i].Z = marker_in_world[i].Z + world_to_marker_t[2];

		markerTranslated[i].X = marker.points[i].X - markerCenter.X;
		markerTranslated[i].Y = marker.points[i].Y - markerCenter.Y;
		markerTranslated[i].Z = marker.points[i].Z - markerCenter.Z;
	}

	cv::Mat A(vertices, 3, CV_64F);
	cv::Mat B(vertices, 3, CV_64F);

	for (int i = 0; i < vertices; i++) {
		A.at<double>(i, 0) = markerTranslated[i].X;
		A.at<double>(i, 1) = markerTranslated[i].Y;
		A.at<double>(i, 2) = markerTranslated[i].Z;

		B.at<double>(i, 0) = markerInWorldTranslated[i].X;
		B.at<double>(i, 1) = markerInWorldTranslated[i].Y;
		B.at<double>(i, 2) = markerInWorldTranslated[i].Z;
	}

	const cv::Mat M = A.t() * B;

	cv::SVD svd;
	svd(M);
	cv::Mat R = svd.u * svd.vt;

	const double det = cv::determinant(R);
	if (det < 0) {
		cv::Mat temp = cv::Mat::eye(3, 3, CV_64F);
		temp.at<double>(2, 2) = -1;
		R = svd.u * temp * svd.vt;
	}

	world_to_marker_r.resize(3);

	for (int i = 0; i < 3; i++) {
		world_to_marker_r[i].resize(3);
		for (int j = 0; j < 3; j++) {
			world_to_marker_r[i][j] = static_cast<float>(R.at<double>(i, j));
		}
	}
}

bool calibration::get_marker_corners_3d(vector<Point3f>& marker_3d, MarkerInfo& marker, Point3f* camera_coordinates, int color_width) {
	for (unsigned int i = 0; i < marker.corners.size(); i++) {  // NOLINT
		const auto min_x = static_cast<int>(marker.corners[i].X);
		const auto max_x = min_x + 1;
		const auto min_y = static_cast<int>(marker.corners[i].Y);
		const auto max_y = min_y + 1;

		const auto dx = marker.corners[i].X - min_x;
		const auto dy = marker.corners[i].Y - min_y;

		const auto point_min = camera_coordinates[min_x + min_y * color_width];
		const auto point_x_max_y_min = camera_coordinates[max_x + min_y * color_width];
		const auto point_x_min_y_max = camera_coordinates[min_x + max_y * color_width];
		const auto point_max = camera_coordinates[max_x + max_y * color_width];

		if (point_min.Z < 0 || point_x_max_y_min.Z < 0 || point_x_min_y_max.Z < 0 || point_max.Z < 0)
			return false;

		marker_3d[i].X = (1 - dx) * (1 - dy) * point_min.X + dx * (1 - dy) * point_x_max_y_min.X + (1 - dx) * dy * point_x_min_y_max.X +
			dx * dy * point_max.X;
		marker_3d[i].Y = (1 - dx) * (1 - dy) * point_min.Y + dx * (1 - dy) * point_x_max_y_min.Y + (1 - dx) * dy * point_x_min_y_max.Y +
			dx * dy * point_max.Y;
		marker_3d[i].Z = (1 - dx) * (1 - dy) * point_min.Z + dx * (1 - dy) * point_x_max_y_min.Z + (1 - dx) * dy * point_x_min_y_max.Z +
			dx * dy * point_max.Z;
	}

	return true;
}

vector<float> InverseRotatePoint(vector<float>& point, std::vector<std::vector<float>>& R) {
	vector<float> res(3);

	res[0] = point[0] * R[0][0] + point[1] * R[1][0] + point[2] * R[2][0];
	res[1] = point[0] * R[0][1] + point[1] * R[1][1] + point[2] * R[2][1];
	res[2] = point[0] * R[0][2] + point[1] * R[1][2] + point[2] * R[2][2];

	return res;
}

vector<float> RotatePoint(vector<float>& point, std::vector<std::vector<float>>& R) {
	vector<float> res(3);

	res[0] = point[0] * R[0][0] + point[1] * R[0][1] + point[2] * R[0][2];
	res[1] = point[0] * R[1][0] + point[1] * R[1][1] + point[2] * R[1][2];
	res[2] = point[0] * R[2][0] + point[1] * R[2][1] + point[2] * R[2][2];

	return res;
}
