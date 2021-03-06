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

#include "utils.h"

// Struktur, die die Daten der Marker speichert
typedef struct MarkerStruct {
	int id;
	// Position der Ecken im Bild
	std::vector<Point2f> corners;
	// Punkte im Marker-Koordinatensystem
	std::vector<Point3f> points;

	MarkerStruct() {
		id = -1;
	}

	MarkerStruct(int id, std::vector<Point2f> corners, std::vector<Point3f> points) {
		this->id = id;

		this->corners = corners;
		this->points = points;
	}
} MarkerInfo;

class IMarker {
public:
	IMarker() {};

	// Finds all markers and stores them into the variable marker
	// Returns: Found at least one marker?
	virtual bool get_marker(RGB* img, int height, int width, MarkerInfo& marker) = 0;
};
