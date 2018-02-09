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

using System;
using System.Collections.Generic;
using System.ComponentModel;

namespace KinectServer {
    [Serializable]
    public class KinectSettings {
        public float[] aMinBounds = new float[3];
        public float[] aMaxBounds = new float[3];

        public bool bFilter = false;
        public int nFilterNeighbors = 10;
        public float fFilterThreshold = 0.1f;

        public BindingList<MarkerPose> markerPoses = new BindingList<MarkerPose>();

        public bool bStreamOnlyBodies = false;
        public bool bShowSkeletons = true;
        public int iCompressionLevel = 2; // 0 for no compression, 2 is recommended

        public int nNumICPIterations = 10;
        public int nNumRefineIters = 2;
        public bool bMergeScansForSave = true;
        public bool bSaveAsBinaryPLY = true;

        public KinectSettings() {
            aMinBounds[0] = -5f;
            aMinBounds[1] = -5f;
            aMinBounds[2] = -5f;

            aMaxBounds[0] = 5f;
            aMaxBounds[1] = 5f;
            aMaxBounds[2] = 5f;
        }

        public List<byte> ToByteList() {
            var lData = new List<byte>();

            var bTemp = new byte[sizeof(float) * 3];

            Buffer.BlockCopy(aMinBounds, 0, bTemp, 0, sizeof(float) * 3);
            lData.AddRange(bTemp);
            Buffer.BlockCopy(aMaxBounds, 0, bTemp, 0, sizeof(float) * 3);
            lData.AddRange(bTemp);

            lData.Add(bFilter ? (byte) 1 : (byte) 0);

            bTemp = BitConverter.GetBytes(nFilterNeighbors);
            lData.AddRange(bTemp);

            bTemp = BitConverter.GetBytes(fFilterThreshold);
            lData.AddRange(bTemp);

            bTemp = BitConverter.GetBytes(markerPoses.Count);
            lData.AddRange(bTemp);

            foreach (var markerPose in markerPoses) {
                bTemp = new byte[sizeof(float) * 9];
                Buffer.BlockCopy(markerPose.pose.R, 0, bTemp, 0, sizeof(float) * 9);
                lData.AddRange(bTemp);

                bTemp = new byte[sizeof(float) * 3];
                Buffer.BlockCopy(markerPose.pose.t, 0, bTemp, 0, sizeof(float) * 3);
                lData.AddRange(bTemp);

                bTemp = BitConverter.GetBytes(markerPose.Id);
                lData.AddRange(bTemp);
            }

            lData.Add(bStreamOnlyBodies ? (byte) 1 : (byte) 0);

            bTemp = BitConverter.GetBytes(iCompressionLevel);
            lData.AddRange(bTemp);

            return lData;
        }
    }
}