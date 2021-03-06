﻿//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
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
using System.Globalization;
using System.IO;

namespace KinectServer {
    public struct Point2f {
        public float X;
        public float Y;
    }

    public struct Point3f {
        public float X;
        public float Y;
        public float Z;
    }

    [Serializable]
    public class AffineTransform {
        public float[,] R = new float[3, 3];
        public float[] t = new float[3];

        public AffineTransform() {
            for (var i = 0; i < 3; i++) {
                for (var j = 0; j < 3; j++) {
                    if (i == j)
                        R[i, j] = 1;
                    else
                        R[i, j] = 0;
                }

                t[i] = 0;
            }
        }
    }

    [Serializable]
    public class MarkerPose {
        public readonly AffineTransform pose = new AffineTransform();
        public int Id = -1;

        public MarkerPose() {
            UpdateRotationMatrix();
        }

        public void SetOrientation(float X, float Y, float Z) {
            r[0] = X;
            r[1] = Y;
            r[2] = Z;

            UpdateRotationMatrix();
        }

        public void GetOrientation(out float X, out float Y, out float Z) {
            X = r[0];
            Y = r[1];
            Z = r[2];
        }

        private void UpdateRotationMatrix() {
            var radX = r[0] * (float) Math.PI / 180.0f;
            var radY = r[1] * (float) Math.PI / 180.0f;
            var radZ = r[2] * (float) Math.PI / 180.0f;

            var c1 = (float) Math.Cos(radZ);
            var c2 = (float) Math.Cos(radY);
            var c3 = (float) Math.Cos(radX);
            var s1 = (float) Math.Sin(radZ);
            var s2 = (float) Math.Sin(radY);
            var s3 = (float) Math.Sin(radX);

            //Z Y X rotation
            pose.R[0, 0] = c1 * c2;
            pose.R[0, 1] = c1 * s2 * s3 - c3 * s1;
            pose.R[0, 2] = s1 * s3 + c1 * c3 * s2;
            pose.R[1, 0] = c2 * s1;
            pose.R[1, 1] = c1 * c3 + s1 * s2 * s3;
            pose.R[1, 2] = c3 * s1 * s2 - c1 * s3;
            pose.R[2, 0] = -s2;
            pose.R[2, 1] = c2 * s3;
            pose.R[2, 2] = c2 * c3;
        }

        private float[] r = new float[3];
    }

    public enum TrackingState {
        TrackingState_NotTracked = 0,
        TrackingState_Inferred = 1,
        TrackingState_Tracked = 2
    }

    public enum JointType {
        JointType_SpineBase = 0,
        JointType_SpineMid = 1,
        JointType_Neck = 2,
        JointType_Head = 3,
        JointType_ShoulderLeft = 4,
        JointType_ElbowLeft = 5,
        JointType_WristLeft = 6,
        JointType_HandLeft = 7,
        JointType_ShoulderRight = 8,
        JointType_ElbowRight = 9,
        JointType_WristRight = 10,
        JointType_HandRight = 11,
        JointType_HipLeft = 12,
        JointType_KneeLeft = 13,
        JointType_AnkleLeft = 14,
        JointType_FootLeft = 15,
        JointType_HipRight = 16,
        JointType_KneeRight = 17,
        JointType_AnkleRight = 18,
        JointType_FootRight = 19,
        JointType_SpineShoulder = 20,
        JointType_HandTipLeft = 21,
        JointType_ThumbLeft = 22,
        JointType_HandTipRight = 23,
        JointType_ThumbRight = 24,
        JointType_Count = (JointType_ThumbRight + 1)
    }

    public struct Joint {
        public Point3f position;
        public JointType jointType;
        public TrackingState trackingState;
    }

    public struct Body {
        public bool bTracked;
        public List<Joint> lJoints;
        public List<Point2f> lJointsInColorSpace;
    }

    public static class Utils {
        public static void SaveToPly(string filename, List<float> vertices, List<byte> colors, bool binary) {
            var nVertices = vertices.Count / 3;

            var fileStream = File.Open(filename, FileMode.Create);

            var streamWriter = new StreamWriter(fileStream);
            var binaryWriter = new BinaryWriter(fileStream);

            //PLY file header is written here.
            if (binary)
                streamWriter.WriteLine("ply\nformat binary_little_endian 1.0");
            else
                streamWriter.WriteLine("ply\nformat ascii 1.0\n");
            streamWriter.Write("element vertex " + nVertices + "\n");
            streamWriter.Write("property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n");
            streamWriter.Flush();

            //Vertex and color data are written here.
            if (binary) {
                for (var j = 0; j < vertices.Count / 3; j++) {
                    for (var k = 0; k < 3; k++)
                        binaryWriter.Write(vertices[j * 3 + k]);
                    for (var k = 0; k < 3; k++) {
                        var temp = colors[j * 3 + k];
                        binaryWriter.Write(temp);
                    }
                }
            } else {
                for (var j = 0; j < vertices.Count / 3; j++) {
                    var s = "";
                    for (var k = 0; k < 3; k++)
                        s += vertices[j * 3 + k].ToString(CultureInfo.InvariantCulture) + " ";
                    for (var k = 0; k < 3; k++)
                        s += colors[j * 3 + k].ToString(CultureInfo.InvariantCulture) + " ";
                    streamWriter.WriteLine(s);
                }
            }

            streamWriter.Flush();
            binaryWriter.Flush();
            fileStream.Close();
        }
    }
}