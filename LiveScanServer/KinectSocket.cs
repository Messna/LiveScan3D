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
using System.Net.Sockets;

namespace KinectServer {
    public delegate void SocketChangedHandler();

    public class KinectSocket {
        Socket oSocket;
        byte[] byteToSend = new byte[1];
        public bool bFrameCaptured;
        public bool bLatestFrameReceived;
        public bool bStoredFrameReceived;
        public bool bNoMoreStoredFrames = true;

        public bool bCalibrated;

        //The pose of the sensor in the scene (used by the OpenGLWindow to show the sensor)
        public AffineTransform oCameraPose = new AffineTransform();

        //The transform that maps the vertices in the sensor coordinate system to the world corrdinate system.
        public AffineTransform oWorldTransform = new AffineTransform();

        public string sSocketState;

        public List<byte> lFrameRGB = new List<byte>();
        public List<float> lFrameVerts = new List<float>();
        public List<Body> lBodies = new List<Body>();

        public event SocketChangedHandler eChanged;

        public KinectSocket(Socket clientSocket) {
            oSocket = clientSocket;
            sSocketState = oSocket.RemoteEndPoint + " Calibrated = false";
        }

        public void CaptureFrame() {
            bFrameCaptured = false;
            byteToSend[0] = 0;
            SendByte();
        }

        public void Calibrate() {
            bCalibrated = false;
            sSocketState = oSocket.RemoteEndPoint + " Calibrated = false";

            byteToSend[0] = 1;
            SendByte();

            UpdateSocketState();
        }

        public void SendSettings(KinectSettings settings) {
            var lData = settings.ToByteList();

            var bTemp = BitConverter.GetBytes(lData.Count);
            lData.InsertRange(0, bTemp);
            lData.Insert(0, 2);

            if (SocketConnected())
                oSocket.Send(lData.ToArray());
        }

        public void RequestStoredFrame() {
            byteToSend[0] = 3;
            SendByte();
            bNoMoreStoredFrames = false;
            bStoredFrameReceived = false;
        }

        public void RequestLastFrame() {
            byteToSend[0] = 4;
            SendByte();
            bLatestFrameReceived = false;
        }

        public void SendCalibrationData() {
            var size = 1 + (9 + 3) * sizeof(float);
            var data = new byte[size];
            var i = 0;

            data[i] = 5;
            i++;

            Buffer.BlockCopy(oWorldTransform.R, 0, data, i, 9 * sizeof(float));
            i += 9 * sizeof(float);
            Buffer.BlockCopy(oWorldTransform.t, 0, data, i, 3 * sizeof(float));
            i += 3 * sizeof(float);

            if (SocketConnected())
                oSocket.Send(data);
        }

        public void ClearStoredFrames() {
            byteToSend[0] = 6;
            SendByte();
        }

        public void ReceiveCalibrationData() {
            bCalibrated = true;

            var buffer = Receive(sizeof(int) * 1);
            //currently not used
            var markerId = BitConverter.ToInt32(buffer, 0);

            buffer = Receive(sizeof(float) * 9);
            Buffer.BlockCopy(buffer, 0, oWorldTransform.R, 0, sizeof(float) * 9);

            buffer = Receive(sizeof(float) * 3);
            Buffer.BlockCopy(buffer, 0, oWorldTransform.t, 0, sizeof(float) * 3);

            oCameraPose.R = oWorldTransform.R;
            for (var i = 0; i < 3; i++) {
                oCameraPose.t[i] = 0.0f;
                for (var j = 0; j < 3; j++) {
                    oCameraPose.t[i] += oWorldTransform.t[j] * oWorldTransform.R[i, j];
                }
            }

            UpdateSocketState();
        }

        public void ReceiveFrame() {
            lFrameRGB.Clear();
            lFrameVerts.Clear();
            lBodies.Clear();

            var buffer = new byte[1024];

            while (oSocket.Available == 0) {
                if (!SocketConnected())
                    return;
            }

            oSocket.Receive(buffer, 8, SocketFlags.None);
            var bitsToRead = BitConverter.ToInt32(buffer, 0);
            var iCompressed = BitConverter.ToInt32(buffer, 4);

            if (bitsToRead == -1) {
                bNoMoreStoredFrames = true;
                return;
            }

            buffer = new byte[bitsToRead];
            var alreadyRead = 0;

            while (alreadyRead != bitsToRead) {
                while (oSocket.Available == 0) {
                    if (!SocketConnected())
                        return;
                }

                alreadyRead += oSocket.Receive(buffer, alreadyRead, bitsToRead - alreadyRead, SocketFlags.None);
            }


            if (iCompressed == 1)
                buffer = ZstdDecompressor.Decompress(buffer);

            //Receive depth and color data
            var startIdx = 0;

            var vertices = BitConverter.ToInt32(buffer, startIdx);
            startIdx += 4;

            for (var i = 0; i < vertices; i++) {
                for (var j = 0; j < 3; j++) {
                    lFrameRGB.Add(buffer[startIdx++]);
                }

                for (var j = 0; j < 3; j++) {
                    float val = BitConverter.ToInt16(buffer, startIdx);
                    //converting from milimeters to meters
                    val /= 1000.0f;
                    lFrameVerts.Add(val);
                    startIdx += 2;
                }
            }

            //Receive body data
            var nBodies = BitConverter.ToInt32(buffer, startIdx);
            startIdx += 4;
            for (var i = 0; i < nBodies; i++) {
                var tempBody = new Body {bTracked = BitConverter.ToBoolean(buffer, startIdx++)};
                var nJoints = BitConverter.ToInt32(buffer, startIdx);
                startIdx += 4;

                tempBody.lJoints = new List<Joint>(nJoints);
                tempBody.lJointsInColorSpace = new List<Point2f>(nJoints);

                for (var j = 0; j < nJoints; j++) {
                    var tempJoint = new Joint();
                    var tempPoint = new Point2f();

                    tempJoint.jointType = (JointType) BitConverter.ToInt32(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.trackingState = (TrackingState) BitConverter.ToInt32(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.position.X = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.position.Y = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.position.Z = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;

                    tempPoint.X = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;
                    tempPoint.Y = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;

                    tempBody.lJoints.Add(tempJoint);
                    tempBody.lJointsInColorSpace.Add(tempPoint);
                }

                lBodies.Add(tempBody);
            }
        }

        public byte[] Receive(int nBytes) {
            byte[] buffer;
            if (oSocket.Available != 0) {
                buffer = new byte[Math.Min(nBytes, oSocket.Available)];
                oSocket.Receive(buffer, nBytes, SocketFlags.None);
            } else
                buffer = new byte[0];

            return buffer;
        }

        public bool SocketConnected() {
            var part1 = oSocket.Poll(1000, SelectMode.SelectRead);
            var part2 = (oSocket.Available == 0);

            if (part1 && part2) 
                return false;

            return true;
        }

        private void SendByte() {
            oSocket.Send(byteToSend);
        }

        public void UpdateSocketState() {
            if (bCalibrated)
                sSocketState = oSocket.RemoteEndPoint + " Calibrated = true";
            else
                sSocketState = oSocket.RemoteEndPoint + " Calibrated = false";

            eChanged?.Invoke();
        }
    }
}