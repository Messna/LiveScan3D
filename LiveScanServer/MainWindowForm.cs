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
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using System.Threading;
using System.Windows.Forms;
using Timer = System.Timers.Timer;

namespace KinectServer {
    public partial class MainWindowForm : Form {
        [DllImport("ICP.dll")]
        static extern float ICP(IntPtr verts1, IntPtr verts2, int nVerts1, int nVerts2, float[] R, float[] t,
                                int maxIter = 200);

        KinectServer oServer;
        TransferServer oTransferServer;

        //Those three variables are shared with the OpenGLWindow class and are used to exchange data with it.
        //Vertices from all of the sensors
        List<float> lAllVertices = new List<float>();

        //Color data from all of the sensors
        List<byte> lAllColors = new List<byte>();

        //Sensor poses from all of the sensors
        List<AffineTransform> lAllCameraPoses = new List<AffineTransform>();

        //Body data from all of the sensors
        List<Body> lAllBodies = new List<Body>();

        bool bServerRunning;
        bool bRecording;
        bool bSaving;

        //Live view open or not
        bool bLiveViewRunning;

        Timer oStatusBarTimer = new Timer();

        KinectSettings oSettings = new KinectSettings();

        //The live view window class
        OpenGlWindow oOpenGLWindow;

        public MainWindowForm() {
            //This tries to read the settings from "settings.bin", if it failes the settings stay at default values.
            try {
                IFormatter formatter = new BinaryFormatter();
                Stream stream = new FileStream("settings.bin", FileMode.Open, FileAccess.Read);
                oSettings = (KinectSettings) formatter.Deserialize(stream);
                stream.Close();
            } catch (Exception) {
                // ignored
            }

            oServer = new KinectServer(oSettings);
            oServer.ESocketListChanged += UpdateListView;
            oTransferServer = new TransferServer {
                Vertices = lAllVertices,
                Colors = lAllColors
            };

            InitializeComponent();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e) {
            //The current settings are saved to a files.
            IFormatter formatter = new BinaryFormatter();

            Stream stream = new FileStream("settings.bin", FileMode.Create, FileAccess.Write);
            formatter.Serialize(stream, oSettings);
            stream.Close();

            oServer.StopServer();
            oTransferServer.StopServer();
        }

        //Starts the server
        private void btStart_Click(object sender, EventArgs e) {
            bServerRunning = !bServerRunning;

            if (bServerRunning) {
                oServer.StartServer();
                oTransferServer.StartServer();
                btStart.Text = "Stop server";
            } else {
                oServer.StopServer();
                oTransferServer.StopServer();
                btStart.Text = "Start server";
            }
        }

        //Opens the settings form
        private void btSettings_Click(object sender, EventArgs e) {
            var form = new SettingsForm {
                Settings = oSettings,
                Server = oServer
            };
            form.Show();
        }

        //Performs recording which is synchronized frame capture.
        //The frames are downloaded from the clients and saved once recording is finished.
        private void recordingWorker_DoWork(object sender, DoWorkEventArgs e) {
            oServer.ClearStoredFrames();

            var nCaptured = 0;
            var worker = (BackgroundWorker) sender;
            while (!worker.CancellationPending) {
                oServer.CaptureSynchronizedFrame();

                nCaptured++;
                SetStatusBarOnTimer("Captured frame " + (nCaptured) + ".", 5000);
            }
        }

        private void recordingWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e) {
            //After recording has been terminated it is time to begin saving the frames.
            //Saving is downloading the frames from clients and saving them locally.
            bSaving = true;

            btRecord.Text = "Stop saving";
            btRecord.Enabled = true;

            savingWorker.RunWorkerAsync();
        }

        //Opens the live view window
        private void OpenGLWorker_DoWork(object sender, DoWorkEventArgs e) {
            bLiveViewRunning = true;
            oOpenGLWindow = new OpenGlWindow();

            //The variables below are shared between this class and the OpenGLWindow.
            lock (lAllVertices) {
                oOpenGLWindow.vertices = lAllVertices;
                oOpenGLWindow.colors = lAllColors;
                oOpenGLWindow.cameraPoses = lAllCameraPoses;
                oOpenGLWindow.bodies = lAllBodies;
                oOpenGLWindow.settings = oSettings;
            }

            oOpenGLWindow.Run();
        }

        private void OpenGLWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e) {
            bLiveViewRunning = false;
            updateWorker.CancelAsync();
        }

        private void savingWorker_DoWork(object sender, DoWorkEventArgs e) {
            var nFrames = 0;

            var outDir = "out" + "\\" + txtSeqName.Text + "\\";
            var di = Directory.CreateDirectory(outDir);

            var worker = (BackgroundWorker) sender;
            //This loop is running till it is either cancelled (using the btRecord button), or till there are no more stored frames.
            while (!worker.CancellationPending) {
                var frameRgbAllDevices = new List<List<byte>>();
                var frameVertsAllDevices = new List<List<float>>();

                var success = oServer.GetStoredFrame(frameRgbAllDevices, frameVertsAllDevices);

                //This indicates that there are no more stored frames.
                if (!success)
                    break;

                nFrames++;
                var nVerticesTotal = 0;
                for (var i = 0; i < frameRgbAllDevices.Count; i++) {
                    nVerticesTotal += frameVertsAllDevices[i].Count;
                }

                var frameRgb = new List<byte>();
                var frameVerts = new List<float>();

                SetStatusBarOnTimer("Saving frame " + (nFrames) + ".", 5000);
                for (var i = 0; i < frameRgbAllDevices.Count; i++) {
                    frameRgb.AddRange(frameRgbAllDevices[i]);
                    frameVerts.AddRange(frameVertsAllDevices[i]);

                    //This is ran if the frames from each client are to be placed in separate files.
                    if (!oSettings.bMergeScansForSave) {
                        var outputFilename = outDir + "\\" + nFrames.ToString().PadLeft(5, '0') + i + ".ply";
                        Utils.SaveToPly(outputFilename, frameVertsAllDevices[i], frameRgbAllDevices[i],
                                        oSettings.bSaveAsBinaryPLY);
                    }
                }

                //This is ran if the frames from all clients are to be placed in a single file.
                if (oSettings.bMergeScansForSave) {
                    var outputFilename = outDir + "\\" + nFrames.ToString().PadLeft(5, '0') + ".ply";
                    Utils.SaveToPly(outputFilename, frameVerts, frameRgb, oSettings.bSaveAsBinaryPLY);
                }
            }
        }

        private void savingWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e) {
            oServer.ClearStoredFrames();
            bSaving = false;

            //If the live view window was open, we need to restart the UpdateWorker.
            if (bLiveViewRunning)
                RestartUpdateWorker();

            btRecord.Enabled = true;
            btRecord.Text = "Start recording";
            btRefineCalib.Enabled = true;
            btCalibrate.Enabled = true;
        }

        //Continually requests frames that will be displayed in the live view window.
        private void updateWorker_DoWork(object sender, DoWorkEventArgs e) {
            var lFramesRGB = new List<List<byte>>();
            var lFramesVerts = new List<List<float>>();
            var lFramesBody = new List<List<Body>>();

            var worker = (BackgroundWorker) sender;
            while (!worker.CancellationPending) {
                Thread.Sleep(1);
                oServer.GetLatestFrame(lFramesRGB, lFramesVerts, lFramesBody);

                //Update the vertex and color lists that are common between this class and the OpenGLWindow.
                lock (lAllVertices) {
                    lAllVertices.Clear();
                    lAllColors.Clear();
                    lAllBodies.Clear();
                    lAllCameraPoses.Clear();

                    for (var i = 0; i < lFramesRGB.Count; i++) {
                        lAllVertices.AddRange(lFramesVerts[i]);
                        lAllColors.AddRange(lFramesRGB[i]);
                        lAllBodies.AddRange(lFramesBody[i]);
                    }

                    lAllCameraPoses.AddRange(oServer.CameraPoses);
                }

                //Notes the fact that a new frame was downloaded, this is used to estimate the FPS.
                if (oOpenGLWindow != null)
                    oOpenGLWindow.CloudUpdateTick();
            }
        }

        //Performs the ICP based pose refinement.
        private void refineWorker_DoWork(object sender, DoWorkEventArgs e) {
            if (oServer.AllCalibrated == false) {
                SetStatusBarOnTimer("Not all of the devices are calibrated.", 5000);
                return;
            }

            //Download a frame from each client.
            var lAllFrameVertices = new List<List<float>>();
            var lAllFrameColors = new List<List<byte>>();
            var lAllFrameBody = new List<List<Body>>();
            oServer.GetLatestFrame(lAllFrameColors, lAllFrameVertices, lAllFrameBody);

            //Initialize containers for the poses.
            var Rs = new List<float[]>();
            var Ts = new List<float[]>();
            for (var i = 0; i < lAllFrameVertices.Count; i++) {
                var tempR = new float[9];
                var tempT = new float[3];
                for (var j = 0; j < 3; j++) {
                    tempT[j] = 0;
                    tempR[j + j * 3] = 1;
                }

                Rs.Add(tempR);
                Ts.Add(tempT);
            }

            //Use ICP to refine the sensor poses.
            //This part is explained in more detail in our article (name on top of this file).

            for (var refineIter = 0; refineIter < oSettings.nNumRefineIters; refineIter++) {
                for (var i = 0; i < lAllFrameVertices.Count; i++) {
                    var otherFramesVertices = new List<float>();
                    for (var j = 0; j < lAllFrameVertices.Count; j++) {
                        if (j == i)
                            continue;
                        otherFramesVertices.AddRange(lAllFrameVertices[j]);
                    }

                    var verts1 = otherFramesVertices.ToArray();
                    var verts2 = lAllFrameVertices[i].ToArray();

                    var pVerts1 = Marshal.AllocHGlobal(otherFramesVertices.Count * sizeof(float));
                    var pVerts2 = Marshal.AllocHGlobal(lAllFrameVertices[i].Count * sizeof(float));

                    Marshal.Copy(verts1, 0, pVerts1, verts1.Length);
                    Marshal.Copy(verts2, 0, pVerts2, verts2.Length);

                    ICP(pVerts1, pVerts2, otherFramesVertices.Count / 3, lAllFrameVertices[i].Count / 3, Rs[i], Ts[i],
                        oSettings.nNumICPIterations);

                    Marshal.Copy(pVerts2, verts2, 0, verts2.Length);
                    lAllFrameVertices[i].Clear();
                    lAllFrameVertices[i].AddRange(verts2);
                }
            }

            //Update the calibration data in client machines.
            var worldTransforms = oServer.WorldTransforms;
            var cameraPoses = oServer.CameraPoses;

            for (var i = 0; i < worldTransforms.Count; i++) {
                var tempT = new float[3];
                var tempR = new float[3, 3];
                for (var j = 0; j < 3; j++) {
                    for (var k = 0; k < 3; k++) {
                        tempT[j] += Ts[i][k] * worldTransforms[i].R[k, j];
                    }

                    worldTransforms[i].t[j] += tempT[j];
                    cameraPoses[i].t[j] += Ts[i][j];
                }

                for (var j = 0; j < 3; j++) {
                    for (var k = 0; k < 3; k++) {
                        for (var l = 0; l < 3; l++) {
                            tempR[j, k] += Rs[i][l * 3 + j] * worldTransforms[i].R[l, k];
                        }

                        worldTransforms[i].R[j, k] = tempR[j, k];
                        cameraPoses[i].R[j, k] = tempR[j, k];
                    }
                }
            }

            oServer.WorldTransforms = worldTransforms;
            oServer.CameraPoses = cameraPoses;

            oServer.SendCalibrationData();
        }

        private void refineWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e) {
            //Re-enable all of the buttons after refinement.
            btRefineCalib.Enabled = true;
            btCalibrate.Enabled = true;
            btRecord.Enabled = true;
        }

        //This is used for: starting/stopping the recording worker, stopping the saving worker
        private void btRecord_Click(object sender, EventArgs e) {
            if (oServer.ClientCount < 1) {
                SetStatusBarOnTimer("At least one client needs to be connected for recording.", 5000);
                return;
            }

            //If we are saving frames right now, this button stops saving.
            if (bSaving) {
                btRecord.Enabled = false;
                savingWorker.CancelAsync();
                return;
            }

            bRecording = !bRecording;
            if (bRecording) {
                //Stop the update worker to reduce the network usage (provides better synchronization).
                updateWorker.CancelAsync();

                recordingWorker.RunWorkerAsync();
                btRecord.Text = "Stop recording";
                btRefineCalib.Enabled = false;
                btCalibrate.Enabled = false;
            } else {
                btRecord.Enabled = false;
                recordingWorker.CancelAsync();
            }
        }

        private void btCalibrate_Click(object sender, EventArgs e) {
            oServer.Calibrate();
        }

        private void btRefineCalib_Click(object sender, EventArgs e) {
            if (oServer.ClientCount < 2) {
                SetStatusBarOnTimer("To refine calibration you need at least 2 connected devices.", 5000);
                return;
            }

            btRefineCalib.Enabled = false;
            btCalibrate.Enabled = false;
            btRecord.Enabled = false;

            refineWorker.RunWorkerAsync();
        }

        void RestartUpdateWorker() {
            if (!updateWorker.IsBusy)
                updateWorker.RunWorkerAsync();
        }

        private void btShowLive_Click(object sender, EventArgs e) {
            RestartUpdateWorker();

            //Opens the live view window if it is not open yet.
            if (!OpenGLWorker.IsBusy)
                OpenGLWorker.RunWorkerAsync();
        }

        private void SetStatusBarOnTimer(string message, int milliseconds) {
            statusLabel.Text = message;

            oStatusBarTimer.Stop();
            oStatusBarTimer = new Timer();

            oStatusBarTimer.Interval = milliseconds;
            oStatusBarTimer.Elapsed += delegate {
                oStatusBarTimer.Stop();
                statusLabel.Text = "";
            };
            oStatusBarTimer.Start();
        }

        //Updates the ListBox contaning the connected clients, called by events inside KinectServer.
        private void UpdateListView(List<KinectSocket> socketList) {
            var listBoxItems = socketList.Select(t => t.sSocketState).ToList();

            lClientListBox.DataSource = listBoxItems;
        }
    }
}