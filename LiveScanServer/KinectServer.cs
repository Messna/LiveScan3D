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

using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Timer = System.Timers.Timer;

namespace KinectServer {
    public delegate void SocketListChangedHandler(List<KinectSocket> list);

    public class KinectServer {
        private Socket _serverSocket;

        private bool _serverRunning;

        private readonly KinectSettings _settings;
        private readonly object _clientSocketLock = new object();
        private readonly object _frameRequestLock = new object();

        private readonly List<KinectSocket> _clientSockets = new List<KinectSocket>();

        public event SocketListChangedHandler ESocketListChanged;

        public int ClientCount {
            get {
                int clients;
                lock (_clientSocketLock) {
                    clients = _clientSockets.Count;
                }

                return clients;
            }
        }

        public List<AffineTransform> CameraPoses {
            get {
                var cameraPoses = new List<AffineTransform>();
                lock (_clientSocketLock) {
                    foreach (var socket in _clientSockets) {
                        cameraPoses.Add(socket.oCameraPose);
                    }
                }

                return cameraPoses;
            }
            set {
                lock (_clientSocketLock) {
                    for (var i = 0; i < _clientSockets.Count; i++) {
                        _clientSockets[i].oCameraPose = value[i];
                    }
                }
            }
        }

        public List<AffineTransform> WorldTransforms {
            get {
                var worldTransforms = new List<AffineTransform>();
                lock (_clientSocketLock) {
                    foreach (var clientSocket in _clientSockets) {
                        worldTransforms.Add(clientSocket.oWorldTransform);
                    }
                }

                return worldTransforms;
            }

            set {
                lock (_clientSocketLock) {
                    for (var i = 0; i < _clientSockets.Count; i++) {
                        _clientSockets[i].oWorldTransform = value[i];
                    }
                }
            }
        }

        public bool AllCalibrated {
            get {
                var allCalibrated = true;
                lock (_clientSocketLock) {
                    foreach (var socket in _clientSockets) {
                        if (!socket.bCalibrated) {
                            allCalibrated = false;
                            break;
                        }
                    }
                }

                return allCalibrated;
            }
        }

        public KinectServer(KinectSettings settings) {
            _settings = settings;
        }

        private void SocketListChanged() {
            ESocketListChanged?.Invoke(_clientSockets);
        }

        public void StartServer() {
            if (_serverRunning) return;
            
            _serverSocket = new Socket(SocketType.Stream, ProtocolType.Tcp) {Blocking = false};

            var endPoint = new IPEndPoint(IPAddress.Any, 48001);
            _serverSocket.Bind(endPoint);
            _serverSocket.Listen(10);

            _serverRunning = true;
            var listeningThread = new Thread(ListeningWorker);
            listeningThread.Start();
            var receivingThread = new Thread(ReceivingWorker);
            receivingThread.Start();
        }

        public void StopServer() {
            if (!_serverRunning) return;
            _serverRunning = false;

            _serverSocket.Close();
            lock (_clientSocketLock)
                _clientSockets.Clear();
        }

        public void CaptureSynchronizedFrame() {
            lock (_clientSocketLock) {
                foreach (var socket in _clientSockets)
                    socket.CaptureFrame();
            }

            //Wait till frames captured
            var allGathered = false;
            while (!allGathered) {
                allGathered = true;

                lock (_clientSocketLock) {
                    if (_clientSockets.Any(t => !t.bFrameCaptured)) {
                        allGathered = false;
                    }
                }
            }
        }

        public void Calibrate() {
            lock (_clientSocketLock) {
                foreach (var socket in _clientSockets) {
                    socket.Calibrate();
                }
            }
        }

        public void SendSettings() {
            lock (_clientSocketLock) {
                foreach (var socket in _clientSockets) {
                    socket.SendSettings(_settings);
                }
            }
        }

        public void SendCalibrationData() {
            lock (_clientSocketLock) {
                foreach (var socket in _clientSockets) {
                    socket.SendCalibrationData();
                }
            }
        }

        public bool GetStoredFrame(List<List<byte>> framesRgb, List<List<float>> framesVerts) {
            bool bNoMoreStoredFrames;
            framesRgb.Clear();
            framesVerts.Clear();

            lock (_frameRequestLock) {
                //Request frames
                lock (_clientSocketLock) {
                    foreach (var socket in _clientSockets)
                        socket.RequestStoredFrame();
                }

                //Wait till frames received
                var allGathered = false;
                bNoMoreStoredFrames = false;
                while (!allGathered) {
                    allGathered = true;
                    lock (_clientSocketLock) {
                        foreach (var socket in _clientSockets) {
                            if (!socket.bStoredFrameReceived) {
                                allGathered = false;
                                break;
                            }

                            if (socket.bNoMoreStoredFrames)
                                bNoMoreStoredFrames = true;
                        }
                    }
                }

                //Store received frames
                lock (_clientSocketLock) {
                    foreach (var socket in _clientSockets) {
                        framesRgb.Add(new List<byte>(socket.lFrameRGB));
                        framesVerts.Add(new List<float>(socket.lFrameVerts));
                    }
                }
            }

            if (bNoMoreStoredFrames)
                return false;
            
            return true;
        }

        public void GetLatestFrame(List<List<byte>> framesRgb, List<List<float>> framesVerts,
                                   List<List<Body>> framesBody) {
            framesRgb.Clear();
            framesVerts.Clear();
            framesBody.Clear();

            lock (_frameRequestLock) {
                //Request frames
                lock (_clientSocketLock) {
                    foreach (var socket in _clientSockets)
                        socket.RequestLastFrame();
                }

                //Wait till frames received
                var allGathered = false;
                while (!allGathered) {
                    allGathered = true;

                    lock (_clientSocketLock) {
                        if (_clientSockets.Any(t => !t.bLatestFrameReceived)) {
                            allGathered = false;
                        }
                    }
                }

                //Store received frames
                lock (_clientSocketLock) {
                    foreach (var socket in _clientSockets) {
                        framesRgb.Add(new List<byte>(socket.lFrameRGB));
                        framesVerts.Add(new List<float>(socket.lFrameVerts));
                        framesBody.Add(new List<Body>(socket.lBodies));
                    }
                }
            }
        }

        public void ClearStoredFrames() {
            lock (_clientSocketLock) {
                foreach (var socket in _clientSockets) {
                    socket.ClearStoredFrames();
                }
            }
        }

        private void ListeningWorker() {
            while (_serverRunning) {
                try {
                    var newClient = _serverSocket.Accept();

                    //we do not want to add new clients while a frame is being requested
                    lock (_frameRequestLock) {
                        lock (_clientSocketLock) {
                            _clientSockets.Add(new KinectSocket(newClient));
                            _clientSockets[_clientSockets.Count - 1].SendSettings(_settings);
                            _clientSockets[_clientSockets.Count - 1].eChanged += SocketListChanged;
                            ESocketListChanged?.Invoke(_clientSockets);
                        }
                    }
                } catch (SocketException) { }

                Thread.Sleep(100);
            }

            lock (_clientSocketLock) {
                ESocketListChanged?.Invoke(_clientSockets);
            }
        }

        private void ReceivingWorker() {
            var checkConnectionTimer = new Timer {Interval = 1000};

            checkConnectionTimer.Elapsed += delegate {
                lock (_clientSocketLock) {
                    for (var i = 0; i < _clientSockets.Count; i++) {
                        if (_clientSockets[i].SocketConnected()) continue;
                        _clientSockets.RemoveAt(i);
                        ESocketListChanged?.Invoke(_clientSockets);
                    }
                }
            };

            checkConnectionTimer.Start();

            while (_serverRunning) {
                lock (_clientSocketLock) {
                    foreach (var socket in _clientSockets) {
                        var buffer = socket.Receive(1);

                        while (buffer.Length != 0) {
                            if (buffer[0] == 0) {
                                socket.bFrameCaptured = true;
                            } else if (buffer[0] == 1) {
                                socket.ReceiveCalibrationData();
                            }
                            //stored frame
                            else if (buffer[0] == 2) {
                                socket.ReceiveFrame();
                                socket.bStoredFrameReceived = true;
                            }
                            //last frame
                            else if (buffer[0] == 3) {
                                socket.ReceiveFrame();
                                socket.bLatestFrameReceived = true;
                            }

                            buffer = socket.Receive(1);
                        }
                    }
                }

                Thread.Sleep(10);
            }

            checkConnectionTimer.Stop();
        }
    }
}