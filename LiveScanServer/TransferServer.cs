using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Timer = System.Timers.Timer;

namespace KinectServer {
    public class TransferServer {
        public List<float> Vertices = new List<float>();
        public List<byte> Colors = new List<byte>();

        private TcpListener _oListener;
        private readonly List<TransferSocket> _clientSockets = new List<TransferSocket>();

        private readonly object _clientSocketLock = new object();
        private bool _serverRunning;

        ~TransferServer() {
            StopServer();
        }

        public void StartServer() {
            if (_serverRunning) return;
            _oListener = new TcpListener(IPAddress.Any, 48002);
            _oListener.Start();

            _serverRunning = true;
            var listeningThread = new Thread(ListeningWorker);
            listeningThread.Start();
            var receivingThread = new Thread(ReceivingWorker);
            receivingThread.Start();
        }

        public void StopServer() {
            if (!_serverRunning) return;
            _serverRunning = false;

            _oListener.Stop();
            lock (_clientSocketLock)
                _clientSockets.Clear();
        }

        private void ListeningWorker() {
            while (_serverRunning) {
                try {
                    var newClient = _oListener.AcceptTcpClient();

                    lock (_clientSocketLock) {
                        _clientSockets.Add(new TransferSocket(newClient));
                    }
                } catch (SocketException) { }

                Thread.Sleep(100);
            }
        }

        private void ReceivingWorker() {
            var checkConnectionTimer = new Timer {Interval = 1000};

            checkConnectionTimer.Elapsed += delegate {
                lock (_clientSocketLock) {
                    for (var i = 0; i < _clientSockets.Count; i++) {
                        if (!_clientSockets[i].SocketConnected()) {
                            _clientSockets.RemoveAt(i);
                            i--;
                        }
                    }
                }
            };

            checkConnectionTimer.Start();

            while (_serverRunning) {
                lock (_clientSocketLock) {
                    foreach (var clientSocket in _clientSockets) {
                        var buffer = clientSocket.Receive(1);

                        while (buffer.Length != 0) {
                            if (buffer[0] == 0) {
                                lock (Vertices) {
                                    clientSocket.SendFrame(Vertices, Colors);
                                }
                            }

                            buffer = clientSocket.Receive(1);
                        }
                    }
                }

                Thread.Sleep(10);
            }

            checkConnectionTimer.Stop();
        }
    }
}