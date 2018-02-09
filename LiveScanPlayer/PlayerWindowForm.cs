using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Threading;
using System.Windows.Forms;
using KinectServer;

namespace LiveScanPlayer {
    public partial class PlayerWindowForm : Form {
        private readonly BindingList<IFrameFileReader> _frameFiles = new BindingList<IFrameFileReader>();
        private bool _playerRunning;

        private readonly List<float> _allVertices = new List<float>();
        private readonly List<byte> _allColors = new List<byte>();

        private readonly TransferServer _transferServer = new TransferServer();

        private readonly AutoResetEvent _updateWorkerFinished = new AutoResetEvent(false);

        public PlayerWindowForm() {
            InitializeComponent();

            _transferServer.Vertices = _allVertices;
            _transferServer.Colors = _allColors;

            lFrameFilesListView.Columns.Add("Current frame", 75);
            lFrameFilesListView.Columns.Add("Filename", 300);
        }

        private void PlayerWindowForm_FormClosing(object sender, FormClosingEventArgs e) {
            _playerRunning = false;
            _transferServer.StopServer();
        }

        private void btSelect_Click(object sender, EventArgs e) {
            var dialog = new OpenFileDialog {Multiselect = true};
            dialog.ShowDialog();

            lock (_frameFiles) {
                foreach (var fileName in dialog.FileNames) {
                    _frameFiles.Add(new FrameFileReaderBin(fileName));

                    var item = new ListViewItem(new[] {"0", fileName});
                    lFrameFilesListView.Items.Add(item);
                }
            }
        }

        private void btnSelectPly_Click(object sender, EventArgs e) {
            var dialog = new OpenFileDialog {Multiselect = true};
            dialog.ShowDialog();

            if (dialog.FileNames.Length == 0)
                return;

            lock (_frameFiles) {
                _frameFiles.Add(new FrameFileReaderPly(dialog.FileNames));


                var item = new ListViewItem(new[] {"0", Path.GetDirectoryName(dialog.FileNames[0])});
                lFrameFilesListView.Items.Add(item);
            }
        }

        private void btStart_Click(object sender, EventArgs e) {
            _playerRunning = !_playerRunning;

            if (_playerRunning) {
                _transferServer.StartServer();
                updateWorker.RunWorkerAsync();
                btStart.Text = "Stop player";
            } else {
                _transferServer.StopServer();
                btStart.Text = "Start player";
                _updateWorkerFinished.WaitOne();
            }
        }

        private void btRemove_Click(object sender, EventArgs e) {
            if (lFrameFilesListView.SelectedIndices.Count == 0)
                return;

            lock (_frameFiles) {
                var idx = lFrameFilesListView.SelectedIndices[0];
                lFrameFilesListView.Items.RemoveAt(idx);
                _frameFiles.RemoveAt(idx);
            }
        }

        private void btRewind_Click(object sender, EventArgs e) {
            lock (_frameFiles) {
                for (var i = 0; i < _frameFiles.Count; i++) {
                    _frameFiles[i].Rewind();
                    lFrameFilesListView.Items[i].Text = "0";
                }
            }
        }

        private void btShow_Click(object sender, EventArgs e) {
            if (!OpenGLWorker.IsBusy)
                OpenGLWorker.RunWorkerAsync();
        }

        private void updateWorker_DoWork(object sender, DoWorkEventArgs e) {
            var curFrameIdx = 0;
            const string outDir = "outPlayer\\";
            Directory.CreateDirectory(outDir);

            while (_playerRunning) {
                Thread.Sleep(50);

                var tempAllVertices = new List<float>();
                var tempAllColors = new List<byte>();

                lock (_frameFiles) {
                    foreach (var file in _frameFiles) {
                        var vertices = new List<float>();
                        var colors = new List<byte>();
                        file.ReadFrame(vertices, colors);

                        tempAllVertices.AddRange(vertices);
                        tempAllColors.AddRange(colors);
                    }
                }

                var frameIdxUpdate = new Thread(() => Invoke((MethodInvoker) UpdateDisplayedFrameIndices));
                frameIdxUpdate.Start();

                lock (_allVertices) {
                    _allVertices.Clear();
                    _allColors.Clear();
                    _allVertices.AddRange(tempAllVertices);
                    _allColors.AddRange(tempAllColors);
                }

                if (chSaveFrames.Checked)
                    SaveCurrentFrameToFile(outDir, curFrameIdx);


                curFrameIdx++;
            }

            _updateWorkerFinished.Set();
        }

        private void OpenGLWorker_DoWork(object sender, DoWorkEventArgs e) {
            var openGlWindow = new OpenGlWindow {
                vertices = _allVertices,
                colors = _allColors
            };


            openGlWindow.Run();
        }

        private void lFrameFilesListView_DoubleClick(object sender, EventArgs e) {
            lFrameFilesListView.SelectedItems[0].BeginEdit();
        }

        private void lFrameFilesListView_AfterLabelEdit(object sender, LabelEditEventArgs e) {
            var fileIdx = lFrameFilesListView.SelectedIndices[0];
            int frameIdx;
            var res = int.TryParse(e.Label, out frameIdx);

            if (!res) {
                e.CancelEdit = true;
                return;
            }

            lock (_frameFiles) {
                _frameFiles[fileIdx].JumpToFrame(frameIdx);
            }
        }

        private void UpdateDisplayedFrameIndices() {
            lock (_frameFiles) {
                for (var i = 0; i < _frameFiles.Count; i++) {
                    lFrameFilesListView.Items[i].SubItems[0].Text = _frameFiles[i].FrameIdx.ToString();
                }
            }
        }

        private void SaveCurrentFrameToFile(string outDir, int frameIdx) {
            var lVertices = new List<float>();
            var lColors = new List<byte>();

            lock (_allVertices) {
                lVertices.AddRange(_allVertices);
                lColors.AddRange(_allColors);
            }

            var outputFilename = outDir + frameIdx.ToString().PadLeft(5, '0') + ".ply";
            Utils.SaveToPly(outputFilename, lVertices, lColors, true);
        }
    }
}