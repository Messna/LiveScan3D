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
using System.Globalization;
using System.Windows.Forms;

namespace KinectServer {
    public partial class SettingsForm : Form {
        public KinectSettings Settings;
        public KinectServer Server;

        private bool _formLoaded;

        public SettingsForm() {
            InitializeComponent();
        }

        private void SettingsForm_Load(object sender, EventArgs e) {
            txtMinX.Text = Settings.aMinBounds[0].ToString(CultureInfo.InvariantCulture);
            txtMinY.Text = Settings.aMinBounds[1].ToString(CultureInfo.InvariantCulture);
            txtMinZ.Text = Settings.aMinBounds[2].ToString(CultureInfo.InvariantCulture);

            txtMaxX.Text = Settings.aMaxBounds[0].ToString(CultureInfo.InvariantCulture);
            txtMaxY.Text = Settings.aMaxBounds[1].ToString(CultureInfo.InvariantCulture);
            txtMaxZ.Text = Settings.aMaxBounds[2].ToString(CultureInfo.InvariantCulture);

            chFilter.Checked = Settings.bFilter;
            txtFilterNeighbors.Text = Settings.nFilterNeighbors.ToString();
            txtFilterDistance.Text = Settings.fFilterThreshold.ToString(CultureInfo.InvariantCulture);

            lisMarkers.DataSource = Settings.markerPoses;

            chBodyData.Checked = Settings.bStreamOnlyBodies;
            chSkeletons.Checked = Settings.bShowSkeletons;

            cbCompressionLevel.SelectedText = Settings.iCompressionLevel.ToString();

            chMerge.Checked = Settings.bMergeScansForSave;
            txtICPIters.Text = Settings.nNumICPIterations.ToString();
            txtRefinIters.Text = Settings.nNumRefineIters.ToString();
            if (Settings.bSaveAsBinaryPLY) {
                rBinaryPly.Checked = true;
                rAsciiPly.Checked = false;
            } else {
                rBinaryPly.Checked = false;
                rAsciiPly.Checked = true;
            }

            _formLoaded = true;
        }

        void UpdateClients() {
            if (_formLoaded)
                Server.SendSettings();
        }

        void UpdateMarkerFields() {
            if (lisMarkers.SelectedIndex >= 0) {
                var pose = Settings.markerPoses[lisMarkers.SelectedIndex];

                float X, Y, Z;
                pose.GetOrientation(out X, out Y, out Z);

                txtOrientationX.Text = X.ToString(CultureInfo.InvariantCulture);
                txtOrientationY.Text = Y.ToString(CultureInfo.InvariantCulture);
                txtOrientationZ.Text = Z.ToString(CultureInfo.InvariantCulture);

                txtTranslationX.Text = pose.pose.t[0].ToString(CultureInfo.InvariantCulture);
                txtTranslationY.Text = pose.pose.t[1].ToString(CultureInfo.InvariantCulture);
                txtTranslationZ.Text = pose.pose.t[2].ToString(CultureInfo.InvariantCulture);

                txtId.Text = pose.Id.ToString(CultureInfo.InvariantCulture);
            } else {
                txtOrientationX.Text = "";
                txtOrientationY.Text = "";
                txtOrientationZ.Text = "";

                txtTranslationX.Text = "";
                txtTranslationY.Text = "";
                txtTranslationZ.Text = "";

                txtId.Text = "";
            }
        }

        private void txtMinX_TextChanged(object sender, EventArgs e) {
            float.TryParse(txtMinX.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Settings.aMinBounds[0]);
            UpdateClients();
        }

        private void txtMinY_TextChanged(object sender, EventArgs e) {
            float.TryParse(txtMinY.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Settings.aMinBounds[1]);
            UpdateClients();
        }

        private void txtMinZ_TextChanged(object sender, EventArgs e) {
            float.TryParse(txtMinZ.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Settings.aMinBounds[2]);
            UpdateClients();
        }

        private void txtMaxX_TextChanged(object sender, EventArgs e) {
            float.TryParse(txtMaxX.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Settings.aMaxBounds[0]);
            UpdateClients();
        }

        private void txtMaxY_TextChanged(object sender, EventArgs e) {
            float.TryParse(txtMaxY.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Settings.aMaxBounds[1]);
            UpdateClients();
        }

        private void txtMaxZ_TextChanged(object sender, EventArgs e) {
            float.TryParse(txtMaxZ.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Settings.aMaxBounds[2]);
            UpdateClients();
        }

        private void chFilter_CheckedChanged(object sender, EventArgs e) {
            Settings.bFilter = chFilter.Checked;
            UpdateClients();
        }

        private void txtFilterNeighbors_TextChanged(object sender, EventArgs e) {
            int.TryParse(txtFilterNeighbors.Text, out Settings.nFilterNeighbors);
            UpdateClients();
        }

        private void txtFilterDistance_TextChanged(object sender, EventArgs e) {
            float.TryParse(txtFilterDistance.Text, NumberStyles.Any, CultureInfo.InvariantCulture,
                            out Settings.fFilterThreshold);
            UpdateClients();
        }

        private void txtICPIters_TextChanged(object sender, EventArgs e) {
            int.TryParse(txtICPIters.Text, out Settings.nNumICPIterations);
        }

        private void txtRefinIters_TextChanged(object sender, EventArgs e) {
            int.TryParse(txtRefinIters.Text, out Settings.nNumRefineIters);
        }

        private void chMerge_CheckedChanged(object sender, EventArgs e) {
            Settings.bMergeScansForSave = chMerge.Checked;
        }

        private void btAdd_Click(object sender, EventArgs e) {
            lock (Settings)
                Settings.markerPoses.Add(new MarkerPose());
            lisMarkers.SelectedIndex = Settings.markerPoses.Count - 1;
            UpdateMarkerFields();
            UpdateClients();
        }

        private void btRemove_Click(object sender, EventArgs e) {
            if (Settings.markerPoses.Count > 0) {
                Settings.markerPoses.RemoveAt(lisMarkers.SelectedIndex);
                lisMarkers.SelectedIndex = Settings.markerPoses.Count - 1;
                UpdateMarkerFields();
                UpdateClients();
            }
        }

        private void lisMarkers_SelectedIndexChanged(object sender, EventArgs e) {
            UpdateMarkerFields();
        }

        private void txtOrientationX_TextChanged(object sender, EventArgs e) {
            if (lisMarkers.SelectedIndex >= 0) {
                var pose = Settings.markerPoses[lisMarkers.SelectedIndex];
                float X, Y, Z;
                pose.GetOrientation(out X, out Y, out Z);
                float.TryParse(txtOrientationX.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out X);

                pose.SetOrientation(X, Y, Z);
                UpdateClients();
            }
        }

        private void txtOrientationY_TextChanged(object sender, EventArgs e) {
            if (lisMarkers.SelectedIndex >= 0) {
                var pose = Settings.markerPoses[lisMarkers.SelectedIndex];
                float X, Y, Z;
                pose.GetOrientation(out X, out Y, out Z);
                float.TryParse(txtOrientationY.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Y);

                pose.SetOrientation(X, Y, Z);
                UpdateClients();
            }
        }

        private void txtOrientationZ_TextChanged(object sender, EventArgs e) {
            if (lisMarkers.SelectedIndex >= 0) {
                var pose = Settings.markerPoses[lisMarkers.SelectedIndex];
                float X, Y, Z;
                pose.GetOrientation(out X, out Y, out Z);
                float.TryParse(txtOrientationZ.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Z);

                pose.SetOrientation(X, Y, Z);
                UpdateClients();
            }
        }

        private void txtTranslationX_TextChanged(object sender, EventArgs e) {
            if (lisMarkers.SelectedIndex >= 0) {
                float X;
                var pose = Settings.markerPoses[lisMarkers.SelectedIndex];
                float.TryParse(txtTranslationX.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out X);

                pose.pose.t[0] = X;
                UpdateClients();
            }
        }

        private void txtTranslationY_TextChanged(object sender, EventArgs e) {
            if (lisMarkers.SelectedIndex >= 0) {
                float Y;
                var pose = Settings.markerPoses[lisMarkers.SelectedIndex];
                float.TryParse(txtTranslationY.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Y);

                pose.pose.t[1] = Y;
                UpdateClients();
            }
        }

        private void txtTranslationZ_TextChanged(object sender, EventArgs e) {
            if (lisMarkers.SelectedIndex >= 0) {
                float Z;
                var pose = Settings.markerPoses[lisMarkers.SelectedIndex];
                float.TryParse(txtTranslationZ.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out Z);

                pose.pose.t[2] = Z;
                UpdateClients();
            }
        }

        private void txtId_TextChanged(object sender, EventArgs e) {
            if (lisMarkers.SelectedIndex >= 0) {
                int id;
                var pose = Settings.markerPoses[lisMarkers.SelectedIndex];
                int.TryParse(txtId.Text, out id);

                pose.Id = id;
                UpdateClients();
            }
        }

        private void chBodyData_CheckedChanged(object sender, EventArgs e) {
            Settings.bStreamOnlyBodies = chBodyData.Checked;
            UpdateClients();
        }

        private void PlyFormat_CheckedChanged(object sender, EventArgs e) {
            if (rAsciiPly.Checked) {
                Settings.bSaveAsBinaryPLY = false;
            } else {
                Settings.bSaveAsBinaryPLY = true;
            }
        }

        private void chSkeletons_CheckedChanged(object sender, EventArgs e) {
            Settings.bShowSkeletons = chSkeletons.Checked;
        }

        private void cbCompressionLevel_SelectedIndexChanged(object sender, EventArgs e) {
            var index = cbCompressionLevel.SelectedIndex;
            if (index == 0)
                Settings.iCompressionLevel = 0;
            else if (index == 2)
                Settings.iCompressionLevel = 2;
            else {
                var value = cbCompressionLevel.SelectedItem.ToString();
                var tryParse = int.TryParse(value, out Settings.iCompressionLevel);
                if (!tryParse)
                    Settings.iCompressionLevel = 0;
            }

            UpdateClients();
        }
    }
}