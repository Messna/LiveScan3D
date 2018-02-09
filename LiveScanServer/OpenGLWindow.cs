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
using System.Drawing;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Input;
using static OpenTK.Graphics.GL;
using MatrixMode = OpenTK.Graphics.MatrixMode;

enum ECameraMode {
    CAMERA_NONE,
    CAMERA_TRACK,
    CAMERA_DOLLY,
    CAMERA_ORBIT
}

namespace KinectServer {
    public class OpenGlWindow : GameWindow {
        int PointCount;
        int LineCount;

        private VertexC4ubV3f[] _vbo;
        private float _pointSize;
        private ECameraMode _cameraMode = ECameraMode.CAMERA_NONE;

        private const float KEYBOARD_MOVE_SPEED = 0.01f;

        private bool _isFullscreen;

        private const float MOUSE_ORBIT_SPEED = 0.30f; // 0 = SLOWEST, 1 = FASTEST
        private const float MOUSE_DOLLY_SPEED = 0.2f; // same as above...but much more sensitive
        private const float MOUSE_TRACK_SPEED = 0.003f; // same as above...but much more sensitive

        float g_heading;
        float g_pitch;
        float dx;
        float dy;

        byte brightnessModifier;

        Vector2 MousePrevious;
        Vector2 MouseCurrent;
        float[] cameraPosition = new float[3];
        float[] targetPosition = new float[3];

        public List<float> vertices = new List<float>();
        public List<byte> colors = new List<byte>();
        public List<AffineTransform> cameraPoses = new List<AffineTransform>();
        public List<Body> bodies = new List<Body>();
        public KinectSettings settings = new KinectSettings();

        DateTime tFPSUpdateTimer = DateTime.Now;
        int nTickCounter;

        bool bDrawMarkings = true;

        // this struct is used for drawing
        struct VertexC4ubV3f {
            public byte R, G, B, A;
            public Vector3 Position;

            public static int SizeInBytes = 16;
        }

        uint VBOHandle;

        /// <summary>Creates a 800x600 window with the specified title.</summary>
        public OpenGlWindow()
            : base(800, 600, GraphicsMode.Default, "LiveScan") {
            VSync = VSyncMode.Off;
            MouseUp += OnMouseButtonUp;
            MouseDown += OnMouseButtonDown;
            MouseMove += OnMouseMove;
            MouseWheel += OnMouseWheelChanged;

            KeyDown += OnKeyDown;

            cameraPosition[0] = 0;
            cameraPosition[1] = 0;
            cameraPosition[2] = 1.0f;
            targetPosition[0] = 0;
            targetPosition[1] = 0;
            targetPosition[2] = 0;
        }

        public void CloudUpdateTick() {
            nTickCounter++;
        }

        public void ToggleFullscreen() {
            if (_isFullscreen) {
                WindowBorder = WindowBorder.Resizable;
                WindowState = WindowState.Normal;
                ClientSize = new Size(800, 600);
                CursorVisible = true;
            } else {
                CursorVisible = false;
                WindowBorder = WindowBorder.Hidden;
                WindowState = WindowState.Fullscreen;
            }

            _isFullscreen = !_isFullscreen;
        }

        void OnKeyDown(object sender, KeyboardKeyEventArgs e) {
            var keyboard = e.Keyboard;
            if (keyboard[Key.Escape]) {
                Exit();
            }

            if (keyboard[Key.Plus]) {
                _pointSize += 0.1f;
                PointSize(_pointSize);
            }

            if (keyboard[Key.Minus]) {
                if (_pointSize != 0)
                    _pointSize -= 0.1f;
                PointSize(_pointSize);
            }

            if (keyboard[Key.W])
                cameraPosition[2] -= KEYBOARD_MOVE_SPEED;
            if (keyboard[Key.A])
                cameraPosition[0] -= KEYBOARD_MOVE_SPEED;
            if (keyboard[Key.S])
                cameraPosition[2] += KEYBOARD_MOVE_SPEED;
            if (keyboard[Key.D])
                cameraPosition[0] += KEYBOARD_MOVE_SPEED;
            if (keyboard[Key.F])
                ToggleFullscreen();
            if (keyboard[Key.M])
                bDrawMarkings = !bDrawMarkings;
            if (keyboard[Key.O])
                brightnessModifier = (byte) Math.Max(0, brightnessModifier - 10);
            if (keyboard[Key.P])
                brightnessModifier = (byte) Math.Min(255, brightnessModifier + 10);
        }

        /// <summary>Load resources here.</summary>
        /// <param name="e">Not used.</param>
        protected override void OnLoad(EventArgs e) {
            base.OnLoad(e);

            var version = new Version(GetString(StringName.Version).Substring(0, 3));
            var target = new Version(1, 5);
            if (version < target) {
                throw new NotSupportedException($"OpenGL {target} is required (you only have {version}).");
            }

            ClearColor(.1f, 0f, .1f, 0f);
            Enable(EnableCap.DepthTest);

            // Setup parameters for Points
            PointSize(_pointSize);
            Enable(EnableCap.PointSmooth);
            Hint(HintTarget.PointSmoothHint, HintMode.Nicest);

            // Setup VBO state
            EnableClientState(EnableCap.ColorArray);
            EnableClientState(EnableCap.VertexArray);

            GenBuffers(1, out VBOHandle);

            // Since there's only 1 VBO in the app, might aswell setup here.
            BindBuffer(BufferTarget.ArrayBuffer, VBOHandle);
            ColorPointer(4, ColorPointerType.UnsignedByte, VertexC4ubV3f.SizeInBytes, (IntPtr) 0);
            VertexPointer(3, VertexPointerType.Float, VertexC4ubV3f.SizeInBytes, (IntPtr) (4 * sizeof(byte)));

            PointCount = 0;
            LineCount = 12;
            _vbo = new VertexC4ubV3f[PointCount + 2 * LineCount];
        }

        protected override void OnUnload(EventArgs e) {
            DeleteBuffers(1, ref VBOHandle);
        }

        /// <summary>
        /// Called when your window is resized. Set your viewport here. It is also
        /// a good place to set up your projection matrix (which probably changes
        /// along when the aspect ratio of your window).
        /// </summary>
        /// <param name="e">Contains information on the new Width and Size of the GameWindow.</param>
        protected override void OnResize(EventArgs e) {
            Viewport(0, 0, Width, Height);

            MatrixMode(MatrixMode.Projection);
            var p = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver4, Width / (float) Height, 0.1f, 50.0f);
            LoadMatrix(ref p);

            MatrixMode(MatrixMode.Modelview);
            var mv = Matrix4.LookAt(Vector3.UnitZ, Vector3.Zero, Vector3.UnitY);
            LoadMatrix(ref mv);
        }

        void OnMouseWheelChanged(object sender, MouseWheelEventArgs e) {
            dy = e.Delta * MOUSE_DOLLY_SPEED;

            cameraPosition[2] -= dy;

            //if (cameraPosition[2] < 0)
            //    cameraPosition[2] = 0;
        }

        void OnMouseMove(object sender, MouseMoveEventArgs e) {
            MouseCurrent.X = e.Mouse.X;
            MouseCurrent.Y = e.Mouse.Y;

            // Now use mouse_delta to move the camera

            switch (_cameraMode) {
                case ECameraMode.CAMERA_TRACK:
                    dx = MouseCurrent.X - MousePrevious.X;
                    dx *= MOUSE_TRACK_SPEED;

                    dy = MouseCurrent.Y - MousePrevious.Y;
                    dy *= MOUSE_TRACK_SPEED;

                    cameraPosition[0] -= dx;
                    cameraPosition[1] += dy;

                    //targetPosition[0] -= dx;
                    //targetPosition[1] += dy;

                    break;

                case ECameraMode.CAMERA_DOLLY:
                    dy = MouseCurrent.Y - MousePrevious.Y;
                    dy *= MOUSE_DOLLY_SPEED;

                    cameraPosition[2] -= dy;

                    //    if (cameraPosition[2] < 0)
                    //       cameraPosition[2] = 0;

                    break;

                case ECameraMode.CAMERA_ORBIT:
                    dx = MouseCurrent.X - MousePrevious.X;
                    dx *= MOUSE_ORBIT_SPEED;

                    dy = MouseCurrent.Y - MousePrevious.Y;
                    dy *= MOUSE_ORBIT_SPEED;

                    g_heading += dx;
                    g_pitch += dy;

                    break;
            }

            MousePrevious.X = MouseCurrent.X;
            MousePrevious.Y = MouseCurrent.Y;
        }

        void OnMouseButtonUp(object sender, MouseButtonEventArgs e) {
            _cameraMode = ECameraMode.CAMERA_NONE;
        }

        void OnMouseButtonDown(object sender, MouseButtonEventArgs e) {
            switch (e.Button) {
                case MouseButton.Left:
                    _cameraMode = ECameraMode.CAMERA_ORBIT;
                    break;
                case MouseButton.Middle:
                    _cameraMode = ECameraMode.CAMERA_DOLLY;
                    break;
                case MouseButton.Right:
                    _cameraMode = ECameraMode.CAMERA_TRACK;
                    break;
            }

            MousePrevious.X = Mouse.X;
            MousePrevious.Y = Mouse.Y;
        }

        protected override void OnUpdateFrame(FrameEventArgs e) {
            if ((DateTime.Now - tFPSUpdateTimer).Seconds >= 1) {
                var FPS = nTickCounter / (DateTime.Now - tFPSUpdateTimer).TotalSeconds;
                Title = "FPS: " + string.Format("{0:F}", FPS);

                tFPSUpdateTimer = DateTime.Now;
                nTickCounter = 0;
            }


            lock (vertices) {
                lock (settings) {
                    var bShowSkeletons = settings.bShowSkeletons;

                    PointCount = vertices.Count / 3;
                    LineCount = 0;
                    if (bDrawMarkings) {
                        //bounding box
                        LineCount += 12;
                        //markers
                        LineCount += settings.markerPoses.Count * 3;
                        //cameras
                        LineCount += cameraPoses.Count * 3;
                        if (bShowSkeletons)
                            LineCount += 24 * bodies.Count;
                    }

                    _vbo = new VertexC4ubV3f[PointCount + 2 * LineCount];

                    for (var i = 0; i < PointCount; i++) {
                        _vbo[i].R = (byte) Math.Max(0, Math.Min(255, (colors[i * 3] + brightnessModifier)));
                        _vbo[i].G = (byte) Math.Max(0, Math.Min(255, (colors[i * 3 + 1] + brightnessModifier)));
                        _vbo[i].B = (byte) Math.Max(0, Math.Min(255, (colors[i * 3 + 2] + brightnessModifier)));
                        _vbo[i].A = 255;
                        _vbo[i].Position.X = vertices[i * 3];
                        _vbo[i].Position.Y = vertices[i * 3 + 1];
                        _vbo[i].Position.Z = vertices[i * 3 + 2];
                    }

                    if (bDrawMarkings) {
                        var iCurLineCount = 0;
                        iCurLineCount += AddBoundingBox(PointCount + 2 * iCurLineCount);
                        foreach (var markerPose in settings.markerPoses) {
                            iCurLineCount += AddMarker(PointCount + 2 * iCurLineCount, markerPose.pose);
                        }

                        foreach (var cameraPose in cameraPoses) {
                            iCurLineCount += AddCamera(PointCount + 2 * iCurLineCount, cameraPose);
                        }

                        if (bShowSkeletons)
                            AddBodies(PointCount + 2 * iCurLineCount);
                    }
                }
            }
        }

        /// <summary>
        /// Called when it is time to render the next frame. Add your rendering code here.
        /// </summary>
        /// <param name="e">Contains timing information.</param>
        protected override void OnRenderFrame(FrameEventArgs e) {
            Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            PushMatrix();

            MatrixMode(MatrixMode.Modelview);
            Translate(-cameraPosition[0], -cameraPosition[1], -cameraPosition[2]);
            Rotate(g_pitch, 1.0f, 0.0f, 0.0f);
            Rotate(g_heading, 0.0f, 1.0f, 0.0f);

            // Tell OpenGL to discard old VBO when done drawing it and reserve memory _now_ for a new buffer.
            // without this, GL would wait until draw operations on old VBO are complete before writing to it
            BufferData(BufferTarget.ArrayBuffer, (IntPtr) (VertexC4ubV3f.SizeInBytes * (PointCount + 2 * LineCount)),
                          IntPtr.Zero, BufferUsageHint.StreamDraw);
            // Fill newly allocated buffer
            BufferData(BufferTarget.ArrayBuffer, (IntPtr) (VertexC4ubV3f.SizeInBytes * (PointCount + 2 * LineCount)),
                          _vbo, BufferUsageHint.StreamDraw);

            DrawArrays(BeginMode.Points, 0, PointCount);
            DrawArrays(BeginMode.Lines, PointCount, 2 * LineCount);

            PopMatrix();

            SwapBuffers();
        }

        private int AddBoundingBox(int startIdx) {
            var nLinesBeingAdded = 12;
            //2 points per line
            var nPointsToAdd = 2 * nLinesBeingAdded;

            for (var i = startIdx; i < startIdx + nPointsToAdd; i++) {
                _vbo[i].R = 255;
                _vbo[i].G = 255;
                _vbo[i].B = 0;
                _vbo[i].A = 0;
            }

            var n = 0;

            //bottom vertices
            //first vertex
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                    settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                    settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                    settings.aMinBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //second vertex
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                    settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                    settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //third vertex
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2],
                    settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2],
                    settings.aMinBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //fourth vertex
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2],
                    settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //top vertices
            //fifth vertex 
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2],
                    settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2],
                    settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //sixth vertex
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2],
                    settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2],
                    settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2]);
            n += 2;

            return nLinesBeingAdded;
        }

        private int AddMarker(int startIdx, AffineTransform pose) {
            var nLinesBeingAdded = 3;
            //2 points per line
            var nPointsToAdd = 2 * nLinesBeingAdded;

            for (var i = startIdx; i < startIdx + nPointsToAdd; i++) {
                _vbo[i].R = 255;
                _vbo[i].G = 0;
                _vbo[i].B = 0;
                _vbo[i].A = 0;
            }

            var n = 0;

            var x0 = pose.t[0];
            var y0 = pose.t[1];
            var z0 = pose.t[2];

            var x1 = 0.1f;
            var y1 = 0.1f;
            var z1 = 0.1f;

            var x2 = pose.R[0, 0] * x1;
            var y2 = pose.R[1, 0] * x1;
            var z2 = pose.R[2, 0] * x1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 1] * y1;
            y2 = pose.R[1, 1] * y1;
            z2 = pose.R[2, 1] * y1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 2] * z1;
            y2 = pose.R[1, 2] * z1;
            z2 = pose.R[2, 2] * z1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);

            return nLinesBeingAdded;
        }

        private int AddCamera(int startIdx, AffineTransform pose) {
            const int linesBeingAdded = 3;
            //2 points per line
            const int pointsToAdd = 2 * linesBeingAdded;

            for (var i = startIdx; i < startIdx + pointsToAdd; i++) {
                _vbo[i].R = 0;
                _vbo[i].G = 255;
                _vbo[i].B = 0;
                _vbo[i].A = 0;
            }

            var n = 0;

            var x0 = pose.t[0];
            var y0 = pose.t[1];
            var z0 = pose.t[2];

            var x1 = 0.1f;
            var y1 = 0.1f;
            var z1 = 0.1f;

            var x2 = pose.R[0, 0] * x1;
            var y2 = pose.R[1, 0] * x1;
            var z2 = pose.R[2, 0] * x1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 1] * y1;
            y2 = pose.R[1, 1] * y1;
            z2 = pose.R[2, 1] * y1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 2] * z1;
            y2 = pose.R[1, 2] * z1;
            z2 = pose.R[2, 2] * z1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            return linesBeingAdded;
        }

        private int AddBone(int bodyIdx, JointType jointType0, JointType jointType1, int startIdx) {
            var joint0 = bodies[bodyIdx].lJoints[(int) jointType0].position;
            var joint1 = bodies[bodyIdx].lJoints[(int) jointType1].position;
            AddLine(startIdx, joint0.X, joint0.Y, joint0.Z, joint1.X, joint1.Y, joint1.Z);
            return 2;
        }

        private int AddBodies(int startIdx) {
            var nLinesToAdd = 24 * bodies.Count;
            var nPointsToAdd = nLinesToAdd * 2;

            for (var i = startIdx; i < startIdx + nPointsToAdd; i++) {
                _vbo[i].R = 0;
                _vbo[i].G = 255;
                _vbo[i].B = 0;
                _vbo[i].A = 0;
            }

            var n = 0;

            for (var bodyIdx = 0; bodyIdx < bodies.Count; bodyIdx++) {
                if (bodies[bodyIdx].bTracked == false)
                    continue;

                //Torso
                n += AddBone(bodyIdx, JointType.JointType_Head, JointType.JointType_Neck, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_Neck, JointType.JointType_SpineShoulder, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineShoulder, JointType.JointType_SpineMid, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineMid, JointType.JointType_SpineBase, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineShoulder, JointType.JointType_ShoulderRight,
                             startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineShoulder, JointType.JointType_ShoulderLeft,
                             startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineBase, JointType.JointType_HipRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineBase, JointType.JointType_HipLeft, startIdx + n);

                // Right Arm    
                n += AddBone(bodyIdx, JointType.JointType_ShoulderRight, JointType.JointType_ElbowRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_ElbowRight, JointType.JointType_WristRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_WristRight, JointType.JointType_HandRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_HandRight, JointType.JointType_HandTipRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_WristRight, JointType.JointType_ThumbRight, startIdx + n);

                // Left Arm
                n += AddBone(bodyIdx, JointType.JointType_ShoulderLeft, JointType.JointType_ElbowLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_ElbowLeft, JointType.JointType_WristLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_WristLeft, JointType.JointType_HandLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_HandLeft, JointType.JointType_HandTipLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_WristLeft, JointType.JointType_ThumbLeft, startIdx + n);

                // Right Leg
                n += AddBone(bodyIdx, JointType.JointType_HipRight, JointType.JointType_KneeRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_KneeRight, JointType.JointType_AnkleRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_AnkleRight, JointType.JointType_FootRight, startIdx + n);

                // Left Leg
                n += AddBone(bodyIdx, JointType.JointType_HipLeft, JointType.JointType_KneeLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_KneeLeft, JointType.JointType_AnkleLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_AnkleLeft, JointType.JointType_FootLeft, startIdx + n);
            }

            return nLinesToAdd;
        }

        private void AddLine(int startIdx, float x0, float y0, float z0,
                             float x1, float y1, float z1) {
            _vbo[startIdx].Position.X = x0;
            _vbo[startIdx].Position.Y = y0;
            _vbo[startIdx].Position.Z = z0;

            _vbo[startIdx + 1].Position.X = x1;
            _vbo[startIdx + 1].Position.Y = y1;
            _vbo[startIdx + 1].Position.Z = z1;
        }
    }
}