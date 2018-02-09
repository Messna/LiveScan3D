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
#include "stdafx.h"
#include "resource.h"
#include "liveScanClient.h"
#include "filter.h"
#include <chrono>
#include <strsafe.h>
#include <fstream>
#include "zstd.h"
#include <mutex>

std::mutex m_m_socket_thread_mutex;

int APIENTRY wWinMain(_In_ HINSTANCE instance,
	_In_opt_ HINSTANCE prev_instance,
	_In_ LPWSTR cmd_line,
	_In_ int show_cmd
) {
	UNREFERENCED_PARAMETER(prev_instance);
	UNREFERENCED_PARAMETER(cmd_line);

	LiveScanClient application;
	application.Run(instance, show_cmd);
	return 0;
}

LiveScanClient::LiveScanClient() :
	m_bSocketThread(true),
	m_bCalibrate(false),
	m_bFilter(false),
	m_bStreamOnlyBodies(false),
	m_nFilterNeighbors(10),
	m_fFilterThreshold(0.01f),
	m_bCaptureFrame(false),
	m_connected(false),
	m_bConfirmCaptured(false),
	m_bConfirmCalibrated(false),
	m_bShowDepth(false),
	m_bFrameCompression(true),
	m_iCompressionLevel(2),
	m_pClientSocket(nullptr),
	m_hWnd(nullptr),
	m_nLastCounter(0),
	m_fFreq(0),
	m_nNextStatusTime(0LL),
	m_nFramesSinceUpdate(0),
	m_pCameraSpaceCoordinates(nullptr),
	m_pColorCoordinatesOfDepth(nullptr),
	m_pDepthCoordinatesOfColor(nullptr),
	m_pDrawColor(nullptr),
	m_pD2DFactory(nullptr),
	m_pDepthRGBX(nullptr) {
	pCapture = new KinectCapture();

	LARGE_INTEGER qpf = {0};
	if (QueryPerformanceFrequency(&qpf)) {
		m_fFreq = double(qpf.QuadPart);
	}

	m_vBounds.push_back(-0.5);
	m_vBounds.push_back(-0.5);
	m_vBounds.push_back(-0.5);
	m_vBounds.push_back(0.5);
	m_vBounds.push_back(0.5);
	m_vBounds.push_back(0.5);

	calibration.load_calibration();
}

LiveScanClient::~LiveScanClient() {
	// clean up Direct2D renderer
	if (m_pDrawColor) {
		delete m_pDrawColor;
		m_pDrawColor = nullptr;
	}

	if (pCapture) {
		delete pCapture;
		pCapture = nullptr;
	}

	if (m_pDepthRGBX) {
		delete[] m_pDepthRGBX;
		m_pDepthRGBX = nullptr;
	}

	if (m_pCameraSpaceCoordinates) {
		delete[] m_pCameraSpaceCoordinates;
		m_pCameraSpaceCoordinates = nullptr;
	}

	if (m_pColorCoordinatesOfDepth) {
		delete[] m_pColorCoordinatesOfDepth;
		m_pColorCoordinatesOfDepth = nullptr;
	}

	if (m_pDepthCoordinatesOfColor) {
		delete[] m_pDepthCoordinatesOfColor;
		m_pDepthCoordinatesOfColor = nullptr;
	}

	if (m_pClientSocket) {
		delete m_pClientSocket;
		m_pClientSocket = nullptr;
	}
	// clean up Direct2D
	SafeRelease(m_pD2DFactory);
}

int LiveScanClient::Run(HINSTANCE hInstance, int nCmdShow) {
	MSG msg = {nullptr};
	WNDCLASS wc;

	// Dialog custom window class
	ZeroMemory(&wc, sizeof(wc));
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.cbWndExtra = DLGWINDOWEXTRA;
	wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
	wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
	wc.lpfnWndProc = DefDlgProcW;
	wc.lpszClassName = L"LiveScanClientAppDlgWndClass";

	if (!RegisterClassW(&wc)) {
		return 0;
	}

	// Create main application window
	HWND hWndApp = CreateDialogParamW(
		nullptr,
		MAKEINTRESOURCE(IDD_APP),
		nullptr,
		static_cast<DLGPROC>(MessageRouter),
		reinterpret_cast<LPARAM>(this));

	// Show window
	ShowWindow(hWndApp, nCmdShow);

	std::thread t1(&LiveScanClient::SocketThreadFunction, this);
	// Main message loop
	while (WM_QUIT != msg.message) {
		//HandleSocket();
		UpdateFrame();

		while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
			// If a dialog message will be taken care of by the dialog proc
			if (hWndApp && IsDialogMessageW(hWndApp, &msg)) {
				continue;
			}

			TranslateMessage(&msg);
			DispatchMessageW(&msg);
		}
	}

	m_bSocketThread = false;
	t1.join();
	return static_cast<int>(msg.wParam);
}


void LiveScanClient::UpdateFrame() {
	if (!pCapture->bInitialized) {
		return;
	}

	const bool new_frame_acquired = pCapture->AcquireFrame();

	if (!new_frame_acquired)
		return;

	pCapture->MapDepthFrameToCameraSpace(m_pCameraSpaceCoordinates);
	pCapture->MapDepthFrameToColorSpace(m_pColorCoordinatesOfDepth);
	{
		std::lock_guard<std::mutex> lock(m_m_socket_thread_mutex);
		StoreFrame(m_pCameraSpaceCoordinates, m_pColorCoordinatesOfDepth, pCapture->pColorRGBX, pCapture->vBodies,
		           pCapture->pBodyIndex);

		if (m_bCaptureFrame) {
			m_framesFileWriterReader.writeFrame(m_vLastFrameVertices, m_vLastFrameRGB);
			m_bConfirmCaptured = true;
			m_bCaptureFrame = false;
		}
	}

	if (m_bCalibrate) {
		std::lock_guard<std::mutex> lock(m_m_socket_thread_mutex);
		const auto camera_coordinates = new Point3f[pCapture->nColorFrameWidth * pCapture->nColorFrameHeight];
		pCapture->MapColorFrameToCameraSpace(camera_coordinates);

		bool res = calibration.calibrate(pCapture->pColorRGBX, camera_coordinates, pCapture->nColorFrameWidth,
		                                 pCapture->nColorFrameHeight);

		delete[] camera_coordinates;

		if (res) {
			m_bConfirmCalibrated = true;
			m_bCalibrate = false;
		}
	}

	if (!m_bShowDepth)
		ProcessColor(pCapture->pColorRGBX, pCapture->nColorFrameWidth, pCapture->nColorFrameHeight);
	else
		ProcessDepth(pCapture->pDepth, pCapture->nDepthFrameWidth, pCapture->nDepthFrameHeight);

	ShowFPS();
}

LRESULT CALLBACK LiveScanClient::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
	LiveScanClient* p_this;

	if (WM_INITDIALOG == uMsg) {
		p_this = reinterpret_cast<LiveScanClient*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(p_this));
	}
	else {
		p_this = reinterpret_cast<LiveScanClient*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (p_this) {
		return p_this->DlgProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}

LRESULT CALLBACK LiveScanClient::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
	UNREFERENCED_PARAMETER(wParam);
	UNREFERENCED_PARAMETER(lParam);

	switch (message) {
	case WM_INITDIALOG: {
		// Bind application window handle
		m_hWnd = hWnd;

		// Init Direct2D
		D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

		// Get and initialize the default Kinect sensor
		bool res = pCapture->Initialize();
		if (res) {
			m_pDepthRGBX = new RGB[pCapture->nColorFrameWidth * pCapture->nColorFrameHeight];

			m_pCameraSpaceCoordinates = new Point3f[pCapture->nDepthFrameWidth * pCapture->nDepthFrameHeight];
			m_pColorCoordinatesOfDepth = new Point2f[pCapture->nDepthFrameWidth * pCapture->nDepthFrameHeight];
			m_pDepthCoordinatesOfColor = new Point2f[pCapture->nColorFrameWidth * pCapture->nColorFrameHeight];
		}
		else {
			SetStatusMessage(L"Capture device failed to initialize!", 10000, true);
		}

		// Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
		// We'll use this to draw the data we receive from the Kinect to the screen
		HRESULT hr;
		m_pDrawColor = new ImageRenderer();
		hr = m_pDrawColor->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, pCapture->nColorFrameWidth,
		                              pCapture->nColorFrameHeight, pCapture->nColorFrameWidth * sizeof(RGB));
		if (FAILED(hr)) {
			SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
		}

		ReadIPFromFile();
	}
		break;

		// If the titlebar X is clicked, destroy app
	case WM_CLOSE:
		WriteIPToFile();
		DestroyWindow(hWnd);
		break;
	case WM_DESTROY:
		// Quit the main message pump
		PostQuitMessage(0);
		break;

		// Handle button press
	case WM_COMMAND:
		if (IDC_BUTTON_CONNECT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)) {
			std::lock_guard<std::mutex> lock(m_m_socket_thread_mutex);
			if (m_connected) {
				delete m_pClientSocket;
				m_pClientSocket = nullptr;

				m_connected = false;
				SetDlgItemTextA(m_hWnd, IDC_BUTTON_CONNECT, "Connect");
			}
			else {
				try {
					char address[20];
					GetDlgItemTextA(m_hWnd, IDC_IP, address, 20);
					m_pClientSocket = new SocketClient(address, 48001);

					m_connected = true;
					if (calibration.calibrated)
						m_bConfirmCalibrated = true;

					SetDlgItemTextA(m_hWnd, IDC_BUTTON_CONNECT, "Disconnect");
					//Clear the status bar so that the "Failed to connect..." disappears.
					SetStatusMessage(L"", 1, true);
				}
				catch (...) {
					SetStatusMessage(L"Failed to connect. Did you start the server?", 10000, true);
				}
			}
		}
		if (IDC_BUTTON_SWITCH == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam)) {
			m_bShowDepth = !m_bShowDepth;

			if (m_bShowDepth) {
				SetDlgItemTextA(m_hWnd, IDC_BUTTON_SWITCH, "Show color");
			}
			else {
				SetDlgItemTextA(m_hWnd, IDC_BUTTON_SWITCH, "Show depth");
			}
		}
		break;
	}

	return FALSE;
}

void LiveScanClient::ProcessDepth(const UINT16* pBuffer, int nWidth, int nHeight) {
	// Make sure we've received valid data
	if (m_pDepthRGBX && m_pDepthCoordinatesOfColor && pBuffer && (nWidth == pCapture->nDepthFrameWidth) && (nHeight ==
		pCapture->nDepthFrameHeight)) {
		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		pCapture->MapColorFrameToDepthSpace(m_pDepthCoordinatesOfColor);

		for (int i = 0; i < pCapture->nColorFrameWidth * pCapture->nColorFrameHeight; i++) {
			Point2f depthPoint = m_pDepthCoordinatesOfColor[i];
			BYTE intensity = 0;

			if (depthPoint.X >= 0 && depthPoint.Y >= 0) {
				int depthIdx = (int)(depthPoint.X + depthPoint.Y * pCapture->nDepthFrameWidth);
				USHORT depth = pBuffer[depthIdx];
				intensity = static_cast<BYTE>(depth % 256);
			}

			m_pDepthRGBX[i].rgbRed = intensity;
			m_pDepthRGBX[i].rgbGreen = intensity;
			m_pDepthRGBX[i].rgbBlue = intensity;
		}

		// Draw the data with Direct2D
		m_pDrawColor->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX),
		                   pCapture->nColorFrameWidth * pCapture->nColorFrameHeight * sizeof(RGB), pCapture->vBodies);
	}
}

void LiveScanClient::ProcessColor(RGB* pBuffer, int nWidth, int nHeight) {
	// Make sure we've received valid data
	if (pBuffer && (nWidth == pCapture->nColorFrameWidth) && (nHeight == pCapture->nColorFrameHeight)) {
		// Draw the data with Direct2D
		m_pDrawColor->Draw(reinterpret_cast<BYTE*>(pBuffer),
		                   pCapture->nColorFrameWidth * pCapture->nColorFrameHeight * sizeof(RGB), pCapture->vBodies);
	}
}

bool LiveScanClient::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce) {
	INT64 now = GetTickCount64();

	if (m_hWnd && (bForce || (m_nNextStatusTime <= now))) {
		SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
		m_nNextStatusTime = now + nShowTimeMsec;

		return true;
	}

	return false;
}

void LiveScanClient::SocketThreadFunction() {
	while (m_bSocketThread) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		HandleSocket();
	}
}

void LiveScanClient::HandleSocket() {
	char byteToSend;
	std::lock_guard<std::mutex> lock(m_m_socket_thread_mutex);

	if (!m_connected) {
		return;
	}

	string received = m_pClientSocket->ReceiveBytes();
	for (unsigned int i = 0; i < received.length(); i++) {
		//capture a frame
		if (received[i] == MSG_CAPTURE_FRAME)
			m_bCaptureFrame = true;
			//calibrate
		else if (received[i] == MSG_CALIBRATE)
			m_bCalibrate = true;
			//receive settings
			//TODO: what if packet is split?
		else if (received[i] == MSG_RECEIVE_SETTINGS) {
			vector<float> bounds(6);
			i++;
			int nBytes = *(int*)(received.c_str() + i);
			i += sizeof(int);

			for (int j = 0; j < 6; j++) {
				bounds[j] = *(float*)(received.c_str() + i);
				i += sizeof(float);
			}

			m_bFilter = (received[i] != 0);
			i++;

			m_nFilterNeighbors = *(int*)(received.c_str() + i);
			i += sizeof(int);

			m_fFilterThreshold = *(float*)(received.c_str() + i);
			i += sizeof(float);

			m_vBounds = bounds;

			const int n_markers = *(int*)(received.c_str() + i);
			i += sizeof(int);

			calibration.marker_poses.resize(n_markers);

			for (int j = 0; j < n_markers; j++) {
				for (auto& marker_pose_R : calibration.marker_poses[j].R) {
					for (float& el : marker_pose_R) {
						el = *(float*)(received.c_str() + i);
						i += sizeof(float);
					}
				}

				for (float& marker_pose_t : calibration.marker_poses[j].t) {
					marker_pose_t = *(float*)(received.c_str() + i);
					i += sizeof(float);
				}

				calibration.marker_poses[j].marker_id = *(int*)(received.c_str() + i);
				i += sizeof(int);
			}

			m_bStreamOnlyBodies = received[i] != 0;
			i += 1;

			m_iCompressionLevel = *(int*)(received.c_str() + i);
			i += sizeof(int);
			if (m_iCompressionLevel > 0)
				m_bFrameCompression = true;
			else
				m_bFrameCompression = false;

			//so that we do not lose the next character in the stream
			i--;
		}
			//send stored frame
		else if (received[i] == MSG_REQUEST_STORED_FRAME) {
			byteToSend = MSG_STORED_FRAME;
			m_pClientSocket->SendBytes(&byteToSend, 1);

			vector<Point3s> points;
			vector<RGB> colors;
			const bool res = m_framesFileWriterReader.readFrame(points, colors);
			if (!res) {
				int size = -1;
				m_pClientSocket->SendBytes(reinterpret_cast<char*>(&size), 4);
			}
			else
				SendFrame(points, colors, m_vLastFrameBody);
		}
			//send last frame
		else if (received[i] == MSG_REQUEST_LAST_FRAME) {
			byteToSend = MSG_LAST_FRAME;
			m_pClientSocket->SendBytes(&byteToSend, 1);

			SendFrame(m_vLastFrameVertices, m_vLastFrameRGB, m_vLastFrameBody);
		}
			//receive calibration data
		else if (received[i] == MSG_RECEIVE_CALIBRATION) {
			i++;
			for (int j = 0; j < 3; j++) {
				for (int k = 0; k < 3; k++) {
					calibration.world_r[j][k] = *(float*)(received.c_str() + i);
					i += sizeof(float);
				}
			}
			for (int j = 0; j < 3; j++) {
				calibration.world_t[j] = *(float*)(received.c_str() + i);
				i += sizeof(float);
			}

			//so that we do not lose the next character in the stream
			i--;
		}
		else if (received[i] == MSG_CLEAR_STORED_FRAMES) {
			m_framesFileWriterReader.closeFileIfOpened();
		}
	}

	if (m_bConfirmCaptured) {
		byteToSend = MSG_CONFIRM_CAPTURED;
		m_pClientSocket->SendBytes(&byteToSend, 1);
		m_bConfirmCaptured = false;
	}

	if (m_bConfirmCalibrated) {
		const int size = (9 + 3) * sizeof(float) + sizeof(int) + 1;
		char* buffer = new char[size];
		buffer[0] = MSG_CONFIRM_CALIBRATED;
		int i = 1;

		memcpy(buffer + i, &calibration.used_marker_id, 1 * sizeof(int));
		i += 1 * sizeof(int);
		memcpy(buffer + i, calibration.world_r[0].data(), 3 * sizeof(float));
		i += 3 * sizeof(float);
		memcpy(buffer + i, calibration.world_r[1].data(), 3 * sizeof(float));
		i += 3 * sizeof(float);
		memcpy(buffer + i, calibration.world_r[2].data(), 3 * sizeof(float));
		i += 3 * sizeof(float);
		memcpy(buffer + i, calibration.world_t.data(), 3 * sizeof(float));
		i += 3 * sizeof(float);

		m_pClientSocket->SendBytes(buffer, size);
		m_bConfirmCalibrated = false;
	}
}

void LiveScanClient::SendFrame(vector<Point3s> vertices, vector<RGB> RGB, vector<Body> body) {
	int size = RGB.size() * (3 + 3 * sizeof(short)) + sizeof(int);

	vector<char> buffer(size);
	char* ptr2 = (char*)vertices.data();
	int pos = 0;

	int nVertices = RGB.size();
	memcpy(buffer.data() + pos, &nVertices, sizeof(nVertices));
	pos += sizeof(nVertices);

	for (auto& color : RGB) {
		buffer[pos++] = color.rgbRed;
		buffer[pos++] = color.rgbGreen;
		buffer[pos++] = color.rgbBlue;

		memcpy(buffer.data() + pos, ptr2, sizeof(short) * 3);
		ptr2 += sizeof(short) * 3;
		pos += sizeof(short) * 3;
	}

	int nBodies = body.size();
	size += sizeof(nBodies);
	for (int i = 0; i < nBodies; i++) {
		size += sizeof(body[i].bTracked);
		int nJoints = body[i].vJoints.size();
		size += sizeof(nJoints);
		size += nJoints * (3 * sizeof(float) + 2 * sizeof(int));
		size += nJoints * 2 * sizeof(float);
	}
	buffer.resize(size);

	memcpy(buffer.data() + pos, &nBodies, sizeof(nBodies));
	pos += sizeof(nBodies);

	for (int i = 0; i < nBodies; i++) {
		memcpy(buffer.data() + pos, &body[i].bTracked, sizeof(body[i].bTracked));
		pos += sizeof(body[i].bTracked);

		int nJoints = body[i].vJoints.size();
		memcpy(buffer.data() + pos, &nJoints, sizeof(nJoints));
		pos += sizeof(nJoints);

		for (int j = 0; j < nJoints; j++) {
			//Joint
			memcpy(buffer.data() + pos, &body[i].vJoints[j].JointType, sizeof(JointType));
			pos += sizeof(JointType);
			memcpy(buffer.data() + pos, &body[i].vJoints[j].TrackingState, sizeof(TrackingState));
			pos += sizeof(TrackingState);
			//Joint position
			memcpy(buffer.data() + pos, &body[i].vJoints[j].Position.X, sizeof(float));
			pos += sizeof(float);
			memcpy(buffer.data() + pos, &body[i].vJoints[j].Position.Y, sizeof(float));
			pos += sizeof(float);
			memcpy(buffer.data() + pos, &body[i].vJoints[j].Position.Z, sizeof(float));
			pos += sizeof(float);

			//JointInColorSpace
			memcpy(buffer.data() + pos, &body[i].vJointsInColorSpace[j].X, sizeof(float));
			pos += sizeof(float);
			memcpy(buffer.data() + pos, &body[i].vJointsInColorSpace[j].Y, sizeof(float));
			pos += sizeof(float);
		}
	}

	int iCompression = static_cast<int>(m_bFrameCompression);

	if (m_bFrameCompression) {
		// *2, because according to zstd documentation, increasing the size of the output buffer above a 
		// bound should speed up the compression.
		int cBuffSize = ZSTD_compressBound(size) * 2;
		vector<char> compressedBuffer(cBuffSize);
		int cSize = ZSTD_compress(compressedBuffer.data(), cBuffSize, buffer.data(), size, m_iCompressionLevel);
		size = cSize;
		buffer = compressedBuffer;
	}
	char header[8];
	memcpy(header, (char*)&size, sizeof(size));
	memcpy(header + 4, (char*)&iCompression, sizeof(iCompression));

	m_pClientSocket->SendBytes((char*)&header, sizeof(int) * 2);
	m_pClientSocket->SendBytes(buffer.data(), size);
}

void LiveScanClient::StoreFrame(Point3f* vertices, Point2f* mapping, RGB* color, vector<Body>& bodies,
                                BYTE* body_index) {
	std::vector<Point3f> goodVertices;
	std::vector<RGB> goodColorPoints;

	const unsigned int n_vertices = pCapture->nDepthFrameWidth * pCapture->nDepthFrameHeight;

	for (unsigned int vertexIndex = 0; vertexIndex < n_vertices; vertexIndex++) {
		if (m_bStreamOnlyBodies && body_index[vertexIndex] >= bodies.size())
			continue;

		if (vertices[vertexIndex].Z >= 0 && mapping[vertexIndex].Y >= 0 && mapping[vertexIndex].Y < pCapture->
			nColorFrameHeight) {
			Point3f temp = vertices[vertexIndex];
			RGB tempColor = color[static_cast<int>(mapping[vertexIndex].X) + static_cast<int>(mapping[vertexIndex].Y) * pCapture->nColorFrameWidth];
			if (calibration.calibrated) {
				temp.X += calibration.world_t[0];
				temp.Y += calibration.world_t[1];
				temp.Z += calibration.world_t[2];
				temp = RotatePoint(temp, calibration.world_r);

				if (temp.X < m_vBounds[0] || temp.X > m_vBounds[3]
					|| temp.Y < m_vBounds[1] || temp.Y > m_vBounds[4]
					|| temp.Z < m_vBounds[2] || temp.Z > m_vBounds[5])
					continue;
			}

			goodVertices.push_back(temp);
			goodColorPoints.push_back(tempColor);
		}
	}

	vector<Body> tempBodies = bodies;

	for (auto& temp_body : tempBodies) {
		for (auto& vJoint : temp_body.vJoints) {
			if (calibration.calibrated) {
				vJoint.Position.X += calibration.world_t[0];
				vJoint.Position.Y += calibration.world_t[1];
				vJoint.Position.Z += calibration.world_t[2];

				Point3f tempPoint(vJoint.Position.X, vJoint.Position.Y,
				                  vJoint.Position.Z);

				tempPoint = RotatePoint(tempPoint, calibration.world_r);

				vJoint.Position.X = tempPoint.X;
				vJoint.Position.Y = tempPoint.Y;
				vJoint.Position.Z = tempPoint.Z;
			}
		}
	}

	if (m_bFilter)
		filter(goodVertices, goodColorPoints, m_nFilterNeighbors, m_fFilterThreshold);

	vector<Point3s> goodVerticesShort(goodVertices.size());

	for (unsigned int i = 0; i < goodVertices.size(); i++) {
		goodVerticesShort[i] = goodVertices[i];
	}

	m_vLastFrameBody = tempBodies;
	m_vLastFrameVertices = goodVerticesShort;
	m_vLastFrameRGB = goodColorPoints;
}

void LiveScanClient::ShowFPS() {
	if (m_hWnd) {
		double fps = 0.0;

		LARGE_INTEGER qpcNow = {0};
		if (m_fFreq) {
			if (QueryPerformanceCounter(&qpcNow)) {
				if (m_nLastCounter) {
					m_nFramesSinceUpdate++;
					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
				}
			}
		}

		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f", fps);

		if (SetStatusMessage(szStatusMessage, 1000, false)) {
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
	}
}

void LiveScanClient::ReadIPFromFile() {
	ifstream file;
	file.open("lastIP.txt");
	if (file.is_open()) {
		char lastUsedIPAddress[20];
		file.getline(lastUsedIPAddress, 20);
		file.close();
		SetDlgItemTextA(m_hWnd, IDC_IP, lastUsedIPAddress);
	}
}

void LiveScanClient::WriteIPToFile() {
	ofstream file;
	file.open("lastIP.txt");
	char lastUsedIPAddress[20];
	GetDlgItemTextA(m_hWnd, IDC_IP, lastUsedIPAddress, 20);
	file << lastUsedIPAddress;
	file.close();
}
