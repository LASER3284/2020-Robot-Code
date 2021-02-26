#include <cstdio>
#include <string>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>
#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>
#include <cameraserver/CameraServer.h>

using namespace cv;
using namespace cs;
using namespace nt;
using namespace frc;
using namespace wpi;
using namespace std;

/*
	JSON format:
	{
		"team": <team number>,
		"ntmode": <"client" or "server", "client" if unspecified>
		"cameras": [
			{
				"name": <camera name>
				"path": <path, e.g. "/dev/video0">
				"pixel format": <"MJPEG", "YUYV", etc>	// optional
				"width": <video mode width>			  // optional
				"height": <video mode height>			// optional
				"fps": <video mode fps>				  // optional
				"brightness": <percentage brightness>	// optional
				"white balance": <"auto", "hold", value> // optional
				"exposure": <"auto", "hold", value>	  // optional
				"properties": [						  // optional
					{
						"name": <property name>
						"value": <property value>
					}
				],
				"stream": {							  // optional
					"properties": [
						{
							"name": <stream property name>
							"value": <stream property value>
						}
					]
				}
			}
		]
		"switched cameras": [
			{
				"name": <virtual camera name>
				"key": <network table key used for selection>
				// if NT value is a string, it's treated as a name
				// if NT value is a double, it's treated as an integer index
			}
		]
	}
 */

// Store config file.
static const char* m_cConfigFile = "/boot/frc.json";

/****************************************************************************
		Description:	Implements the FPS Class

		Classes:		FPS

		Project:		2020 DeepSpace Vision Code

		Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
class FPS
{
public:
	/****************************************************************************
			Description:	FPS constructor.

			Arguments:		None

			Derived From:	Nothing
	****************************************************************************/
	FPS()
	{
		// Initialize member variables.
		m_nIterations = 0;
		m_pStartTime = time(0);
		m_nTick = 0;
		m_nFPS = 0;
	}

	/****************************************************************************
			Description:	FPS destructor.

			Arguments:		None

			Derived From:	Nothing
	****************************************************************************/
	~FPS()
	{
	}

	/****************************************************************************
			Description:	Counts number of interations.

			Arguments: 		None
	
			Returns: 		Nothing
	****************************************************************************/
	void Increment()
	{
		m_nIterations++;
	}

	/****************************************************************************
			Description:	Calculate average FPS.

			Arguments: 		None
	
			Returns: 		INT
	****************************************************************************/
	int FramesPerSec()
	{
		// Create instance variables.
		time_t m_pTimeNow = time(0) - m_pStartTime;

		// Calculate FPS.
		if (m_pTimeNow - m_nTick >= 1)
		{
			m_nTick++;
			m_nFPS = m_nIterations;
			m_nIterations = 0;
		}

		// Print debug.
		//cout << "Start Time: " << m_pStartTime << endl;
		//cout << "Time Now: " << m_pTimeNow << endl;
		//cout << "Iterations: " << m_nIterations << endl;
		//cout << "nFPS: " << m_nFPS << endl;

		// Return FPS value.
		return m_nFPS;
	}

private:
	time_t					m_pStartTime;
	int						m_nIterations;
	int						m_nTick;
	int						m_nFPS;
};

namespace 
{
	// Create namespace variables, stucts, and objects.
	unsigned int m_nTeam;
	bool m_bServer = false;

	struct CameraConfig 
	{
		string name;
		string path;
		json config;
		json streamConfig;
	};

	struct SwitchedCameraConfig 
	{
		string name;
		string key;
	};

	vector<CameraConfig> m_vCameraConfigs;
	vector<SwitchedCameraConfig> m_vSwitchedCameraConfigs;
	vector<UsbCamera> m_vCameras;
	vector<CvSink> m_vCameraSinks;
	vector<CvSource> m_vCameraSources;

	raw_ostream& ParseError() 
	{
		return errs() << "config error in '" << m_cConfigFile << "': ";
	}

	/****************************************************************************
			Description:	Read camera config file from the web dashboard.

			Arguments: 		CONST JSON&
	
			Returns: 		BOOL
	****************************************************************************/
	bool ReadCameraConfig(const json& fConfig) 
	{
		// Create instance variables.
		CameraConfig m_pCamConfig;

		// Get camera name.
		try 
		{
			m_pCamConfig.name = fConfig.at("name").get<string>();
		} 
		catch (const json::exception& e) 
		{
			ParseError() << "Could not read camera name: " << e.what() << "\n";
			return false;
		}

		// Get camera path.
		try 
		{
			m_pCamConfig.path = fConfig.at("path").get<string>();
		} 
		catch (const json::exception& e) 
		{
			ParseError() << "Camera '" << m_pCamConfig.name << "': could not read path: " << e.what() << "\n";
			return false;
		}

		// Get stream properties.
		if (fConfig.count("stream") != 0)
		{
			m_pCamConfig.streamConfig = fConfig.at("stream");
		}

		m_pCamConfig.config = fConfig;

		m_vCameraConfigs.emplace_back(move(m_pCamConfig));
		return true;
	}

	/****************************************************************************
			Description:	Read config file from the web dashboard.

			Arguments: 		None
	
			Returns: 		BOOL
	****************************************************************************/
	bool ReadConfig() 
	{
		// Open config file.
		error_code m_pErrorCode;
		raw_fd_istream is(m_cConfigFile, m_pErrorCode);
		if (m_pErrorCode) 
		{
			errs() << "Could not open '" << m_cConfigFile << "': " << m_pErrorCode.message() << "\n";
			return false;
		}

		// Parse file.
		json m_fParseFile;
		try 
		{
			m_fParseFile = json::parse(is);
		} 
		catch (const json::parse_error& e) 
		{
			ParseError() << "Byte " << e.byte << ": " << e.what() << "\n";
			return false;
		}

		// Check if the top level is an object.
		if (!m_fParseFile.is_object()) 
		{
			ParseError() << "Must be JSON object!" << "\n";
			return false;
		}

		// Get team number.
		try 
		{
			m_nTeam = m_fParseFile.at("team").get<unsigned int>();
		} 
		catch (const json::exception& e) 
		{
			ParseError() << "Could not read team number: " << e.what() << "\n";
			return false;
		}

		// Get NetworkTable mode.
		if (m_fParseFile.count("ntmode") != 0) 
		{
			try 
			{
				auto str = m_fParseFile.at("ntmode").get<string>();
				StringRef s(str);
				if (s.equals_lower("client")) 
				{
					m_bServer = false;
				} 
				else 
				{
					if (s.equals_lower("server")) 
					{
						m_bServer = true;
					}
					else 
					{
						ParseError() << "Could not understand ntmode value '" << str << "'" << "\n";
					}
				} 
			} 
			catch (const json::exception& e) 
			{
				ParseError() << "Could not read ntmode: " << e.what() << "\n";
			}
		}

		// Read camera configs and get cameras.
		try 
		{
			for (auto&& camera : m_fParseFile.at("cameras")) 
			{
				if (!ReadCameraConfig(camera))
				{
					return false;
				}
			}
		} 
		catch (const json::exception& e) 
		{
			ParseError() << "Could not read cameras: " << e.what() << "\n";
			return false;
		}

		return true;
	}

	/****************************************************************************
			Description:	Starts cameras and camera streams.

			Arguments: 		CONST CAMERACONFIG&
	
			Returns: 		Nothing
	****************************************************************************/
	void StartCamera(const CameraConfig& fConfig) 
	{
		// Print debug
		outs() << "Starting camera '" << m_vCameraConfigs[0].name << "' on " << m_vCameraConfigs[0].path << "\n";

		// Create new CameraServer instance and start camera.
		CameraServer* m_Inst = CameraServer::GetInstance();
		UsbCamera m_Camera{fConfig.name, fConfig.path};
		MjpegServer m_pServer = m_Inst->StartAutomaticCapture(m_Camera);

		// Set camera parameters.
		m_Camera.SetConfigJson(fConfig.config);
		m_Camera.SetConnectionStrategy(VideoSource::kConnectionKeepOpen);

		// Check for unexpected parameters.
		if (fConfig.streamConfig.is_object())
		{
			m_pServer.SetConfigJson(fConfig.streamConfig);
		}

		// Store the camera video in a vector. (so we can access it later)
		CvSink m_cvSink = m_Inst->GetVideo();
		CvSource m_cvSource = m_Inst->PutVideo(fConfig.name + "Processed", 640, 480);
		m_vCameras.emplace_back(m_Camera);
		m_vCameraSinks.emplace_back(m_cvSink);
		m_vCameraSources.emplace_back(m_cvSource);
	}

	/****************************************************************************
			Description:	Implements the VideoGet Class

			Classes:		VideoGet

			Project:		2020 DeepSpace Vision Code

			Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
	****************************************************************************/
	class VideoGet
	{
	public:
		// Create objects and variables.
		bool					m_bIsStopping;
		bool					m_bIsStopped;

		/****************************************************************************
				Description:	VideoGet constructor.

				Arguments:		None

				Derived From:	Nothing
		****************************************************************************/
		VideoGet()
		{
			// Create objects.
			m_pFPS									= new FPS();

			// Initialize Variables.
			m_bIsStopping							= false;
			m_bIsStopped							= false;
		}

		/****************************************************************************
				Description:	VideoGet destructor.

				Arguments:		None

				Derived From:	Nothing
		****************************************************************************/
		~VideoGet()
		{
			// Delete object pointers.
			delete m_pFPS;

			// Set object pointers as nullptrs.
			m_pFPS				 = nullptr;
		}

		/****************************************************************************
				Description:	Grabs frames from camera.

				Arguments: 		MAT&, SHARED_TIMED_MUTEX&
	
				Returns: 		Nothing
		****************************************************************************/
		void StartCapture(Mat &m_pFrame, bool &bCameraSourceIndex, bool &bDrivingMode, shared_timed_mutex &m_pMutex)
		{
			// Continuously grab camera frames.
			while (1)
			{
				// Increment FPS counter.
				m_pFPS->Increment();

				try
				{
					// Acquire resource lock for thread.
					lock_guard<shared_timed_mutex> guard(m_pMutex);		// unique_lock

					// If the frame is empty, stop the capture.
					if (m_vCameraSinks.empty() || m_vCameraSources.empty())
					{
						break;
					}

					// Grab frame from either camera1 or camera2.
					if (bDrivingMode)
					{
						// Set camera properties.
						m_vCameras[0].SetBrightness(10);
						m_vCameras[0].SetExposureAuto();
						m_vCameras[0].SetWhiteBalanceAuto();
						m_vCameras[1].SetBrightness(10);
						m_vCameras[1].SetExposureAuto();
						m_vCameras[1].SetWhiteBalanceAuto();
						// Get camera frame.
						m_vCameraSinks[0].GrabFrame(m_pFrame);
					}
					else
					{
						static bool bToggle = false;
						if (bCameraSourceIndex)
						{
							// Only set properties once.
							if (bToggle)
							{
								// Set camera properties.
								m_vCameras[0].SetBrightness(10);
								m_vCameras[0].SetExposureAuto();
								m_vCameras[0].SetWhiteBalanceAuto();
								m_vCameras[1].SetBrightness(10);
								m_vCameras[1].SetExposureAuto();
								m_vCameras[1].SetWhiteBalanceAuto();
								// Set toggle var.
								bToggle = false;
							}

							// Get camera frame.
							m_vCameraSinks[0].GrabFrame(m_pFrame);
						}
						if (!bCameraSourceIndex)
						{
							// Only set properties once.
							if (!bToggle)
							{
								// Set camera properties.
								m_vCameras[0].SetBrightness(0);
								m_vCameras[0].SetExposureManual(20);
								// Set toggle var.
								bToggle = true;
							}

							// Get camera frame.
							m_vCameraSinks[0].GrabFrame(m_pFrame);
						}
					}
				}
				catch (const exception& e)
				{
					//SetIsStopping(true);
					cout << "WARNING: Video data empty or camera not present." << "\n";
				}

				// Calculate FPS.
				m_nFPS = m_pFPS->FramesPerSec();

				// If the program stops shutdown the thread.
				if (m_bIsStopping)
				{
					break;
				}
			}

			// Clean-up.
			m_bIsStopped = true;
			return;
		}

		/****************************************************************************
				Description:	Signals the thread to stop.

				Arguments: 		BOOL
	
				Returns: 		Nothing
		****************************************************************************/
		void SetIsStopping(bool bIsStopping)
		{
			this->m_bIsStopping = bIsStopping;
		}

		/****************************************************************************
				Description:	Gets if the thread has stopped.

				Arguments: 		None
	
				Returns: 		BOOL
		****************************************************************************/
		bool GetIsStopped()
		{
			return m_bIsStopped;
		}

		/****************************************************************************
				Description:	Gets the current FPS of the thread.

				Arguments: 		None
	
				Returns: 		Int
		****************************************************************************/
		int GetFPS()
		{
			return m_nFPS;
		}

	private:
		// Create objects and variables.
		FPS*					m_pFPS;
		int						m_nFPS;
	};

	/****************************************************************************
			Description:	Implements the VideoProcess Class

			Classes:		VideoProcess

			Project:		2020 DeepSpace Vision Code

			Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
	****************************************************************************/
	class VideoProcess
	{
	public:
		/****************************************************************************
				Description:	VideoProcess constructor.

				Arguments:		None

				Derived From:	Nothing
		****************************************************************************/
		VideoProcess()
		{
			// Create object pointers.
			m_pFPS									= new FPS();
			
			// Initialize member variables.
			m_pKernel								= getStructuringElement(MORPH_RECT, Size(3, 3));
			m_nFPS									= 0;
			m_nNumberOfPolyCorners					= 8;
			m_nMinimumNumberOfTapeVertices			= 6;
			m_nMinimumPowerCellRadius				= 1;
			m_nMaxContours							= 50;
			m_nGreenBlurRadius						= 3;
			m_nOrangeBlurRadius						= 3;
			m_nHorizontalAspect						= 4;
			m_nVerticalAspect						= 3;
			m_dCameraFOV							= 75;
			m_nScreenWidth							= 640;
			m_nScreenHeight							= 480;
			m_dFocalLength							= (m_nScreenWidth / 2.0) / tan((m_dCameraFOV * PI / 180.0) / 2.0);
			m_bIsStopping							= false;
			m_bIsStopped							= false;

			////
			// Setup SolvePNP data.
			////

			// Reference object points.
			m_pObjectPoints.emplace_back(Point3f(39.50, 0.0, 0.0));
			m_pObjectPoints.emplace_back(Point3f(29.50, -17.0, 0.0));
			m_pObjectPoints.emplace_back(Point3f(9.75, -17.0, 0.0));
			m_pObjectPoints.emplace_back(Point3f(0.0, 0.0, 0.0));

			// Precalibrated camera matrix values.
			double mtx[3][3] = {{516.5613698781304, 0.0, 320.38297194779585},		//// PSEye Cam
								{0.0, 515.9356734667019, 231.73585601568368},
								{0.0, 0.0, 1.0}};
			// double mtx[3][3] = {{659.3851992714341, 0.0, 306.98918779442675},		//// Lifecam
			// 					{0.0, 659.212123568372, 232.07157473243464},
			// 					{0.0, 0.0, 1.0}};
			m_pCameraMatrix = Mat(3, 3, CV_64FC1, mtx).clone();

			// Precalibration distance/distortion values.
			double dist[5] = {-0.0841024904469607, 0.014864043816324026, -0.00013887041018197853, -0.0014661216967276468, 0.5671907234987197};	//// PSEye Cam
			// double dist[5] = {0.1715327237204972, -1.3255106761114646, 7.713495040297368e-07, -0.0035865453000784634, 2.599132082766894};	//// Lifecam
			m_pDistanceCoefficients = Mat(1, 5, CV_64FC1, dist).clone();
		}

		/****************************************************************************
				Description:	VideoProcess destructor.

				Arguments:		None

				Derived From:	Nothing
		****************************************************************************/
		~VideoProcess()
		{
			// Delete object pointers.
			delete m_pFPS;

			// Set object pointers as nullptrs.
			m_pFPS = nullptr;
		}

		/****************************************************************************
				Description:	Processes frames with OpenCV.

				Arguments: 		MAT&, MAT&, BOOL&, BOOL&, SHARED_TIMED_MUTEX&, SHARED_TIMED_MUTEX&
	
				Returns: 		Nothing
		****************************************************************************/
		void Process(Mat &m_pFrame, Mat &m_pFinalImg, int &m_nTargetCenterX, int &m_nTargetCenterY, double &m_dHoodPosition, double &m_dTargetAngle, bool &m_bDrivingMode, bool &m_bTrackingMode, bool &m_bSolvePNPEnabled, vector<int> &m_vTrackbarValues, vector<double> &m_vSolvePNPValues, VideoGet &pVideoGetter, shared_timed_mutex &m_pMutexGet, shared_timed_mutex &m_pMutexShow)
		{
			// Give other threads enough time to start before processing camera frames.
			this_thread::sleep_for(std::chrono::milliseconds(800));

			while (1)
			{
				// Increment FPS counter.
				m_pFPS->Increment();

				// Make sure frame is not corrupt.
				try
				{
					// Acquire resource lock for read thread. NOTE: This line has been commented out to improve processing speed. VideoGet takes to long with the resources.
					//shared_lock<shared_timed_mutex> guard(m_pMutexGet);

					if (!m_pFrame.empty())
					{
						// Convert image from RGB to HSV.
						//cvtColor(m_pFrame, m_pHSVImg, COLOR_BGR2HSV);
						// Acquire resource lock for show thread only after m_pFrame has been used.
						unique_lock<shared_timed_mutex> guard(m_pMutexShow);
						// Copy frame to a new mat.
						m_pFinalImg = m_pFrame.clone();
						// Blur the image.
						blur(m_pFrame, m_pBlurImg, Size(m_nGreenBlurRadius, m_nGreenBlurRadius));
						// Filter out specific color in image.
						inRange(m_pBlurImg, Scalar(m_vTrackbarValues[0], m_vTrackbarValues[2], m_vTrackbarValues[4]), Scalar(m_vTrackbarValues[1], m_vTrackbarValues[3], m_vTrackbarValues[5]), m_pFilterImg);
						// Apply blur to image.
						dilate(m_pFilterImg, m_pDilateImg, m_pKernel);

						// Find countours of image.
						findContours(m_pDilateImg, m_pContours, m_pHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);		//// TRY CHAIN_APPROX_SIMPLE		//// Not sure what this method of detection does, but it worked before: CHAIN_APPROX_TC89_KCOS

						// Driving mode.
						if (!m_bDrivingMode)
						{
							// Tracking mode. (Tape or PowerCells)
							if (m_bTrackingMode)
							{
								// Track tape target.

								// Create variables and arrays.
								int nCX = 0;
								int nCY = 0;
								vector<vector<double>> vTapeTargets;
								vector<vector<vector<Point>>> vTapeTargets2;
								vector<vector<double>> vBiggestContours;
								vector<vector<vector<Point>>> vBiggestContours2;
								vector<Point2f> vImagePoints;

								// Draw all contours in white.
								drawContours(m_pFinalImg, m_pContours, -1, Scalar(255, 255, 210), 1, LINE_4, m_pHierarchy);

								if (m_pContours.size() > 0)
								{
									// Sort contours from biggest to smallest.
									sort(m_pContours.begin(), m_pContours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {	return fabs(contourArea(c1, false)) < fabs(contourArea(c2, false)); });

									// Loop through detected contours.
									for (vector<Point> pContour : m_pContours)
									{
										// Limit number of objects to do calculations for.
										if (vBiggestContours.size() < m_nMaxContours)
										{
											// Gets the (x, y) and radius of the enclosing circle of contour.
											Point2f pCenter;
											float dRadius = 0;
											minEnclosingCircle(pContour, pCenter, dRadius);
											// Makes bounding rectangle of contour.
											Rect rCoordinates = boundingRect(pContour);
											// Draws contour of bounding rectangle and enclosing circle in green.
											rectangle(m_pFinalImg, rCoordinates, Scalar(23, 184, 80), 1, LINE_4, 0);
											circle(m_pFinalImg, pCenter, dRadius, Scalar(23, 184, 80), 1, LINE_4, 0);

											// Find convex hull. (Bounding polygon on contour.)
											vector<Point> pHull;
											convexHull(pContour, pHull, false);
											// Calculate hull area.
											double dHullArea = fabs(contourArea(pHull));

											// Calculate contour's distance from the camera.
											double dAngle = CalculateXAngle(pCenter.x);
											double dHoodPosition = CalculateHood(pCenter.y);

											// Approximate contour with accuracy proportional to contour perimeter.
											vector<Point> vApprox;
											approxPolyDP(Mat(pContour), vApprox, arcLength(Mat(pContour), true) * 0.02, true);

											// Approximate the contours corners.
											vector<Point> vPolyCorners;
											approxPolyDP(Mat(pHull), vPolyCorners, m_nNumberOfPolyCorners, true); // Only 4 corners.
											// approxPolyDP(Mat(pContour), vPolyCorners, m_nNumberOfPolyCorners, true); // All corners.

											// Appends contour data to arrays after checking for duplicates.
											vector<double> points;
											points.emplace_back(double(pCenter.x));
											points.emplace_back(double(pCenter.y));
											points.emplace_back(dHoodPosition);
											points.emplace_back(dAngle);
											points.emplace_back(vApprox.size());
											points.emplace_back(dHullArea);
	
											vBiggestContours.emplace_back(points);

											// Appends special contour data to a secondary array.
											vector<vector<Point>> points2;
											points2.emplace_back(vPolyCorners);
											points2.emplace_back(vApprox);

											vBiggestContours2.emplace_back(points2);
										}
									}

									// Sort array based on coordinates (leftmost to rightmost) to make sure contours are adjacent.
									//sort(vBiggestContours.begin(), vBiggestContours.end(), [](const vector<double>& points1, const vector<double>& points2) { return points1[0] < points2[0]; }); 		// Sorts using nCX location.

									// Target checking.
									for (int i = 0; i < int(vBiggestContours.size()); i++)
									{
										// X and Y coordinates of contours.
										int nCX1 = vBiggestContours[i][0];
										int nCY1 = vBiggestContours[i][1];

										// Distance of contour from camera.
										double dHoodPosition = vBiggestContours[i][2];
										// Angle of contour from camera.
										double dAngle = vBiggestContours[i][3];
										// Radius of contour.
										double dArea = vBiggestContours[i][5];
										// Bounding polygon of contour;
										vector<Point> vPolyCorners = vBiggestContours2[i][0];
										vector<Point> vApprox = vBiggestContours2[i][1];

										// Skip small or non convex contours that don't have more than 6 vertices.
										if (int(vBiggestContours[i][4]) >= m_nMinimumNumberOfTapeVertices)
										{
											// Appends contour data to arrays after checking for duplicates.
											vector<double> points;
											points.emplace_back(double(nCX1));
											points.emplace_back(double(nCY1));
											points.emplace_back(dHoodPosition);
											points.emplace_back(dAngle);
											points.emplace_back(dArea);

											vTapeTargets.emplace_back(points);

											// Appends special contour data to a secondary array.
											vector<vector<Point>> points2;
											points2.emplace_back(vPolyCorners);
											points2.emplace_back(vApprox);

											vTapeTargets2.emplace_back(points2);
										}
									}

									// Draw how many targets are detected on screen.
									putText(m_pFinalImg, ("Targets Detected: " + to_string(vTapeTargets.size())), Point(10, m_pFinalImg.rows - 40), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
								}
								
								// Check if there are targets seen.
								if (int(vTapeTargets.size()) > 0)
								{
									// Track the biggest target closest to center.
									int nTargetPositionX;
									int nTargetPositionY;
									double dHoodPosition;
									double dAngle;
									double dBiggestContour = 0;
									vector<Point> vPolyCorners;

									for (int i = 0; i < int(vTapeTargets.size()); i++)
									{
										// Check the object size and distance from center.
										if (vTapeTargets[i][4] > dBiggestContour)
										{
											// Store the object location and distance.
											nTargetPositionX = vTapeTargets[i][0];
											nTargetPositionY = vTapeTargets[i][1];
											dHoodPosition = vTapeTargets[i][2];
											dAngle = vTapeTargets[i][3];

											// Store the object corners.
											vPolyCorners = vTapeTargets2[i][0];

											// Store new biggest contour.
											dBiggestContour = vTapeTargets[i][4];
										}
									}

									// If SolvePNP toggle is enabled, then estimate the object pose using the raw contour points.
									if (m_bSolvePNPEnabled)
									{
										for (Point pPoint : vPolyCorners)
										{
											// Draw corner point onto image for viewing.
											circle(m_pFinalImg, Point(pPoint.x, pPoint.y), 0,  Scalar(0, 0, 0), 5, 8, 0);
											vImagePoints.emplace_back(Point2f(pPoint.x, pPoint.y));
										}
										
										// Sort the image points from greatest to least based on X value.
										sort(vImagePoints.begin(), vImagePoints.end(), [](const Point2f& point1, const Point2f& point2) { return int(point1.x) > int(point2.x); });;

										// DEBUG.
										cout << vPolyCorners << "\n\n";

										// Calculate object pose.
										m_vSolvePNPValues = SolveObjectPose(vImagePoints, ref(m_pFinalImg), ref(m_pFrame), nTargetPositionX, nTargetPositionY);
									} 

									// Push position of tracked target.
									m_nTargetCenterX = nTargetPositionX - (m_nScreenWidth / 2);
									m_nTargetCenterY = -(nTargetPositionY - (m_nScreenHeight / 2));
									m_dHoodPosition = dHoodPosition;
									m_dTargetAngle = dAngle;

									// Draw target distance and target crosshairs with error line.
									putText(m_pFinalImg, ("size:" + to_string(dBiggestContour)), Point(10, m_pFinalImg.rows - 100), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
									line(m_pFinalImg, Point(nTargetPositionX, m_nScreenHeight), Point(nTargetPositionX, 0), Scalar(0, 0, 200), 1, LINE_4, 0);
									line(m_pFinalImg, Point(0, nTargetPositionY), Point(m_nScreenWidth, nTargetPositionY), Scalar(0, 0, 200), 1, LINE_4, 0);
									//line(m_pFinalImg, Point((m_nScreenWidth / 2), (m_nScreenHeight / 2)), Point(nTargetPositionX, nTargetPositionY), Scalar(200, 0, 0), 2, LINE_4, 0);
								}
							}
							else
							{
								// Track power cells.

								// Create variables and arrays.
								int nCX = 0;
								int nCY = 0;
								vector<vector<double>> vBallTargets;
								vector<vector<double>> vBiggestContours;

								// Draw all contours in white.
								drawContours(m_pFinalImg, m_pContours, -1, Scalar(255, 255, 210), 1, LINE_4, m_pHierarchy);

								if (m_pContours.size() > 0)
								{
									// Sort contours from biggest to smallest.
									sort(m_pContours.begin(), m_pContours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {	return fabs(contourArea(c1, false)) < fabs(contourArea(c2, false)); });

									// Loop through detected contours.
									for (vector<Point> pContour : m_pContours)
									{
										// Limit number of objects to do calculations for.
										if (vBiggestContours.size() < m_nMaxContours)
										{
											// Gets the (x, y) and radius of the enclosing circle of contour.
											Point2f pCenter;
											float dRadius = 0;
											minEnclosingCircle(pContour, pCenter, dRadius);
											// Draws contour of bounding rectangle and enclosing circle in green.
											circle(m_pFinalImg, pCenter, dRadius, Scalar(0, 0, 250), 1, LINE_4, 0);

											// Find convex hull. (Bounding polygon on contour.)
											vector<Point> pHull;
											convexHull(pContour, pHull, false);
											// Calculate hull area.
											double dHullArea = fabs(contourArea(pHull));

											// Calculate contour's distance from the camera.
											double dAngle = CalculateXAngle(pCenter.x);

											// Appends contour data to arrays after checking for duplicates.
											vector<double> points;
											points.emplace_back(double(pCenter.x));
											points.emplace_back(double(pCenter.y));
											points.emplace_back(dAngle);
											points.emplace_back(dHullArea);
											points.emplace_back(dRadius);
	
											vBiggestContours.emplace_back(points);
										}
									}

									// Sort array based on coordinates (leftmost to rightmost) to make sure contours are adjacent.
									sort(vBiggestContours.begin(), vBiggestContours.end(), [](const vector<double>& points1, const vector<double>& points2) { return points1[0] < points2[0]; }); 		// Sorts using nCX location.

									// Target checking.
									for (int i = 0; i < int(vBiggestContours.size()); i++)
									{
										// X and Y coordinates of contours.
										int nCX1 = vBiggestContours[i][0];
										int nCY1 = vBiggestContours[i][1];
										// Angle of contour from camera.
										double dAngle = vBiggestContours[i][2];
										// Hull area of contour.
										double dArea = vBiggestContours[i][3];
										// Radius of contour.
										double dRadius = vBiggestContours[i][4];

										// Skip small contours that don't have a radius greater than a number.
										if (int(vBiggestContours[i][4]) >= m_nMinimumPowerCellRadius)
										{
											// Appends contour data to arrays after checking for duplicates.
											vector<double> points;
											points.emplace_back(double(nCX1));
											points.emplace_back(double(nCY1));
											points.emplace_back(dAngle);
											points.emplace_back(dArea);
											points.emplace_back(dRadius);

											vBallTargets.emplace_back(points);
										}
									}

									// Draw how many targets are detected on screen.
									putText(m_pFinalImg, ("Targets Detected: " + to_string(vBallTargets.size())), Point(10, m_pFinalImg.rows - 40), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
								}
								
								// Check if there are targets seen.
								if (int(vBallTargets.size()) > 0)
								{
									// Create instance variables.
									int nTargetPositionX;
									int nTargetPositionY;
									double dAngle;
									double dHullArea;
									double dBiggestContour = 0;

									// Track the biggest target.
									for (int i = 0; i < int(vBallTargets.size()); i++)
									{
										// Check the object size and distance from center.
										if (vBallTargets[i][4] > dBiggestContour)
										{
											// Store the object location and distance.
											nTargetPositionX = vBallTargets[i][0];
											nTargetPositionY = vBallTargets[i][1];
											dAngle = vBallTargets[i][2];
											dHullArea = vBallTargets[i][3];

											// Store new biggest contour.
											dBiggestContour = vBallTargets[i][4];
										}
									}

									// Push position of tracked target.
									m_nTargetCenterX = nTargetPositionX - (m_nScreenWidth / 2);
									m_nTargetCenterY = -(nTargetPositionY - (m_nScreenHeight / 2));
									m_dHoodPosition = 0.0;
									m_dTargetAngle = dAngle;

									// Draw target distance and target crosshairs with error line.
									putText(m_pFinalImg, ("Radius:" + to_string(dBiggestContour)), Point(10, m_pFinalImg.rows - 80), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
									putText(m_pFinalImg, ("Hull Area:" + to_string(dHullArea)), Point(10, m_pFinalImg.rows - 100), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
									line(m_pFinalImg, Point(nTargetPositionX, m_nScreenHeight), Point(nTargetPositionX, 0), Scalar(0, 0, 200), 1, LINE_4, 0);
									line(m_pFinalImg, Point(0, nTargetPositionY), Point(m_nScreenWidth, nTargetPositionY), Scalar(0, 0, 200), 1, LINE_4, 0);
									//line(m_pFinalImg, Point((m_nScreenWidth / 2), (m_nScreenHeight / 2)), Point(nTargetPositionX, nTargetPositionY), Scalar(200, 0, 0), 2, LINE_4, 0);
								}

								// This is for tracking the chessboard.
								// vector<Point2f> vImagePoints;
								// vImagePoints.emplace_back(Point2f(0.0, 0.0));
								// int nTargetPositionX = 0; 
								// int nTargetPositionY = 0;
								// m_vSolvePNPValues = SolveObjectPose(vImagePoints, ref(m_pFinalImg), ref(m_pFrame), nTargetPositionX, nTargetPositionY);
							}
						}

						// Put FPS on image.
						m_nFPS = m_pFPS->FramesPerSec();
						putText(m_pFinalImg, ("Camera FPS: " + to_string(pVideoGetter.GetFPS())), Point(420, m_pFinalImg.rows - 40), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
						putText(m_pFinalImg, ("Processor FPS: " + to_string(m_nFPS)), Point(420, m_pFinalImg.rows - 20), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);

						// Release garbage mats.
						m_pHSVImg.release();
						m_pBlurImg.release();
						m_pFilterImg.release();
					}
				}
				catch (const exception& e)
				{
					//SetIsStopping(true);
					// Print error to console and show that an error has occured on the screen.
					putText(m_pFinalImg, "Image Processing ERROR", Point(280, m_pFinalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.65, Scalar(0, 0, 250), 1);
					cout << "\nWARNING: MAT corrupt or a runtime error has occured! Frame has been dropped." << "\n" << e.what();
				}

				// If the program stops shutdown the thread.
				if (m_bIsStopping)
				{
					break;
				}
			}

			// Clean-up.
			m_bIsStopped = true;
		}

		/****************************************************************************
				Description:	Turn negative numbers into -1, positive numbers 
								into 1, and returns 0 when 0.

				Arguments: 		DOUBLE
	
				Returns: 		INT
		****************************************************************************/
		int SignNum(double dVal)
		{
			return (double(0) < dVal) - (dVal < double(0));
		}

		/****************************************************************************
				Description:	Calculate the Z distance of object with the focal 
								length of the camera and Y location of contour. Then,
								use that distance to interpolate the hood angle.

				Arguments: 		INT
	
				Returns: 		DOUBLE
		****************************************************************************/
		double CalculateHood(int nObjectY)
		{
			// Create instance variables.
			double dCameraCenterY = (m_nScreenHeight - 1) / 2.0;
			double dTargetHeight = 98.25;
			double dTurretHeight = 16.75;
			double dCameraAngleOffset = 22 * (PI / 180);	// Convert to radians.
			double dBallExitOffsetX = 4.0;					// X offset in inches of the ball exiting the shooter from the camera.
			double dBallExitOffsetY = 7.0;					// Y offset in inches of the ball exiting the shooter from the camera.

			// // Calculate the angle of the robot from the robot.
			// double dAngle = atan(-(nObjectY - dCameraCenterY) / m_dFocalLength);
			// // Account for camera offset.
			// dAngle += dCameraAngleOffset;

			// // Find the distance of the camera to the target.
			// double dDistance = (dTargetHeight - dTurretHeight) / tan(dAngle);

			double nObjectCenter = -(nObjectY - dCameraCenterY);

			double dHoodAngle = (1e-11 * pow(nObjectCenter, 6)) - (4e-9 * pow(nObjectCenter, 5)) + (2e-7 * pow(nObjectCenter, 4)) + (7e-5 * pow(nObjectCenter, 3)) - (0.0065 * pow(nObjectCenter, 2)) - (0.7871 * nObjectCenter) + 234.35;

			return dHoodAngle;
		}

		/****************************************************************************
				Description:	Calculate the X angle of object with the focal 
								length of the camera and X location of contour.

				Arguments: 		INT
	
				Returns: 		DOUBLE
		****************************************************************************/
		double CalculateXAngle(int nObjectX)
		{
			// Create instance variables.
			double dCameraCenterX = (m_nScreenWidth - 1) / 2.0;

			// Calculate the angle of the robot from the object.
			double dAngle = atan((nObjectX - dCameraCenterX) / m_dFocalLength) * (180.0 / PI);

			return dAngle;
		}

		/****************************************************************************
				Description:	Use the detected object points and real world reference
								points to estimated the 3D pose of the object.

				Arguments: 		INPUT VECTOR, OUTPUT VECTOR
	
				Returns: 		OUTPUT VECTOR (6 values)
		****************************************************************************/
		vector<double> SolveObjectPose(vector<Point2f> m_pImagePoints, Mat &m_pFinalImg, Mat &m_pFrame, int nTargetPositionX, int nTargetPositionY)
		{
			// Create instance variables.
			static int nCount = 0;
			Vec3d					vEulerAngles;
			vector<Point2f>			vPositionVector;
			Mat						vMTXR;
			Mat						vMTXQ;
			Mat						vRotationVectors;
			Mat						vRotationMatrix;
			Mat						vTranslationVectors;
			Mat						vTranslationMatrix;
			Mat						vTRNSP;
			Mat 					mNewCameraMatrix;
			Mat						mUndistort;
			Mat						mGray;
			TermCriteria mTermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.001);

			// Create a vector that stores 0s by default. 
			vector<double>	vObjectPosition;
			vObjectPosition.emplace_back(1);
			vObjectPosition.emplace_back(2);
			vObjectPosition.emplace_back(3);
			vObjectPosition.emplace_back(4);
			vObjectPosition.emplace_back(5);
			vObjectPosition.emplace_back(6);

			// This is for tracking the chessboard.
			// vector<Point3f> chessboards;
			// for (double i = 0; i < 9; i++)			//// These were switched around.
			// {
			// 	for (double j = 0; j < 6; j++)		//// These were switched around.
			// 	{
			// 		chessboards.emplace_back(Point3f(i, j, 0.0));
			// 	}
			// }

			// Catch any anomalies from SolvePNP process. (Like bad input data errors.)
			try
			{
				// Refine the corners from image points.
				// vector<Point2f> corners;
				cvtColor(m_pFinalImg, mGray, COLOR_BGR2GRAY);
				// bool bFound = findChessboardCorners(mGray, Size(6, 9), corners);			// These have a possibility of being switched around.
				cornerSubPix(mGray, m_pImagePoints, Size(11, 11), Size(-1, -1), mTermCriteria);
				// drawChessboardCorners(m_pFinalImg, Size(6, 9), corners, bFound);			// These have a possibility of being switched around.

				// Use real world reference points and image points to estimate object pose.
				bool bSuccess = solvePnP(m_pObjectPoints,					// Object reference points in 3D space.			
											m_pImagePoints,					// Object points from the 2D camera image.
											m_pCameraMatrix,				// Precalibrated camera matrix. (camera specific)
											m_pDistanceCoefficients,		// Precalibrated camera config. (camera specific)
											vRotationVectors,				// Storage vector for rotation values.
											vTranslationVectors,			// Storage vector for translation values.
											false,							// Use the provided rvec and tvec values as initial approximations of the rotation and translation vectors, and further optimize them? (useExtrensicGuess)
				 							SOLVEPNP_ITERATIVE				// Method used for the PNP problem.
										);
				// bool bSuccess = solvePnPRansac(m_pObjectPoints,						// Object reference points in 3D space.			
				// 									m_pImagePoints,						// Object points from the 2D camera image.
				// 									m_pCameraMatrix,				// Precalibrated camera matrix. (camera specific)
				// 									m_pDistanceCoefficients,		// Precalibrated camera config. (camera specific)
				// 									vRotationVectors,				// Storage vector for rotation values.
				// 									vTranslationVectors,			// Storage vector for translation values.
				// 									false,							// Use the provided rvec and tvec values as initial approximations of the rotation and translation vectors, and further optimize them? (useExtrensicGuess)
				//  									100,							// Number of iterations. (adjust for performance?)
				//  									15.0,							// Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier.
			 	// 									0.99,							// Confidence value that the algorithm produces a useful result. 
				//  									noArray(),						// Output vector that contains indices of inliers in objectPoints and imagePoints.
				//  									SOLVEPNP_ITERATIVE				// Method used for the PNP problem.
				// 								);

				// If SolvePNP reports a success, then continue with calculations. Else, keep searching. 
				if (bSuccess)
				{
					// Convert the rotation matrix from the solvePNP function to a rotation vector, or vise versa.
					Rodrigues(vRotationVectors, vRotationMatrix);

					// Calculate the camera x, y, z translation.
					transpose(vRotationMatrix, vTRNSP);
					vTranslationMatrix = -vTRNSP * vTranslationVectors;

					// Calculate the pitch, roll, yaw angles of the camera.
					vEulerAngles = RQDecomp3x3(vRotationMatrix, vMTXR, vMTXQ);

					// Store the calculated object values in the vector.
					vObjectPosition.at(0) = vTranslationMatrix.at<double>(0);
					vObjectPosition.at(1) = vTranslationMatrix.at<double>(1);
					vObjectPosition.at(2) = vTranslationMatrix.at<double>(2);
					vObjectPosition.at(3) = vEulerAngles[0];
					vObjectPosition.at(4) = vEulerAngles[1];
					vObjectPosition.at(5) = vEulerAngles[2];

					// Draw axis vectors.
					drawFrameAxes(m_pFinalImg, m_pCameraMatrix, m_pDistanceCoefficients, vRotationVectors, vTranslationVectors, 20.0);

					// Print status onto image.
					putText(m_pFinalImg, "PNP Status: found match!", Point(50, m_pFinalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.40, Scalar(0, 0, 250), 1);
				}
				else
				{
					// If the object is not found, then put 0s in the vector.
					vObjectPosition.emplace_back(0);
					vObjectPosition.emplace_back(0);
					vObjectPosition.emplace_back(0);
					vObjectPosition.emplace_back(0);
					vObjectPosition.emplace_back(0);
					vObjectPosition.emplace_back(0);

					// Print status onto image.
					putText(m_pFinalImg, "PNP Status: searching...", Point(50, m_pFinalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.40, Scalar(0, 0, 250), 1);
				}

				// Reset toggle if the code ran successfully.
				nCount = 0;
			}
			catch (const exception& e)
			{
				// Print status on screen.
				putText(m_pFinalImg, "PNP Status: point data unsolvable...", Point(50, m_pFinalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.40, Scalar(0, 0, 250), 1);

				// Only print the message to the console once per fail.
				if (nCount <= 100)
				{
					// Print message to console.
					cout << "\nMESSAGE: SolvePNP was unable to process the image data. Moving on...\n" << e.what();

					// Add one error count to toggle.
					nCount++;
				}
			}

			// Return useless stuff for now.
			return vObjectPosition;
		}

		/****************************************************************************
				Description:	Signals the thread to stop.

				Arguments: 		BOOL
	
				Returns: 		Nothing
		****************************************************************************/
		void SetIsStopping(bool bIsStopping)
		{
			this->m_bIsStopping = bIsStopping;
		}

		/****************************************************************************
				Description:	Gets if the thread has stopped.

				Arguments: 		None
	
				Returns: 		BOOL
		****************************************************************************/
		bool GetIsStopped()
		{
			return m_bIsStopped;
		}

		/****************************************************************************
				Description:	Gets the current FPS of the thread.

				Arguments: 		None
	
				Returns: 		INT
		****************************************************************************/
		int GetFPS()
		{
			return m_nFPS;
		}

	private:
		// Create objects and variables.
		Mat							m_pHSVImg;
		Mat							m_pBlurImg;
		Mat							m_pFilterImg;
		Mat							m_pDilateImg;
		Mat							m_pCorners;
		Mat							m_pCornersNormalized;
		Mat							m_pCornersScaled;
		Mat							m_pKernel;
		Mat							m_pCameraMatrix;
		Mat							m_pDistanceCoefficients;
		vector<Point3f>				m_pObjectPoints;
		vector<vector<Point>>		m_pContours;
		vector<Vec4i>				m_pHierarchy;
		FPS*						m_pFPS;

		int							m_nFPS;
		int							m_nMinimumNumberOfTapeVertices;
		int							m_nMinimumPowerCellRadius;
		int							m_nMaxContours;
		int							m_nScreenHeight;
		int							m_nScreenWidth;
		int							m_nGreenBlurRadius;
		int							m_nOrangeBlurRadius;
		int							m_nHorizontalAspect;
		int							m_nVerticalAspect;
		int							m_nNumberOfPolyCorners;
		double						m_dCameraFOV;
		double						m_dFocalLength;
		const double				PI = 3.14159265358979323846;
		bool						m_bIsStopping;
		bool						m_bIsStopped;
	};

	/****************************************************************************
			Description:	Implements the VideoShow Class

			Classes:		VideoShow

			Project:		2020 DeepSpace Vision Code

			Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
	****************************************************************************/
	class VideoShow
	{
	public:
		/****************************************************************************
				Description:	VideoShow constructor.

				Arguments:		None

				Derived From:	Nothing
		****************************************************************************/
		VideoShow()
		{
			// Create objects.
			m_pFPS									= new FPS();

			// Initialize member variables.
			m_bIsStopping							= false;
			m_bIsStopped							= false;
		}

		/****************************************************************************
				Description:	VideoShow destructor.

				Arguments:		None

				Derived From:	Nothing
		****************************************************************************/
		~VideoShow()
		{
			// Delete object pointers.
			delete m_pFPS;

			// Set object pointers as nullptrs.
			m_pFPS = nullptr;
		}

		/****************************************************************************
				Description:	Method that gives the processed frame to CameraServer.

				Arguments: 		MAT&, SHARED_TIMED_MUTEX&
	
				Returns: 		Nothing
		****************************************************************************/
		void ShowFrame(Mat &m_pFrame, shared_timed_mutex &m_pMutex)
		{
			// Give other threads some time.
			this_thread::sleep_for(std::chrono::milliseconds(1000));

			while (1)
			{
				// Increment FPS counter.
				m_pFPS->Increment();

				// Check to make sure frame is not corrupt.
				try
				{
					// Slow thread down to save bandwidth.
					this_thread::sleep_for(std::chrono::milliseconds(25));

					// Acquire resource lock for thread.
					shared_lock<shared_timed_mutex> guard(m_pMutex);

					if (!m_pFrame.empty())
					{
						// Output frame to camera stream.
						m_vCameraSources[0].PutFrame(m_pFrame);
					}
					else
					{
						// Print that frame is empty.
						cout << "WARNING: Frame is empty!" << "\n";
					}
				}
				catch (const exception& e)
				{
					//SetIsStopping(true);
					cout << "WARNING: MAT corrupt. Frame has been dropped." << "\n";
				}

				// Calculate FPS.
				m_nFPS = m_pFPS->FramesPerSec();

				// If the program stops shutdown the thread.
				if (m_bIsStopping)
				{
					break;
				}
			}

			// Clean-up.
			m_bIsStopped = true;
		}

		/****************************************************************************
				Description:	Signals the thread to stop.

				Arguments: 		BOOL
	
				Returns: 		Nothing
		****************************************************************************/
		void SetIsStopping(bool bIsStopping)
		{
			this->m_bIsStopping = bIsStopping;
		}

		/****************************************************************************
				Description:	Gets if the thread has stopped.

				Arguments: 		Nothing
	
				Returns: 		BOOL 
		****************************************************************************/
		bool GetIsStopped()
		{
			return m_bIsStopped;
		}

		/****************************************************************************
				Description:	Gets the current FPS of the thread.

				Arguments: 		Nothing
	
				Returns: 		INT
		****************************************************************************/
		int GetFPS()
		{
			return m_nFPS;
		}

	private:
		// Create objects and variables.
		FPS*						m_pFPS;

		int							m_nFPS;
		bool						m_bIsStopping;
		bool						m_bIsStopped;
	};
}  // End of namespace.


/****************************************************************************
	Description:	Main method

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
int main(int argc, char* argv[]) 
{
	/************************************************************************** 
	  			Read Configurations
	 * ************************************************************************/
	if (argc >= 2) 
	{
		m_cConfigFile = argv[1];
	}

	if (!ReadConfig())
	{
		return EXIT_FAILURE;
	}

	/**************************************************************************
	  			Start NetworkTables
	 * ************************************************************************/
	// Create instance.
	auto NetworkTablesInstance = NetworkTableInstance::GetDefault();
	auto NetworkTable = NetworkTablesInstance.GetTable("SmartDashboard");

	// Start Networktables as a client or server.
	if (m_bServer) 
	{
		outs() << "Setting up NetworkTables server" << "\n";
		NetworkTablesInstance.StartServer();
	} 
	else 
	{
		outs() << "Setting up NetworkTables client for team " << m_nTeam << "\n";
		NetworkTablesInstance.StartClientTeam(m_nTeam);
	}

	// Populate NetworkTables.
	NetworkTable->PutBoolean("Camera Source", false);
	NetworkTable->PutBoolean("Driving Mode", false);
	NetworkTable->PutBoolean("Tape Tracking Mode", true);
	NetworkTable->PutBoolean("Enable SolvePNP", false);
	NetworkTable->PutNumber("HMN", 157);
	NetworkTable->PutNumber("HMX", 255);
	NetworkTable->PutNumber("SMN", 119);
	NetworkTable->PutNumber("SMX", 255);
	NetworkTable->PutNumber("VMN", 0);
	NetworkTable->PutNumber("VMX", 110);

	/**************************************************************************
	 			Start Cameras
	 * ************************************************************************/
	for (const auto& config : m_vCameraConfigs)
	{
		StartCamera(config);
	}

	/**************************************************************************
	 			Start Image Processing on Camera 0
	 * ************************************************************************/
	if (m_vCameraSinks.size() >= 1) 
	{
		// Create object pointers for threads.
		VideoGet m_pVideoGetter;
		VideoProcess m_pVideoProcessor;
		VideoShow m_pVideoShower;

		// Preallocate image objects.
		Mat	m_pFrame(480, 640, CV_8U, 1);
		Mat m_pFinalImg(480, 640, CV_8U, 1);

		// Create a global instance of mutex to protect it.
		shared_timed_mutex m_pMutexGet;
		shared_timed_mutex m_pMutexShow;

		// Vision options and values.
		int m_nTargetCenterX = 0;
		int m_nTargetCenterY = 0;
		double m_dHoodPosition = 0;
		double m_dTargetAngle = 0;
		bool m_bCameraSourceIndex = false;
		bool m_bDrivingMode = false;
		bool m_bTrackingMode = true;
		bool m_bEnableSolvePNP = false;
		vector<int> m_vTrackbarValues {1, 255, 1, 255, 1, 255};
		vector<double> m_vSolvePNPValues {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Start multi-threading.
		thread m_pVideoGetThread(&VideoGet::StartCapture, &m_pVideoGetter, ref(m_pFrame), ref(m_bCameraSourceIndex), ref(m_bDrivingMode), ref(m_pMutexGet));
		thread m_pVideoProcessThread(&VideoProcess::Process, &m_pVideoProcessor, ref(m_pFrame), ref(m_pFinalImg), ref(m_nTargetCenterX), ref(m_nTargetCenterY), ref(m_dHoodPosition), ref(m_dTargetAngle), ref(m_bDrivingMode), ref(m_bTrackingMode), ref(m_bEnableSolvePNP), ref(m_vTrackbarValues), ref(m_vSolvePNPValues), ref(m_pVideoGetter), ref(m_pMutexGet), ref(m_pMutexShow));
		thread m_pVideoShowerThread(&VideoShow::ShowFrame, &m_pVideoShower, ref(m_pFinalImg), ref(m_pMutexShow));

		while (1)
		{
			try
			{
				// Check if any of the threads have stopped.
				if (!m_pVideoGetter.GetIsStopped() && !m_pVideoProcessor.GetIsStopped() && !m_pVideoShower.GetIsStopped())
				{
					// Get NetworkTables data.
					m_bCameraSourceIndex = NetworkTable->GetBoolean("Camera Source", false);
					m_bDrivingMode = NetworkTable->GetBoolean("Driving Mode", false);
					m_bTrackingMode = NetworkTable->GetBoolean("Tape Tracking Mode", true);
					m_bEnableSolvePNP = NetworkTable->GetBoolean("Enable SolvePNP", false);
					m_vTrackbarValues[0] = int(NetworkTable->GetNumber("HMN", 1));
					m_vTrackbarValues[1] = int(NetworkTable->GetNumber("HMX", 255));
					m_vTrackbarValues[2] = int(NetworkTable->GetNumber("SMN", 1));
					m_vTrackbarValues[3] = int(NetworkTable->GetNumber("SMX", 255));
					m_vTrackbarValues[4] = int(NetworkTable->GetNumber("VMN", 1));
					m_vTrackbarValues[5] = int(NetworkTable->GetNumber("VMX", 255));

					// Put NetworkTables data.
					NetworkTable->PutNumber("Target Center X", m_nTargetCenterX);
					NetworkTable->PutNumber("Target Center Y", m_nTargetCenterY);
					NetworkTable->PutNumber("Target Distance", m_dHoodPosition);
					NetworkTable->PutNumber("Target Angle", m_dTargetAngle);
					NetworkTable->PutNumber("SPNP X Dist", m_vSolvePNPValues[0]);
					NetworkTable->PutNumber("SPNP Y Dist", m_vSolvePNPValues[1]);
					NetworkTable->PutNumber("SPNP Z Dist", m_vSolvePNPValues[2]);
					NetworkTable->PutNumber("SPNP Roll", m_vSolvePNPValues[3]);
					NetworkTable->PutNumber("SPNP Pitch", m_vSolvePNPValues[4]);
					NetworkTable->PutNumber("SPNP Yaw", m_vSolvePNPValues[5]);

					// Put different trackbar values on smartdashboard depending on camera source.
					static bool bSetValuesToggle = false;
					if (!m_bCameraSourceIndex && bSetValuesToggle == false)		// Turret Camera.
					{
						// Put trackbar values for tape tracking.
						NetworkTable->PutNumber("HMN", 157);
						NetworkTable->PutNumber("HMX", 255);
						NetworkTable->PutNumber("SMN", 119);
						NetworkTable->PutNumber("SMX", 255);
						NetworkTable->PutNumber("VMN", 0);
						NetworkTable->PutNumber("VMX", 110);

						// Set toggle.
						bSetValuesToggle = true;
					}
					else
					{
						if (m_bCameraSourceIndex && bSetValuesToggle == true)	// Bottom Camera.
						{
							// Put trackbar values for tape tracking.
							NetworkTable->PutNumber("HMN", 0);
							NetworkTable->PutNumber("HMX", 0);
							NetworkTable->PutNumber("SMN", 0);
							NetworkTable->PutNumber("SMX", 0);
							NetworkTable->PutNumber("VMN", 0);
							NetworkTable->PutNumber("VMX", 0);

							// Set toggle.
							bSetValuesToggle = false;
						}
					}
					

					// Sleep.
					this_thread::sleep_for(std::chrono::milliseconds(20));

					// Print debug info.
					//cout << "Getter FPS: " << m_pVideoGetter.GetFPS() << "\n";
					//cout << "Processor FPS: " << m_pVideoProcessor.GetFPS() << "\n";
					//cout << "Shower FPS: " << m_pVideoShower.GetFPS() << "\n";
				}
				else
				{
					// Notify other threads the program is stopping.
					m_pVideoGetter.SetIsStopping(true);
					m_pVideoProcessor.SetIsStopping(true);
					m_pVideoShower.SetIsStopping(true);
					break;
				}
			}
			catch (const exception& e)
			{
				cout << "CRITICAL: A main thread error has occured!" << "\n";
			}
		}

		// Stop all threads.
		m_pVideoGetThread.join();
		m_pVideoProcessThread.join();
		m_pVideoShowerThread.join();

		// Print that program has safely and successfully shutdown.
		cout << "All threads have been released! Program will now stop..." << "\n";
		
		// Kill program.
		return 0;
	}
}
