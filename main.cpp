#include <cstdio>
#include <string>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <iostream>
#include <vector>
#include <math.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
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
		void StartCapture(Mat &m_pFrame, shared_timed_mutex &m_pMutex)
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

					// Grab frame from camera.
					m_vCameraSinks[0].GrabFrame(m_pFrame);
				}
				catch (const exception& e)
				{
					//SetIsStopping(true);
					cout << "WARNING: Video data empty." << "\n";
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
			mKernel									= getStructuringElement(MORPH_RECT, Size(3, 3));
			m_nFPS									= 0;
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
			m_pObjectPoints.emplace_back(Point3f());

			// Precalibrated camera matrix values.
			double mtx[3][3] = {{700.4178771192215, 0.0, 323.3391386812556},
								{0.0, 699.1701795469227, 232.428813933943},
								{0.0, 0.0, 1.0}};
			m_pCameraMatrix = Mat(3, 3, CV_64FC1, mtx);

			// Precalibration distance/distortion values.
			double dist[5] = {0.32376580532500226, -1.852588223166456, -0.0008142808002410602, 0.0018634708027606384, 3.0524527683705562};
			m_pDistanceCoefficients = Mat(5, 1, CV_64FC1, dist);
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
						//cvtColor(m_pFrame, mHSVImg, COLOR_BGR2HSV);
						// Acquire resource lock for show thread only after m_pFrame has been used.
						unique_lock<shared_timed_mutex> guard(m_pMutexShow);
						// Copy frame to a new mat.
						m_pFinalImg = m_pFrame.clone();
						// Blur the image.
						blur(m_pFrame, mBlurImg, Size(m_nGreenBlurRadius, m_nGreenBlurRadius));
						// Filter out specific color in image.
						inRange(mBlurImg, Scalar(m_vTrackbarValues[0], m_vTrackbarValues[2], m_vTrackbarValues[4]), Scalar(m_vTrackbarValues[1], m_vTrackbarValues[3], m_vTrackbarValues[5]), mFilterImg);
						// Apply blur to image.
						dilate(mFilterImg, mDilateImg, mKernel);

						// Find countours of image.
						findContours(mDilateImg, m_pContours, m_pHierarchy, RETR_EXTERNAL, CHAIN_APPROX_TC89_KCOS);		//// TRY CHAIN_APPROX_SIMPLE		//// Not sure what this method of detection does, but it worked before: CHAIN_APPROX_TC89_KCOS

						// Driving mode.
						if (!m_bDrivingMode)
						{
							// Tracking mode. (Tape or PowerCells)
							if (m_bTrackingMode)
							{
								// Create variables and arrays.
								int nCX = 0;
								int nCY = 0;
								vector<vector<double>> vTapeTargets;
								vector<vector<double>> vBiggestContours;
								vector<vector<Point>> vContourStorageArray;
								vector<vector<Point>> vFilteredContourStorageArray;
								vector<Point> vImagePoints;

								// Draw all contours in white.
								drawContours(m_pFinalImg, m_pContours, -1, Scalar(255, 255, 210), 1, LINE_4, m_pHierarchy);

								if (m_pContours.size() > 0)
								{
									// Sort contours from biggest to smallest.
									sort(m_pContours.begin(), m_pContours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {	return fabs(contourArea(c1, false)) < fabs(contourArea(c2, false)); });

									// Loop through detected contours.
									for (vector<Point> m_pContour: m_pContours)
									{
										// Limit number of objects to do calculations for.
										if (vBiggestContours.size() < 50)
										{
											// Gets the (x, y) and radius of the enclosing circle of contour.
											Point2f pCenter;
											float dRadius = 0;
											minEnclosingCircle(m_pContour, pCenter, dRadius);
											// Makes bounding rectangle of contour.
											Rect rCoordinates = boundingRect(m_pContour);
											// Draws contour of bounding rectangle and enclosing circle in green.
											rectangle(m_pFinalImg, rCoordinates, Scalar(23, 184, 80), 1, LINE_4, 0);
											circle(m_pFinalImg, pCenter, dRadius, Scalar(23, 184, 80), 1, LINE_4, 0);

											// Find convex hull. (Bounding polygon on contour.)
											vector<Point> m_pHull;
											convexHull(m_pContour, m_pHull, false);
											// Calculate hull area.
											double dHullArea = fabs(contourArea(m_pHull));

											// Calculate contour's distance from the camera.
											double dAngle = CalculateXAngle(pCenter.x);
											double dHoodPosition = CalculateHood(pCenter.y);

											// Approximate contour with accuracy proportional to contour perimeter.
											vector<Point> vApprox;
											approxPolyDP(Mat(m_pContour), vApprox, arcLength(Mat(m_pContour), true) * 0.02, true);

											// Appends contour data to arrays after checking for duplicates.
											vector<double> points;
											points.emplace_back(double(pCenter.x));
											points.emplace_back(double(pCenter.y));
											points.emplace_back(dHoodPosition);
											points.emplace_back(dAngle);
											points.emplace_back(vApprox.size());
											points.emplace_back(dHullArea);
	
											vBiggestContours.emplace_back(points);

											// Store the raw contour data for each object detected.
											vContourStorageArray.emplace_back(m_pContour); 									//// Use CHAIN_SPPROX_SIMPLE and pass through m_pContour with only corner points? Need to test CHAIN_APPROX_SIMPLE on RPI to find out!
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

										// Skip small or non convex contours that don't have more than 4 vertices.
										if (int(vBiggestContours[i][4]) >= 6)
										{
											// Appends contour data to arrays after checking for duplicates.
											vector<double> points;
											points.emplace_back(double(nCX1));
											points.emplace_back(double(nCY1));
											points.emplace_back(dHoodPosition);
											points.emplace_back(dAngle);
											points.emplace_back(dArea);

											vTapeTargets.emplace_back(points);

											// Store the raw contour data of the filtered object. 
											vFilteredContourStorageArray.emplace_back(vContourStorageArray[i]);
										}
									}

									// Draw how many targets are detected on screen.
									putText(m_pFinalImg, ("Targets Detected: " + to_string(vTapeTargets.size())), Point(10, m_pFinalImg.rows - 40), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
								}
								
								// Check if there are targets seen.
								if (int(vTapeTargets.size()) > 0)
								{
									// Track the biggest target closest to center.
									int nTargetPositionX = 0;
									int nTargetPositionY = 0;
									double dHoodPosition = 0.0;
									double dAngle = 0.0;
									double dBiggestContour = 0;

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

											// Store this objects raw contour points for SolvePNP calculation.
											vImagePoints = vFilteredContourStorageArray[i];

											// Store new biggest contour.
											dBiggestContour = vTapeTargets[i][4];
										}
									}

									// If SolvePNP toggle is enabled, then estimate the object pose using the raw contour points.
									if (m_bSolvePNPEnabled)
									{
										////The Try caught a runtime error
										// Go SolvePNP!
										m_vSolvePNPValues = SolveObjectPose(vImagePoints);
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
									line(m_pFinalImg, Point((m_nScreenWidth / 2), (m_nScreenHeight / 2)), Point(nTargetPositionX, nTargetPositionY), Scalar(200, 0, 0), 2, LINE_4, 0);
								}
							}
							else
							{
								/*
								
								 This section serves as a placeholder for future code used to track the power cells. This may or may not actually be implemented... 

								*/ 
							}
						}

						// Put FPS on image.
						m_nFPS = m_pFPS->FramesPerSec();
						putText(m_pFinalImg, ("Camera FPS: " + to_string(pVideoGetter.GetFPS())), Point(420, m_pFinalImg.rows - 40), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
						putText(m_pFinalImg, ("Processor FPS: " + to_string(m_nFPS)), Point(420, m_pFinalImg.rows - 20), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);

						// Release garbage mats.
						mHSVImg.release();
						mBlurImg.release();
						mFilterImg.release();
					}
				}
				catch (const exception& e)
				{
					//SetIsStopping(true);
					cout << "WARNING: MAT corrupt. Frame has been dropped." << "\n";
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
		vector<double> SolveObjectPose(vector<Point> m_pImagePoints)
		{
			// Create instance variables.
			vector<vector<double>>	vRotationVectors;
			vector<vector<double>>	vTranslationVectors;
			vector<vector<double>>	vRotationMatrix;
			vector<vector<double>>	vTranslationMatrix;
			vector<vector<double>>	vMTXR;
			vector<vector<double>>	vMTXQ;
			vector<vector<double>>	vTRNSP;

			vector<double>	vUselessPlaceholderVector;
			vUselessPlaceholderVector.emplace_back(1);
			vUselessPlaceholderVector.emplace_back(2);
			vUselessPlaceholderVector.emplace_back(3);
			vUselessPlaceholderVector.emplace_back(4);
			vUselessPlaceholderVector.emplace_back(5);
			vUselessPlaceholderVector.emplace_back(6);

			// The golden stuff...
			bool bSuccess = solvePnPRansac(m_pObjectPoints,				// Object reference points in 3D space.			
										m_pImagePoints,					// Object points from the 2D camera image.
										m_pCameraMatrix,				// Precalibrated camera matrix. (camera specific)
										m_pDistanceCoefficients,		// Precalibrated camera config. (camera specific)
										vRotationVectors,				// Storage vector for rotation values.
										vTranslationVectors,			// Storage vector for translation values.
										true,							// Use the provided rvec and tvec values as initial approximations of the rotation and translation vectors, and further optimize them? (useExtrensicGuess)
										100,							// Number of iterations. (adjust for performance?)
										8.f,							// Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier.
										0.99,							// Confidence value that the algorithm produces a useful result. 
										noArray(),						// Output vector that contains indices of inliers in objectPoints and imagePoints.
										SOLVEPNP_ITERATIVE				// Method used for the PNP problem.
									);

			// Convert the rotation matrix from the solvePNP function to a rotation vector, or vise versa.
			Rodrigues(vRotationVectors, vRotationMatrix);

			// Calculate the pitch, roll, yaw angles of the object.
			RQDecomp3x3(vRotationMatrix, vMTXR, vMTXQ);

			// Calculate the camera x, y, z translation.
			//transpose(vRotationMatrix, vTRNSP);
			//vTranslationMatrix = -vTRNSP * vTranslationVectors;

			// Return useless stuff for now.
			return vUselessPlaceholderVector;
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
		Mat							mHSVImg;
		Mat							mBlurImg;
		Mat							mFilterImg;
		Mat							mDilateImg;
		Mat							mKernel;
		Mat							m_pCameraMatrix;
		Mat							m_pDistanceCoefficients;
		vector<Point3f>				m_pObjectPoints;
		vector<vector<Point>>		m_pContours;
		vector<Vec4i>				m_pHierarchy;
		FPS*						m_pFPS;

		int							m_nFPS;
		int							m_nScreenHeight;
		int							m_nScreenWidth;
		int							m_nGreenBlurRadius;
		int							m_nOrangeBlurRadius;
		int							m_nHorizontalAspect;
		int							m_nVerticalAspect;
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
		bool m_bDrivingMode = false;
		bool m_bTrackingMode = true;
		bool m_bEnableSolvePNP = false;
		vector<int> m_vTrackbarValues {1, 255, 1, 255, 1, 255};
		vector<double> m_vSolvePNPValues {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Start multi-threading.
		thread m_pVideoGetThread(&VideoGet::StartCapture, &m_pVideoGetter, ref(m_pFrame), ref(m_pMutexGet));
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
