/*
 * OpticalFlow.cpp
 *
 *  Created on: Jan 20, 2014
 *      Author: windsdon
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";

using namespace std;
using namespace cv;

#define PI 3.141592653

//class Vector2f {
//	public:
//		Vector2f(float x, float y) :
//				x(x), y(y) {
//
//		}
//
//		float x;
//		float y;
//
//		Vector2f operator+(Vector2f &right) const {
//			return Vector2f(x + right.x, y + right.y);
//		}
//
//		void operator-() {
//			x = -x;
//			y = -y;
//		}
//
//		Vector2f operator*(float scalar) const {
//			return Vector2f(x * scalar, y * scalar);
//		}
//
//		void operator*(float scalar) {
//			x *= scalar;
//			y *= scalar;
//		}
//
//		float operator*(Vector2f& v) const {
//			return (x * v.x + y * v.y);
//		}
//
//		float size2() const {
//			return (pow(x, 2) + pow(y, 2));
//		}
//
//		float size() const {
//			return sqrt(size2());
//		}
//
//		Vector2f perp() const {
//			return Vector2f(-y, x);
//		}
//
//};

typedef Point2f Vector2f;

class MathHelper {
	public:
		static Vector2f perp(Vector2f& v) {
			return Vector2f(-v.y, v.x);
		}

		static float size2(Vector2f& v) {
			return (pow(v.x, 2) + pow(v.y, 2));
		}

		static float size(Vector2f &v) {
			return sqrt(size2(v));
		}

		static Vector2f norm(Vector2f& v) {
			float s = size(v);
			if (s == 0) {
				return Vector2f();
			}

			return v * (1 / s);
		}

		static Point2f converge(vector<vector<Point2f> > &v) {
			int skip = 0;

			//vector<Point2f> intersections;

			int countIntersect = 0;
			float cx = 0, cy = 0;

			//calculate all intersections
			for (vector<vector<Point2f> >::iterator it = v.begin(); it != v.end(); ++it) {
				for (vector<vector<Point2f> >::iterator it2 = v.begin() + skip; it2 != v.end(); ++it2) {
					Point2f intersect;
					if (*it == *it2) {
						continue;
					}
					if (lineIntersect((*it)[0], (*it)[1], (*it2)[0], (*it2)[1], intersect)) {
						cx += intersect.x;
						cy += intersect.y;
						countIntersect++;
						//intersections.push_back(intersect);
					}
				}

				skip++;
			}

			if (countIntersect > 0) {
				return Point2f(cx / countIntersect, cy / countIntersect);
			} else {
				return Point2f();
			}

		}

		static Point2f circleCentre(vector<Point2f> points) {
			int pcount = points.size();

			float cx = 0;
			float cy = 0;
			int interCount = 0;

			for (int i = 0; i < pcount; i++) {
				Point2f b = points[(i + 1) % pcount];
				Point2f a = points[i];
				Vector2f r = b - a;
				Point2f p0 = a + r * 0.5;
				Point2f p1 = p0 + perp(r);

				for (int j = 0; j < pcount; j++) {
					if (i == j) {
						continue;
					}

					Point2f b2 = points[(j + 1) % pcount];
					Point2f a2 = points[j];
					Vector2f r2 = b2 - a2;
					Point2f q0 = a2 + r2 * 0.5;
					Point2f q1 = q0 + perp(r2);

					Vector2f intersect;

					if (lineIntersect(p0, p1, q0, q1, intersect)) {
						cx += intersect.x;
						cy += intersect.y;

						interCount++;
					}

				}
			}

			if (interCount > 0) {
				return Point2f(cx / interCount, cy / interCount);
			} else {
				return Point2f();
			}

		}

		static bool lineIntersect(Point2f p0, Point2f p1, Point2f q0, Point2f q1, Point2f& intersect) {
			Vector2f u = p1 - p0;
			Vector2f v = q1 - q0;

			float det = (u.x * v.y - u.y * v.x);

			if (det == 0) {
				return false;
			}

			float s = ((p0.y - q0.y) * v.x + (q0.x - p0.x) * v.y) / det;
			float t = ((q0.x - p0.x) * u.y + (p0.y - q0.y) * u.x) / det;

			/*
			 if (s <= 0 || (s >= 1 && !ray) || t <= 0 || t >= 1) {
			 return 0;
			 }*/

			intersect = p0 + s * u;

			return true;
		}
};

struct VelocityPoint {
		Vector2f velocity;
		Point2f point;
};

void makeVectors(const vector<Point2f> previous, const vector<Point2f> now, const vector<uchar> status, vector<VelocityPoint> &out, vector<
		vector<Point2f> > &pairs) {
	int pos = 0;

	for (int i = 0; i < previous.size(); i++) {
		if (status.at(i) != 1) {
			continue;
		}

		Point2f p0 = previous.at(i);
		Point2f p1 = now.at(pos++);

		VelocityPoint v;
		v.point = p0;
		v.velocity = p1 - p0;

		out.push_back(v);

		vector<Point2f> pair;
		pair.push_back(p0);
		pair.push_back(p1);

		pairs.push_back(pair);
	}
}

class VectorClassifier {
	public:
		static void classify(const vector<VelocityPoint> raw, vector<VelocityPoint> &scene, vector<VelocityPoint> &object) {

		}
};

float operator*(const Vector2f& left, const Vector2f& right) {
	return (left.x * right.x + left.y + right.y);
}

class ImageConverter {
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;

	public:
		ImageConverter() :
				it_(nh_) {
			image_sub_ = it_.subscribe("input", 1, &ImageConverter::imageCb, this);
			cv::namedWindow(OPENCV_WINDOW);

			alpha = 255;

			//createTrackbar("alpha", OPENCV_WINDOW, &alpha, 255);

			lastAvg = 100;
			frameCount = 0;
			angles[0] = 0;
			angles[1] = 0;
			angles[2] = 0;

			anglePos = 0;

			cout << "Created!" << endl;
		}

		~ImageConverter() {
			cv::destroyWindow(OPENCV_WINDOW);
		}

		void createPoints(vector<Point2f> &points, unsigned int countX, unsigned int countY, unsigned int width, unsigned int height) {
			points.clear();

			float xstep = width / ((float) countX);
			float ystep = height / ((float) countY);

			for (int i = 0; i < countX; ++i) {
				for (int j = 0; j < countY; ++j) {
					points.push_back(Point2f(i * xstep, j * ystep));
				}
			}
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg) {
			cout << "\033[33mReceived image #" << ++frameCount << endl;
			cv_bridge::CvImagePtr cv_ptr;

			try {
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			} catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

			Mat image = cv_ptr->image;
			Mat temp;
			Mat temp2;

			//vector<Point2d> corners;
			vector<Point2f> nextPoints;
			vector<uchar> status;
			vector<float> err;

			bilateralFilter(image, temp, 5, 100, 100);

			cvtColor(temp, temp, CV_BGR2GRAY);

			for (int i = 0; i < temp.cols; i++) {
				for (int j = 0; j < temp.rows; j++) {
					temp.at<uchar>(j, i) = saturate_cast<uchar>((alpha / 255.0) * (temp.at<uchar>(j, i)));
				}
			}

			Laplacian(temp, temp, CV_8U, 3);

			/*temp.copyTo(image);

			 cvtColor(image, image, CV_GRAY2BGR);*/

			resize(image, image, Size(image.cols * 2, image.rows * 2));

			prev.copyTo(temp2);
			temp.copyTo(prev);

			if (frameCount != 1) {

				cout << "\033[33mCalculating optical flow... ";

				calcOpticalFlowPyrLK(temp2, temp, points, nextPoints, status, err);

				cout << "\033[32mDone.\033[39;49m" << endl;

				if (points.size() < 30) {
					circle(image, Point2f(10, 10), 10, Scalar(0, 0, 255), -1);
				} else {
					circle(image, Point2f(10, 10), 10, Scalar(0, 255, 0), -1);
				}

				int validPoints = 0;
				int missedPoints = 0;
				int pTotal = 0;
				float error = 0;

				vector<vector<Point2f> > pointPairs;
				vector<VelocityPoint> vPoints;

				/*for (vector<Point2f>::iterator it = points.begin(); it != points.end(); ++it) {
					circle(image, *it * 2, 1, Scalar(255, 0, 0), -1);
				}

				vector<Point2f>::iterator it = nextPoints.begin();
				for (vector<uchar>::iterator it2 = status.begin(); it2 != status.end(); ++it2) {
					if ((*it2) != 1) {
						missedPoints++;
						continue;
					}

					Point2f p = points[pTotal++];

					vector<Point2f> pair;
					pair.push_back(p);
					pair.push_back(*it);

					pointPairs.push_back(pair);

					error += err[validPoints];

					circle(image, *it * 2, 1, Scalar(0, 0, 255), -1);

					line(image, *it * 2, p * 2, Scalar(0, 0, 255), 1);

					++it;
					validPoints++;
				}

				cout << "\033[32mDone. " << points.size() << " points tested, \033[1m" << validPoints << " valid, \033[31m"
						<< missedPoints << " not found. " << (lastAvg = error / validPoints) << " avg error." << "\033[0m"
						<< endl;*/

				makeVectors(points, nextPoints, status, vPoints, pointPairs);

				Point2f centre = Point2f(temp.cols / 2, temp.rows / 2);
				Point2f horizon = MathHelper::converge(pointPairs);

				Vector2f move = horizon - centre;

				circle(image, (centre - MathHelper::norm(move) * log(MathHelper::size(move)) * 10) * 2, 10, Scalar(0, 255, 0), -1);

				circle(image, centre * 2, 3, Scalar(0, 255, 0), -1);

			}

			points.clear();
			goodFeaturesToTrack(temp, points, 300, 0.1, 1);

			// Update GUI Window
			cv::imshow(OPENCV_WINDOW, image);
			cv::waitKey(3);
		}

	private:
		Mat prev;
		unsigned int frameCount;
		vector<Point2f> points;
		float lastAvg;

		float angles[3];
		int anglePos;

		int alpha;
};

int main(int argc, char **argv) {
	cout << "\033[32mStarting!" << endl;
	ros::init(argc, argv, "optical_flow");
	ImageConverter im;
	ros::spin();

	return 0;
}

