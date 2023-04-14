#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <vector>
#include <Windows.h>
#include <iostream>
#include <filesystem>
using namespace std;
using namespace cv;
namespace fs = std::filesystem;

Mat refImage;
Mat rawDepth;
bool btnDown;
bool rightBtnDown;
Point3f camPos = Point3f(0, 0, -255);
Point3f targetPoint = Point3f(0, 0, 255);
int focal = 500;
Point2f camOri = Point2f(0, 0);


struct depthUnit {
	int depth;
	Point2i pos;
	int b;
	int g;
	int r;
};

vector<depthUnit> pointsCloud;

//Point2i MatcoorToXYcoor(int x, int y) {
//	Point2i result;
//	result.x=
//	return 
//}

bool compareByDepth(const depthUnit& a, const depthUnit& b)
{
	return a.depth > b.depth;
}



Mat computeNewView(int baseLineFactor, int X, int Y) {
	Mat newView = Mat(rawDepth.rows, rawDepth.cols, CV_8UC3, Scalar(0, 0, 0));
	int Xbaseline = X - rawDepth.cols / 2;
	int Ybaseline = Y - rawDepth.rows / 2;

	for (int i = 0; i < pointsCloud.size(); i++) {
		int depth = pointsCloud[i].depth;
		int deltaX = baseLineFactor * Xbaseline / (256 - depth);
		int deltaY = baseLineFactor * Ybaseline / (256 - depth);
		int targetX = pointsCloud[i].pos.x - deltaX;
		int targetY = pointsCloud[i].pos.y - deltaY;
		if (targetX < newView.cols && targetX>0 && targetY > 0 && targetY < newView.rows) {
			newView.at<Vec3b>(targetY, targetX)[0] = refImage.at<Vec3b>(pointsCloud[i].pos.y, pointsCloud[i].pos.x)[0];
			newView.at<Vec3b>(targetY, targetX)[1] = refImage.at<Vec3b>(pointsCloud[i].pos.y, pointsCloud[i].pos.x)[1];
			newView.at<Vec3b>(targetY, targetX)[2] = refImage.at<Vec3b>(pointsCloud[i].pos.y, pointsCloud[i].pos.x)[2];
		}
	}
	return newView;
}



int itemp = 0;
Mat computeNewView2(Point3f camPos, float focal) {
	Mat newView = Mat(rawDepth.rows, rawDepth.cols, CV_8UC3, Scalar(0, 0, 0));

	for (int i = 0; i < pointsCloud.size(); i++) {
		if (camPos.z < pointsCloud[i].depth) {
			Point2f desPos;
			desPos.x = (newView.cols/2)+(focal * (pointsCloud[i].pos.x - camPos.x) / (pointsCloud[i].depth - camPos.z));
			desPos.y = (newView.rows/2)-(focal * (pointsCloud[i].pos.y - camPos.y) / (pointsCloud[i].depth - camPos.z));
			if (desPos.x < newView.cols && desPos.x > 0 && desPos.y < newView.rows && desPos.y > 0) {
				newView.at<Vec3b>(desPos.y, desPos.x)[0] = refImage.at<Vec3b>(newView.rows / 2 - pointsCloud[i].pos.y, pointsCloud[i].pos.x + newView.cols / 2)[0];
				newView.at<Vec3b>(desPos.y, desPos.x)[1] = refImage.at<Vec3b>(newView.rows / 2 - pointsCloud[i].pos.y, pointsCloud[i].pos.x + newView.cols / 2)[1];
				newView.at<Vec3b>(desPos.y, desPos.x)[2] = refImage.at<Vec3b>(newView.rows / 2 - pointsCloud[i].pos.y, pointsCloud[i].pos.x + newView.cols / 2)[2];
			}
		}
	}
	/*std::ostringstream oss;
	oss << "frames/" << itemp << ".png";
	std::string dir = oss.str();
	imwrite(dir, newView);
	itemp++;*/
	return newView;
}

Mat computeNewView3(Point3f camPos, float focal, Point2f camOri) {
	Mat newView = Mat(rawDepth.rows, rawDepth.cols, CV_8UC3, Scalar(0, 0, 0));

	for (int i = 0; i < pointsCloud.size(); i++) {
		Point2f pointXZcameraDvalue = Point2f(pointsCloud[i].pos.x - camPos.x, pointsCloud[i].depth - camPos.z);
		float normXZ = sqrt(pow(pointXZcameraDvalue.x, 2) + pow(pointXZcameraDvalue.y, 2));
		float maxCitaXZ = acos(focal / normXZ);

		float absoluteCitaXZ = atan2(pointXZcameraDvalue.y, pointXZcameraDvalue.x);
		/*float leftLimitXZ = maxCitaXZ + (3.1415926 / 2 - absoluteCitaXZ);
		float rightLimitXZ = (3.1415926 / 2 - absoluteCitaXZ) - maxCitaXZ;*/


		Point2f pointYZcameraDvalue = Point2f(pointsCloud[i].pos.y - camPos.y, pointsCloud[i].depth - camPos.z);
		float normYZ = sqrt(pow(pointYZcameraDvalue.x, 2) + pow(pointYZcameraDvalue.y, 2));
		float maxCitaYZ = acos(focal / normYZ);

		float absoluteCitaYZ = atan2(pointYZcameraDvalue.y, pointYZcameraDvalue.x);
		/*float leftLimitYZ = maxCitaYZ + (3.1415926 / 2 - absoluteCitaYZ);
		float rightLimitYZ = (3.1415926 / 2 - absoluteCitaYZ) - maxCitaYZ;*/

		//if (camOri.x < leftLimitXZ && camOri.x > rightLimitXZ && camOri.y < leftLimitYZ && camOri.y > rightLimitYZ) {
			Point2f result;
			result.x = focal / tan(absoluteCitaXZ + camOri.x);
			result.y = focal / tan(absoluteCitaYZ + camOri.y);

			Point2f desPos;
			desPos.x = (newView.cols / 2) + result.x;
			desPos.y = (newView.rows / 2) - result.y;

			if (desPos.x < newView.cols && desPos.x > 0 && desPos.y < newView.rows && desPos.y > 0) {
				newView.at<Vec3b>(desPos.y, desPos.x)[0] = pointsCloud[i].b;
				newView.at<Vec3b>(desPos.y, desPos.x)[1] = pointsCloud[i].g;
				newView.at<Vec3b>(desPos.y, desPos.x)[2] = pointsCloud[i].r;
			}
		//}

	}
	return newView;
}

Mat fillPieceToESLF(Mat ESLF, Mat piece, int xOfUnit, int yOfUnit, int unitSize) {
	for (int i = 0; i < refImage.cols; i++) {
		for (int j = 0; j < refImage.rows; j++) {
			ESLF.at<Vec3b>(j * unitSize + yOfUnit, i * unitSize + xOfUnit)[0] = piece.at<Vec3b>(j, i)[0];
			ESLF.at<Vec3b>(j * unitSize + yOfUnit, i * unitSize + xOfUnit)[1] = piece.at<Vec3b>(j, i)[1];
			ESLF.at<Vec3b>(j * unitSize + yOfUnit, i * unitSize + xOfUnit)[2] = piece.at<Vec3b>(j, i)[2];
		}
	}

	return ESLF;
}

Mat generateESLF(int startPoint[2], int unitSize) {
	Mat ESLF = Mat(refImage.rows * unitSize, refImage.cols * unitSize, CV_8UC3, Scalar(0, 0, 0));
	for (int i = 0; i < unitSize; i++) {
		for (int j = 0; j < unitSize; j++) {
			fillPieceToESLF(ESLF, computeNewView(150, startPoint[0] + i, startPoint[1] + j), i, j, unitSize);
		}
	}

	//FOUR CORNER ONLY
	/*fillPieceToESLF(ESLF, computeNewView(50, startPoint[0] + 0, startPoint[1] + 0), 0, 0, unitSize);
	fillPieceToESLF(ESLF, computeNewView(50, startPoint[0] + 0, startPoint[1] + 7), 0, 7, unitSize);
	fillPieceToESLF(ESLF, computeNewView(50, startPoint[0] + 7, startPoint[1] + 0), 7, 0, unitSize);
	fillPieceToESLF(ESLF, computeNewView(50, startPoint[0] + 7, startPoint[1] + 7), 7, 7, unitSize);*/

	return ESLF;
}

//vector<Mat> restoreFromESLF(Mat ESLF, int unitSize) {
//
//}

vector<Mat> generateSynthesicViewPath(float density, int radius) {
	vector<Mat> viewList;
	for(float rad = 0;rad<6.283;rad = rad+ (1 / density)){
		int X = rawDepth.cols/2+cos(rad)* radius;
		int Y = rawDepth.rows/2+sin(rad)* radius;
		viewList.push_back(computeNewView(10, X, Y));
	}
	return viewList;
}

vector<Point3f> generateSynthesicViewPath2(Point3f startingPos, Point3f endingPos, float frames) {
	Point3f dis = endingPos-startingPos;
	Point3f changes = dis/frames;

	Point3f currentPos = startingPos;
	vector<Point3f> OUTPUT;

	for (int i = 0; i < frames; i++) {
		currentPos = currentPos + changes;
		OUTPUT.push_back(currentPos);
	}
	return OUTPUT;
}


void mouseCallBack(int event, int x, int y, int flags, void* param) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		btnDown = true;
	}
	else if (event == cv::EVENT_LBUTTONUP) {
		btnDown = false;
	}
	else if (event == cv::EVENT_MOUSEMOVE) {
		if (btnDown) {
			Point3f targetCamDvalue = targetPoint - camPos;
			float tanX = atan2(targetCamDvalue.x, targetCamDvalue.z);
			float tanY = atan2(targetCamDvalue.y, targetCamDvalue.z);
			camOri.x = tanX;
			camOri.y = tanY;
			camPos.x = (float)(x - rawDepth.cols / 2);
			camPos.y = (float)(rawDepth.rows / 2 - y);
			imshow("Synthesic View", computeNewView3(camPos, focal, camOri));
		}
	}
	else if (event == cv::EVENT_RBUTTONDOWN) {
		rightBtnDown = true;
	}
	else if (event == cv::EVENT_RBUTTONUP) {
		rightBtnDown = false;
	}
	else if (event == cv::EVENT_MOUSEWHEEL) {
		if (getMouseWheelDelta(flags) > 0) {
			if (rightBtnDown) {
				focal = focal + 10;
				camPos.z = camPos.z - 10;
				imshow("Synthesic View", computeNewView3(camPos, focal, camOri));
			}
			else {
				camPos.z = camPos.z + 10;
				imshow("Synthesic View", computeNewView3(camPos, focal, camOri));
			}
		}
		else {
			if (rightBtnDown) {
				focal = focal - 10;
				camPos.z = camPos.z + 10;
				if (focal < 1) {
					focal = 1;
				}
				imshow("Synthesic View", computeNewView3(camPos, focal, camOri));
			}
			else {
				camPos.z = camPos.z - 10;
				imshow("Synthesic View", computeNewView3(camPos, focal, camOri));
			}
		}
	}

}

int main() {
	srand((int)time(0));

	String inputName = "Printer";

	rawDepth = imread(inputName + "/depth.png", IMREAD_GRAYSCALE);
	refImage = imread(inputName + "/refImage.png");

	//Build PointsCloud with Depth+Ref PNGs
	for (int width = 0; width < rawDepth.cols; width++) {
		for (int height = 0; height < rawDepth.rows; height++)
		{
			pointsCloud.push_back({255 - rawDepth.at<uchar>(height, width), Point2i(width - rawDepth.cols / 2,rawDepth.rows / 2 - height), refImage.at<Vec3b>(height, width)[0],refImage.at<Vec3b>(height, width)[1],refImage.at<Vec3b>(height, width)[2] });
		}
	}
	

	//Build PointsCloud with MPI
	
	std::string path = "./MPI";
	for (const auto& entry : fs::directory_iterator(path)) {
		String pathStr = entry.path().string();
		int MPIdepth = stoi(pathStr.substr(6, pathStr.find(".", 6) - 6));
		
		Mat MPI = imread(pathStr, IMREAD_UNCHANGED);
		for (int width = 0; width < MPI.cols; width++) {
			for (int height = 0; height < MPI.rows; height++) {
				if (MPI.at<Vec4b>(height, width)[3]!=0) {
					pointsCloud.push_back({MPIdepth, Point2i(width - MPI.cols / 2,MPI.rows / 2 - height), MPI.at<Vec4b>(height, width)[0],MPI.at<Vec4b>(height, width)[1],MPI.at<Vec4b>(height, width)[2] });
				}
			}
		}
	}
	
	//printf("pointsCloudSize: %d", pointsCloud.size());

	

	//GENERATING RANDOM DEPTH//
	/*rawDepth = Mat(600, 800, CV_8UC3, Scalar(0, 0, 0));

	for (int i = 0; i < 30; i++) {
		int randomX = rand() % 700;
		int randomY= rand() % 500;
		int randomDepth = rand() % 255;

		for (int j = 0; j < 20; j++) {
			for (int k = 0; k < 20; k++) {
				rawDepth.at<Vec3b>(randomY + j, randomX + k)[0] = randomDepth;
				rawDepth.at<Vec3b>(randomY + j, randomX + k)[1] = randomDepth;
				rawDepth.at<Vec3b>(randomY + j, randomX + k)[2] = randomDepth;
			}
		}
	}*/
	//GENERATING RANDOM DEPTH//

	


	sort(pointsCloud.begin(), pointsCloud.end(), compareByDepth);

	//REAL-TIME VIEWING
	/*
	namedWindow("Synthesic View");
	imshow("Synthesic View", refImage);
	imshow("depthView", rawDepth);
	setMouseCallback("Synthesic View", mouseCallBack);
	*/
	//REAL-TIME VIEWING

	//GENERATING ESLF
	/*int startPoint[2] = {refImage.cols / 2,refImage.rows / 2 };
	Mat ESLF = generateESLF(startPoint, 8);
	imshow("ESLF", ESLF);
	imwrite("./ESLF.png", ESLF);*/
	//GENERATING ESLF

	//OUTDATED
	//GENERATE SYNTHETIC PATH
	/*vector<Mat> listView = generateSynthesicViewPath(10,200);
	for (int i = 0; i < listView.size(); i++) {
		std::ostringstream oss;
		oss << "frames/" << i << ".png";
		std::string dir = oss.str();
		imwrite(dir, listView[i]);
	}*/
	//GENERATE SYNTHETIC PATH

	//GENERATE SYNTHETIC PATH 2
	vector<Point3f> listView = generateSynthesicViewPath2(Point3f(0,0,-500),Point3f(-50, 0, -500), 100);
	for (int i = 0; i < listView.size(); i++) {
		camPos = listView[i];
		Point3f targetCamDvalue = targetPoint - camPos;
		float tanX = atan2(targetCamDvalue.x, targetCamDvalue.z);
		float tanY = atan2(targetCamDvalue.y, targetCamDvalue.z);
		camOri.x = tanX;
		camOri.y = tanY;
		Mat outputImg = computeNewView3(camPos, focal, camOri);

		std::ostringstream oss;
		oss << "frames/" << i << ".png";
		std::string dir = oss.str();
		
		imwrite(dir, outputImg);
	}
	//GENERATE SYNTHETIC PATH 2
	
	waitKey(0);
}