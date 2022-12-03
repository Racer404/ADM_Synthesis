#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <vector>
#include <Windows.h>

using namespace std;
using namespace cv;

Mat refImage;
Mat rawDepth;
bool btnDown;
bool rightBtnDown;
Point3f camPos = Point3f(0, 0, 0);
int focal = 500;


struct depthUnit {
	int depth;
	Point2i pos;
};

vector<depthUnit> depthMap;

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

	for (int i = 0; i < depthMap.size(); i++) {
		int depth = depthMap[i].depth;
		int deltaX = baseLineFactor * Xbaseline / (256 - depth);
		int deltaY = baseLineFactor * Ybaseline / (256 - depth);
		int targetX = depthMap[i].pos.x - deltaX;
		int targetY = depthMap[i].pos.y - deltaY;
		if (targetX < newView.cols && targetX>0 && targetY > 0 && targetY < newView.rows) {
			newView.at<Vec3b>(targetY, targetX)[0] = refImage.at<Vec3b>(depthMap[i].pos.y, depthMap[i].pos.x)[0];
			newView.at<Vec3b>(targetY, targetX)[1] = refImage.at<Vec3b>(depthMap[i].pos.y, depthMap[i].pos.x)[1];
			newView.at<Vec3b>(targetY, targetX)[2] = refImage.at<Vec3b>(depthMap[i].pos.y, depthMap[i].pos.x)[2];
		}
	}
	return newView;
}

int itemp = 0;
Mat computeNewView2(Point3f camPos, float focal) {
	Mat newView = Mat(rawDepth.rows, rawDepth.cols, CV_8UC3, Scalar(0, 0, 0));

	for (int i = 0; i < depthMap.size(); i++) {
		if (camPos.z < depthMap[i].depth) {
			Point2f desPos;
			desPos.x = (newView.cols/2)+(focal * (depthMap[i].pos.x - camPos.x) / (depthMap[i].depth - camPos.z));
			desPos.y = (newView.rows/2)-(focal * (depthMap[i].pos.y - camPos.y) / (depthMap[i].depth - camPos.z));
			if (desPos.x < newView.cols && desPos.x > 0 && desPos.y < newView.rows && desPos.y > 0) {
				newView.at<Vec3b>(desPos.y, desPos.x)[0] = refImage.at<Vec3b>(newView.rows / 2 - depthMap[i].pos.y, depthMap[i].pos.x + newView.cols / 2)[0];
				newView.at<Vec3b>(desPos.y, desPos.x)[1] = refImage.at<Vec3b>(newView.rows / 2 - depthMap[i].pos.y, depthMap[i].pos.x + newView.cols / 2)[1];
				newView.at<Vec3b>(desPos.y, desPos.x)[2] = refImage.at<Vec3b>(newView.rows / 2 - depthMap[i].pos.y, depthMap[i].pos.x + newView.cols / 2)[2];
			}
		}
	}
	std::ostringstream oss;
	oss << "frames/" << itemp << ".png";
	std::string dir = oss.str();
	imwrite(dir, newView);
	itemp++;
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

void mouseCallBack(int event, int x, int y, int flags, void* param) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		btnDown = true;
	}
	else if (event == cv::EVENT_LBUTTONUP) {
		btnDown = false;
	}
	else if (event == cv::EVENT_MOUSEMOVE) {
		if (btnDown) {
			camPos.x = x - rawDepth.cols / 2;
			camPos.y = rawDepth.rows / 2 - y;
			imshow("Synthesic View", computeNewView2(camPos, focal));
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
				imshow("Synthesic View", computeNewView2(camPos, focal));
			}
			else {
				camPos.z = camPos.z + 10;
				imshow("Synthesic View", computeNewView2(camPos, focal));
			}
		}
		else {
			if (rightBtnDown) {
				focal = focal - 10;
				if (focal < 1) {
					focal = 1;
				}
				imshow("Synthesic View", computeNewView2(camPos, focal));
			}
			else {
				camPos.z = camPos.z - 10;
				imshow("Synthesic View", computeNewView2(camPos, focal));
			}
		}
	}

}

int main() {
	srand((int)time(0));

	String inputName = "Sculpture_GT";

	rawDepth = imread(inputName + "/depth.png");
	refImage = imread(inputName + "/refImage.png");

	//converting to grayscale
	for (int width = 0; width < rawDepth.cols; width++) {
		for (int height = 0; height < rawDepth.rows; height++)
		{
			int leftGrayScale = 255-(rawDepth.at<Vec3b>(height, width)[2]);
			rawDepth.at<Vec3b>(height, width)[0] = leftGrayScale;
			rawDepth.at<Vec3b>(height, width)[1] = leftGrayScale;
			rawDepth.at<Vec3b>(height, width)[2] = leftGrayScale;
		}
	}

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


	for (int i = 0; i < rawDepth.rows; i++) {
		for (int j = 0; j < rawDepth.cols; j++) {
			//depthMap.push_back({rawDepth.at<Vec3b>(i,j)[0], Point2i(j,i)});
			depthMap.push_back({rawDepth.at<Vec3b>(i,j)[0], Point2i(j-rawDepth.cols/2,rawDepth.rows/2-i)});
		}
	}

	sort(depthMap.begin(), depthMap.end(), compareByDepth);

	//REAL-TIME VIEWING
	namedWindow("Synthesic View");
	imshow("Synthesic View", refImage);
	imshow("depthView", rawDepth);
	setMouseCallback("Synthesic View", mouseCallBack);
	//REAL-TIME VIEWING

	//GENERATING ESLF
	/*int startPoint[2] = {refImage.cols / 2,refImage.rows / 2 };
	Mat ESLF = generateESLF(startPoint, 8);
	imshow("ESLF", ESLF);
	imwrite("./ESLF.png", ESLF);*/
	//GENERATING ESLF


	//GENERATE SYNTHETIC PATH
	/*vector<Mat> listView = generateSynthesicViewPath(10,200);
	for (int i = 0; i < listView.size(); i++) {
		std::ostringstream oss;
		oss << "frames/" << i << ".png";
		std::string dir = oss.str();
		imwrite(dir, listView[i]);
	}*/
	//GENERATE SYNTHETIC PATH


	
	waitKey(0);
}