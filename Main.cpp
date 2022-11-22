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


struct depthUnit {
	int depth;
	Vec2i pos;
};

vector<depthUnit> depthMap;


bool compareByDepth(const depthUnit& a, const depthUnit& b)
{
	return a.depth < b.depth;
}


Mat computeNewView(int baseLineFactor, int X, int Y) {
	Mat newView = Mat(rawDepth.rows, rawDepth.cols, CV_8UC3, Scalar(0, 0, 0));
	int Xbaseline = X - rawDepth.cols / 2;
	int Ybaseline = Y - rawDepth.rows / 2;

	for (int i = 0; i < depthMap.size(); i++) {
		int depth = depthMap[i].depth;
		int deltaX = baseLineFactor * Xbaseline / (256 - depth);
		int deltaY = baseLineFactor * Ybaseline / (256 - depth);
		int targetX = depthMap[i].pos[0] - deltaX;
		int targetY = depthMap[i].pos[1] - deltaY;
		if (targetX < newView.cols && targetX>0 && targetY > 0 && targetY < newView.rows) {
			newView.at<Vec3b>(targetY, targetX)[0] = refImage.at<Vec3b>(depthMap[i].pos[1], depthMap[i].pos[0])[0];
			newView.at<Vec3b>(targetY, targetX)[1] = refImage.at<Vec3b>(depthMap[i].pos[1], depthMap[i].pos[0])[1];
			newView.at<Vec3b>(targetY, targetX)[2] = refImage.at<Vec3b>(depthMap[i].pos[1], depthMap[i].pos[0])[2];
		}
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
			fillPieceToESLF(ESLF, computeNewView(50, startPoint[0] + i, startPoint[1] + j), i, j, unitSize);
		}
	}

	//FOUR CORNER ONLY
	
	/*fillPieceToESLF(ESLF, computeNewView(50, startPoint[0] + 0, startPoint[1] + 0), 0, 0, unitSize);
	fillPieceToESLF(ESLF, computeNewView(50, startPoint[0] + 0, startPoint[1] + 7), 0, 7, unitSize);
	fillPieceToESLF(ESLF, computeNewView(50, startPoint[0] + 7, startPoint[1] + 0), 7, 0, unitSize);
	fillPieceToESLF(ESLF, computeNewView(50, startPoint[0] + 7, startPoint[1] + 7), 7, 7, unitSize);*/

	return ESLF;
}

vector<Mat> restoreFromESLF(Mat ESLF, int unitSize) {

}

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
			imshow("Synthesic View", computeNewView(10, x, y));
		}
	}
}

int main() {
	srand((int)time(0));

	String inputName = "Sculpture";

	rawDepth = imread(inputName + "/depth.png");
	refImage = imread(inputName + "/refImage.png");

	//converting to grayscale
	for (int width = 0; width < rawDepth.cols; width++) {
		for (int height = 0; height < rawDepth.rows; height++)
		{
			int leftGrayScale = rawDepth.at<Vec3b>(height, width)[2] * 0.299 + rawDepth.at<Vec3b>(height, width)[1] * 0.387 + rawDepth.at<Vec3b>(height, width)[0] * 0.114;
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



	for (int i = 0; i < rawDepth.rows; i++) {
		for (int j = 0; j < rawDepth.cols; j++) {
			depthMap.push_back({rawDepth.at<Vec3b>(i,j)[0], Vec2i(j,i)});
		}
	}

	sort(depthMap.begin(), depthMap.end(), compareByDepth);

	//REAL-TIME VIEWING
	/*namedWindow("Synthesic View");
	imshow("Synthesic View", refImage);
	imshow("depthView", rawDepth);
	setMouseCallback("Synthesic View", mouseCallBack);*/
	//REAL-TIME VIEWING

	int startPoint[2] = {refImage.cols / 2,refImage.rows / 2 };
	Mat ESLF = generateESLF(startPoint, 8);
	imshow("ESLF", ESLF);
	imwrite("./ESLF.png", ESLF);


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