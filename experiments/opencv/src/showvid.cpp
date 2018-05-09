#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
	
	cv::String path(argv[1]);

	cv::VideoCapture vid(path);
	cv::Mat frame;

	cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);
	
	while(true) {
		vid >> frame;
		if(frame.empty())
			break;
		cv::imshow("Display", frame);
		cv::waitKey(20);
	}

	cv::waitKey(0);
	return 0;
}