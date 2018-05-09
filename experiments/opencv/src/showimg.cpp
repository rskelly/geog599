#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
	
	cv::String path(argv[1]);

	cv::Mat img = imread(path, cv::IMREAD_COLOR);

	if(img.empty()) {
		std::cerr << "Couldn't read.\n";
		return -1;
	}

	cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);
	cv::imshow("Display", img);

	cv::waitKey(0);
	return 0;
}