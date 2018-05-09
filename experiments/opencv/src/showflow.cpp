#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/optflow.hpp>

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
	
	cv::String path(argv[1]);
	std::cout << "c\n";
	cv::VideoCapture vid(path);
	cv::Mat frame;

	cv::namedWindow("Display1", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Display2", cv::WINDOW_AUTOSIZE);
	std::cout << "d\n";
	vid >> frame;
	if(frame.empty())
		return -1;

	cv::Mat last(frame.rows, frame.cols, frame.type());
	cv::Mat flow(frame.rows, frame.cols, CV_32FC2);
	cv::Mat flow0(frame.rows, frame.cols, CV_32F);
	while(true) {
		frame.copyTo(last);
		vid >> frame;
		if(frame.empty())
			break;
		cv::optflow::calcOpticalFlowSparseToDense(last, frame, flow);
		std::cout << flow.channels() << "\n";
		std::vector<cv::Mat> channels(2);
		cv::split(flow, channels);
		cv::imshow("Display1", frame);
		cv::imshow("Display2", channels[1]);
		cv::waitKey(20);
	}

	cv::waitKey(0);
	return 0;
}