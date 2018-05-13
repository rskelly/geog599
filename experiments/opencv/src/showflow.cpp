#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/optflow.hpp>

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
	
	cv::namedWindow("Display1", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Display2", cv::WINDOW_AUTOSIZE);

	cv::String path(argv[1]);
	cv::VideoCapture vid(path);
	cv::Mat frame;

	vid >> frame;
	if(frame.empty())
		return -1;

	cv::Mat mono(frame.rows, frame.cols, CV_8U);
	cv::Mat last(mono.rows, mono.cols, mono.type());
	cv::Mat lines(mono.rows, mono.cols, frame.type());

	std::vector<cv::Mat> channels(3);
	cv::split(frame, channels);
	channels[0].convertTo(mono, CV_8U);

	cv::Mat err;
	std::vector<uchar> found;
	std::vector<cv::Point2f> next;
	std::vector<cv::Point2f> current;
	std::vector<cv::Point2f> previous;

	cv::goodFeaturesToTrack(mono, current, 256, 0.01, 30);

	while(true) {
		previous.assign(current.begin(), current.end());
		mono.copyTo(last);
		vid >> frame;
		if(frame.empty())
			break;

		std::vector<cv::Mat> channels(3);
		cv::split(frame, channels);
		channels[0].convertTo(mono, CV_8U);

		cv::goodFeaturesToTrack(mono, current, 256, 0.01, 64);
		cv::calcOpticalFlowPyrLK(last, mono, previous, next, found, err);

		frame.copyTo(lines);

		for(size_t i = 0; i < next.size(); ++i) {
			if(found[i])
				cv::line(lines, previous[i], next[i], cv::Scalar(0, 0, 255), 2);
		}

		cv::imshow("Display1", lines);
		//cv::imshow("Display2", lines);

		cv::waitKey(20);
	}

	cv::waitKey(0);
	return 0;
}
