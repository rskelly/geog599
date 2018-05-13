#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <list>

int main(int argc, char** argv) {

	cv::String path(argv[1]);
	std::cout << "c\n";
	cv::VideoCapture vid(path);
	std::list<cv::Mat> frames;
	cv::Mat frame;

	float f  = atof(argv[2]);
	float cx = atof(argv[3]);
	float cy = atof(argv[4]);
	cv::Matx33d K = cv::Matx33d( f, 0, cx,
	                       0, f, cy,
	                       0, 0,  1);
	  bool is_projective = true;
	  std::vector<cv::Mat> Rs_est, ts_est, points3d_estimated;
	  cv::sfm::reconstruct(images_paths, Rs_est, ts_est, K, points3d_estimated, is_projective);

	while(true) {
		vid >> frame;
		frames.push_front(frame);
		if(frames.size() > 5) {


			frames.pop_back();
		}

	}

	cv::waitKey(0);
	return 0;
}
