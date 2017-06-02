#include "libfreenect2opencv.h"
#include <iostream>
#include <fstream>
int depth_height = 424;
int depth_width = 512;
int main() {
	libfreenect2opencv::Libfreenect2OpenCV libfreeTool;
	//_sleep(1000);
//	do {
		libfreeTool.updateMat();
		cv::Mat c_2_d = libfreeTool.getRGBMapDepth();
		cv::imwrite("color.png", libfreeTool.getRGBMat());
		cv::Mat depth_img, depth_img_16U = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);
		libfreeTool.getDepthMatUndistorted().copyTo(depth_img);
		for (int y = 0; y < depth_height; y++)
			for (int x = 0; x < depth_width; x++)
			{
				int index = y * depth_width + x;
				if (isnan(depth_img.at<float>(y, x))
					|| depth_img.at<float>(y, x) < 0
					|| depth_img.at<float>(y, x) == std::numeric_limits<float>::infinity()
					|| depth_img.at<float>(y, x) == -std::numeric_limits<float>::infinity())
					depth_img.at<float>(y, x) = 0;
				depth_img_16U.at<unsigned short>(y, x) = static_cast<unsigned short>(depth_img.at<float>(y, x));
			}
		cv::imwrite("depth.png", depth_img_16U);
		cv::imwrite("color_register.png", libfreeTool.getRGB2Depth());
		std::ofstream ofs;
		ofs.open("./colordepthmap.txt");
		ofs <<c_2_d<< std::endl;
		ofs.close();
		
	//} while (cv::waitKey(1) != 27);

	//cv::Mat img = cv::imread("H:/data/aiyu/color_to_depth_map0.png", CV_LOAD_IMAGE_UNCHANGED);
	//if (img.data == NULL) {
	//	std::cout << "read image error!" << std::endl;
	//	return 0;
	//}
	//std::cout << "16sc3:" << CV_8UC3 << std::endl;
	//std::cout << "type:" << img.type() << std::endl;
	//std::cout << "width:" << img.cols << ", height:" << img.rows << std::endl;
	//for (int y = 0; y < img.rows; y++)
	//	for (int x = 0; x < img.cols; x++) {
	//		if (img.at<cv::Vec3s>(y, x) != cv::Vec3s(-1, -1, -1)) {
	//			std::cout << img.at<cv::Vec3s>(y, x)[0] << ", " << img.at<cv::Vec3s>(y, x)[1] << std::endl;
	//		}
	//	}
	return 0;
}