#include "ImgViewer.h"
#include "PcdViewer.h"
#include "typedef.h"
#include "util.h"
#include <Eigen/Core>
#include <thread>


void createColorMap(std::vector<Eigen::Vector3i>& color_map)
{
	int step = 32;
	for (int r = 0; r < 256; r+=step) {
		for (int g = 0; g < 256; g += step) {
			for (int b = 0; b < 256; b += step) {
				if (r == g && g == b)
					continue;
				if ((float)(r + g + b) / std::sqrt(3.0 * (r*r + g*g + b*b)) > std::sqrt(0.5))
					continue;
				color_map.push_back(Eigen::Vector3i(r, g, b));
			}
		}
	}
	std::random_shuffle(color_map.begin(), color_map.end());
}


void save_point_pairs(const std::string& filename, 
	const std::vector<cv::Point2f>& point2d, const std::vector<cv::Point3f>& point3d) 
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	fs << "point2d" << "[";
	for (int i = 0; i < point2d.size(); i++) {
		fs << point2d[i];
	}
	fs << "]";
	fs << "point3d" << "[";
	for (int i = 0; i < point3d.size(); i++) {
		fs << point3d[i];
	}
	fs << "]";
}


int main(int argc, char * argv[])
{
	//std::string calib_file = AskQuestionGetString("Calibration File: ");
	//std::string out_file = AskQuestionGetString("Output File (.txt): ");

	std::vector<Eigen::Vector3i> color_map;
	createColorMap(color_map);

	bool exitf = false;
	std::vector<cv::Point3f> pcd_points;
	std::vector<cv::Point2f> img_points;
	while (!exitf) {
		std::string pcd_file = AskQuestionGetString("Point Cloud File: ");
		PcdViewer pcd_viewer(color_map);
		std::thread th_pcd(&PcdViewer::run, &pcd_viewer, pcd_file);
		std::string img_file = AskQuestionGetString("Image File: ");
		ImgViewer img_viewer(color_map);
		std::thread th_img(&ImgViewer::run, &img_viewer, img_file);
		th_pcd.join();
		th_img.join();

		/// get plotted points coordinates ///
		std::vector<Eigen::Vector3f> pcd_pts;
		pcd_viewer.get3DPoints(pcd_pts);
		std::vector<cv::Point2f> img_pts;
		img_viewer.get2DPoints(img_pts);
		for (int i = 0; i < pcd_pts.size() && i < img_pts.size(); i++) {
			img_points.push_back(img_pts[i]);
			Eigen::Vector3f pt = pcd_pts[i];
			pcd_points.push_back(cv::Point3f(pt(0), pt(1), pt(2)));
		}
		int ans = AskQuestionGetInt("Quit? (1:Yes, 0:No): ");
		exitf = ans == 1;
	}
	std::string save_file = AskQuestionGetString("file name to save points: ");
	save_point_pairs(save_file, img_points, pcd_points);

	return 0;
}