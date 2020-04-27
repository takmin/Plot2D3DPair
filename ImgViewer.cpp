#include "ImgViewer.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "util.h"


ImgViewer::ImgViewer(const std::vector<Eigen::Vector3i>& color_map)
{
	_color_map = color_map;
	_radius = 2.0;
	_max_size = cv::Size(2048, 1536);
}


ImgViewer::~ImgViewer()
{
}

void ImgViewer::printHelp()
{
	printf("H: print help\n");
	printf("Q: quit\n");
	printf("S: change max window size\n");
	printf("R: change plot circle radius\n");
	printf("D: delete the last point\n");
	printf("-: zoom out\n");
	printf("shift + click: zoom in\n");
}

void ImgViewer::on_mouse(int event, int x, int y, int flag, void * param)
{
	ImgViewer* viewer = (ImgViewer*)param;

	// If left mouse button is released
	if (event == cv::EVENT_LBUTTONUP)
	{
		viewer->MouseButtonUp(x,y, flag & cv::EVENT_FLAG_SHIFTKEY);
	}

}


void ImgViewer::MouseButtonUp(int x, int y, bool shift_key) {
	if(shift_key)
		zoomInImage(cv::Point(x, y));
	else
		_points.push_back(srcPoint(cv::Point(x,y)));
}


cv::Point2f ImgViewer::srcPoint(const cv::Point& plot_pt) const
{
	float x = (float)(plot_pt.x * _crop_area.width) / _image.cols + _crop_area.x;
	float y = (float)(plot_pt.y * _crop_area.height) / _image.rows + _crop_area.y;
	return cv::Point2f(x, y);
}

cv::Point ImgViewer::plotPoint(const cv::Point2f& src_pt) const
{
	float x = src_pt.x - _crop_area.x;
	float y = src_pt.y - _crop_area.y;
	return cv::Point(x * _image.cols / _crop_area.width, y * _image.rows/ _crop_area.height);
}

void ImgViewer::drawImage()
{
	cv::Mat img = _image.clone();
	for (int i = 0; i < _points.size(); i++) {
		if (!isInsideRect(_points[i], _crop_area))
			continue;
		int color_id = i % _color_map.size();
		cv::Scalar color(_color_map[color_id](2),
			_color_map[color_id](1),
			_color_map[color_id](0));
		cv::circle(img, plotPoint(_points[i]), _radius, color, -1);
	}
	cv::imshow("image", img);
}


cv::Mat ImgViewer::cropImage(const cv::Mat& img)
{
	_crop_area = cv::Rect(0, 0, img.cols, img.rows);
	if (img.cols <= _max_size.width && img.rows <= _max_size.height) {
		return img;
	}

	float w_scale = (float)_max_size.width / img.cols;
	float h_scale = (float)_max_size.height / img.rows;
	float scale = w_scale < h_scale ? w_scale : h_scale;
	cv::Mat dst_img;
	cv::Size dst_size((float)img.cols * scale, (float)img.rows * scale);
	cv::resize(img, dst_img, dst_size);
	return dst_img;
}


void ImgViewer::zoomInImage(const cv::Point& center)
{
	cv::Size img_size = _image.size();
	cv::Rect area;
	area.width = _crop_area.width / 2;
	area.height = _crop_area.height / 2;
	cv::Point2f src_pt = srcPoint(center);
	area.x = src_pt.x - area.width / 2;
	area.y = src_pt.y - area.height / 2;
	if (area.x < 0)
		area.x = 0;
	if (area.y < 0)
		area.y = 0;
	_crop_area = area;
	cv::Mat dst_img;
	cv::resize(_src_image(area), dst_img, img_size);
	_image = dst_img;
}



bool ImgViewer::run(const std::string& img_file)
{
	cv::namedWindow("image");
	cv::setMouseCallback("image", ImgViewer::on_mouse, this);
	cv::Mat img = cv::imread(img_file);
	if (img.empty())
		return false;
	_src_image = img;
	_image = cropImage(_src_image);

	bool exitf = false;
	while (!exitf) {
		drawImage();
		int key = cv::waitKey(1);
		if (key == 'q' || key == 'Q') {
			exitf = true;
		}
		else if (key == 'h' || key == 'H') {
			printHelp();
		}
		else if (key == 'r' || key == 'R') {
			float r = AskQuestionGetDouble("radius: ");
			if (r > 0)
				_radius = r;
		}
		else if (key == '-') {
			_image = cropImage(_src_image);
		}
		else if (key == 's' || key == 'S') {
			float w = AskQuestionGetInt("max window width: ");
			if (w > 0)
				_max_size.width = w;
			float h = AskQuestionGetInt("max window height: ");
			if (h > 0)
				_max_size.height = h;
			_image = cropImage(_src_image);
		}
		else if (key == 'd' || key == 'D') {
			if (!_points.empty())
				_points.pop_back();
		}
	}

	cv::destroyWindow("image");
	return true;
}

