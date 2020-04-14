#ifndef __IMG_VIEWER__
#define __IMG_VIEWER__

#include <opencv2/core.hpp>
#include <Eigen/Core>

class ImgViewer
{
public:
	ImgViewer(const std::vector<Eigen::Vector3i>& color_map);
	~ImgViewer();

	bool run(const std::string& img_file);

	void get2DPoints(std::vector<cv::Point2f>& points) {
		points = _points;
	}

	static void on_mouse(int event, int x, int y, int flag, void * param);

	void MouseButtonUp(int x, int y, bool shift_key);


private:
	cv::Mat _src_image;
	cv::Mat _image;
	float _radius;
	cv::Size _max_size;
	cv::Rect _crop_area;
	std::vector<Eigen::Vector3i> _color_map;

	std::vector<cv::Point2f> _points;

	void drawImage();
	cv::Mat cropImage(const cv::Mat& img);
	void zoomInImage(const cv::Point& center);
	cv::Point plotPoint(const cv::Point2f& src_pt) const;
	cv::Point2f srcPoint(const cv::Point& plot_pt) const;
	bool isInsideRect(const cv::Point2f& pt, const cv::Rect& area) const {
		return (pt.x >= area.x && pt.x < area.x + area.width &&
			pt.y >= area.y && pt.y < area.y + area.height);
	}
	static void printHelp();
};

#endif