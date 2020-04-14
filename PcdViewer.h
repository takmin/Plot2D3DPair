#ifndef __PCD_VIEWER__
#define __PCD_VIEWER__

#include <pcl/visualization/pcl_visualizer.h>
#include "typedef.h"

class PcdViewer
{
public:
	PcdViewer(const std::vector<Eigen::Vector3i>& color_map);
	~PcdViewer();

	bool run(const std::string& pcd_file);

	void get3DPoints(std::vector<Eigen::Vector3f>& points);

	bool plot(int id);

	bool unplot(int id);

	// Simple callbacks.
	void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* cookie);

	void mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void* cookie);

	void picking_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void);

	static void printHelp();

private:
	bool _update;
	PointCloudPtr _clouds;
	PointCloudPtr _src_clouds;
	float _radius;

	pcl::visualization::PCLVisualizer::Ptr _viewer;

	std::vector<int> _pcd_point_id;

	std::vector<Eigen::Vector3i> _color_map;
};

#endif