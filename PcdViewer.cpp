#include "PcdViewer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <boost/filesystem/path.hpp>
#include "util.h"


PcdViewer::PcdViewer(const std::vector<Eigen::Vector3i>& color_map)
{
	//_src_clouds = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	_clouds = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	_radius = 10.0;
	_color_map = color_map;
}


PcdViewer::~PcdViewer()
{
}


void PcdViewer::printHelp()
{
	using pcl::console::print_info;
	print_info("    h, H:    Print this help\n");
	//print_info("Shift + left click: pick a point\n");
	print_info("    >/<:    enlarge/ensmall plotted sphere\n");
	print_info("    d, D:    delete the last seletected point\n");
	print_info("    q, Q:    Quit and Save\n");
	print_info("\n");
}

// Simple callbacks.
void PcdViewer::keyboard_callback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
	if (!event.keyUp())
		return;

	unsigned char KeyCode = event.getKeyCode();
	if (KeyCode == 'h' || KeyCode == 'H') {
		printHelp();
	}
	else if (KeyCode == 'd' || KeyCode == 'D') {
		if (!_pcd_point_id.empty()) {
			int id = _pcd_point_id.back();
			_pcd_point_id.pop_back();
			//_clouds->at(id) = _src_clouds->at(id);
			bool ret = unplot(id);
			//_update = true;
		}
	}
	else if (KeyCode == '>') {
		_radius *= 2;
		updateAllShpare();
	}
	else if (KeyCode == '<') {
		_radius /= 2;
		updateAllShpare();	
	}
}


void PcdViewer::mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void* cookie)
{
	if (mouse_event.getButton() != pcl::visualization::MouseEvent::LeftButton ||
		mouse_event.getType() != pcl::visualization::MouseEvent::MouseButtonRelease ||
		mouse_event.getKeyboardModifiers() != pcl::visualization::KeyboardEvent::Alt)
		return;
}


void PcdViewer::picking_callback(const pcl::visualization::PointPickingEvent& pp_event, void* viewer_void)
{
	int id = pp_event.getPointIndex();
	if (id < 0)
		return;
	//int cur_color_id = _pcd_point_id.size() % _color_map.size();
	//_clouds->at(id).r = _color_map[cur_color_id](0);
	//_clouds->at(id).g = _color_map[cur_color_id](1);
	//_clouds->at(id).b = _color_map[cur_color_id](2);
	//PointT pt = _clouds->at(id);
	//pcl::console::print_info("%d:[%f,%f,%f]\n", id, pt.x, pt.y, pt.z);
	bool ret = plot(id);
	_pcd_point_id.push_back(id);
	//_update = true;
}


bool PcdViewer::plot(int id) 
{
	int cur_color_id = _pcd_point_id.size() % _color_map.size();
	float r = (float)_color_map[cur_color_id](0) / 255;
	float g = (float)_color_map[cur_color_id](1) / 255;
	float b = (float)_color_map[cur_color_id](2) / 255;
	PointT pt = _clouds->at(id);
	pcl::console::print_info("%d:[%f,%f,%f]\n", id, pt.x, pt.y, pt.z);
	std::string sphere_id = "sphere_" + Int2String(id);
	bool ret  = _viewer->addSphere(pt, _radius, r, g, b, sphere_id);
	return ret;
}


bool PcdViewer::unplot(int id)
{
	std::string sphere_id = "sphere_" + Int2String(id);
	bool ret = _viewer->removeShape(sphere_id);
	return ret;
}


void PcdViewer::updateAllShpare()
{
	for (int i = 0; i < _pcd_point_id.size(); i++) {
		int cur_color_id = i % _color_map.size();
		float r = (float)_color_map[cur_color_id](0) / 255;
		float g = (float)_color_map[cur_color_id](1) / 255;
		float b = (float)_color_map[cur_color_id](2) / 255;
		int id = _pcd_point_id[i];
		PointT pt = _clouds->at(id);
		std::string sphere_id = "sphere_" + Int2String(id);
		bool ret = _viewer->updateSphere(pt, _radius, r, g, b, sphere_id);
	}
}


void PcdViewer::get3DPoints(std::vector<Eigen::Vector3f>& points)
{
	for (int i = 0; i < _pcd_point_id.size(); i++) {
		PointT pt = _clouds->at(_pcd_point_id[i]);
		points.push_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
	}
}


bool PcdViewer::run(const std::string& pcd_file)
{
	// load point cloud
	std::string ext = boost::filesystem::path(pcd_file).extension().string();
	int ret = -1;
	if (ext == ".pcd" || ext == ".PCD") {
		//ret = pcl::io::loadPCDFile(pcd_file, *_src_clouds);
		ret = pcl::io::loadPCDFile(pcd_file, *_clouds);
	}
	else if (ext == ".ply" || ext == ".PLY") {
		//ret = pcl::io::loadPLYFile(pcd_file, *_src_clouds);
		ret = pcl::io::loadPLYFile(pcd_file, *_clouds);
	}
	if (ret < 0) {
		std::cerr << "Fail to load " << pcd_file << std::endl;
		return false;
	}

	// Initialize Viewer
	std::string cloud_id = "cloud0";
	_viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("PCL Viewer"));
	//_viewer->initCameraParameters();
	_viewer->registerMouseCallback(&PcdViewer::mouse_callback, *this);
	_viewer->registerKeyboardCallback(&PcdViewer::keyboard_callback, *this);
	_viewer->registerPointPickingCallback(&PcdViewer::picking_callback, *this);

	//*_clouds = *_src_clouds;

	_viewer->addPointCloud(_clouds, cloud_id);

	// Loop
	while (!_viewer->wasStopped())
	{
		// Render and process events in the two interactors
		_viewer->spinOnce(100);
		//if (_update) {
		//	_viewer->updatePointCloud(_clouds, cloud_id);
		//	_update = false;
		//}
		std::this_thread::sleep_for(std::chrono::microseconds(100));
		//boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}
	return true;
}

