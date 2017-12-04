#include <windows.h>
/*
#include <osgViewer/viewer>
#include <osg/Node>
#include <osg/geode>
#include <osg\group>
#include <osgDB/readfile>
#include <osgDB/writefile>
#include <osgUtil\optimizer>
/ *
int main()
{
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();
	osg::ref_ptr<osg::Group> root = new osg::Group();
	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("E:\\osg\\osgearth\\tests\\feature_geom.earth");
	root->addChild(node.get());
	osgUtil::Optimizer optimizer;
	optimizer.optimize(root.get());
	viewer->setSceneData(root.get());
	viewer->realize();
	viewer->run();
	return 0;
}* /

#pragma comment(lib,"osgViewerd.lib")
#pragma comment(lib,"osgDBd.lib")
#pragma comment(lib,"OpenThreadsd.lib")
#pragma comment(lib,"osgd.lib")


#include <osgViewer/Viewer>

#include <osg/Transform>
#include <osg/Billboard>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Notify>

#include <osgDB/Registry>
#include <osgDB/ReadFile>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>

#include <osgUtil/Optimizer>

#include <iostream>

class UpdateCallback : public osg::NodeCallback
{
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		//std::cout << "update callback - pre traverse" << node << std::endl;
		traverse(node, nv);
		//std::cout << "update callback - post traverse" << node << std::endl;
	}
};

class CullCallback : public osg::NodeCallback
{
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		//std::cout << "cull callback - pre traverse" << node << std::endl;
		traverse(node, nv);
		//std::cout << "cull callback - post traverse" << node << std::endl;
	}
};

class DrawableDrawCallback : public osg::Drawable::DrawCallback
{
	virtual void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const
	{
		//std::cout << "draw call back - pre drawImplementation" << drawable << std::endl;
		drawable->drawImplementation(renderInfo);
		//std::cout << "draw call back - post drawImplementation" << drawable << std::endl;
	}
};

struct DrawableUpdateCallback : public osg::Drawable::UpdateCallback
{
	virtual void update(osg::NodeVisitor*, osg::Drawable* drawable)
	{
		//std::cout << "Drawable update callback " << drawable << std::endl;
	}
};

struct DrawableCullCallback : public osg::Drawable::CullCallback
{
	/ ** do customized cull code.* /
	virtual bool cull(osg::NodeVisitor*, osg::Drawable* drawable, osg::State* / *state* /) const
	{
		//std::cout << "Drawable cull callback " << drawable << std::endl;
		return false;
	}
};

class InsertCallbacksVisitor : public osg::NodeVisitor
{

public:

	InsertCallbacksVisitor() :osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{
	}

	virtual void apply(osg::Node& node)
	{
		node.setUpdateCallback(new UpdateCallback());
		node.setCullCallback(new CullCallback());
		traverse(node);
	}

	virtual void apply(osg::Geode& geode)
	{
		geode.setUpdateCallback(new UpdateCallback());

		//note, it makes no sense to attach a cull callback to the node
		//at there are no nodes to traverse below the geode, only
		//drawables, and as such the Cull node callbacks is ignored.
		//If you wish to control the culling of drawables
		//then use a drawable cullback...

		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			geode.getDrawable(i)->setUpdateCallback(new DrawableUpdateCallback());
			geode.getDrawable(i)->setCullCallback(new DrawableCullCallback());
			geode.getDrawable(i)->setDrawCallback(new DrawableDrawCallback());
		}
	}

	virtual void apply(osg::Transform& node)
	{
		apply((osg::Node&)node);
	}
};

class MyReadFileCallback : public osgDB::Registry::ReadFileCallback
{
public:
	virtual osgDB::ReaderWriter::ReadResult readNode(const std::string& fileName, const osgDB::ReaderWriter::Options* options)
	{
		//std::cout << "before readNode" << std::endl;
		// note when calling the Registry to do the read you have to call readNodeImplementation NOT readNode, as this will
		// cause on infinite recusive loop.
		osgDB::ReaderWriter::ReadResult result = osgDB::Registry::instance()->readNodeImplementation(fileName, options);
		//std::cout << "after readNode" << std::endl;
		return result;
	}
};

class CameraUpdateCallback : public osg::NodeCallback
{
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		//std::cout << "Camera update callback - pre traverse" << node << std::endl;
		traverse(node, nv);
		//std::cout << "Camera update callback - post traverse" << node << std::endl;
	}
};

class CameraEventCallback : public osg::NodeCallback
{
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		//std::cout << "Camera event callback - pre traverse" << node << std::endl;
		traverse(node, nv);
		//std::cout << "Camera event callback - post traverse" << node << std::endl;
	}
};


struct TestDrawableUpdateCallback : public osg::Drawable::UpdateCallback
{
	TestDrawableUpdateCallback(const std::string &message) : _message(message) {}

	virtual void update(osg::NodeVisitor*, osg::Drawable* drw) {
		printf("%s\n", _message.c_str());
	}
	std::string _message;
};

struct TestNodeUpdateCallback : public osg::NodeCallback
{
	TestNodeUpdateCallback(const std::string &message) : _message(message) {}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
		printf("%s\n", _message.c_str());
	}
	std::string _message;
};

int main()
{

	// initialize the viewer.
	osgDB::Registry::instance()->setReadFileCallback(new MyReadFileCallback());
	osgViewer::Viewer viewer;	// load the nodes from the commandline arguments.
	osg::ref_ptr<osg::Node> rootnode;	rootnode = osgDB::readNodeFile("E:\\osg\\osgearth\\tests\\feature_geom.earth");	if (!rootnode)
	{
		osg::notify(osg::NOTICE) << "Please specify a file on the command line" << std::endl;

		return 1;
	}

	// run optimization over the scene graph
	osgUtil::Optimizer optimzer;
	optimzer.optimize(rootnode.get());

	// insert all the callbacks
	InsertCallbacksVisitor icv;
	rootnode->accept(icv);
	viewer.getCamera()->setUpdateCallback(new CameraUpdateCallback());
	viewer.getCamera()->setEventCallback(new CameraEventCallback());
	// set the scene to render
	viewer.setSceneData(rootnode.get());
	viewer.setCameraManipulator(new osgGA::TrackballManipulator);
	viewer.realize();
	viewer.run();

	return 0;
}
*/
//引入osg和osgEarth的头文件和命名空间
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ElevationQuery>
#include <osgEarth/StringUtils>
#include <osgEarth/Terrain>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/LatLongFormatter>
#include <iomanip>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;

static MapNode*       s_mapNode = 0L;
static LabelControl*  s_posLabel = 0L;
static LabelControl*  s_vdaLabel = 0L;
static LabelControl*  s_mslLabel = 0L;
static LabelControl*  s_haeLabel = 0L;
static LabelControl*  s_mapLabel = 0L;
static LabelControl*  s_resLabel = 0L;


// An event handler that will print out the elevation at the clicked point
//查询高程的一个事件回调，在场景有事件更新触发时调用，详细参考osg或者osgGA::GUIEventHandler 
struct QueryElevationHandler : public osgGA::GUIEventHandler
{
	//构造函数
	QueryElevationHandler()
		: _mouseDown(false),
		_terrain(s_mapNode->getTerrain()),
		_query(s_mapNode->getMap())
	{
		_map = s_mapNode->getMap();

		//初始化最大查询LOD级别
		_query.setMaxTilesToCache(10);

		_path.push_back(s_mapNode->getTerrainEngine());
	}
	//更新回调，具体的内容可以参考父类，传进来的参数是屏幕坐标xy，和osgViewer
	void update(float x, float y, osgViewer::View* view)
	{
		bool yes = false;

		// look under the mouse:
		//采用线去对地球做碰撞检测，根据鼠标点击点去检测，得到交点，就是当前点的xyz
		osg::Vec3d world;
		osgUtil::LineSegmentIntersector::Intersections hits;
		//判断求交结果是否为空
		if (view->computeIntersections(x, y, hits))
		{
			//得到世界坐标系下面的坐标，就是osg的xyz坐标
			world = hits.begin()->getWorldIntersectPoint();

			// convert to map coords:
			//将其转换为地球的地理坐标，转换方法都照抄即可
			GeoPoint mapPoint;
			mapPoint.fromWorld(_terrain->getSRS(), world);

			// do an elevation query:
			double query_resolution = 0; // 1/10th of a degree
			double out_hamsl = 0.0;
			double out_resolution = 0.0;

			//根据输入参数查询当前点位置的高程，需要设置分辨率，就是查询精度
			bool ok = _query.getElevation(
				mapPoint,
				out_hamsl,
				query_resolution,
				&out_resolution);

			//如果查询成功
			if (ok)
			{
				// convert to geodetic to get the HAE:
				mapPoint.z() = out_hamsl;
				GeoPoint mapPointGeodetic(s_mapNode->getMapSRS()->getGeodeticSRS(), mapPoint);

				//经纬度坐标的格式化工具，也可以自己用字符串去拼接xyz数字
				static LatLongFormatter s_f;
				
				//更新显示的xyz值，label是传入的控件
				s_posLabel->setText(Stringify()
					<< std::fixed << std::setprecision(2)
					<< s_f.format(mapPointGeodetic)
					);

				//还可以输出分辨率，椭球体信息等
				s_mslLabel->setText(Stringify() << out_hamsl);
				s_haeLabel->setText(Stringify() << mapPointGeodetic.z());
				s_resLabel->setText(Stringify() << out_resolution);

				yes = true;
			}

			// finally, get a normal ISECT HAE point.
			GeoPoint isectPoint;
			isectPoint.fromWorld(_terrain->getSRS()->getGeodeticSRS(), world);
			s_mapLabel->setText(Stringify() << isectPoint.alt());
		}

		//如果查询不到高程的话
		if (!yes)
		{
			s_posLabel->setText("-");
			s_mslLabel->setText("-");
			s_haeLabel->setText("-");
			s_resLabel->setText("-");
		}
	}

	//参数一个是事件的动作，另外一个是对应的操作
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		//判断如果是移动鼠标事件才进行更新当前的坐标显示
		if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE &&
			aa.asView()->getFrameStamp()->getFrameNumber() % 10 == 0)
		{
			osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
			update(ea.getX(), ea.getY(), view);
		}

		return false;
	}

	//Map对象
	const Map*       _map;
	//地形对象
	const Terrain*   _terrain;
	bool             _mouseDown;
	//查询高程使用的对象
	ElevationQuery   _query;
	osg::NodePath    _path;
};

//main函数，
int main(int argc, char** argv)
{
	//这儿两个参数，第一个是命令参数的个数为，后面是字符串数组输入earth文件的路径
	osg::ArgumentParser arguments(&argc,argv);
	//osg的场景
	osgViewer::Viewer viewer(arguments);
	//构造MapNode，arguments里面有earth文件的路径，命令行输入
	s_mapNode = MapNode::load(arguments);
	//如果路径不正确或者earth文件错误，没有构造好MapNode
	if (!s_mapNode)
	{
		OE_WARN << "Unable to load earth file." << std::endl;
		return -1;
	}

	//建立一个组节点
	osg::Group* root = new osg::Group();
	//将组节点设置为场景节点
	viewer.setSceneData(root);

	// install the programmable manipulator.
	//设置earth操作器
	viewer.setCameraManipulator(new osgEarth::Util::EarthManipulator());

	// The MapNode will render the Map object in the scene graph.
	//将MapNode添加到组节点中去

	root->addChild(s_mapNode);

	//下面是设置一个控件，grid的意思是用格网去布局里面的小控件
	// Make the readout:
	Grid* grid = new Grid();
	//设置几个Label文字控件显示在场景中的第行
	grid->setControl(0, 0, new LabelControl("Coords (Lat, Long):"));
	grid->setControl(0, 1, new LabelControl("Vertical Datum:"));
	grid->setControl(0, 2, new LabelControl("Height (MSL):"));
	grid->setControl(0, 3, new LabelControl("Height (HAE):"));
	grid->setControl(0, 4, new LabelControl("Isect  (HAE):"));
	grid->setControl(0, 5, new LabelControl("Resolution:"));
	//设置几个Label文字控件显示在场景中的第行
	s_posLabel = grid->setControl(1, 0, new LabelControl(""));
	s_vdaLabel = grid->setControl(1, 1, new LabelControl(""));
	s_mslLabel = grid->setControl(1, 2, new LabelControl(""));
	s_haeLabel = grid->setControl(1, 3, new LabelControl(""));
	s_mapLabel = grid->setControl(1, 4, new LabelControl(""));
	s_resLabel = grid->setControl(1, 5, new LabelControl(""));

	//得到空间参考，椭球面信息，并显示对应上面的label
	const SpatialReference* mapSRS = s_mapNode->getMapSRS();
	s_vdaLabel->setText(mapSRS->getVerticalDatum() ?
		mapSRS->getVerticalDatum()->getName() :
		Stringify() << "geodetic (" << mapSRS->getEllipsoid()->getName() << ")");

	//控件绘制容器
	ControlCanvas* canvas = new ControlCanvas();
	//将要显示的控件加入到root组节点中去
	root->addChild(canvas);
	canvas->addControl(grid);

	//添加刚刚自定义的查询高程的事件回调
	// An event handler that will respond to mouse clicks:
	viewer.addEventHandler(new QueryElevationHandler());
	//添加状态显示，窗口改变等事件回调
	// add some stock OSG handlers:
	viewer.addEventHandler(new osgViewer::StatsHandler());
	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

	//run
	return viewer.run();
}