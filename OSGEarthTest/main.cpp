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
//����osg��osgEarth��ͷ�ļ��������ռ�
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
//��ѯ�̵߳�һ���¼��ص����ڳ������¼����´���ʱ���ã���ϸ�ο�osg����osgGA::GUIEventHandler 
struct QueryElevationHandler : public osgGA::GUIEventHandler
{
	//���캯��
	QueryElevationHandler()
		: _mouseDown(false),
		_terrain(s_mapNode->getTerrain()),
		_query(s_mapNode->getMap())
	{
		_map = s_mapNode->getMap();

		//��ʼ������ѯLOD����
		_query.setMaxTilesToCache(10);

		_path.push_back(s_mapNode->getTerrainEngine());
	}
	//���»ص�����������ݿ��Բο����࣬�������Ĳ�������Ļ����xy����osgViewer
	void update(float x, float y, osgViewer::View* view)
	{
		bool yes = false;

		// look under the mouse:
		//������ȥ�Ե�������ײ��⣬�����������ȥ��⣬�õ����㣬���ǵ�ǰ���xyz
		osg::Vec3d world;
		osgUtil::LineSegmentIntersector::Intersections hits;
		//�ж��󽻽���Ƿ�Ϊ��
		if (view->computeIntersections(x, y, hits))
		{
			//�õ���������ϵ��������꣬����osg��xyz����
			world = hits.begin()->getWorldIntersectPoint();

			// convert to map coords:
			//����ת��Ϊ����ĵ������꣬ת���������ճ�����
			GeoPoint mapPoint;
			mapPoint.fromWorld(_terrain->getSRS(), world);

			// do an elevation query:
			double query_resolution = 0; // 1/10th of a degree
			double out_hamsl = 0.0;
			double out_resolution = 0.0;

			//�������������ѯ��ǰ��λ�õĸ̣߳���Ҫ���÷ֱ��ʣ����ǲ�ѯ����
			bool ok = _query.getElevation(
				mapPoint,
				out_hamsl,
				query_resolution,
				&out_resolution);

			//�����ѯ�ɹ�
			if (ok)
			{
				// convert to geodetic to get the HAE:
				mapPoint.z() = out_hamsl;
				GeoPoint mapPointGeodetic(s_mapNode->getMapSRS()->getGeodeticSRS(), mapPoint);

				//��γ������ĸ�ʽ�����ߣ�Ҳ�����Լ����ַ���ȥƴ��xyz����
				static LatLongFormatter s_f;
				
				//������ʾ��xyzֵ��label�Ǵ���Ŀؼ�
				s_posLabel->setText(Stringify()
					<< std::fixed << std::setprecision(2)
					<< s_f.format(mapPointGeodetic)
					);

				//����������ֱ��ʣ���������Ϣ��
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

		//�����ѯ�����̵߳Ļ�
		if (!yes)
		{
			s_posLabel->setText("-");
			s_mslLabel->setText("-");
			s_haeLabel->setText("-");
			s_resLabel->setText("-");
		}
	}

	//����һ�����¼��Ķ���������һ���Ƕ�Ӧ�Ĳ���
	bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
	{
		//�ж�������ƶ�����¼��Ž��и��µ�ǰ��������ʾ
		if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE &&
			aa.asView()->getFrameStamp()->getFrameNumber() % 10 == 0)
		{
			osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
			update(ea.getX(), ea.getY(), view);
		}

		return false;
	}

	//Map����
	const Map*       _map;
	//���ζ���
	const Terrain*   _terrain;
	bool             _mouseDown;
	//��ѯ�߳�ʹ�õĶ���
	ElevationQuery   _query;
	osg::NodePath    _path;
};

//main������
int main(int argc, char** argv)
{
	//���������������һ������������ĸ���Ϊ���������ַ�����������earth�ļ���·��
	osg::ArgumentParser arguments(&argc,argv);
	//osg�ĳ���
	osgViewer::Viewer viewer(arguments);
	//����MapNode��arguments������earth�ļ���·��������������
	s_mapNode = MapNode::load(arguments);
	//���·������ȷ����earth�ļ�����û�й����MapNode
	if (!s_mapNode)
	{
		OE_WARN << "Unable to load earth file." << std::endl;
		return -1;
	}

	//����һ����ڵ�
	osg::Group* root = new osg::Group();
	//����ڵ�����Ϊ�����ڵ�
	viewer.setSceneData(root);

	// install the programmable manipulator.
	//����earth������
	viewer.setCameraManipulator(new osgEarth::Util::EarthManipulator());

	// The MapNode will render the Map object in the scene graph.
	//��MapNode��ӵ���ڵ���ȥ

	root->addChild(s_mapNode);

	//����������һ���ؼ���grid����˼���ø���ȥ���������С�ؼ�
	// Make the readout:
	Grid* grid = new Grid();
	//���ü���Label���ֿؼ���ʾ�ڳ����еĵ���
	grid->setControl(0, 0, new LabelControl("Coords (Lat, Long):"));
	grid->setControl(0, 1, new LabelControl("Vertical Datum:"));
	grid->setControl(0, 2, new LabelControl("Height (MSL):"));
	grid->setControl(0, 3, new LabelControl("Height (HAE):"));
	grid->setControl(0, 4, new LabelControl("Isect  (HAE):"));
	grid->setControl(0, 5, new LabelControl("Resolution:"));
	//���ü���Label���ֿؼ���ʾ�ڳ����еĵ���
	s_posLabel = grid->setControl(1, 0, new LabelControl(""));
	s_vdaLabel = grid->setControl(1, 1, new LabelControl(""));
	s_mslLabel = grid->setControl(1, 2, new LabelControl(""));
	s_haeLabel = grid->setControl(1, 3, new LabelControl(""));
	s_mapLabel = grid->setControl(1, 4, new LabelControl(""));
	s_resLabel = grid->setControl(1, 5, new LabelControl(""));

	//�õ��ռ�ο�����������Ϣ������ʾ��Ӧ�����label
	const SpatialReference* mapSRS = s_mapNode->getMapSRS();
	s_vdaLabel->setText(mapSRS->getVerticalDatum() ?
		mapSRS->getVerticalDatum()->getName() :
		Stringify() << "geodetic (" << mapSRS->getEllipsoid()->getName() << ")");

	//�ؼ���������
	ControlCanvas* canvas = new ControlCanvas();
	//��Ҫ��ʾ�Ŀؼ����뵽root��ڵ���ȥ
	root->addChild(canvas);
	canvas->addControl(grid);

	//��Ӹո��Զ���Ĳ�ѯ�̵߳��¼��ص�
	// An event handler that will respond to mouse clicks:
	viewer.addEventHandler(new QueryElevationHandler());
	//���״̬��ʾ�����ڸı���¼��ص�
	// add some stock OSG handlers:
	viewer.addEventHandler(new osgViewer::StatsHandler());
	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

	//run
	return viewer.run();
}