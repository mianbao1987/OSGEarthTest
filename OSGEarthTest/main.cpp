#include <windows.h>
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
	osgViewer::Viewer viewer(arguments);//����MapNode��arguments������earth�ļ���·��������������

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