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
#include <osgEarthDrivers/kml/KML>
#include <osgEarthSymbology/TextSymbol>
#include <osgEarthAnnotation/PlaceNode>
#include <iomanip>
#include <string>



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

void unicodeToUTF8(const std::wstring &src, std::string& result)
{
	int n = WideCharToMultiByte(CP_UTF8, 0, src.c_str(), -1, 0, 0, 0, 0);
	result.resize(n);
	::WideCharToMultiByte(CP_UTF8, 0, src.c_str(), -1, (char*)result.c_str(), result.length(), 0, 0);
}

void gb2312ToUnicode(const std::string& src, std::wstring& result)
{
	int n = MultiByteToWideChar(CP_ACP, 0, src.c_str(), -1, NULL, 0);
	result.resize(n);
	::MultiByteToWideChar(CP_ACP, 0, src.c_str(), -1, (LPWSTR)result.c_str(), result.length());
}

void gb2312ToUtf8(const std::string& src, std::string& result)
{
	std::wstring strWideChar;
	gb2312ToUnicode(src, strWideChar);
	unicodeToUTF8(strWideChar, result);
}

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
	

/*
	osgEarth::Style style;
	osgEarth::Symbology::TextSymbol *textStyle = style.getOrCreateSymbol<osgEarth::Symbology::TextSymbol>();
	textStyle->font() = "E:\\osg\\222\\simhei.ttc";
	textStyle->size() = 30.0;
	textStyle->encoding() = osgEarth::Symbology::TextSymbol::ENCODING_UTF8;
*/


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
/*
	std::string kmlFile("D:/SXEarth_Downloads/doc.kml");
	osg::ref_ptr<osgDB::Options> options = new osgDB::Options();
	options->setPluginData("osgEarth::MapNode", s_mapNode);
	osg::Node* kml = osgDB::readNodeFile(kmlFile, options.get());
	if (kml)
		root->addChild(kml);*/
	//osgEarth::URI url("F:\\doc.kml");
	//osgEarth::URI url("E:\\osg\\osgearth\\src\\osgEarthDriversDisabled\\engine_seamless\\doc\\euler.kml");


/*

	osgEarth::URI url("E:\\osg\\222\\doc.kml");
	MapNode* mapNode = static_cast<MapNode*>(s_mapNode);
	osgEarth::Drivers::KMLOptions optionskml;
	/ *osg::ref_ptr<osgEarth::Symbology::TextSymbol> textStyle = optionskml.defaultTextSymbol();
	textStyle->font() = "C:/Windows/Fonts/simhei.ttf";
	textStyle->size() = 30.0;
	textStyle->encoding() = osgEarth::Symbology::TextSymbol::ENCODING_UTF8;* /
	static osg::Node* kml = osgEarth::Drivers::KML::load(url, mapNode, optionskml);


	if (kml)
	{
		root->addChild(kml);
	}

*/





	MapNode* mapNode = static_cast<MapNode*>(s_mapNode);
	Style pin;


	pin.getOrCreate<osgEarth::Symbology::TextSymbol>()->font() = "simhei.ttf";//ָ����������·��
	pin.getOrCreate<osgEarth::Symbology::TextSymbol>()->encoding() = osgEarth::Symbology::TextSymbol::ENCODING_UTF8;
	pin.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
	pin.getOrCreate<TextSymbol>()->fill()->color() = Color::Red;


	std::string _strName;

	_strName = "���";

	std::wstring _strWideName1;
	std::string _strWideName;
	osg::Group* labelGroup = new osg::Group();
	//gb2312ToUnicode(_strName, _strWideName1);
	gb2312ToUtf8(_strName, _strWideName);//��ʱ��_strWideName���ǿ��ֽ�������ʾ����ȷ��
	const SpatialReference* geoSRS = mapNode->getMapSRS()->getGeographicSRS();
	labelGroup->addChild(new osgEarth::Annotation::PlaceNode(mapNode, GeoPoint(geoSRS, 117.5, 39.38), _strName, pin));
	root->addChild(labelGroup);




	//��Ӹո��Զ���Ĳ�ѯ�̵߳��¼��ص�
	// An event handler that will respond to mouse clicks:
	viewer.addEventHandler(new QueryElevationHandler());
	//viewer.addEventHandler(new osgViewer::LODScaleHandler());
	//���״̬��ʾ�����ڸı���¼��ص�
	// add some stock OSG handlers:
	viewer.addEventHandler(new osgViewer::StatsHandler());
	viewer.addEventHandler(new osgViewer::WindowSizeHandler());
	viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

	//run
	return viewer.run();
}