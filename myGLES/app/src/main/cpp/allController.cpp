#include <cvrUtil/AndroidHelper.h>
#include <osgGA/TrackballManipulator>
#include <osg/ShapeDrawable>
#include <cvrMenu/MenuManager.h>
#include <cvrKernel/SceneManager.h>
#include <cvrConfig/ConfigManager.h>

#include "allController.h"
#include "utils.h"
//#include "osg_utils.h"
#include <cvrMenu/SubMenu.h>
#include <cvrKernel/PluginManager.h>
#include <cvrInput/TrackingManager.h>
#include <cvrKernel/ComController.h>
#include <cvrUtil/AndroidPreloadPlugins.h>
//#include <MenuBasics.h>
#include <osg/ComputeBoundsVisitor>
#include <plugins/PhysxBall/PhysxBall.h>
#include <osg/PolygonMode>
#include <MenuBasics.h>

using namespace controller;
using namespace osg;
using namespace cvr;
REGISTER(PhysxBall);
REGISTER(MenuBasics);

allController::allController(AAssetManager *assetManager)
        :_asset_manager(assetManager){
    _viewer = new cvr::CVRViewer();
    _viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    _root = new Group;
    _menu =  cvr::MenuManager::instance();
    _scene = cvr::SceneManager::instance();
    _config = cvr::ConfigManager::instance();
    _interactionManager = cvr::InteractionManager::instance();
    _tracking = TrackingManager::instance();
    _navigation = cvr::Navigation::instance();
    _communication = cvr::ComController::instance();
    _plugins = cvr::PluginManager::instance();

    _ar_controller = new arcoreController;
    _bgDrawable = new bgDrawable();
    _sceneGroup = new Group;
    _strokeDrawable = new strokeDrawable;
    _fpsMonitor = new perfMonitor();

    initialize_camera();
}

allController::~allController(){
    // should be friend class to call deconstructor
//    if(_file)
//    {
//        delete _file;
//    }
    /*if(_menu)
    {
        delete _menu;
    }
//    if(_threadedLoader)
//    {
//        delete _threadedLoader;
//    }
    if(_communication)
    {
        delete _communication;
    }
    if(_scene)
    {
        delete _scene;
    }
//    if(_screens)
//    {
//        delete _screens;
//    }
    if(_viewer)
    {
        delete _viewer;
    }
    if(_navigation)
    {
        delete _navigation;
    }
    if(_interactionManager)
    {
        delete _interactionManager;
    }
    if(_tracking)
    {
        delete _tracking;
    }
    if(_communication)
    {
        delete _communication;
    }
    if(_config)
    {
        delete _config;
    }*/
}


void allController::initialize_camera() {
    osg::ref_ptr<osg::Camera> mainCam = _viewer->getCamera();
    mainCam->setClearColor(osg::Vec4f(0.81, 0.77, 0.75,1.0));
    osg::Vec3d eye = osg::Vec3d(0,-10,0);
    osg::Vec3d center = osg::Vec3d(0,0,.0);
    osg::Vec3d up = osg::Vec3d(0,0,1);
    mainCam->setViewMatrixAsLookAt(eye,center,up); // usual up vector
    mainCam->setRenderOrder(osg::Camera::NESTED_RENDER);
//    mainCam->setCullingMode( osg::CullSettings::NO_CULLING );
}

void allController::setupDefaultEnvironment(const char *root_path) {
    std::string homeDir = std::string(root_path) + "/";
    setenv("CALVR_HOST_NAME", "calvrHost");
    setenv("CALVR_HOME", homeDir);
    setenv("CALVR_ICON_DIR", homeDir+"icons/");
    setenv("CALVR_CONFIG_DIR", homeDir+"config/");
    setenv("CALVR_RESOURCE_DIR", homeDir+"resources/");
    setenv("CALVR_CONFIG_FILE", homeDir+"config/config.xml");
}

void allController::onCreate(const char * calvr_path){
//    calvr = new cvr::CalVR();
//    if(!calvr->init(calvr_path)){
//        delete calvr;
//        return;
//    }
    setupDefaultEnvironment(calvr_path);
//    ref_ptr<Geode> sphereNode = createDebugOSGSphere(osg::Vec3(.0f,0.5f,.0f));
//    _sceneGroup->addChild(sphereNode.get());

    _sceneGroup->addChild( _strokeDrawable->createDrawableNode(_asset_manager,&_glStateStack));

    //Initialization should follow a specific order

    if(!_config->init())
        LOGE("==========CONFIG INITIALIZATION FAIL========");
    if(!_communication->init())
        LOGE("==========INTERACTION MANAGER FAIL=========");
    if(!_tracking->init())
        LOGE("==========TRACKING MANAGER FAIL=========");
    if(!_interactionManager->init())
        LOGE("==========INTERACTION MANAGER FAIL=========");
    if(!_navigation->init())
        LOGE("=========NAVIGATION FAIL===========");
    if(!_scene->init())
        LOGE("==========SCENE INITIALIZATION FAIL=========");
    _scene->setViewerScene(_viewer);
    _viewer->setReleaseContextAtEndOfFrameHint(false);

    if(!_menu->init())
        LOGE("==========MENU INITIALIZATION FAIL=========");

    if(!_plugins->init(_asset_manager))
        LOGE("==========PLUG IN  FAIL=========");

//    _bgDrawable->createDrawableNode(_asset_manager, &_glStateStack);
    _root->addChild(_bgDrawable->createDrawableNode(_asset_manager, &_glStateStack));

    //This will make sure camera always in the background
    _sceneGroup->getOrCreateStateSet()->setRenderBinDetails(2,"RenderBin");
    _sceneGroup->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::ON);
    _root->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
    _sceneGroup->addChild(_scene->getSceneRoot());
//    _scene->addChild(createDebugOSGSphere(osg::Vec3(.0f,0.5f,.0f)));

    _root->addChild(_sceneGroup);
    _viewer->setSceneData(_root.get());
}

void allController::onDrawFrame(){
    _ar_controller->onDrawFrame(_bgDrawable->GetTextureId());

    osg::Matrixd* mat = new osg::Matrixd(glm::value_ptr(_ar_controller->view_mat));
    Vec3d eye, center, up;
    mat->getLookAt(eye, center, up);
    _viewer->getCamera()->setViewMatrixAsLookAt(Vec3d(eye.x(), -eye.z(), eye.y()),
                                                Vec3d(center.x(), -center.z(), center.y()),
                                                Vec3d(up.x(), -up.z(), up.y()));

    float * transUV = _ar_controller->updateBackgroundRender();
    if(nullptr != transUV)
        _bgDrawable->updateOnFrame(transUV);

    _viewer->frameStart();
    _viewer->advance(USE_REFERENCE_TIME);
//    _viewer->eventTraversal();

    _tracking->update();
    _scene->update();
    _menu->update();
    _interactionManager->update();

    _navigation->update();
    _scene->postEventUpdate();
    _plugins->preFrame();
//    _viewer->frame();
    _viewer->updateTraversal();
    _viewer->renderingTraversals();
    if(_communication->getIsSyncError())
        LOGE("Sync error");
    _plugins->postFrame();
    DrawRay();
}

void allController::onViewChanged(int rot, int width, int height){
    _viewer->setUpViewerAsEmbeddedInWindow(0,0,width,height);
    _ar_controller->onViewChanged(rot, width, height);
    _touchX = width/2; _touchY = height/2;
    _screenWidth = width;   _screenHeight = height;
}
void allController::onPause() {
    _ar_controller->onPause();
}

void allController::onResume(void *env, void *context, void *activity) {
    _ar_controller->onResume(env, context, activity);
}
void allController::onResourceLoaded(const char *path) {
    FILE* fp = fopen(path, "r");
    if(nullptr == fp){
        LOGE("FAILED TO LOAD RESOURCE FILE");
        return;
    }
}
void toEulerAngle(const osg::Quat& q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}

void allController::commonMouseEvent(cvr::MouseInteractionEvent * mie,
                                     int pointer_num, float x, float y){
    mie->setButton(pointer_num - 1);
    mie->setHand(0);
    mie->setX(x);
    mie->setY(y);

    float * camera_pos = _ar_controller->getCameraPose();
    osg::Quat camRot = osg::Quat(camera_pos[0],-camera_pos[2],camera_pos[1],camera_pos[3]);
    Vec3f hand_pose = Vec3f(camera_pos[4], -camera_pos[6], camera_pos[5]);
    osg::Matrix m, n;
    m.makeRotate(camRot);

    n.makeTranslate(hand_pose);
    mie->setTransform(m * n);

    double roll, pitch, yaw;
    toEulerAngle(osg::Quat(camera_pos[0],camera_pos[1],camera_pos[2],camera_pos[3]), roll, pitch, yaw);

    _tracking->setCameraRotation(m, roll, -yaw, pitch);
    _tracking->setTouchEventMatrix(m*n);
    _interactionManager->addEvent(mie);
}

void allController::onSingleTouchDown(int pointer_num, float x, float y) {
    MouseInteractionEvent * mie = new MouseInteractionEvent();
    mie->setInteraction(BUTTON_DOWN);
    commonMouseEvent(mie, pointer_num, x, y);
}

void allController::onSingleTouchUp(int pointer_num, float x, float y){
    MouseInteractionEvent * mie = new MouseInteractionEvent();
    mie->setInteraction(BUTTON_UP);
    commonMouseEvent(mie, pointer_num, x, y);
}

void allController::onDoubleTouch(int pointer_num, float x, float y){
    MouseInteractionEvent * mie = new MouseInteractionEvent();
    mie->setInteraction(BUTTON_DOUBLE_CLICK);
    commonMouseEvent(mie, pointer_num, x, y);
}

void allController::onTouchMove(int pointer_num, float x, float y) {
    _touchX = x; _touchY = y;
    MouseInteractionEvent * mie = new MouseInteractionEvent();
    mie->setInteraction(BUTTON_DRAG);
    commonMouseEvent(mie, pointer_num, x, y);
}

void allController::DrawRay(){
    float offset[2];
    offset[0] = 2*(_touchX - _screenWidth/2) / _screenWidth;
    offset[1] = 2*(_screenHeight/2 - _touchY) / _screenHeight;

    Vec3f isPoint;
    if(TrackingManager::instance()->getIsPoint(isPoint)){
        _strokeDrawable->getGLNode()->setNodeMask(0xFFFFFF);
        _ar_controller->renderStroke(_strokeDrawable,
                                     TrackingManager::instance()->getHandMat(0).getTrans(),
                                     isPoint,
                                     offset);
    }
    else
        _strokeDrawable->getGLNode()->setNodeMask(0x0);
    //shoot the ray to check the interaction with menu
    MouseInteractionEvent * mie = new MouseInteractionEvent();
    mie->setInteraction(BUTTON_DRAG);
    commonMouseEvent(mie, 1, _touchX, _touchY);
}
ref_ptr<osg::Geode> allController::createDebugOSGSphere(osg::Vec3 pos) {
//    osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable;
//    shape->setShape(new osg::Sphere(pos, 0.05f));
//    shape->setColor(osg::Vec4f(1.0f,.0f,.0f,1.0f));
//    osg::ref_ptr<osg::Geode> node = new osg::Geode;
//    Program * program = osg_utils::createShaderProgram("shaders/lightingOSG.vert","shaders/lightingOSG.frag",_asset_manager);
//
//    osg::StateSet * stateSet = shape->getOrCreateStateSet();
//    stateSet->setAttributeAndModes(program);
//
//    stateSet->addUniform( new osg::Uniform("lightDiffuse",
//                                           osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f)) );
//    stateSet->addUniform( new osg::Uniform("lightSpecular",
//                                           osg::Vec4(1.0f, 1.0f, 0.4f, 1.0f)) );
//    stateSet->addUniform( new osg::Uniform("shininess", 64.0f) );
//
//    stateSet->addUniform( new osg::Uniform("lightPosition", osg::Vec3(0,0,1)));
//
//    node->addDrawable(shape.get());
//
//    return node;
}