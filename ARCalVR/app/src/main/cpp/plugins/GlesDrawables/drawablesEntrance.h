#ifndef DRAWABLE_ENTRANCE_H
#define DRAWABLE_ENTRANCE_H

// STD
#include <queue>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>

// CVR
#include <cvrKernel/CVRPlugin.h>
#include <cvrKernel/ScreenBase.h>
#include <cvrKernel/SceneManager.h>
#include <cvrKernel/SceneObject.h>
#include <cvrKernel/Navigation.h>
#include <cvrKernel/PluginHelper.h>
#include <cvrKernel/ComController.h>
#include <cvrMenu/MenuSystem.h>
#include <cvrMenu/SubMenu.h>
#include <cvrMenu/MenuButton.h>
#include <cvrMenu/MenuCheckbox.h>
#include <cvrMenu/MenuRangeValue.h>
#include <cvrConfig/ConfigManager.h>
#include <cvrKernel/FileHandler.h>
#include <cvrUtil/AndroidHelper.h>
// OSG
#include <osg/Group>
#include <osg/Vec3>
#include <osg/Uniform>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>

#include "pointDrawable.h"
#include "planeDrawable.h"
#include "strokeDrawable.h"

typedef struct IntersetctObj{
    osg::Uniform * uTexture;
    osg::MatrixTransform * matrixTrans;
}isectObj;
enum sceneState{
    FREE = 0,
    ROTATE,
    TRANSLATE
};
class GlesDrawables : public cvr::CVRPlugin, public cvr::MenuCallback
{
typedef osg::ref_ptr<osg::MatrixTransform> Transform;
protected:
    cvr::SubMenu *_mainMenu;
    cvr::MenuCheckbox *_pointButton, *_planeButton, *_strokeButton;
    osg::Group *_root, *_objects;
    cvr::SceneObject *rootSO, *objSO;

    bool drawPoints, drawPlanes, drawStroke;

    int _plane_num = 0, _objNum = 0;
    osg::ref_ptr<pointDrawable> _pointcloudDrawable;
    std::vector<planeDrawable*> _planeDrawables;
    osg::ref_ptr<strokeDrawable> _strokeDrawable;

    // for interactions with particular object
    std::unordered_map<osg::Node*, isectObj> _map;
    osg::Node* _selectedNode = nullptr;
    sceneState _selectState = FREE;
    osg::Vec2f _mPreviousPos;
    const bool use_sh = true;

    void initMenuButtons();
    void createObject(osg::Group *, const char*, const char*, osg::Matrixf);
    void createObject(osg::Group *, const char*, const char*, osg::Matrixf, bool opengl);
    bool tackleHitted(osgUtil::LineSegmentIntersector::Intersection result );
public:
    bool init();
    void menuCallback(cvr::MenuItem * item);
    void postFrame();
    bool processEvent(cvr::InteractionEvent * event);
};

#endif
