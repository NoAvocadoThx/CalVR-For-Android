//
// Created by menghe on 7/12/2018.
//

#ifndef MYGLES_OSGCONTROLLER_H
#define MYGLES_OSGCONTROLLER_H
#include <unordered_map>
#include "osg_utils.h"

#include "osg_cameraRenderer.h"
#include "osg_planeRenderer.h"
#include "osg_pointcloudRenderer.h"
#include "osg_objectRenderer.h"
#include "arcoreController.h"
#include <osgGA/KeySwitchMatrixManipulator>
namespace osg_controller{
    class osgController {
    private:
        AAssetManager *const _asset_manager;


        int _plane_num = 0;

        bool _this_is_the_first_plane = true;
        float _color_correction[4] = {1.f, 1.f, 1.f, 1.f};
        std::unordered_map<ArPlane*,  glm::vec3> _plane_color_map;

        osgViewer::Viewer * _viewer;
        osg::ref_ptr<osg::Group>  _root;
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> _manipulator;

        arcoreController * _ar_controller;
        osg_cameraRenderer * _camera_renderer;
        osg_planeRenderer * _plane_renderer;
        osg_pointcloudRenderer * _pointcloud_renderer;
        osg_objectRenderer *_object_renderer;
    public:
        osgController(AAssetManager * manager);

        ~osgController();

        void onCreate();

        void onPause();

        void onResume(void * env, void* context, void* activity);

        void onDrawFrame(bool btn_status_normal);
        void onViewChanged(int rot, int width, int height);
        void onTouched(float x, float y);
        bool hasDetectedPlane(){ return  _plane_num > 0;}
        void debug_loadScene(const char* filename);
    };
}



#endif //MYGLES_OSGCONTROLLER_H