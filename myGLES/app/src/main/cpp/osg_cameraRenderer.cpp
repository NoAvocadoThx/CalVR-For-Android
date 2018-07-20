//
// Created by menghe on 7/12/2018.
//

#include <type_traits>
#include "osg_cameraRenderer.h"
#include "arcore_utils.h"

using namespace osg;
namespace {
    // Positions of the quad vertices in clip space (X, Y, Z).
    const GLfloat kVertices[] = {
            -1.0f, -1.0f, 0.0f, +1.0f, -1.0f, 0.0f,
            -1.0f, +1.0f, 0.0f, +1.0f, +1.0f, 0.0f
    };

    // UVs of the quad vertices (S, T)
    const GLfloat kUvs[] = {
            0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    };
}

osg::ref_ptr<osg::Geode> osg_cameraRenderer::createNode(AAssetManager *manager) {
    _bgNode = new osg::Geode();
     _bgGeo = new osg::Geometry();
    _vertices = new osg::Vec3Array();
    _uvs = new osg::Vec2Array();

    for(int i=0; i<4; i++){
        _vertices->push_back(Vec3(kVertices[3*i], kVertices[3*i+1], kVertices[3*i+2]));
        _uvs ->push_back(Vec2(kUvs[2*i], kUvs[2*i+1]));
    }
    _bgGeo->setVertexArray(_vertices.get());
    _bgGeo->setTexCoordArray(0, _uvs.get());

    _bgGeo->addPrimitiveSet(new DrawArrays(GL_TRIANGLE_STRIP, 0, 4));

    _bgNode->addDrawable(_bgGeo.get());


    _bgTexture = new osg::TextureAC();
    _bgTexture->setFilter(osg::Texture::FilterParameter::MIN_FILTER, osg::Texture::FilterMode::LINEAR);
    _bgTexture->setFilter(osg::Texture::FilterParameter::MAG_FILTER, osg::Texture::FilterMode::LINEAR);
    _bgTexture->setDataVariance(osg::Object::DYNAMIC);

    _samUniform = new osg::Uniform("uTexture", GL_TEXTURE_EXTERNAL_OES);
    _samUniform->set(0);

    osg::StateSet* stateset = new osg::StateSet;
    stateset->addUniform(_samUniform);
    stateset->setTextureAttributeAndModes(0, _bgTexture, osg::StateAttribute::ON);

    stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    stateset->setAttribute(
            osg_utils::createShaderProgram("shaders/osgTexture.vert", "shaders/osgTexture.frag", manager));

    _bgGeo->setStateSet(stateset);
//    _bgNode->setStateSet(stateset);
    return _bgNode.get();
}

void osg_cameraRenderer::Draw(arcoreController* arController, bool btn_status_normal) {
    // If display rotation changed (also includes view size change), we need to
    // re-query the uv coordinates for the on-screen portion of the camera image.
    if(arController->updateBackgroundRender(kUvs))
     {
        _uvs->clear();
        for(int i=0; i<4; i++)
            _uvs ->push_back(Vec2(arController->transformed_camera_uvs[2*i], arController->transformed_camera_uvs[2*i+1]));
        _bgGeo->setVertexArray(_vertices.get());
        _bgGeo->setTexCoordArray(0, _uvs.get());

    }

//    glDisable(GL_DEPTH_TEST);
//    glDepthMask(GL_FALSE);
//    if(!utils::LoadPngFromAssetManager(GL_TEXTURE_EXTERNAL_OES, "textures/pd.png", "JniInterfaceOSG"))
//        LOGE("Failed to load png image");
//    glActiveTexture(GL_TEXTURE0);
//    glBindTexture(GL_TEXTURE_EXTERNAL_OES, _bgTexture->textureID);



//    glDepthMask(GL_TRUE);
//    glEnable(GL_DEPTH_TEST);
    /*ArImage *ar_image;
    const AImage *ndk_image = nullptr;
    ArStatus status =
            ArFrame_acquireCameraImage(session, frame, &ar_image);
    if (status == AR_SUCCESS) {
        ArImage_getNdkImage(ar_image, &ndk_image);
        ArImage_release(ar_image);
    } else {
        LOGE("ComputerVisionApplication::OnDrawFrame acquire camera image not ready.");
    }
    if (ndk_image == nullptr)
        return;
    int32_t format = 0, width = 0, height = 0, num_plane = 0, stride = 0;
    if (arcore_utils::getNdkImageProperties(ndk_image, &format, &width, &height, &num_plane,
                              &stride)) {
        if (format == AIMAGE_FORMAT_YUV_420_888) {
            if (width > 0 || height > 0 || num_plane > 0 || stride > 0) {
                ref_ptr<Image> osgImg = new Image;
                uint8_t* input_pixels = nullptr;
                int length = 0;
                media_status_t status =
                        AImage_getPlaneData(ndk_image, 0, &input_pixels, &length);
                if (status != AMEDIA_OK)
                    return;
                osgImg->setImage(width, height, 3,
                                 GL_RGB, GL_RGB, GL_UNSIGNED_BYTE,
                                 (unsigned char*)input_pixels,
                                 osg::Image::AllocationMode::NO_DELETE, 1);
                _bgTexture->setImage(osgImg);
            }
        }
    }*/

}