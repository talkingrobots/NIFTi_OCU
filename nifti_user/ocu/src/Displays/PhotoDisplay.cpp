// Benoit 2013-03

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>

#include "AnnotatedPicUtil.h"

#include "Displays/PhotoDisplay.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                // Because it's static, the image must be initialized here
                Ogre::Image PhotoDisplay::DEFAULT_IMAGE = Ogre::Image();

                PhotoDisplay::PhotoDisplay(const std::string& name, Ogre::SceneManager* sceneMgr, rviz::FrameTransformer* frameMgr, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue, Ogre::RenderWindow* renderWindow)
                : rviz::Display(name, sceneMgr, frameMgr, updateQueue, threadQueue)
                , renderWindow(renderWindow)
                , zoomLevel(0)
                , displacement_X(0)
                , displacement_Y(0)
                , renderRequested(false)
                {
                    // Loads the default image only once (because it is static)
                    if (DEFAULT_IMAGE.getWidth() == 0)
                    {
                        DEFAULT_IMAGE.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                    }

                    static uint32_t count = 0;
                    std::stringstream ss;
                    ss << "PhotoDisplay" << count++;
                    texture = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, DEFAULT_IMAGE);


                    ss << "Material";
                    material = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                    material->setDepthWriteEnabled(false);
                    material->setReceiveShadows(false);
                    material->setDepthCheckEnabled(false);

                    material->getTechnique(0)->setLightingEnabled(false);
                    Ogre::TextureUnitState* tu = material->getTechnique(0)->getPass(0)->createTextureUnitState();
                    tu->setTextureName(texture->getName());
                    tu->setTextureFiltering(Ogre::TFO_NONE);
                    tu->setAlphaOperation(Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 1.0f);

                    material->setCullingMode(Ogre::CULL_NONE);
                    material->setSceneBlending(Ogre::SBT_REPLACE);

                    screenRect = new Ogre::Rectangle2D(true);
                    screenRect->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

                    screenRect->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE); // Makes it always included in the part of the scene to be drawn
                    screenRect->setMaterial(material->getName());

                    sceneNode = sceneMgr->getRootSceneNode()->createChildSceneNode();
                    sceneNode->attachObject(screenRect);
                    sceneNode->setVisible(false);


                    setStatus("Photo", eu::nifti::ocu::STATUS_LEVEL_OK, "");
                }

                PhotoDisplay::~PhotoDisplay()
                {
                    delete screenRect;

                    sceneNode->getParentSceneNode()->removeAndDestroyChild(sceneNode->getName());

                    texture->unload();
                }

                void PhotoDisplay::zoom(double zoom)
                {
                    this->zoomLevel *= zoom;

                    checkZoomLimits();

                    adjustPhoto();
                }

                void PhotoDisplay::move(int x, int y)
                {
                    this->displacement_X -= x / zoomLevel;
                    this->displacement_Y += y / zoomLevel;

                    checkDisplacementLimits();

                    adjustPhoto();
                }

                void PhotoDisplay::forceRender()
                {
                    renderRequested = true;
                }

                void PhotoDisplay::fixedFrameChanged()
                {

                }

                void PhotoDisplay::update(float wall_dt, float ros_dt)
                {
                    if (renderRequested)
                    {
                        renderRequested = false;

                        renderWindow->update();
                    }
                }

                void PhotoDisplay::reset()
                {
                    Display::reset();

                    zoomLevel = 0;
                    displacement_X = 0;
                    displacement_Y = 0;

                    checkZoomLimits();
                    
                    adjustPhoto();
                }

                void PhotoDisplay::handleSizeEvent(wxSizeEvent& evt)
                {
                    adjustDisplayRatios();
                    adjustPhoto();
                }

                void PhotoDisplay::setPhoto(const EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    //ROS_INFO_STREAM("IN void PhotoDisplay::setPhoto(const EXIFReader_msgs::AnnotatedPicture* picture): " << picture->filename);

                    Ogre::Image* ogreImage = eu::nifti::ocu::AnnotatedPicUtil::createOgreImage(picture);
                    assert(ogreImage != NULL);

                    texture->unload();
                    texture->loadImage(*ogreImage);

                    delete ogreImage;

                    adjustDisplayRatios();
                    reset();
                    
                    //ROS_INFO_STREAM("OUT void PhotoDisplay::setPhoto(const EXIFReader_msgs::AnnotatedPicture* picture): " << picture->filename);
                }

                void PhotoDisplay::onEnable()
                {

                }

                void PhotoDisplay::onDisable()
                {

                }

                void PhotoDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
                {
                    sceneNode->setVisible(true);
                }

                void PhotoDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
                {
                    sceneNode->setVisible(false);
                }

                void PhotoDisplay::adjustDisplayRatios()
                {
                    bool resetToMinZoom = (zoomLevel == zoomMin); // With this it can leave the picture in mode 'size to fit'
                    
                    compressionRatioX = (double) texture->getWidth() / (double) renderWindow->getWidth();
                    compressionRatioY = (double) texture->getHeight() / (double) renderWindow->getHeight();
                    zoomMin = 1 / std::max(compressionRatioX, compressionRatioY);

                    if(resetToMinZoom) zoomLevel = 0;
                    checkZoomLimits();

                    //ROS_INFO_STREAM("compressionRatio: (" << compressionRatioX << ", " << compressionRatioY << ")" << ", zoomMin: " << zoomMin);
                }

                void PhotoDisplay::adjustPhoto()
                {
                    double left = ((-1.0f * compressionRatioX) - (2.0 * displacement_X / renderWindow->getWidth())) * zoomLevel;
                    double top = ((1.0f * compressionRatioY) - (2.0 * displacement_Y / renderWindow->getHeight())) * zoomLevel;
                    double right = ((1.0f * compressionRatioX) - (2.0 * displacement_X / renderWindow->getWidth())) * zoomLevel;
                    double bottom = ((-1.0f * compressionRatioY) - (2.0 * displacement_Y / renderWindow->getHeight())) * zoomLevel;

                    //ROS_INFO_STREAM("left: " << left << ", top: " << top << ", right: " << right << ", bottom: " << bottom);

                    // Moves and stretches the image w.r.t the viewport
                    screenRect->setCorners(left, top, right, bottom, false);

                    renderRequested = true;
                }

                void PhotoDisplay::checkZoomLimits()
                {
                    if (zoomLevel < zoomMin) zoomLevel = zoomMin;
                    if (zoomLevel > 1) zoomLevel = 1; // Does not zoom in more than real size

                    //ROS_INFO_STREAM("zoomLevel: " << zoomLevel);
                }

                void PhotoDisplay::checkDisplacementLimits()
                {
                    if (displacement_X < (int) -texture->getWidth() / 2)
                    {
                        displacement_X = -texture->getWidth() / 2;
                    }
                    else if (displacement_X > (int) texture->getWidth() / 2)
                    {
                        displacement_X = texture->getWidth() / 2;
                    }

                    if (displacement_Y < (int) -texture->getHeight() / 2)
                    {
                        displacement_Y = -texture->getHeight() / 2;
                    }
                    else if (displacement_Y > (int) texture->getHeight() / 2)
                    {
                        displacement_Y = texture->getHeight() / 2;
                    }

                    //ROS_INFO_STREAM("Center " << displacement_X << ", " << displacement_Y);
                }

            }
        }
    }
}
