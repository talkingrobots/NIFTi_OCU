// Benoit 2011-10-13

#ifndef EU_NIFTI_OCU_VIEW_VIRTUAL_PTZ_VIEW_CONTROLLER_H
#define EU_NIFTI_OCU_VIEW_VIRTUAL_PTZ_VIEW_CONTROLLER_H

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <omnicamera_msgs/VirtualCameraConfig.h> // For virtual PTZ commands

#include "ViewControllers/CameraViewController.h"

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace view
            {

                /**
                 * This controller controls the virtual cam from CTU 
                 */
                class VirtualPTZViewController : public CameraViewController
                {
                public:
                    VirtualPTZViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer);
                    virtual ~VirtualPTZViewController();
                    
                    void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                    std::string toString() const;

                    double getFieldOfViewHorizontal() const;
                    double getFieldOfViewVertical() const;
                    
                    void lookAt(const Ogre::Vector3& point);
                    void resetView();
                    
                protected:

                    ros::NodeHandle rosNodeHandle;
                    ros::Publisher publisher;

                    void updateOrientation(int currentX, int currentY);

                    // Last position of the cursor, used to calculate the displacement with the current position of the cursor
                    int lastX, lastY;

                    omnicamera_msgs::VirtualCameraConfig virtualPTZmsg;
                    
                    static const double PAN_OFFSET; // This is due to the position of the camera wrt to the robot body
                    
                    static const double MAX_UP;
                    static const double MAX_DOWN;
                    static const double MAX_LEFT;
                    static const double MAX_RIGHT;
                    static const double MAX_FOV;
                    static const double MIN_FOV;
                    static const double DEFAULT_PAN;
                    static const double DEFAULT_TILT;
                    static const double DEFAULT_FOV;
                    static const int VIEWPORT_WIDTH;

                    // This influences how much the camera will move for each pixel of mouse move
                    // THIS NEEDS TO BE CALIBRATED (AND ADJUSTABLE DEPENDING ON FOCAL LENGTH)
                    static const double PAN_SPEED;
                    static const double TILT_SPEED;
                    
                    static const double ZOOM_SPEED;

                    static const char* ROS_PUBLISHING_TOPIC;

                };

            }
        }
    }
}

#endif // EU_NIFTI_OCU_VIEW_VIRTUAL_PTZ_VIEW_CONTROLLER_H
