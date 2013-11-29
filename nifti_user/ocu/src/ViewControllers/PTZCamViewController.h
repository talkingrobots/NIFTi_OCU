// Benoit August 2010

#ifndef NIFTI_PTZ_CAM_VIEW_CONTROLLER_H
#define NIFTI_PTZ_CAM_VIEW_CONTROLLER_H

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <sensor_msgs/JointState.h> // For PTU commands

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
                 * This controller does nothing for now (I need to import the functionality
                 * from the move tool
                 */
                class PTZCamViewController : public CameraViewController
                {
                public:
                    PTZCamViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer);
                    virtual ~PTZCamViewController();

                    void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                    std::string toString() const;

                    double getFieldOfViewHorizontal() const;
                    double getFieldOfViewVertical() const;
                    
                    void lookAt(const Ogre::Vector3& point);
                    void resetView();
                    
                    float getFieldOfView();

                protected:

                    ros::NodeHandle rosNodeHandle;
                    ros::Publisher publisher;

                    void publishNewPosition(int currentX, int currentY);

                    // Last position of the cursor, used to calculate the displacement with the current position of the cursor
                    int lastX, lastY;

                    // Represents the absolute position of the PTU.
                    sensor_msgs::JointState desiredPosition;
                    static const int PAN_SLOT;
                    static const int TILT_SLOT;

                    // I guess that this is the speed at which the PTU moves TODO verify that
                    std::vector<double> velocity;

                    // For the robot P3-AT, these values seem to be the maximal points where
                    // the PTU can go.
                    static const double MAX_UP;
                    static const double MAX_DOWN;
                    static const double MAX_LEFT;
                    static const double MAX_RIGHT;
                    static const double HOME_POSITION;

                    // For the robot P3-AT, this is a convenient speed (it moves fast enough but not too fast)
                    // Harmish said that 0.22 is the default
                    static const double VELOCITY;

                    // This influences how much the PTU will move for each pixel of mouse move
                    // THIS NEEDS TO BE CALIBRATED (AND ADJUSTABLE DEPENDING ON FOCAL LENGTH)
                    static const double PIXEL_TO_MOVEMENT_RATIO;

                    static const char* ROS_PUBLISHING_TOPIC;


                };

            }
        }
    }
}

#endif // NIFTI_PTZ_CAM_VIEW_CONTROLLER_H
