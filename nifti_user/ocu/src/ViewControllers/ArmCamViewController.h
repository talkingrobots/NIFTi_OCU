// Benoit 2012-03-06

#ifndef NIFTI_ARM_CAM_VIEW_CONTROLLER_H
#define NIFTI_ARM_CAM_VIEW_CONTROLLER_H

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <std_msgs/Float64.h>

#include "nifti_arm_msgs/msg_height.h"

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
                 * Moves the arm and PTU
                 */
                class ArmCamViewController : public CameraViewController
                {
                public:
                    ArmCamViewController(Ogre::SceneManager* sceneMgr, eu::nifti::ocu::IVisualizationManager* vizMgr, Ogre::Camera* camera, rviz::FrameTransformer* frameTransformer);
                    virtual ~ArmCamViewController();

                    void handleMouseEvent(eu::nifti::ocu::gui::MultiVizMouseEvent& evt);

                    double getFieldOfViewHorizontal() const;
                    double getFieldOfViewVertical() const;
                    
                    void lookAt(const Ogre::Vector3& point);
                    void resetView();
                    
                    std::string toString() const;
                    
                    float getFieldOfView();

                protected:

                    ros::NodeHandle rosNodeHandle;
                    ros::Publisher publisherHeight;
                    ros::Publisher publisherPan;
                    ros::Publisher publisherTilt;

                    void publishDueToMouseMotion(int currentX, int currentY);
                    void publishDueToMouseWheel(int scrollAmount);

                    // Last position of the cursor, used to calculate the displacement with the current position of the cursor
                    int lastX, lastY;

                    // Represents the absolute position of the PTU.
                    nifti_arm_msgs::msg_height desiredHeight;
                    std_msgs::Float64 desiredPan;
                    std_msgs::Float64 desiredTilt;

                    // I guess that this is the speed at which the PTU moves TODO verify that
                    std::vector<double> velocity;

                    // For the robot P3-AT, these values seem to be the maximal points where
                    // the PTU can go.
                    static const double MAX_UP;
                    static const double MAX_DOWN;
                    static const double MAX_LEFT;
                    static const double MAX_RIGHT;
                    static const double MAX_TILT_UP;
                    static const double MAX_TILT_DOWN;
                    static const double HOME_POSITION_HEIGHT;
                    static const double HOME_POSITION_PAN;
                    static const double HOME_POSITION_TILT;

                    // This influences how much the camera will move for each pixel of mouse move
                    static const double PIXEL_TO_MOVEMENT_RATIO_HEIGHT;
                    static const double PIXEL_TO_MOVEMENT_RATIO_PAN;
                    static const double PIXEL_TO_MOVEMENT_RATIO_TILT;

                    static const char* ROS_TOPIC_HEIGHT;
                    static const char* ROS_TOPIC_PAN;
                    static const char* ROS_TOPIC_TILT;


                };

            }
        }
    }
}

#endif // NIFTI_ARM_CAM_VIEW_CONTROLLER_H
