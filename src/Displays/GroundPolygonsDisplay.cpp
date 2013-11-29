// Benoit September 2010
// Inspired from RVIZ polygon_display

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/Point32.h>

#include "Displays/Transformers/FrameTransformer.h"

#include "FloatValidator.h"

#include "Displays/GroundPolygonsDisplay.h"

namespace rviz
{

    const float GroundPolygonsDisplay::DEFAULT_HEIGHT = -0.01; // 1 cm below ground

    GroundPolygonsDisplay::GroundPolygonsDisplay(const std::string& name, const std::string& topic, Ogre::SceneManager* sceneMgr, FrameTransformer* frameTransformer, ros::CallbackQueueInterface* updateQueue, ros::CallbackQueueInterface* threadQueue)
    : Display(name, sceneMgr, frameTransformer, updateQueue, threadQueue)
    , numPolygonsEverGenerated(0)
    , msgReceivedSinceLastClear(0)
    , sceneNode(sceneMgr->getRootSceneNode()->createChildSceneNode())
    , polygons(new std::set<Ogre::ManualObject*>())
    , colors(std::vector< Ogre::ColourValue* >((size_t) 0, NULL)) // size_t is due to a compiler problem with STL vector
    , filled(true)
    , tf_filter_(*frameTransformer->getTFClient(), "", 10, update_nh_)
    {
        this->topic = topic;
        
        // Todo Remove that and use GroundPolygonUtil instead
        // Creates a set of 125 distinct colors (make it better generated, to vary the colors more)
        colors.reserve(128);
        for (float r = 0.875; r > 0.250; r -= 0.125)
        {
            for (float g = 0.875; g > 0.250; g -= 0.125)
            {
                for (float b = 0.875; b > 0.250; b -= 0.125)
                {
                    colors.push_back(new Ogre::ColourValue(r, g, b, 1)); // Last argument is alpha (transparency I think)
                }
            }
        }

        ;
        //        // TEST TO PLACE THE COLORS DIFFERENTLY IN THE ARRAY
        //        // Creates a set of 125 distinct colors (make it better generated, to vary the colors more)
        //        int colorIndex = 0;
        //        // Need to create the vector with 128 slots instead of 0
        //        for (float r = 0.875; r > 0.250; r-=0.125)
        //        {
        //            for (float g = 0.875; g > 0.250; g-=0.125)
        //            {
        //                for (float b = 0.875; b > 0.250; b-=0.125)
        //                {
        //                    colors[colorIndex] = new Ogre::ColourValue(r, g, b, 1);
        //
        //                    colorIndex = (colorIndex+25)%125;
        //                }
        //                colorIndex = (colorIndex+5)%125;
        //            }
        //            colorIndex = (colorIndex+1)%125;
        //        }
        ;
        //        // TEST TO PLACE THE COLORS DIFFERENTLY IN THE ARRAY

        //        // Creates a set of 125 distinct colors (make it better generated, to vary the colors more)
        //        //int colorIndex = 0;
        //        colors.reserve(128);
        //
        //        // Almost Randomly chosen seeds
        //        int r = 450;
        //        int g = 750;
        //        int b = 150;
        //        for (int i = 0; i < 125; i++)
        //        {
        //            colors.push_back(new Ogre::ColourValue(r/1000.0, g/1000.0, b/1000.0, 1));
        //
        //            std::cout << "Color " << r/1000.0 << " " << g/1000.0 << " " << b/1000.0 << std::endl;
        //
        //
        //                r = (r+300)%750;
        //
        //                g = (g+300)%750;
        //
        //                b = (b+300)%750;
        //
        //
        ////            if(i%5 == 4)
        ////            {
        ////                r = (r+300)%750;
        ////            }
        ////            if(i%25 == 24)
        ////            {
        ////                g = (g+300)%750;
        ////            }
        //
        //        }
        //        std::cout << "Colors created: " << colors.size() << std::endl;


        tf_filter_.connectInput(sub_);
        tf_filter_.registerCallback(boost::bind(&GroundPolygonsDisplay::incomingMessage, this, _1));
        frameTransformer->registerFilterForTransformStatusCheck(tf_filter_, this);
    }

    GroundPolygonsDisplay::~GroundPolygonsDisplay()
    {
        unsubscribe();
        clear();

        delete polygons;

        // Deletes all created colors
        while (colors.empty() == false)
        {
            delete colors.back();
            colors.pop_back();
        }

        sceneMgr->destroySceneNode(sceneNode);
    }

    void GroundPolygonsDisplay::clear()
    {
        clearPolygons();

        msgReceivedSinceLastClear = 0;
        setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_WARNING, "No messages received");
    }

    void GroundPolygonsDisplay::subscribe()
    {
        if (!isEnabled())
        {
            return;
        }

        sub_.subscribe(update_nh_, this->topic, 10);
    }

    void GroundPolygonsDisplay::unsubscribe()
    {
        sub_.unsubscribe();
    }

    void GroundPolygonsDisplay::onEnable()
    {
        sceneNode->setVisible(true);
        subscribe();
    }

    void GroundPolygonsDisplay::onDisable()
    {
        unsubscribe();
        clear();
        sceneNode->setVisible(false);
    }

    void GroundPolygonsDisplay::fixedFrameChanged()
    {
        clear();

        tf_filter_.setTargetFrame(fixed_frame_);
    }

    void GroundPolygonsDisplay::update(float wall_dt, float ros_dt)
    {
        //std::cout << "void GroundPolygonsDisplay::update(float wall_dt, float ros_dt) " << this << std::endl;
    }

    void GroundPolygonsDisplay::clearPolygons()
    {
        for (std::set<Ogre::ManualObject*>::iterator it = polygons->begin(); it != polygons->end(); it++)
        {
            sceneMgr->destroyManualObject(*it); // This handles de-allocation of memory
        }
        polygons->clear();
        
        //std::cout << "All polygons cleared" << std::endl;
    }

    bool validateFloats(const geometry_msgs::PolygonStamped& msg)
    {
        return FloatValidator::validateFloats(msg.polygon.points);
    }

    void GroundPolygonsDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)
    {
        //std::cout << "IN void GroundPolygonsDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg) " << this << " at time " << ros::Time::now() << std::endl;
        
        assert(msg != NULL);

        //        if (!msg)
        //        {
        //            return;
        //        }

        ++msgReceivedSinceLastClear;

        if (!validateFloats(*msg))
        {
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_ERROR, "Message contained invalid floating point values (nans or infs)");
            return;
        }

        {
            std::stringstream ss;
            ss << msgReceivedSinceLastClear << " messages received";
            setStatus("Topic", eu::nifti::ocu::STATUS_LEVEL_OK, ss.str());
        }

        //std::cout << msgReceivedSinceLastClear << " messages received" << std::endl;

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!frameTransformer->getTransform(msg->header, position, orientation))
        {
            ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str());
        }

        sceneNode->setPosition(position);
        sceneNode->setOrientation(orientation);

        // Ensures that we don't process the same message twice
        static geometry_msgs::PolygonStamped::ConstPtr lastMsg;
        if (lastMsg != msg)
        {
            modifyPolygons(msg);
            lastMsg = msg;
        }

        //std::cout << "OUT void GroundPolygonsDisplay::processMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)" << std::endl;
    }

    void GroundPolygonsDisplay::modifyPolygons(const geometry_msgs::PolygonStamped::ConstPtr& msg)
    {
        //std::cout << "IN void GroundPolygonsDisplay::modifyPolygons(const geometry_msgs::PolygonStamped::ConstPtr& msg) " << msg.get() << std::endl;


        // Clears the old polygons (will regenerate every thing from the message)
        clearPolygons();

        // Creates a variable to access the points directly hereafter
        const std::vector<geometry_msgs::Point32_<std::allocator<void> > >& points = msg->polygon.points;

        // If there are no polygons in the message, just return
        if (points.size() == 0)
            return;

        uint32_t currentIndex = 0;

        while (currentIndex < points.size()) // While there are more polygons in the list
        {

            

            uint32_t startIndexOfCurrentPolygon = currentIndex;

            // Determines the extent of the current polygon (start and end index)
            float label = points[startIndexOfCurrentPolygon].z;
            while (points[currentIndex].z == label)
            {
                currentIndex++;
            }
            uint32_t endIndexOfCurrentPolygon = currentIndex - 1;

            // Creates and sets-up a polygon
            std::stringstream polygonName;
            polygonName << "Polygon #" << numPolygonsEverGenerated++;

            //std::cout << "BEFORE creating manual object \"" << polygonName.str() << "\" at z-index " <<  points[endIndexOfCurrentPolygon].z << " with " << (endIndexOfCurrentPolygon - startIndexOfCurrentPolygon) << " vertices" << std::endl;

            Ogre::ManualObject* polygon = sceneMgr->createManualObject(polygonName.str());

            //std::cout << "AFTER creating a manual object [" <<  points[endIndexOfCurrentPolygon].z << "]: " << std::endl;

            sceneNode->attachObject(polygon);
            polygon->estimateVertexCount(endIndexOfCurrentPolygon - startIndexOfCurrentPolygon);
            
            if(filled)
            {
                polygon->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN); // Draws filled triangles in order to fill the entire polygon
            }
            else
            {
                polygon->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP); // Draws only the contour (ICMI 2012)
            }

            // This code chooses the color for the polygon
            // Currently, it uses the index of the polygon (as sent in the message)
            int c = (int) points[startIndexOfCurrentPolygon].z; // Converts the index from float to int
            polygon->colour(*(colors.at(c % 125)));

            // Adds all vertices to the polygon object
            // Todo Remove that and use GroundPolygonUtil instead
            for (uint32_t i = startIndexOfCurrentPolygon; i <= endIndexOfCurrentPolygon; i++)
            //for (uint32_t i = endIndexOfCurrentPolygon; i >= startIndexOfCurrentPolygon; i--)
            {
                //std::cout << "BEFORE adding a vertex @ (" <<  points[i].x << ", " << points[i].y << ")";

                Ogre::Vector3 pos(points[i].x, points[i].y, DEFAULT_HEIGHT);

                //std::cout << " : " <<  pos << std::endl;

                polygon->position(pos);
            }

            polygon->end();

            //std::cout << "Bounding box radius : " <<  polygon->getBoundingRadius() << std::endl;

            polygons->insert(polygon);
        }

        //std::cout << "OUT void GroundPolygonsDisplay::modifyPolygons(const geometry_msgs::PolygonStamped::ConstPtr& msg) " << msg.get() << std::endl;
    }

    void GroundPolygonsDisplay::incomingMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)
    {
        processMessage(msg);
    }

    void GroundPolygonsDisplay::reset()
    {
        Display::reset();
        clear();
    }
    
    void GroundPolygonsDisplay::setFillPolygons(bool filled)
    {
        this->filled = filled;
    }

} // namespace rviz

