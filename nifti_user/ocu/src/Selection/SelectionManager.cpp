// Benoit 2011-05-27

#include <ros/node_handle.h>

#include <eu_nifti_ocu_msg_ros/SelectionMessage.h>
#include <eu_nifti_ocu_msg_ros/PictureSelectionMessage.h>

#include <eu_nifti_cast/WorkingMemoryPointer.h>

#include "Displays/Markers/InFieldPictureMarker.h"
#include "Displays/Markers/LocationOfInterestMarker.h"
#include "Displays/Markers/ObjectOfInterestMarker.h"

#include "Selection/LocationSelectionManager.h"
#include "Selection/MarkerSelectionManager.h"

#include "IMultiVisualizationManager.h"
#include "IVisualizationManager.h"
#include "NIFTiROSUtil.h"

#include "Selection/SelectionManager.h"

using namespace eu::nifti::ocu::display;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace selection
            {
                const std::string TOPIC_SELECTION_EOI = "ocu/selection";
                const std::string TOPIC_SELECTION_PICTURE = "ocu/selectionPicture";

                SelectionManager::SelectionManager(eu::nifti::ocu::IMultiVisualizationManager* multiVizMgr)
                : multiVizMgr(multiVizMgr)
                , locationSelMgr(new LocationSelectionManager(multiVizMgr))
                , markerSelMgr(new MarkerSelectionManager(multiVizMgr))
                , selectedPicture(NULL)
                , selectionType(SELECTION_TYPE_NONE)
                {
                    publisherEOISelection = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<eu_nifti_ocu_msg_ros::SelectionMessage > (TOPIC_SELECTION_EOI, 1);
                    publisherPictureSelection = eu::nifti::ocu::NIFTiROSUtil::getNodeHandle()->advertise<eu_nifti_ocu_msg_ros::PictureSelectionMessage > (TOPIC_SELECTION_PICTURE, 1);
                }

                SelectionManager::~SelectionManager()
                {
                    deselect();
                    delete locationSelMgr;
                    delete markerSelMgr;
                }

                SelectionManager::SelectionType SelectionManager::getSelectionType() const
                {
                    return selectionType;
                }

                void SelectionManager::initialize()
                {
                    markerSelMgr->initialize();
                }

                rviz::CollObjectHandle SelectionManager::addTrackedMarker(eu::nifti::ocu::display::IUSARMarker* marker)
                {
                    return markerSelMgr->addTrackedMarker(marker);
                }

                void SelectionManager::removeTrackedMarker(const rviz::CollObjectHandle obj)
                {
                    // First deselect the marker if it happens to be selected

                    switch (selectionType)
                    {
                        case SELECTION_TYPE_OBJECT_OF_INTEREST:
                        case SELECTION_TYPE_ANNOTATED_PICTURE:
                            if (markerSelMgr->getMarker(obj) == markerSelMgr->getSelectedMarker())
                            {
                                deselect();
                            }
                            break;
                        default:
                            break;
                    }


                    markerSelMgr->removeTrackedMarker(obj);
                }

                void SelectionManager::select(eu::nifti::ocu::IVisualizationManager* vizMgr, int x, int y, int range)
                {
                    //ROS_INFO("IN void ElementOfInterestSelectionManager::select(eu::nifti::ocu::IVisualizationManager* vizMgr, int x, int y, int range): %i %i %i", x, y, range);

                    deselect();

                    const eu::nifti::ocu::display::IUSARMarker* selectedMarker = markerSelMgr->select(vizMgr->getViewport(), x - range, y - range, x + range, y + range);


                    if (selectedMarker != NULL && selectedMarker->getMarkerType() == IUSARMarker::ELEMENT_OF_INTEREST)
                    {
                        selectionType = SELECTION_TYPE_OBJECT_OF_INTEREST;

                        const eu::nifti::ocu::display::ObjectOfInterestMarker* selectedOOIMarker = (const eu::nifti::ocu::display::ObjectOfInterestMarker*) selectedMarker;

                        multiVizMgr->onSelectObjectOfInterest(selectedOOIMarker->getObjectOfInterest(), true);

                        publishSelectionMessage(selectedOOIMarker->getObjectOfInterest()->element.uuid, selectedOOIMarker->getCASTWorkingMemoryPointer(), eu_nifti_ocu_msg_ros::SelectionMessage::SELECT);

                        return;
                    }

                    if (selectedMarker != NULL && selectedMarker->getMarkerType() == IUSARMarker::ANNOTATED_PICTURE)
                    {
                        selectionType = SELECTION_TYPE_ANNOTATED_PICTURE;

                        const eu::nifti::ocu::display::InFieldPictureMarker* selectedPictureMarker = (const eu::nifti::ocu::display::InFieldPictureMarker*) selectedMarker;

                        selectedPicture = selectedPictureMarker->getPicture();

                        multiVizMgr->onSelectAnnotatedPicture(selectedPicture, true);

                        publishSelectionMessage(selectedPicture->filename, eu_nifti_ocu_msg_ros::SelectionMessage::SELECT);

                        return;
                    }

                    //ROS_INFO("No object was found, so we need to chose a location in the ground");

                    // No object was found, so we need to chose a location in the ground
                    const eu::nifti::ocu::display::LocationOfInterestMarker* selectedLOIMarker = locationSelMgr->select(vizMgr, x, y);

                    if (selectedLOIMarker != NULL)
                    {
                        selectionType = SELECTION_TYPE_LOCATION_OF_INTEREST;

                        multiVizMgr->onSelectLocationOfInterest(selectedLOIMarker->getLocationOfInterest(), true);

                        publishSelectionMessage(selectedLOIMarker->getLocationOfInterest()->element.uuid, eu_nifti_ocu_msg_ros::SelectionMessage::SELECT);

                        return;
                    }

                    // At this point, nothing could be selected. We leave the selection to NONE and do nothing

                    //ROS_INFO("OUT void ElementOfInterestSelectionManager::select(eu::nifti::ocu::IVisualizationManager* vizMgr, int x, int y, int range)");
                }

                void SelectionManager::selectAnnotatedPicture(const EXIFReader_msgs::AnnotatedPicture* picture)
                {
                    //ROS_INFO("IN void SelectionManager::selectAnnotatedPicture(const std::string& filename)");

                    deselect();

                    selectionType = SELECTION_TYPE_ANNOTATED_PICTURE;

                    selectedPicture = picture;

                    /*const eu::nifti::ocu::display::IUSARMarker* selectedMarker = */ markerSelMgr->selectAnnotatedPicture(picture->filename);

                    multiVizMgr->onSelectAnnotatedPicture(picture, true);

                    publishSelectionMessage(picture->filename, eu_nifti_ocu_msg_ros::SelectionMessage::SELECT);
                }

                void SelectionManager::deselect()
                {
                    //ROS_INFO("IN void ElementOfInterestSelectionManager::deselect()");

                    switch (selectionType)
                    {
                        case SELECTION_TYPE_NONE:
                            break;
                        case SELECTION_TYPE_OBJECT_OF_INTEREST:
                        {
                            const eu::nifti::ocu::display::ObjectOfInterestMarker* ooiMarker = (eu::nifti::ocu::display::ObjectOfInterestMarker*) getSelectedMarker();

                            assert(ooiMarker != NULL);

                            multiVizMgr->onSelectObjectOfInterest(ooiMarker->getObjectOfInterest(), false);
                            publishSelectionMessage(ooiMarker->getObjectOfInterest()->element.uuid, eu_nifti_ocu_msg_ros::SelectionMessage::DESELECT);
                            markerSelMgr->deselect();
                        }
                            break;
                        case SELECTION_TYPE_LOCATION_OF_INTEREST:
                        {
                            const eu::nifti::ocu::display::LocationOfInterestMarker* loiMarker = (eu::nifti::ocu::display::LocationOfInterestMarker*) getSelectedMarker();

                            assert(loiMarker != NULL);

                            multiVizMgr->onSelectLocationOfInterest(loiMarker->getLocationOfInterest(), false);
                            publishSelectionMessage(loiMarker->getLocationOfInterest()->element.uuid, eu_nifti_ocu_msg_ros::SelectionMessage::DESELECT);
                            locationSelMgr->deselect();

                        }
                            break;
                        case SELECTION_TYPE_ANNOTATED_PICTURE:
                        {
                            assert(selectedPicture != NULL);

                            multiVizMgr->onSelectAnnotatedPicture(selectedPicture, false);
                            publishSelectionMessage(selectedPicture->filename, eu_nifti_ocu_msg_ros::SelectionMessage::DESELECT);
                            markerSelMgr->deselect();
                            selectedPicture = NULL;
                        }
                            break;
                    }

                    selectionType = SELECTION_TYPE_NONE;


                    //ROS_INFO("OUT void ElementOfInterestSelectionManager::deselect()");
                }

                const eu::nifti::ocu::display::IUSARMarker* SelectionManager::getSelectedMarker()
                {
                    switch (selectionType)
                    {
                        case SELECTION_TYPE_NONE:
                            return NULL;
                            break;
                        case SELECTION_TYPE_OBJECT_OF_INTEREST:
                        case SELECTION_TYPE_ANNOTATED_PICTURE:
                            return markerSelMgr->getSelectedMarker();
                            break;
                        case SELECTION_TYPE_LOCATION_OF_INTEREST:
                            return locationSelMgr->getSelectedMarker();
                            break;
                        default:
                            assert(false); // Can never get here
                            return NULL;
                    }
                }

                void SelectionManager::focusOnSelection()
                {
                    switch (selectionType)
                    {
                        case SELECTION_TYPE_NONE:
                            return;
                            break;
                        case SELECTION_TYPE_OBJECT_OF_INTEREST:
                        case SELECTION_TYPE_ANNOTATED_PICTURE:
                            markerSelMgr->focusOnSelection();
                            break;
                        case SELECTION_TYPE_LOCATION_OF_INTEREST:
                            locationSelMgr->focusOnSelection();
                            break;
                    }
                }

                void SelectionManager::publishSelectionMessage(int uuid, int action)
                {
                    publishSelectionMessage(uuid, eu_nifti_cast::WorkingMemoryPointer(), action);
                }

                void SelectionManager::publishSelectionMessage(int uuid, const eu_nifti_cast::WorkingMemoryPointer& castWorkingMemoryPointer, int action)
                {
                    eu_nifti_ocu_msg_ros::SelectionMessage selectionMsg;
                    selectionMsg.header.stamp = ros::Time::now();
                    selectionMsg.userID = eu::nifti::ocu::NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                    selectionMsg.action = action;
                    selectionMsg.elementUUID = uuid;
                    selectionMsg.castWorkingMemoryPointer = castWorkingMemoryPointer;

                    publisherEOISelection.publish(selectionMsg);

                    //ROS_INFO("Published a selection msg for uuid #%i with action %i", uuid, action);
                }

                void SelectionManager::publishSelectionMessage(const std::string& filename, int action)
                {
                    eu_nifti_ocu_msg_ros::PictureSelectionMessage selectionMsg;
                    selectionMsg.header.stamp = ros::Time::now();
                    selectionMsg.userID = eu::nifti::ocu::NIFTiROSUtil::getNodeHandleWithPrefix()->getNamespace();
                    selectionMsg.action = action;
                    selectionMsg.filename = filename;

                    publisherPictureSelection.publish(selectionMsg);
                }

            }
        }
    }
}