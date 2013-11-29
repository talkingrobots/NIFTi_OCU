/**
 * UUID manager advertises a service for UUID generation.
 * First it checks whether there already is such a service advertised
 * in which case it stops.
 *
 *  Created on: Jul 4, 2011
 *      Author: petrito1
 */

#include <ros/ros.h>
#include <eu_nifti_env_msg_ros/RequestForUUIDs.h>

using eu_nifti_env_msg_ros::RequestForUUIDs;

namespace {
std::string uuidTopic = "/eoi/RequestForUUIDs";
int16_t currentUuid = 0;

bool requestForUuidsService(RequestForUUIDs::Request& request, RequestForUUIDs::Response& response) {
  ROS_DEBUG("requestForUuidsService");

  response.uuids.resize(request.numRequested);
  for (int i = 0; i < request.numRequested; i++) {
    response.uuids[i] = currentUuid++;
  }

  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uuid_manager");
  ros::NodeHandle nodeHandle;

  ros::NodeHandle pnh("~");
  uuidTopic = nodeHandle.resolveName(uuidTopic, true);
  
  // https://trac.dfki.de/nifti/ticket/131
  // Avoid advertising the service twice.
  if (ros::service::waitForService(uuidTopic, ros::Duration(1.0))) {
    ROS_WARN("UUID manager already running. Stopping this node...");
    return 0;
  }
  
  ROS_INFO("Previous instance of the running service not found.");

  ros::ServiceServer requestForUuidsServer = nodeHandle.advertiseService(uuidTopic, requestForUuidsService);

  ROS_INFO("Service now running.");

  ros::spin();

  return 0;
}
