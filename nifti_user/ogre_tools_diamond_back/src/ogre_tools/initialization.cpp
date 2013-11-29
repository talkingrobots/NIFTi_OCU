#include "initialization.h"

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreLogManager.h>

#include <exception>
#include <stdexcept>

#include <ros/package.h>

namespace ogre_tools
{
void initializeOgre(bool enable_ogre_log)
{
  try
  {
    Ogre::LogManager* log_manager = new Ogre::LogManager();
    log_manager->createLog( "Ogre.log", false, false, !enable_ogre_log );

    std::string plugin_prefix;
#ifdef OGRE_PLUGIN_PATH
    plugin_prefix = OGRE_PLUGIN_PATH + std::string("/");
#endif

    Ogre::Root* root = new Ogre::Root();
    root->loadPlugin( plugin_prefix + "RenderSystem_GL" );
    root->loadPlugin( plugin_prefix + "Plugin_OctreeSceneManager" );
    root->loadPlugin( plugin_prefix + "Plugin_ParticleFX" );
    root->loadPlugin( plugin_prefix + "Plugin_CgProgramManager" );

    // Taken from gazebo
    Ogre::RenderSystem* render_system = NULL;
#if OGRE_VERSION_MAJOR >=1 && OGRE_VERSION_MINOR >= 7
    Ogre::RenderSystemList rsList = root->getAvailableRenderers();
    Ogre::RenderSystemList::iterator renderIt = rsList.begin();
    Ogre::RenderSystemList::iterator renderEnd = rsList.end();
#else
    Ogre::RenderSystemList* rsList = root->getAvailableRenderers();
    Ogre::RenderSystemList::iterator renderIt = rsList->begin();
    Ogre::RenderSystemList::iterator renderEnd = rsList->end();
#endif
    for ( ; renderIt != renderEnd; ++renderIt )
    {
      render_system = *renderIt;

      if ( render_system->getName() == "OpenGL Rendering Subsystem" )
      {
        break;
      }
    }

    if ( render_system == NULL )
    {
      throw std::runtime_error( "Could not find the opengl rendering subsystem!\n" );
    }

    render_system->setConfigOption("Full Screen","No");
    render_system->setConfigOption("FSAA","2");
    render_system->setConfigOption("RTT Preferred Mode", "FBO");

    root->setRenderSystem( render_system );

    root->initialise( false );

    std::string ogre_tools_path = ros::package::getPath(ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media/fonts", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media/models", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media/materials", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media/materials/scripts", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( ogre_tools_path + "/media/materials/programs", "FileSystem", ROS_PACKAGE_NAME );
  }
  catch ( Ogre::Exception& e )
  {
      printf( "Failed to initialize Ogre: %s\n", e.what() );
      throw;
  }
}

void cleanupOgre()
{
  delete Ogre::Root::getSingletonPtr();
}

void initializeResources( const V_string& resource_paths )
{
  V_string::const_iterator path_it = resource_paths.begin();
  V_string::const_iterator path_end = resource_paths.end();
  for( ; path_it != path_end; ++path_it )
  {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( *path_it, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  }

  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

} // namespace ogre_tools
