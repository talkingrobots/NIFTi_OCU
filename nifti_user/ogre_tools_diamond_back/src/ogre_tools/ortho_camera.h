/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef OGRE_TOOLS_ORTHO_CAMERA_H_
#define OGRE_TOOLS_ORTHO_CAMERA_H_

#include "camera_base.h"
#include <OGRE/OgreVector3.h>

namespace ogre_tools
{

class wxOgreRenderWindow;

class OrthoCamera : public CameraBase
{
public:
  OrthoCamera( wxOgreRenderWindow* render_window, Ogre::SceneManager* scene_manager );
  virtual ~OrthoCamera();

  virtual void update();

  virtual void setFrom( CameraBase* camera );
  virtual void yaw( float angle );
  virtual void pitch( float angle );
  virtual void roll( float angle );
  virtual void setOrientation( float x, float y, float z, float w );
  virtual void setPosition( float x, float y, float z );
  virtual void move( float x, float y, float z );

  virtual Ogre::Vector3 getPosition();
  virtual Ogre::Quaternion getOrientation();

  virtual void lookAt( const Ogre::Vector3& point );

  virtual void mouseLeftDrag( int diff_x, int diff_y, bool ctrl, bool alt, bool shift );
  virtual void mouseMiddleDrag( int diff_x, int diff_y, bool ctrl, bool alt, bool shift );
  virtual void mouseRightDrag( int diff_x, int diff_y, bool ctrl, bool alt, bool shift );
  virtual void scrollWheel( int diff, bool ctrl, bool alt, bool shift );

  virtual void fromString(const std::string& str);
  virtual std::string toString();

private:
  float scale_;
  wxOgreRenderWindow* render_window_;
};

} //namespace ogre_tools

#endif /* OGRE_TOOLS_ORTHO_CAMERA_H_ */
