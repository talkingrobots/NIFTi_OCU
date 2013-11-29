#include "wx_ogre_render_window.h"
#include "orthographic.h"

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreStringConverter.h>

#ifdef __WXGTK__
#include <gdk/gdk.h>
#include <gtk/gtk.h>
#include <gdk/gdkx.h>
#include <wx/gtk/win_gtk.h>
#include <GL/glx.h>
#endif

#include <ros/console.h>
#include <ros/assert.h>

namespace ogre_tools
{

IMPLEMENT_CLASS (wxOgreRenderWindow, wxControl)

BEGIN_EVENT_TABLE (wxOgreRenderWindow, wxControl)
EVT_PAINT (wxOgreRenderWindow::onPaint)
EVT_SIZE (wxOgreRenderWindow::onSize)
EVT_MOUSE_EVENTS (wxOgreRenderWindow::onMouseEvents)
EVT_MOVE(wxOgreRenderWindow::onMove)
END_EVENT_TABLE ()

//------------------------------------------------------------------------------
unsigned int wxOgreRenderWindow::sm_NextRenderWindowId = 1;
//------------------------------------------------------------------------------
wxOgreRenderWindow::wxOgreRenderWindow (Ogre::Root* ogre_root, wxWindow *parent, wxWindowID id,
                                        const wxPoint &pos, const wxSize &size, long style, const wxValidator &validator, bool create_render_window)
    : wxControl( parent, id, pos, size, style, validator )
    , render_window_( 0 )
    , ogre_root_( ogre_root )
    , ortho_scale_( 1.0f )
    , auto_render_(true)
{
  SetBackgroundStyle(wxBG_STYLE_CUSTOM);

  if (create_render_window)
  {
    createRenderWindow();
  }
}

//------------------------------------------------------------------------------
wxOgreRenderWindow::~wxOgreRenderWindow ()
{
  if (render_window_)
  {
    render_window_->removeViewport( 0 );
    render_window_->destroy();
    ogre_root_->detachRenderTarget(render_window_);
  }

  render_window_ = 0;
}

//------------------------------------------------------------------------------
inline wxSize wxOgreRenderWindow::DoGetBestSize () const
{
  return wxSize (320, 240);
}
//------------------------------------------------------------------------------
Ogre::RenderWindow* wxOgreRenderWindow::getRenderWindow () const
{
  return render_window_;
}

//------------------------------------------------------------------------------
Ogre::Viewport* wxOgreRenderWindow::getViewport () const
{
  return viewport_;
}

void wxOgreRenderWindow::setCamera( Ogre::Camera* camera )
{
  viewport_->setCamera( camera );

  setCameraAspectRatio();

  Refresh();
}

void wxOgreRenderWindow::setCameraAspectRatio()
{
  Ogre::Camera* camera = viewport_->getCamera();
  if ( camera )
  {
    int width;
    int height;
    GetSize( &width, &height );

    camera->setAspectRatio( Ogre::Real( width ) / Ogre::Real( height ) );

    if ( camera->getProjectionType() == Ogre::PT_ORTHOGRAPHIC )
    {
      Ogre::Matrix4 proj;
      buildScaledOrthoMatrix( proj, -width / ortho_scale_ / 2, width / ortho_scale_ / 2, -height / ortho_scale_ / 2, height / ortho_scale_ / 2,
                              camera->getNearClipDistance(), camera->getFarClipDistance() );
      camera->setCustomProjectionMatrix(true, proj);
    }
  }
}

void wxOgreRenderWindow::setOrthoScale( float scale )
{
  ortho_scale_ = scale;

  setCameraAspectRatio();
}

void wxOgreRenderWindow::setPreRenderCallback( boost::function<void ()> func )
{
  pre_render_callback_ = func;
}

void wxOgreRenderWindow::setPostRenderCallback( boost::function<void ()> func )
{
  post_render_callback_ = func;
}

//------------------------------------------------------------------------------
void wxOgreRenderWindow::onPaint (wxPaintEvent &evt)
{
  evt.Skip();

  if (auto_render_)
  {
    if ( pre_render_callback_ )
    {
      pre_render_callback_();
    }

    if( ogre_root_->_fireFrameStarted() )
    {
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
      ogre_root_->_fireFrameRenderingQueued();
#endif

      render_window_->update();

      ogre_root_->_fireFrameEnded();
    }

    if ( post_render_callback_ )
    {
      post_render_callback_();
    }
  }
}

//------------------------------------------------------------------------------
void wxOgreRenderWindow::onSize (wxSizeEvent &evt)
{
  int width;
  int height;
  wxSize size = evt.GetSize ();
  width = size.GetWidth ();
  height = size.GetHeight ();

  if (render_window_)
  {
#if !defined(__WXMAC__)
    render_window_->resize (width, height);
#endif
    // Letting Ogre know the window has been resized;
    render_window_->windowMovedOrResized ();

    setCameraAspectRatio();

    if (auto_render_)
    {
      Refresh();
    }
  }

  evt.Skip();
}

void wxOgreRenderWindow::onMove(wxMoveEvent& evt)
{
  evt.Skip();

  if (render_window_)
  {

  }

  ROS_INFO("move");
}

//------------------------------------------------------------------------------
void wxOgreRenderWindow::onMouseEvents (wxMouseEvent &evt)
{
  evt.Skip();
}
//------------------------------------------------------------------------------
void wxOgreRenderWindow::createRenderWindow ()
{
  Ogre::NameValuePairList params;
#if defined(__WXMAC__)
  params["externalWindowHandle"] = getOgreHandle();
#else
  params["parentWindowHandle"] = getOgreHandle();
#endif

  // Get wx control window size
  int width;
  int height;
  GetSize (&width, &height);
  
#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
  ogre_root_->getRenderSystem()->setConfigOption("RTT Preferred Mode", "PBuffer");
#endif
  
  // Create the render window
  render_window_ = ogre_root_->createRenderWindow (
                     Ogre::String ("OgreRenderWindow") + Ogre::StringConverter::toString (sm_NextRenderWindowId++),
                     width, height, false, &params);

  render_window_->setActive (true);
  render_window_->setVisible(true);
  render_window_->setAutoUpdated(true);

  viewport_ = render_window_->addViewport( NULL );
}
//------------------------------------------------------------------------------
std::string wxOgreRenderWindow::getOgreHandle () const
{
  std::string handle;

#ifdef __WXMSW__
  // Handle for Windows systems
  handle = Ogre::StringConverter::toString((size_t)((HWND)GetHandle()));
#elif defined(__WXGTK__)
  // Handle for GTK-based systems
  std::stringstream str;
  GtkWidget* widget = m_wxwindow;
  gtk_widget_set_double_buffered (widget, FALSE);
  if (!GTK_WIDGET_REALIZED(widget))
  {
    gtk_widget_realize( widget );
  }

  // Grab the window object
  GtkPizza* pizza = GTK_PIZZA(widget);
  GdkWindow* gdkWin = pizza->bin_window;
  Window wid = GDK_WINDOW_XWINDOW(gdkWin);

  XSync(GDK_WINDOW_XDISPLAY(widget->window), False);
  XSync(GDK_WINDOW_XDISPLAY(pizza->bin_window), False);

  str << wid;// << ':';
  handle = str.str();
#elif defined(__WXMAC__)
  handle = Ogre::StringConverter::toString((size_t)(GetHandle()));
#else
  // Any other unsupported system
  #error("Not supported on this platform.")
#endif

  return handle;
}

bool wxOgreRenderWindow::Reparent(wxWindowBase* new_parent)
{
  // Dear god I hate GTK.  Because gtk_widget_reparent() does not work properly (https://netfiles.uiuc.edu/rvmorale/www/gtk-faq-es/x639.html),
  // When a window is reparented in wx the drawable client area has to be destroyed and re-created, causing its XID to change.  This causes both:
  // 1) Any window that was its child (like the Ogre render window's) is destroyed
  // 2) The XID changes
  // The following seems to be the only thing that actually works when trying to work around this:
  // 1) Reparent the render window's window to the root window
  // 2) Allow wx to reparent the window
  // 3) Reparent the render window's window to the new parent window
#if defined(__WXGTK__)
  Window win;
  Display* disp;

  if (render_window_)
  {
    render_window_->getCustomAttribute("WINDOW", &win);
    render_window_->getCustomAttribute("XDISPLAY", &disp);

    wxWindow* top = wxTheApp->GetTopWindow();
    GtkPizza* pizza = GTK_PIZZA(top->m_wxwindow);
    Window parent = GDK_WINDOW_XWINDOW(pizza->bin_window);

    XSync(disp, False);
    XReparentWindow(disp, win, parent, 0, 0);
    XSync(disp, False);
  }
#endif

  bool ret = wxControl::Reparent(new_parent);

#if defined(__WXGTK__)
  if (render_window_)
  {
    XSync(disp, False);

    gtk_widget_set_double_buffered(m_wxwindow, FALSE);
    if (!GTK_WIDGET_REALIZED(m_wxwindow))
    {
      gtk_widget_realize(m_wxwindow);
    }

    GtkPizza* pizza = GTK_PIZZA(m_wxwindow);

    XSync(GDK_WINDOW_XDISPLAY(m_wxwindow->window), False);
    XSync(GDK_WINDOW_XDISPLAY(pizza->bin_window), False);

    GdkWindow* gdkWin = pizza->bin_window;
    Window parent = GDK_WINDOW_XWINDOW(gdkWin);
    XReparentWindow(disp, win, parent, 0, 0);

//    XSync(disp, False);

    XMapWindow(disp, win);

    XSync(disp, False);
  }
#endif

  return ret;
}

} // namespace ogre_tools
