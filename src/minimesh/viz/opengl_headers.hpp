#pragma once

//
// opengl_header.hpp
//


#ifdef WIN32
#include <windows.h>
#undef near
#undef far
#endif

#ifdef __APPLE__
#  include <OpenGL/gl.h>
#  include <OpenGL/glu.h>
#  include <GLUT/glut.h>
#  include <GL/glui.h>
#else
#  include <GL/gl.h>
#  include <GL/glu.h>
#  include <GL/glut.h>
#  include <GL/glui.h>
#endif

