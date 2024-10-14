/**
 * @file Platform/Camera.h
 *
 * Inclusion of a platform dependent camera interface.
 *
 * @author Colin Graf
 */

#pragma once

#ifdef TARGET_ROBOT  // TODO VINC: modify in #ifndef
#include "Nao/NaoCamera.h"
#define CAMERA_INCLUDED
#endif
