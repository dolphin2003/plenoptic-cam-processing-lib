#pragma once

#include "io/archive.h"

#include "cfg/mia.h"
#include "cfg/mla.h"
#include "cfg/sensor.h"
#include "cfg/thinlens.h"
#include "cfg/distortions.h"
#include "cfg/functions.h"

#include "geometry/camera/plenoptic.h"

V_DEFINE_PROPERTIES(PlenopticCameraConfig)
(   
    V_DEFINE_PROPERTY(mode, (int(2)), "Mode of the camera (0: Unfocused (F=D and f=d); 1: Keplerian (F<D and f<d)) ; 2: Galilean (F>D and f>d)")
    
    V_DEFINE_PROPERTY(mia, MIAConfig(), "Micro-Images Array configuration")
    V_DEFINE_PROPERTY(mla, MLAConfig(), "Micro-Lenses Array configuration")
    V_DEFINE_PROPERTY(sensor