#pragma once

#include <type_traits>

#include "graphic/gui.h"

#include "graphic/viewer_3d.h"
#include "graphic/viewer_2d.h"

#include "io/choice.h"

#include "types.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/observation.h"
#include "geometry/reprojection.h"
#include "geometry/plane.h"
#include "geometry/ray.h"

#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"
#include "geometry/depth/depthimage.h"

#include "geometry/object/checkerboard.h"
#include "geometry/object/constellation.h"

inline void display(const CheckerBoard& checkboard)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("CheckerBoard"), 
		checkboard, 35.
	);
	Viewer::update(Viewer::Mode::m3D);
}

inline void display(const Plate& plate)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("Plate"), 
		plate
	);
	Viewer::update(Viewer::Mode::m3D);
}

inline void display(const PlenopticCamera& model)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("CameraBody"), 
		model, tag::CameraBody{}
	);
	Viewer::update(Viewer::Mode::m3D);
	
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("TCM"), 
		model, tag::ThinLens{}, 35.
	);
	Viewer::update(Viewer::Mode::m3D);
	
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("MLA"), 
		model, tag::MLA{}, 5.
	);
	Viewer::update(Viewer::Mode::m3D);
	
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("Sensor"), 
		model, tag::Sensor{}, 5.
	);
	Viewer::update(Viewer::Mode::m3D);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void display(const Image& white, const MIA& mia)
{
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("White image"), 
		white
	);
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("Micro-Image Array")
			.pen_color(v::red).pen_width(2),
		mia
	);
	
	Viewer::update();
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void display(const CalibrationP