#include "calibration.h"

#include <type_traits> // std::remove_reference_t
#include <variant> //std::variant

#include "unused.h"

//optimization
#include "optimization/optimization.h"
#include "optimization/errors/mic.h" //MICReprojectionError
#include "optimization/errors/corner.h" //CornerReprojectionError
#include "optimization/errors/radius.h" //RadiusReprojectionError
#include "optimization/errors/bap.h" //BlurAwarePlenopticReprojectionError

//io
#include "io/cfg/observations.h" // save and load
#include "io/cfg/poses.h"

#include "io/printer.h"
#include "io/choice.h"

//graphic
#include "graphic/gui.h"
#include "graphic/display.h"

//calibration
#include "init.h"
#include "link.h"
#include "evaluate.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize(
	//OUT
	CalibrationPoses& poses, /* extrinsics */
	PlenopticCamera& model, /* intrinsics */
	//IN
	const CheckerBoard & checkboard,
	const BAPObservations& observations, /*  (u,v,rho?) */
	const MICObservations& centers /* c_{k,l} */
)
{	
	const bool useRadius = (model.focused()); 
	
	using SolverBAP = lma::Solver<CornerReprojectionError, BlurRadiusReprojectionError, MicroImageCenterReprojectionError>;
	using SolverCorner = lma::Solver<CornerReprojectionError, MicroImageCenterReprojectionError>;
	using Solver_t = std::variant<std::monostate, SolverBAP, SolverCorner>;
	
	Solver_t vsolver;
		if(useRadius) vsolver.emplace<SolverBAP>(-1., 65, 1.0 - 1e-6);
		else vsolver.emplace<SolverCorner>(-1., 65, 1.0 - 1e-6);  
	
	//split observations according to frame index
	std::unordered_map<Index /* frame index */, BAPObservations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);
		
	std::visit(
		[&](auto&& s) { 
			using T = std::decay_t<decltype(s)>;
			if constexpr (not std::is_same_v<T, std::monostate>) {
				for (auto & [p, frame] : poses) { //for each frame with its pose
					for (const auto& o : obs[frame]) { //for each observation of this frame
						//ADD CORNER OBSERVATIONS	
						s.add(
							CornerReprojectionError{
								model, checkboard, o
							},
							&p,
							&model.mla().pose(),
							&model.mla(),
							&model.sensor(),
							&model.main_lens(),
							&model.main_lens_distortions()
						);
						//ADD RADII OBSERVATIONS
						if constexpr (std::is_same_v<T, SolverBAP>) {
							s.add(
								BlurRadiusReprojectionError{
									model, checkboard, o
								},
								&p,
								&model.mla().pose(),
								&model.mla(),
								&model.mla().focal_length(o.k, o.l),
								&model.sensor(),
								&model.main_lens(),
								&model.main_lens_distortions()
							);
						}
					} //endfor each observation	
				} //endfor each frame
				
				{
					for (const auto& c : centers) {								
						//ADD CENTER OBSERVATIONS
						s.add(
							MicroImageCenterReprojectionError{model, c},
							&model.mla().pose(),
							&model.mla(),
							&model.sensor()
						);
					}
				}
				s.solve(lma::DENSE, lma::enable_verbose_output());
			}
		}, vsol