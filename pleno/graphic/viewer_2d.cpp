
#include "viewer_2d.h"

/**
 * @Brief libv_wrapper is a wrapper taking an opencv image to convert it into libv format
 * TODO: becarefull if the image if BGR or RGB
 */
void libv_wrapper(v::ViewerContext& v, const Image& input, int col, int row)
{
    if (input.type() == 0)
        v.add_image(col, row, v::Wrapper<v::core::ImageU8cp> (const_cast<cv::Mat&>(input)) );
    else if (input.type() == 16)
        v.add_image(col, row, v::Wrapper<v::core::ImageRGBU8cp> (const_cast<cv::Mat&>(input)) );
}

/**
 * @Brief display an opencv image
 * TODO: becarefull if the image if BGR or RGB
 */
void viewer_2d(v::ViewerContext& v, const Image& input, int col, int row)
{
    constexpr int maxw = 4080;
    constexpr int maxh = 3068;
    
    //fragment image for viewer if too big
    if (input.cols >  maxw or input.rows > maxh)
    {
    	const int cropfactor = std::max(
    		static_cast<int>(input.cols / maxw) + 1,
    		static_cast<int>(input.rows / maxh) + 1
    	);
    	const int stepw = static_cast<int>(input.cols / cropfactor);
    	const int steph = static_cast<int>(input.rows / cropfactor);
    	
    	for (int i = 0; i < cropfactor; ++i)
    	{
    		for (int j = 0; j < cropfactor; ++j)
    		{
    			cv::Rect roi(i * stepw, j * steph, stepw, steph);
    			const Image iroi = input(roi);
    			
    			libv_wrapper(v, iroi, col + i * stepw, row + j * steph);   			
    		}
    	}
    }    
    else libv_wrapper(v, input, col, row);
    
    v.update();
}

void viewer_2d(v::ViewerContext& v, const std::vector<cv::Point2f>& ps)
{
    for (auto& p : ps)
        v.add_point(p.x, p.y);
    v.update();
}

/**
 * @Brief display some P2DS
 */
void viewer_2d(v::ViewerContext& v, const P2D& p)
{
    v.add_point(p[0], p[1]);
    v.update();
}

/**
 * @Brief display some P2DS
 */
void viewer_2d(v::ViewerContext& v, const P2DS& ps)
{
    for (const P2D& p : ps)
        v.add_point(p[0], p[1]);
    v.update();
}

void viewer_2d(v::ViewerContext& v, const Ray2D& r, double d)
{
    const P2D point = r(d);
    v.add_line(r.origin()[0], r.origin()[1], point[0], point[1])
     .update();
}

void viewer_2d(v::ViewerContext& v, const Disk& d)
{
    v.add_point(d.center[0], d.center[1])
     .add_circle(d.center[0], d.center[1], d.radius)
     .update();
}

/**
 * @Brief viewer_2d draw a 2d grid
 */
void viewer_2d(v::ViewerContext& v, const GridMesh2D& gm)
{
    for (const P2D& node : gm)
        v.add_point(node[0], node[1]);
    v.update();
}

void viewer_2d(v::ViewerContext& v, const MIA& mia)
{
    for (std::size_t k = 0; k < mia.width(); ++k)
    {
    	for (std::size_t l = 0; l < mia.height(); ++l)
    	{
    		const P2D p = mia.nodeInWorld(k, l);
    		v.add_point(p[0], p[1]);
    		
    		for (std::size_t nk = std::min(0ul, k-1); nk <= std::min(k+1, mia.width()-1); ++nk)
    		{
    			for (std::size_t nl = std::min(0ul, l-1); nl <= std::min(l+1, mia.height()-1); ++nl)
    			{
    				const P2D n = mia.nodeInWorld(nk,nl);
    				if ((n-p).norm() < mia.diameter()*1.1)
    				{
    					v.add_line(p[0], p[1], n[0], n[1]);
    				}
    			}
    		}   			
    	}
    }
    
    v.update();
}