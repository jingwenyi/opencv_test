#include "image_algorithm.h"
#include <complex>


namespace IMAGE_MOSAIC
{

#define  DUBUG

#ifdef DUBUG
static int num_image = 0;
#endif

#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f


bool Image_algorithm::Is_zero(float a)
{
	return std::fabs(a) < 1.0e-6f ? true : false;
}


/*
 *  extrapolate latitude/longitude given distances north and east
 */
void Image_algorithm::Location_offset(struct Location &loc, float ofs_north, float ofs_east)
{
    if (!Is_zero(ofs_north) || !Is_zero(ofs_east)) {
        int dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / Longitude_scale(loc);
        loc.lat += dlat;
        loc.lng += dlng;
    }
}


/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of 
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void Image_algorithm::Location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = std::cos(bearing * DEG_TO_RAD)*distance;
    float ofs_east  = std::sin(bearing * DEG_TO_RAD)*distance;
    Location_offset(loc, ofs_north, ofs_east);
}



// return bearing in centi-degrees between two locations
float Image_algorithm::Get_bearing_cd(const struct Location &loc1, const struct Location &loc2)
{
    int off_x = loc2.lng - loc1.lng;
    int off_y = (loc2.lat - loc1.lat) / Longitude_scale(loc2);
    int bearing = 9000 + std::atan2(-off_y, off_x) * 5729.57795f;
    if (bearing < 0) bearing += 36000;
    return (float)bearing / 100.0f;
}



float Image_algorithm::Constrain_float(float amt, float low, float high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


float Image_algorithm::Longitude_scale(const struct Location &loc)
{
    float scale = std::cos(loc.lat * 1.0e-7f * DEG_TO_RAD);
    return Constrain_float(scale, 0.01f, 1.0f);
}


// return distance in meters between two locations
float Image_algorithm::Get_distance(const struct Location &loc1, const struct Location &loc2)
{
    float dlat              = (float)(loc2.lat - loc1.lat);
    float dlong             = ((float)(loc2.lng - loc1.lng)) * Longitude_scale(loc2);
    return std::sqrt(std::pow(dlat, 2)  + std::pow(dlong, 2)) * LOCATION_SCALING_FACTOR;
}

} //namespace IMAGE_MOSAIC

