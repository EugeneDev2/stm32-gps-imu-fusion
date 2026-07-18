/* geo.c - local tangent-plane projection (equirectangular ENU), no HAL */
#include "geo.h"
#include <math.h>

#define GEO_EARTH_R 6378137.0
#define GEO_DEG2RAD (3.14159265358979323846 / 180.0)

static double origin_lat_rad;
static double origin_lon_rad;
static double cos_lat0 = 1.0;

void GEO_SetOrigin(double lat_deg, double lon_deg) {
    origin_lat_rad = lat_deg * GEO_DEG2RAD;
    origin_lon_rad = lon_deg * GEO_DEG2RAD;
    cos_lat0 = cos(origin_lat_rad);
}

/* east = R*(lon-lon0)*cos(lat0), north = R*(lat-lat0); валідно поблизу origin */
void GEO_ToENU(double lat_deg, double lon_deg, float *east_m, float *north_m) {
    double dlat = lat_deg * GEO_DEG2RAD - origin_lat_rad;
    double dlon = lon_deg * GEO_DEG2RAD - origin_lon_rad;
    *east_m  = (float)(GEO_EARTH_R * dlon * cos_lat0);
    *north_m = (float)(GEO_EARTH_R * dlat);
}
