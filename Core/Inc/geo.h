/* geo.h - local tangent-plane projection (equirectangular ENU) */
#ifndef GEO_H
#define GEO_H

void GEO_SetOrigin(double lat_deg, double lon_deg);
void GEO_ToENU(double lat_deg, double lon_deg, float *east_m, float *north_m);

#endif /* GEO_H */
