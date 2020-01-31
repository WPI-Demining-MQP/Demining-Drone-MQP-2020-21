#include "Minefield.h"

mine_t mines[MAX_NUM_MINES];
uint16_t mines_index;
uint16_t num_mines;

// Distance between two lat/lon points
// Uses the Haversine formula
double dist_to(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    // Convert from deg*1E7 to radians
    double phi1 = deg_to_rad(lat1) / 1.0E7, phi2 = deg_to_rad(lat2) / 1.0E7;
    double lambda1 = deg_to_rad(lon1) / 1.0E7, lambda2 = deg_to_rad(lon2) / 1.0E7;
    double a = pow(sin((phi2 - phi1) / 2.0), 2) + cos(phi1) * cos(phi2) * pow(sin((lambda2 - lambda1) / 2.0), 2);
    return EARTH_RADIUS * 2.0 * atan2(sqrt(a), sqrt(1 - a));
}