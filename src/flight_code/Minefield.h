#ifndef MINEFIELD_H
#define MINEFIELD_H

#include <Arduino.h>

#define MAX_NUM_MINES 255   // Maximum number of mines that can be stored at once. This can theoretically be as high as 2^16-1 , though we only have memory for ~15k
#define MINES_PER_RUN 6     // Number of mines that can be detonated in one run (number of payloads that can be carried)
#define deg_to_rad(deg) deg * PI/180.0
#define EARTH_RADIUS 6371000.0 // 6371 km

// Mine datatype
struct mine_t {
    int32_t lat;
    int32_t lon;
    int32_t escape_lat;
    int32_t escape_lon;
};

extern mine_t mines[MAX_NUM_MINES];
extern uint16_t mines_index;
extern uint16_t num_mines;

double dist_to(int32_t, int32_t, int32_t, int32_t);     // Distance between two lat/lon points (haversine formula)

#endif
