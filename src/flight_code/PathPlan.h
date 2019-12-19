#ifndef PATHPLAN_H
#define PATHPLAN_H

#include <Arduino.h>
#include <math.h>

#define PI 3.14159265358979311599796346854
#define EARTH_RADIUS 6371000.0 // 6371 km
#define MINES_PER_RUN 6
#define MAX_NUM_MINES 256
#define degToRad(deg) deg * PI/180.0
#define NUM_MINES 64

// Mine datatype
struct mine_t {
    int32_t lat;
    int32_t lon;
    bool isDetonated;
};

// Node datatype for linked list
struct node_t {
    mine_t* mine;
    struct node_t* next;
};

// Structured mine array
extern mine_t mines[(MAX_NUM_MINES / MINES_PER_RUN) + 1][MINES_PER_RUN];

void LL_add(node_t**, mine_t*);
void LL_remove(node_t**, mine_t*);
bool mine_t_equals(mine_t, mine_t);
mine_t* closest_to(int32_t, int32_t, node_t*);
double dist_to(int32_t, int32_t, int32_t, int32_t);
void plan_path(int32_t, int32_t, node_t**);

#endif