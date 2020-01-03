#include "PathPlan.h"

// Structured mine array
mine_t mines[(MAX_NUM_MINES / MINES_PER_RUN) + 1][MINES_PER_RUN];
uint8_t mines_index_row = 0;
uint8_t mines_index_col = 0;
uint16_t num_mines = 0;

void LL_add(node_t** head_ref, mine_t* mine_ptr) {
    struct node_t* new_node = (struct node_t*)malloc(NODE_T_SIZE);
    new_node->mine = mine_ptr;
    new_node->next = NULL;
    if (*head_ref == NULL) {
        *head_ref = new_node;
        return;
    }
    node_t *cur = *head_ref;
    while (cur->next != NULL) {
        cur = cur->next;
    }
    cur->next = new_node;
}

void LL_remove(node_t** head_ref, mine_t* mine_ptr) {
    if (*head_ref == NULL)
        return;   // can't remove from an empty list (head is null)

    struct node_t* cur = *head_ref;
    // if head is the node that is being removed...
    if (mine_t_equals(*(cur->mine), *mine_ptr)) {
        *head_ref = cur->next;
        free(cur);
        return;
    }

    struct node_t* prev = *head_ref;
    cur = (*head_ref)->next;
    while (!mine_t_equals(*(cur->mine), *mine_ptr)) {
        prev = cur;
        cur = cur->next;
    }
    prev->next = cur->next;
    free(cur);
    return;
}

// Definition of equality for two mines
// Based on lat/lon
bool mine_t_equals(mine_t a, mine_t b) {
    return((a.lat == b.lat) && (a.lon == b.lon));
}

mine_t* closest_to(int32_t lat, int32_t lon, node_t* head) {
    double min_dist = dist_to(lat,lon,head->mine->lat,head->mine->lon);
    mine_t *closest_mine = head->mine;
    node_t *cur = head;
    double dist;
    while (cur->next != NULL) {
        dist = dist_to(lat, lon, cur->next->mine->lat, cur->next->mine->lon);
        if (dist < min_dist) {
            closest_mine = cur->next->mine;
            min_dist = dist;
        }
        cur = cur->next;
    }
    return closest_mine;
}

// Distance between two lat/lon points
// Uses the Haversine formula
double dist_to(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    // Convert from deg*1E7 to radians
    double phi1 = deg_to_rad(lat1)/1.0E7, phi2 = deg_to_rad(lat2)/1.0E7;
    double lambda1 = deg_to_rad(lon1) / 1.0E7, lambda2 = deg_to_rad(lon2) / 1.0E7;
    double a = pow(sin((phi2 - phi1) / 2.0), 2) + cos(phi1) * cos(phi2) * pow(sin((lambda2 - lambda1) / 2.0), 2);
    return EARTH_RADIUS * 2.0 * atan2(sqrt(a), sqrt(1 - a));
}

// Plan the path
void plan_path(int32_t home_lat, int32_t home_lon, node_t** head_ref) {
    int row = 0;
    while (*head_ref != NULL) {  // Keep going until the linked list is empty
        mine_t* primary = closest_to(home_lat, home_lon, *head_ref);    // find the closest mine in the list to the home location
        memcpy(&mines[row][0], primary, sizeof(mine_t));                // copy the info for this mine into the structured mines array
        num_mines++;                                                    // increment the total number of mines in the path
        LL_remove(head_ref, primary);                                   // remove this mine from the linked list

        for (int i = 1; i < MINES_PER_RUN; i++) {       // Find MINES_PER_RUN-1 other mines and place them in the sorted array
            mine_t* secondary = closest_to(primary->lat, primary->lon, *head_ref);
            memcpy(&mines[row][i], secondary, sizeof(mine_t));
            num_mines++;
            LL_remove(head_ref, secondary);
            if (*head_ref == NULL)   // if we run out of mines (empty linked list) then exit the loop
                break;
        }
        row++;  // once we've filled one row, increment the row counter
    }
}
