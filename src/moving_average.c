#include "moving_average.h"

void initialize_moving_average_struct(struct moving_average_info* info_struct) {
    info_struct->all_points_filled = FALSE;
    info_struct->next_point_index = 0;
}

ubyte2 filter_point(ubyte2 data_point, struct moving_average_info* info_struct) {
    // add the new data point to the list
    info_struct->points[info_struct->next_point_index] = data_point;

    // wrap around the index of the next point if necessary
    (info_struct->next_point_index)++;
    if (info_struct->next_point_index >= MOVING_AVERAGE_SIZE) {
        info_struct->next_point_index = 0;
        info_struct->all_points_filled = TRUE;
    }

    ubyte4 sum = 0;
    ubyte4 mas = MOVING_AVERAGE_SIZE;
    ubyte4 npi = info_struct->next_point_index + 1;
    if (info_struct->all_points_filled) {
        for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
            sum += info_struct->points[i];
        }

        return (ubyte2) (sum / (mas));
    } else {
        for (int i = 0; i < info_struct->next_point_index; i++) {
            sum += info_struct->points[i];
        }
        return (ubyte2) (sum / (npi));
    }


}