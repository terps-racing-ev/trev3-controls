#ifndef MOVING_AVERAGE_H_INCLUDED
#define MOVING_AVERAGE_H_INCLUDED

#define MOVING_AVERAGE_SIZE 10

#include "APDB.h"

struct moving_average_info {
    // input data points
    ubyte2 points[MOVING_AVERAGE_SIZE];

    // whether all points in the array have been filled 
    // (if so, we overwrite old data points with new ones)
    bool all_points_filled;

    // index of the next point to fill in
    ubyte2 next_point_index;
};

ubyte2 filter_point(ubyte2 data_point, struct moving_average_info* info_struct);

void initialize_moving_average_struct(struct moving_average_info* info_struct);



#endif