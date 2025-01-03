#ifndef MOVING_AVERAGE_H_INCLUDED
#define MOVING_AVERAGE_H_INCLUDED

#define MOVING_AVERAGE_SIZE 10

#include "APDB.h"

// to use this moving average filter, create a "moving_average_info" struct to store the data points\
// call initialize_moving_average_struct on this once to initialize it
// and when you get a new data point, do filtered_point = filter_point(unfiltered_point, &info_struct)
// to get the filtered point and update the struct


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