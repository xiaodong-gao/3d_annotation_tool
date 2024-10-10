#ifndef MyCloudLUT_H
#define MyCloudLUT_H

#include <pcl/pcl_macros.h>

#include <pcl/impl/point_types.hpp>

const uint8_t colorLut[][3] = {{0x0e, 0xbe, 0xff}, {0x29, 0xb0, 0xf7}, {0x44, 0xa2, 0xee},
                               {0x5e, 0x95, 0xe6}, {0x79, 0x87, 0xdd}, {0x94, 0x79, 0xd5},
                               {0xaf, 0x6b, 0xcc}, {0xc9, 0x5e, 0xc4}, {0xe4, 0x50, 0xbb},
                               {0xff, 0x42, 0xb3}};

class MyCloudLUT
{

public:

    /** Get a color from the lookup table with a given id.
        *
        * The id should be less than the size of the LUT (see size()). */
    static pcl::RGB at (unsigned int color_id);

    /** Get the number of colors in the lookup table.
        *
        * Note: the number of colors is different from the number of elements
        * in the lookup table (each color is defined by three bytes). */
    static size_t size ();

    /** Get a raw pointer to the lookup table. */
    static const unsigned char* data ();

};



#endif /* MyCloudLUT_H */

