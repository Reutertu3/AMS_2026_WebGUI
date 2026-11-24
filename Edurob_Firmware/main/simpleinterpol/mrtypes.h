#ifndef _TYPES_H
#define _TYPES_H

/* std C header files */
#include <math.h> /* for M_PI */

/* local header files */
#include "mrclient_common.h"

#if __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------*/
#ifndef PI
#define PI M_PI
#endif

// Beschraenkung eines Winkels auf -PI ... +PI
#ifndef MAXPI
#define MAXPI(x) ((x) > (PI) ? ((x) - (2 * PI)) : ((x) < -PI ? ((x) + (2 * PI)) : x))
#endif

typedef struct {
    double x;
    double y;
    double theta;
} Pose_t;

typedef struct {
    double x;
    double y;
    double theta;
    double vx;
    double vy;
    double omega;
} PoseV_t;

#if __cplusplus
}
#endif

#endif /* #define _TYPES_H */
