#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>

/* GCC compatibility for Keil __packed attribute */
#ifndef __packed
#define __packed __attribute__((packed))
#endif

typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif



