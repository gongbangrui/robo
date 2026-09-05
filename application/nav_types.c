#include "nav_types.h"

navigate_category_t navigate_action_category(navigate_action_t action)
{
    if (action >= 10 && action <= 199)   return CAT_TURN;   /* LEFT_x */
    if (action >= 210 && action <= 399)  return CAT_TURN;   /* RIGHT_x */

    switch (action) {
    case FORWARD:  case BACKWARD: case AUTO:
    case SLOW:     case FAST:
    case LINE_END:
        return CAT_FWD;
    case PLATFORM: return CAT_PLAT;
    case BRIDGE:   return CAT_BRDG;
    case TRAFFIC:  case QRSCAN: case DIGIT: return CAT_VIS;
    case MOVE:     return CAT_MOVE;
    case CLIMB:    return CAT_CLIMB;
    case IR: return CAT_IR;
    default:       return CAT_NONE;
    }
}
