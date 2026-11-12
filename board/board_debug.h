/* 板级开发配置：修改宏之后必须重新编译、烧录，不能在Watch里改宏。 */
#ifndef BOARD_DEBUG_H
#define BOARD_DEBUG_H

/* 1：Debug构建启用调试坐标入口；0：彻底关闭入口。
 * CMake只为Debug定义ROBOT_DEBUG_BUILD；Release不会保留入口。
 * 上电command仍为0；打开宏本身不会使能或运动。
 */
#define BOARD_DEBUG_POSITION_ENABLE 1

#if BOARD_DEBUG_POSITION_ENABLE != 0 && BOARD_DEBUG_POSITION_ENABLE != 1
#error "BOARD_DEBUG_POSITION_ENABLE must be 0 or 1"
#endif

#if BOARD_DEBUG_POSITION_ENABLE && defined(ROBOT_DEBUG_BUILD)
#define ROBOT_DEBUG_POSITION_ENABLED 1
#else
#define ROBOT_DEBUG_POSITION_ENABLED 0
#endif

#endif
