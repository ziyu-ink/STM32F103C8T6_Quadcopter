#ifndef __FW_VERSION_H__
#define __FW_VERSION_H__

/*
 * 固件版本标识：上电横幅与 1s 调试行都会打印，用来确认烧进去的是哪一版。
 *
 * 规则：改功能 / 修 bug 时手动递增 FW_VERSION_STR（主版本.次版本.修订号）。
 */

#define FW_VERSION_STR "0.11.2"

#endif // __FW_VERSION_H__
