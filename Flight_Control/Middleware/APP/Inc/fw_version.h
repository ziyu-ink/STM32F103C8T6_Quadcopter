#ifndef __FW_VERSION_H__
#define __FW_VERSION_H__

/*
 * 固件版本标识：上电横幅与 1s 调试行都会打印，用来确认烧进去的是哪一版。
 *
 * 规则：改功能 / 修 bug 时手动递增 FW_VERSION_STR；
 *       构建日期和时间由编译器自动填入，能精确区分“同版本号的两次编译”。
 */

// FW_VERSION_STR：语义化版本号（主版本.次版本.修订号），手动维护
#define FW_VERSION_STR "0.10.0"

// FW_BUILD_DATE / FW_BUILD_TIME：编译器自动填入的构建时间
#define FW_BUILD_DATE __DATE__
#define FW_BUILD_TIME __TIME__

#endif // __FW_VERSION_H__
