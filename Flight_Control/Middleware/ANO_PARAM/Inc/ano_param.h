#ifndef __ANO_PARAM_H__
#define __ANO_PARAM_H__

#include <stdint.h>

/*
 * ANO_PARAM 模块：ANO PID 参数帧解析 + 热更新 + 掉电保存（Middleware 层）。
 *
 * 分层契约：
 *   ANO_DT 只负责遥控帧（0x03）解析，本模块自带 PID 帧（0x10~0x15）与命令帧（0xE0）的校验
 *   和分发，避免 ANO_DT → ANO_PARAM → CONTROL → REMOTE → ANO_DT 循环依赖。
 *   回显走 ANO_DT_Send_PID（ANO_DT 新增的轻量发送接口），不反向依赖。
 *
 * 载体：EEPROM_EMU（内部 Flash 模拟），虚拟地址 0~23，全部分配给 8 组已落盘 PID
 *       （rate_x/y/z、roll/pitch/yaw、height_rate/high，每组 kp/ki/kd 各 1 个）。
 *       光流 PID（flow_speed_x/pos_x/speed_y/pos_y）约 4~8 组，无空余槽位 → 仅 RAM。
 * 编码：参数 ×1000 取 int16 存 Flash，恢复时 ÷1000。
 *       下发帧的 9 个 int16 直接回显（原值回显，不重编码，避免舍入误差）。
 *
 * 生命周期：
 *   上电 → App_Init 调 ANO_Param_Load_All，加载失败则 Control_Init 编译默认值。
 *   收到 0x10~0x11 → 热更新 RAM + 立刻写 Flash + 回显。
 *   收到 0x12 → 热更新（含 flow_speed_x）+ 写 Flash（只写高度两组）+ 回显。
 *   收到 0x13 → 热更新（flow_pos_x/flow_speed_y/flow_pos_y）+ 回显，不写 Flash。
 *   收到 0x14/0x15 → 仅回显，不应用、不保存。
 *   收到 0xE0-0x02 → Control_Init 恢复默认 + 全量写 Flash。
 *
 * 帧格式：AA AF | 功能字 | 长度 | 数据 | SUM；PID 帧总长 23 字节（数据区 9×int16 大端）。
 */

// ANO_PARAM_ST_OK：加载全部成功
#define ANO_PARAM_ST_OK 0U
// ANO_PARAM_ST_EMPTY：首启无数据
#define ANO_PARAM_ST_EMPTY 1U
// ANO_PARAM_ST_PARTIAL：部分参数缺失
#define ANO_PARAM_ST_PARTIAL 2U
// ANO_PARAM_ST_CORRUPT：存储页无效
#define ANO_PARAM_ST_CORRUPT 3U

/**
 * @brief  上电从 EEPROM 加载 8 组 PID 到 Control 全局对象
 * @param  无
 * @retval 1 全部成功；0 任一缺失（已读到的参数已生效）
 * @note   **故意使用 `ok &=` 而非 `ok &&`**：按位与不短路，保证 8 组都会尝试
 *         加载，实现"部分恢复优于全部丢弃"。返回值只表示是否全部成功。
 *         加载状态细分由 ANO_Param_Get_Load_Status 查询：
 *           OK      = 全部成功
 *           CORRUPT = EEPROM 两页均无效（NO_PAGE）
 *           EMPTY   = 首启，存储系统正常但无数据
 *           PARTIAL = 部分缺失
 */
uint8_t ANO_Param_Load_All(void);

/**
 * @brief  按 ANO 参数组号保存对应 PID 到 Flash
 * @param  group 0x10=角速度内环, 0x11=角度外环, 0x12=高度环,
 *               0x13=光流 PID（仅 RAM），0x14~0x15=仅回显
 * @retval 1 成功；0 组号非法或写入失败
 * @note   0x10 写 9 个 virt，0x11 写 9 个，0x12 写 6 个，共 24 个。
 */
uint8_t ANO_Param_Save_Group(uint8_t group);

/**
 * @brief  恢复编译默认 PID 并全量写回 Flash
 * @param  无
 * @retval 1 成功；0 失败
 * @note   流程：Control_Init() 恢复默认 → 依次 Save_Group(0x10/0x11/0x12)。
 */
uint8_t ANO_Param_Reset_All(void);

/**
 * @brief  读取最近一次 Load_All 的状态
 * @param  无
 * @retval ANO_PARAM_ST_OK / _EMPTY / _PARTIAL / _CORRUPT
 * @note   初始值为 OK，若从未调用 Load_All 也会返回 OK，可能误导；
 *         建议初始值改为"未知"或 EMPTY。
 */
uint8_t ANO_Param_Get_Load_Status(void);

/**
 * @brief  读取最近一次保存失败的虚拟地址
 * @param  无
 * @retval 0~23 = 失败地址；0xFF = 无失败（初始值）
 * @note   s_save_fail_virt 类型为 uint8_t，成功保存后不清零，
 *         会一直保留历史失败记录，诊断时需结合保存返回值判断。
 */
uint8_t ANO_Param_Get_Save_Fail_Virt(void);

/**
 * @brief  解析一帧 ANO 参数帧并分发处理
 * @param  buf 完整帧：AA AF | 功能字 | 长度 | 数据(18) | SUM
 * @param  num 帧总字节数
 * @retval 1 处理成功；0 帧非法或保存失败
 * @note   校验顺序：长度≥5 → 校验和（累加 buf[0..num-2] == buf[num-1]）
 *         → 帧头 AA AF → 按功能字分发：
 *           - 0xE0 + 子命令 0x02：恢复编译默认值并全量写 Flash；
 *           - 0x10/0x11/0x12：解析 9 个 int16 → Apply 3 个 PID → 回发 → Save；
 *           - 0x13：解析 → Apply 3 个光流 PID → 回发 → 不保存（return 1）；
 *           - 0x14/0x15：仅回发，不应用、不保存。
 */
uint8_t ANO_Param_Parse(uint8_t *buf, uint8_t num);

#endif // __ANO_PARAM_H__
