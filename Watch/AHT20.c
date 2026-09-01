/**
 * ============================================================
 *  AHT20.c — 温湿度传感器驱动实现
 *
 *  通信方式: System/I2C 软件 I2C 模块的"位级原语"层
 *  ─────────────────────────────────────────────────────────
 *  为什么走位级原语而不是寄存器级便捷函数?
 *    I2C_WriteReg()/I2C_ReadRegs() 是给"寄存器型"设备用的,
 *    时序里固定夹着一个寄存器地址字节。AHT20 是"命令式"设备:
 *      写: Start → 0x70 → 0xAC 0x33 0x00 → Stop  (没有寄存器)
 *      读: Start → 0x71 → 连读N字节 → Stop       (直接读)
 *    若硬用 I2C_ReadRegs(0x38, 0xAC, buf, 7), 总线上会变成
 *    "0x70 0xAC(被当寄存器) Restart 0x71 ..." —— AHT20 收到
 *    的是残缺命令。I2C.h 把位级原语开出来就是给这种设备用的
 *
 *  命令依据: AHT20 规格书 2024-07 版 + 三库交叉验证 (见 AHT20.h)
 *
 *  测量时序怎么配合 10ms 主循环 (与 Compass 轮询 DRDY 同套路):
 *    StartMeasure 发令即回 → 芯片忙 ~80ms (状态字 bit7=1)
 *    → 页面每个 tick 调 ReadResult, 忙则立刻返回不占时间
 *    → 就绪后连读 7 字节 + CRC 校验, 全程不阻塞主循环
 *    (想偷懒也可以触发后 Delay_ms(80) 再读, 但会卡住 UI,
 *     不推荐)
 * ============================================================
 */

#include "AHT20.h"
#include "I2C.h"
#include "Delay.h"

/*══════════════════════════════════════════════════════════════
 *  地址 / 命令 / 时序参数 (依据见各条注释)
 *══════════════════════════════════════════════════════════════*/
/*7位从机地址 0x38; 位级原语层发的是"地址+R/W"完整字节:
 *  写 = (0x38<<1)|0 = 0x70    读 = (0x38<<1)|1 = 0x71
 *  (手册图17/18 的时序图里直接标了这两个字节)
 */
#define AHT20_ADDR_W    0x70
#define AHT20_ADDR_R    0x71

/*命令字节 (手册 5.1/5.2 节 + 交叉验证, 详见 AHT20.h 文件头)*/
#define AHT20_CMD_INIT          0xBE    /*初始化, 参数 0x08 0x00*/
#define AHT20_CMD_TRIGGER       0xAC    /*触发测量, 参数 0x33 0x00*/
#define AHT20_CMD_SOFT_RESET    0xBA    /*软复位, 无参数*/

/*初始化命令参数: 参数1 的 bit3=1 使能校准计算, 其余保留写 0*/
#define AHT20_INIT_PARAM1       0x08
#define AHT20_INIT_PARAM2       0x00

/*状态字位定义 (手册表9)*/
#define AHT20_STATUS_BUSY       0x80    /*bit7: 1=测量进行中*/
#define AHT20_STATUS_CAL_EN     0x08    /*bit3: 1=校准数据已生效*/

/*命令后的等待时间, 单位 ms
 *INIT : 发完 0xBE 0x08 0x00 等 10ms (三个驱动库的经验值)
 *RESET: 发完 0xBA 等 20ms (Adafruit/DFRobot/enjoyneering 一致)
 */
#define AHT20_INIT_DELAY_MS     10
#define AHT20_RESET_DELAY_MS    20

/*一帧测量数据的长度 (手册图18): 状态(1) + 湿度20位(2.5)
 *+ 温度20位(2.5) + CRC(1) = 7 字节, 其中前 6 字节参与 CRC
 */
#define AHT20_DATA_LEN          7
#define AHT20_CRC_LEN           6

/*══════════════════════════════════════════════════════════════
 *  CRC8 校验
 *══════════════════════════════════════════════════════════════*/

/**
 * CRC8 计算 (手册 5.2 节官方示例代码原样移植)
 * - 多项式 x8+x5+x4+1 = 0x31, 初始值 0xFF, 高位在前
 * - 校验对象 = 7 字节读数的前 6 字节 (状态+湿度+温度),
 *   算出的值应等于第 7 字节, 不等 = 本帧数据在总线上受损
 * @param data 参与校验的字节序列
 * @param len  字节数
 * @return CRC8 校验值
 */
static uint8_t AHT20_CRC8(uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t byte;
    uint8_t crc = 0xFF;                     /*手册规定初始值 0xFF*/

    for (byte = 0; byte < len; byte++)
    {
        crc ^= data[byte];
        for (i = 8; i > 0; i--)
        {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/*══════════════════════════════════════════════════════════════
 *  总线收发 (拼 AHT20 的"命令式"时序, 走 System/I2C 位级原语)
 *══════════════════════════════════════════════════════════════*/

/**
 * 发送一段命令字节 (无寄存器地址阶段)
 * 时序: Start → 地址+W (0x70) → 命令+参数逐字节 → Stop
 * @param cmd 命令字节序列 (如 {0xAC, 0x33, 0x00})
 * @param len 字节数 (软复位 1 字节, 其余 3 字节)
 * @return 1=全部 ACK, 0=有 NACK (模块不在线)
 */
static uint8_t AHT20_WriteCmd(uint8_t *cmd, uint8_t len)
{
    uint8_t i;
    uint8_t ok;

    I2C_Start();
    ok = I2C_SendByte(AHT20_ADDR_W);
    for (i = 0; i < len; i++)
        ok &= I2C_SendByte(cmd[i]);
    I2C_Stop();

    return ok;
}

/**
 * 直接连读 len 个字节 (无寄存器地址阶段, 读到什么由芯片决定:
 * 读 1 字节 = 状态字, 读 7 字节 = 一帧测量数据, 同一个入口)
 * 时序: Start → 地址+R (0x71) → 连读 (末字节 NACK) → Stop
 * @param buf 接收缓冲区
 * @param len 读取字节数
 * @return 1=成功, 0=地址阶段 NACK (模块不在线)
 */
static uint8_t AHT20_ReadBytes(uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t ok;

    I2C_Start();
    ok = I2C_SendByte(AHT20_ADDR_R);
    if (!ok)
    {
        I2C_Stop();         /*地址阶段就被 NACK, 直接收场*/
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        /*最后一个字节回 NACK, 其余回 ACK (I2C 规范要求)*/
        buf[i] = I2C_ReceiveByte((i < (len - 1)) ? 1 : 0);
    }
    I2C_Stop();

    return 1;
}

/*══════════════════════════════════════════════════════════════
 *  校准管理 (Init 和 SoftReset 共用的核心逻辑)
 *══════════════════════════════════════════════════════════════*/

/**
 * 确保校准使能位已置 1
 * 手册表9: bit3=0 时输出的是 ADC 原始数据而非校准数据。
 * 出厂时该位就是 1, 但软复位后不保证, 所以统一走这里;
 * 开头的状态字读取顺便充当"模块在不在线"的探测
 * @return 1=校准使能正常, 0=模块不在线或校准无法恢复
 */
static uint8_t AHT20_EnsureCalibrated(void)
{
    uint8_t status;
    uint8_t cmd[3];

    /*读状态字 (读 1 字节 = 状态字, 无应答 = 模块不在线)*/
    if (!AHT20_ReadBytes(&status, 1))
        return 0;
    if (status & AHT20_STATUS_CAL_EN)
        return 1;               /*出厂默认: 校准已使能, 无事可做*/

    /*发初始化命令 0xBE 0x08 0x00 加载 OTP 校准参数
     *(0xBE 是 AHT20 的; 0xE1 是 AHT10 的, 三个库里只有
     * Adafruit 用 0xE1 且注释承认在新片上可能无效)
     */
    cmd[0] = AHT20_CMD_INIT;
    cmd[1] = AHT20_INIT_PARAM1;
    cmd[2] = AHT20_INIT_PARAM2;
    if (!AHT20_WriteCmd(cmd, 3))
        return 0;
    Delay_ms(AHT20_INIT_DELAY_MS);

    /*复查: 校准位应已置 1, 还是 0 说明芯片有问题*/
    if (!AHT20_ReadBytes(&status, 1))
        return 0;
    return (status & AHT20_STATUS_CAL_EN) ? 1 : 0;
}

/*══════════════════════════════════════════════════════════════
 *  公共函数实现 (接口见 AHT20.h)
 *══════════════════════════════════════════════════════════════*/

uint8_t AHT20_Init(void)
{
    /*总线初始化 (幂等): 配引脚 + 总线卡死恢复
     *手册 2.4 节注2 要求上电 5ms 内别拉高 SCL/SDA —— 实际
     *进到这里时早已过 (开机时 OLED_Init 已把引脚配好拉高)
     */
    I2C_BusInit();

    /*探测 + 确保校准使能, 一步到位*/
    return AHT20_EnsureCalibrated();
}

uint8_t AHT20_StartMeasure(void)
{
    uint8_t cmd[3];

    /*触发测量命令 (手册 5.1/5.2 节): 0xAC 0x33 0x00
     *参数 0x33 0x00 是固定写法; 发出即返回, ~80ms 后就绪
     */
    cmd[0] = AHT20_CMD_TRIGGER;
    cmd[1] = 0x33;
    cmd[2] = 0x00;

    return AHT20_WriteCmd(cmd, 3);
}

uint8_t AHT20_ReadResult(int32_t *temp_c100, uint32_t *humi_rh100)
{
    uint8_t buf[AHT20_DATA_LEN];
    uint32_t srh;
    uint32_t st;
    uint64_t tmp;

    /*第一步: 只读 1 字节状态字, 忙=1 就走人
     *手册 5.2 节注: 忙位=1 时数据还没测完, 读到的是旧值。
     *按 10ms 主循环节奏轮询, 80ms ≈ 8 个 tick 后就绪,
     *不阻塞等待不卡 UI (与 Compass 轮询 DRDY 同套路)
     */
    if (!AHT20_ReadBytes(buf, 1))
        return 0;
    if (buf[0] & AHT20_STATUS_BUSY)
        return 0;

    /*第二步: 连读 7 字节一帧数据 (手册图18 的跨字节排布):
     *  [0]状态      [1]SRH19:12  [2]SRH11:4   [3]SRH3:0+ST19:16
     *  [4]ST15:8    [5]ST7:0     [6]CRC
     */
    if (!AHT20_ReadBytes(buf, AHT20_DATA_LEN))
        return 0;

    /*CRC 校验: 前 6 字节算出的 CRC8 应等于第 7 字节
     *(手册 5.2 节; 总线受扰时能拦下坏数据, 温湿度不乱跳)
     */
    if (AHT20_CRC8(buf, AHT20_CRC_LEN) != buf[6])
        return 2;

    /*拼出两个 20 位原始值 (注意字节 3 是湿温共用的高低位拼缝):
     *湿度 SRH = buf[1] buf[2] buf[3]高4位 (湿度在前)
     *温度 ST  = buf[3]低4位 buf[4] buf[5] (温度在后)
     */
    srh = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4)
        | (uint32_t)(buf[3] >> 4);
    st  = (((uint32_t)buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8)
        | (uint32_t)buf[5];

    /*定点换算 ×100 (公式见手册 5.2 节):
     *  湿度: RH% = SRH/2^20×100      → humi = (SRH×10000) >> 20
     *  温度: T℃  = ST /2^20×200 - 50 → temp = (ST×20000)>>20 - 5000
     *先加 2^19 再右移 = 四舍五入; 20位×10000 最大约 1.05e10,
     *超出 32 位范围, 借 64 位中间量 (只有乘法和移位, 编译后
     *几条指令, 没有除法, M3 上开销很小)
     */
    tmp = (uint64_t)srh * 10000u + (1u << 19);
    *humi_rh100 = (uint32_t)(tmp >> 20);

    tmp = (uint64_t)st * 20000u + (1u << 19);
    *temp_c100 = (int32_t)(tmp >> 20) - 5000;

    return 1;
}

uint8_t AHT20_SoftReset(void)
{
    uint8_t cmd;

    /*软复位命令 0xBA 无参数 (手册未收录, 三库交叉验证一致)。
     *复位后芯片回到默认状态, 需要重新确保校准使能
     */
    cmd = AHT20_CMD_SOFT_RESET;
    if (!AHT20_WriteCmd(&cmd, 1))
        return 0;
    Delay_ms(AHT20_RESET_DELAY_MS);

    /*复位后校准使能状态不保证, 复查并按需重发初始化命令*/
    return AHT20_EnsureCalibrated();
}

uint8_t AHT20_ReadStatus(uint8_t *status)
{
    /*直接读 1 字节 = 状态字 (与 EnsureCalibrated 内部同一路径)*/
    return AHT20_ReadBytes(status, 1);
}
