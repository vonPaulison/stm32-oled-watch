/**
 * ============================================================
 *  QMC5883P.c — 三轴磁传感器 (指南针) 驱动实现
 *
 *  通信方式: 软件 I2C, 与 OLED 共用 PB8(SCL)/PB9(SDA)
 *  ─────────────────────────────────────────────────────────
 *  为什么自己写一套 I2C 而不复用 OLED.c 的?
 *    1. OLED.c 的 I2C_SendByte 在第9个时钟"不处理应答信号",
 *       而本驱动需要检查从机 ACK 来判断模块是否在线
 *    2. OLED.c 全程只写 SDA 不读 SDA, 本驱动要读回传感器数据,
 *       需要额外的 ReceiveByte (开漏模式下释放后读 IDR)
 *    3. 两套时序操作同一对引脚, 但都在主循环里顺序执行,
 *       互不嵌套, 所以可以安全共存 (中断里没碰 I2C)
 *
 *  时序说明:
 *    GPIO 翻转速度很快, 实测 SCL 频率会超出 I2C 400kHz 规格,
 *    所以每个电平后加 QMC_I2C_Delay() 小延时, 压回规格内
 *    (若总线不稳定可加大延时计数)
 *
 *  寄存器/参数依据: QMC5883P 数据手册 Rev.A (QST 矽睿)
 *    - I2C 地址 0x2C .............. 5.4 节
 *    - 上电后需等 250us 才能通信 .. Table 7 (PORT)
 *    - 各寄存器定义 ............... 9.1 / 9.2 节
 *    - 初始化示例 ................. 7.2 节 (Continuous Setup)
 *    - 挂起模式 ................... 6.2.4 节
 *  以上寄存器布局已经 Adafruit_QMC5883P / SensorLib 两个
 *  独立驱动交叉验证一致 (2026-07 验证记录)
 * ============================================================
 */

#include "stm32f10x.h"      /*STM32 标准外设库: GPIO 读写*/
#include "QMC5883P.h"

/*══════════════════════════════════════════════════════════════
 *  硬件连接与总线参数
 *════════════════════════════════════════════════════════════*/
/*与 OLED 完全相同的两条总线 (见 OLED.c 的 OLED_GPIO_Init)*/
#define QMC_SCL_PORT    GPIOB
#define QMC_SCL_PIN     GPIO_Pin_8      /*PB8 = SCL*/
#define QMC_SDA_PORT    GPIOB
#define QMC_SDA_PIN     GPIO_Pin_9      /*PB9 = SDA*/

/*7位从机地址 0x2C (数据手册 5.4 节: "The default I2C address
 *for QMC5883P is 2CH"), 左移1位后拼 R/W 位:
 *  写 = (0x2C<<1)|0 = 0x58    读 = (0x2C<<1)|1 = 0x59
 */
#define QMC_ADDR_W      0x58
#define QMC_ADDR_R      0x59

/*══════════════════════════════════════════════════════════════
 *  寄存器地址定义 (数据手册 9.1 Register Map)
 *════════════════════════════════════════════════════════════*/
#define QMC_REG_CHIP_ID 0x00    /*芯片ID, 只读, 默认 0x80*/
#define QMC_REG_DATA    0x01    /*X轴低字节 (0x01~0x06 六字节连读:
                                  01=X_LSB 02=X_MSB 03=Y_LSB
                                  04=Y_MSB 05=Z_LSB 06=Z_MSB)*/
#define QMC_REG_STATUS  0x09    /*状态: bit1=OVFL bit0=DRDY*/
#define QMC_REG_CTRL1   0x0A    /*[7:6]OSR2 [5:4]OSR1 [3:2]ODR [1:0]MODE*/
#define QMC_REG_CTRL2   0x0B    /*[7]软复位 [6]自测试 [3:2]RNG [1:0]Set/Reset*/
#define QMC_REG_SIGN    0x29    /*轴符号配置 (官方示例用, 手册 7.2 节)*/

/*CTRL1 各字段编码 (手册 Table 17)*/
#define QMC_MODE_SUSPEND    0x00    /*挂起: 功耗最低, 寄存器保持*/
#define QMC_MODE_NORMAL     0x01    /*正常: 连续测量, 间歇省电*/
#define QMC_MODE_SINGLE     0x02    /*单次: 测一次自动回挂起*/
#define QMC_MODE_CONTINUOUS 0x03    /*连续: 不休眠, ODR 最大*/

#define QMC_ODR_10HZ    (0x00 << 2)     /*输出数据率 10Hz*/
#define QMC_ODR_50HZ    (0x01 << 2)
#define QMC_ODR_100HZ   (0x02 << 2)
#define QMC_ODR_200HZ   (0x03 << 2)

#define QMC_OSR_8       (0x00 << 4)     /*过采样率 8 次 (噪声最小, 电流最大)*/
#define QMC_OSR_4       (0x01 << 4)
#define QMC_OSR_2       (0x02 << 4)
#define QMC_OSR_1       (0x03 << 4)

/*CTRL2 量程编码 (手册 Table 18, 已三方交叉验证列序) + 对应灵敏度*/
#define QMC_RNG_30G     (0x00 << 2)     /*±30G, 1000 LSB/Gauss*/
#define QMC_RNG_12G     (0x01 << 2)     /*±12G, 2500 LSB/Gauss*/
#define QMC_RNG_8G      (0x02 << 2)     /*±8G,  3750 LSB/Gauss*/
#define QMC_RNG_2G      (0x03 << 2)     /*±2G,  15000 LSB/Gauss*/

#define QMC_SR_ON       0x00    /*Set/Reset 都开 (消磁校准最彻底, 默认推荐)*/

/*状态寄存器位 (手册 9.2.2 节)*/
#define QMC_STATUS_DRDY 0x01    /*1=三轴新数据已就绪 (读状态寄存器后自动清零)*/
#define QMC_STATUS_OVFL 0x02    /*1=任一轴溢出 (读后自动清零)*/

/*══════════════════════════════════════════════════════════════
 *  底层 I2C 时序 (开漏总线, 写1=释放靠上拉拉高, 写0=拉低)
 *════════════════════════════════════════════════════════════*/

/**
 * I2C 时序小延时
 * GPIO 翻转在 72MHz 下会超过芯片 400kHz 的 I2C 规格,
 * 用短忙等把 SCL 压回 ~100kHz (标准模式)
 * ── 为什么要压这么低 ──
 * 数据手册 Table 8: 标准模式下从机在时钟下降沿后最长
 * 3.45us 才更新 ACK/数据 (tVD;ACK / tVD;DAT 上限),
 * 太快的时钟会采样到"还没就位"的电平 → 误判 NACK/读错位
 * volatile 防止被编译器优化掉; 若总线不稳可继续调大
 */
static void QMC_I2C_Delay(void)
{
    volatile uint8_t t = 40;
    while (t--);
}

/**
 * 释放/拉高 SCL (开漏模式下写1 = 释放总线, 由上拉电阻拉高)
 */
static void QMC_SCL_1(void)
{
    GPIO_SetBits(QMC_SCL_PORT, QMC_SCL_PIN);
    QMC_I2C_Delay();
}

/**
 * 拉低 SCL
 */
static void QMC_SCL_0(void)
{
    GPIO_ResetBits(QMC_SCL_PORT, QMC_SCL_PIN);
    QMC_I2C_Delay();
}

/**
 * 释放/拉高 SDA
 */
static void QMC_SDA_1(void)
{
    GPIO_SetBits(QMC_SDA_PORT, QMC_SDA_PIN);
    QMC_I2C_Delay();
}

/**
 * 拉低 SDA
 */
static void QMC_SDA_0(void)
{
    GPIO_ResetBits(QMC_SDA_PORT, QMC_SDA_PIN);
    QMC_I2C_Delay();
}

/**
 * 读 SDA 当前电平
 * 开漏模式下先写1释放总线, IDR 就是真实线电平
 * (被从机拉低=0, 上拉拉高=1)
 * @return 1=高电平, 0=低电平
 */
static uint8_t QMC_SDA_Read(void)
{
    return (GPIO_ReadInputDataBit(QMC_SDA_PORT, QMC_SDA_PIN) == Bit_SET) ? 1 : 0;
}

/**
 * I2C 起始信号: SCL 高电平期间 SDA 由高变低
 */
static void QMC_I2C_Start(void)
{
    QMC_SDA_1();    /*先释放 SDA, 保证可以从高变低*/
    QMC_SCL_1();    /*释放 SCL, 保证处于高电平*/
    QMC_SDA_0();    /*SCL 高电平期间拉低 SDA = 起始*/
    QMC_SCL_0();    /*拉低 SCL 占住总线, 方便后续时序拼接*/
}

/**
 * I2C 终止信号: SCL 高电平期间 SDA 由低变高
 */
static void QMC_I2C_Stop(void)
{
    QMC_SDA_0();    /*确保 SDA 为低*/
    QMC_SCL_1();    /*释放 SCL 到高电平*/
    QMC_SDA_1();    /*SCL 高电平期间释放 SDA = 终止*/
}

/**
 * 发送一个字节并检查从机应答
 * @param byte  要发送的字节
 * @return 1=收到 ACK (从机在拉低 SDA), 0=NACK 或总线异常
 */
static uint8_t QMC_I2C_SendByte(uint8_t byte)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8; i++)
    {
        /*高位在前, 逐位放到 SDA 上 (0x80>>i 依次取出 bit7~bit0)*/
        if (byte & (0x80 >> i))
            QMC_SDA_1();
        else
            QMC_SDA_0();

        QMC_SCL_1();    /*从机在 SCL 高电平期间采样 SDA*/
        QMC_SCL_0();    /*拉低准备发下一位*/
    }

    /*第9个时钟: 检查应答*/
    QMC_SDA_1();        /*释放 SDA, 交给从机驱动*/
    QMC_SCL_1();        /*第9个时钟高电平*/
    QMC_I2C_Delay();    /*从机置 ACK 最长需 3.45us (标准模式),
                         *不补这刀会采样过早, 把"在路上"的 ACK 误判成 NACK*/
    ack = !QMC_SDA_Read();  /*从机拉低 SDA = ACK*/
    QMC_SCL_0();

    return ack;
}

/**
 * 接收一个字节
 * @param ack  1=本字节后回 ACK (还要继续读), 0=回 NACK (读完最后一个)
 * @return 收到的字节
 */
static uint8_t QMC_I2C_ReceiveByte(uint8_t ack)
{
    uint8_t i;
    uint8_t byte = 0;

    QMC_SDA_1();        /*释放 SDA, 交给从机驱动*/
    for (i = 0; i < 8; i++)
    {
        QMC_SCL_1();                            /*SCL 高电平期间数据有效*/
        byte <<= 1;                             /*左移腾出最低位*/
        if (QMC_SDA_Read())
            byte |= 0x01;                       /*采样到高电平, 该位置1*/
        QMC_SCL_0();
    }

    /*第9个时钟: 主机回应答
     *ACK  = 拉低 SDA, 告诉从机"继续发"
     *NACK = 释放 SDA, 告诉从机"读完了" (I2C 规范要求最后一个字节 NACK)
     */
    if (ack)
        QMC_SDA_0();
    else
        QMC_SDA_1();
    QMC_SCL_1();
    QMC_SCL_0();
    QMC_SDA_1();        /*释放 SDA, 方便后面发 Stop*/

    return byte;
}

/**
 * 向指定寄存器写入一个字节
 * 时序: Start → 地址+W → ACK → 寄存器地址 → ACK → 数据 → ACK → Stop
 * @param reg   寄存器地址
 * @param val   要写入的值
 * @return 1=两步 ACK 都正常, 0=有 NACK (模块不在线等)
 */
static uint8_t QMC_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t ok;

    QMC_I2C_Start();
    ok  = QMC_I2C_SendByte(QMC_ADDR_W);
    ok &= QMC_I2C_SendByte(reg);
    ok &= QMC_I2C_SendByte(val);
    QMC_I2C_Stop();

    return ok;
}

/**
 * 从指定寄存器开始连读 len 个字节 (地址自动递增)
 * 时序: Start → 地址+W → ACK → 寄存器地址 → ACK
 *      → Restart → 地址+R → ACK → 读N-1字节(回ACK) → 读最后字节(NACK) → Stop
 * @param reg   起始寄存器地址
 * @param buf   接收缓冲区
 * @param len   读取字节数
 * @return 1=成功, 0=地址阶段 NACK
 */
static uint8_t QMC_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t ok;

    QMC_I2C_Start();
    ok = QMC_I2C_SendByte(QMC_ADDR_W);
    ok &= QMC_I2C_SendByte(reg);

    QMC_I2C_Start();            /*Repeated Start, 切换到读方向*/
    ok &= QMC_I2C_SendByte(QMC_ADDR_R);
    if (!ok)
    {
        QMC_I2C_Stop();
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        /*最后一个字节回 NACK, 其余回 ACK*/
        buf[i] = QMC_I2C_ReceiveByte((i < (len - 1)) ? 1 : 0);
    }
    QMC_I2C_Stop();

    return 1;
}

/*══════════════════════════════════════════════════════════════
 *  公共函数实现 (接口见 QMC5883P.h)
 *════════════════════════════════════════════════════════════*/

uint8_t QMC5883P_Init(void)
{
    uint8_t id = 0;

    /*释放两条总线 (开漏写1), 保证起始信号能正常产生
     *引脚模式由 OLED_Init() 配好 (Out_OD), 这里不再重复配置
     */
    QMC_SCL_1();
    QMC_SDA_1();

    /*读 CHIP_ID 验证模块在线 (数据手册 9.2.1: 默认值 0x80)
     *读到 0x80 才继续配置, 否则后面写寄存器都是无用功
     */
    if (!QMC_ReadRegs(QMC_REG_CHIP_ID, &id, 1))
        return 0;
    if (id != 0x80)
        return 0;

    /*── 按数据手册 7.2 节官方示例配置 ──
     *1) 0x29 = 0x06: 轴符号配置 (官方示例固定写法)
     *2) 0x0B = 0x08: RNG=10(±8G) + Set/Reset=00(都开)
     *   Set/Reset 开 = 每次测量前对传感器消磁复位,
     *   消除磁滞带来的误差, 精度最好 (编码已经 Adafruit 驱动
     *   与手册示例双重验证: 00=开)
     *3) 0x0A = 连续模式 + 10Hz + OSR1=8
     *   = MODE=11 | ODR=00 | OSR1=00 | OSR2=00 = 0x03
     *   10Hz 对指南针显示绰绰有余, OSR1=8 噪声最小,
     *   功耗约 78uA (手册 Table 2), 只在指南针页面开着
     */
    if (!QMC_WriteReg(QMC_REG_SIGN, 0x06))  return 0;
    if (!QMC_WriteReg(QMC_REG_CTRL2, (uint8_t)(QMC_RNG_8G | QMC_SR_ON))) return 0;
    if (!QMC_WriteReg(QMC_REG_CTRL1, (uint8_t)(QMC_MODE_CONTINUOUS | QMC_ODR_10HZ | QMC_OSR_8))) return 0;

    return 1;
}

uint8_t QMC5883P_ReadRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t status;
    uint8_t buf[6];

    /*先读状态寄存器 (注意: 读操作本身会清 DRDY, 手册 9.2.2 节)
     *DRDY=0 说明 10Hz 周期还没到, 没有新数据, 直接返回
     */
    if (!QMC_ReadRegs(QMC_REG_STATUS, &status, 1))
        return 0;
    if (!(status & QMC_STATUS_DRDY))
        return 0;

    /*溢出保护: 任一轴超出 [-30000,30000] LSB 时 OVFL 置位
     *强磁场下 (比如贴着磁铁) 会发生, 这组数据不可信, 丢弃
     */
    if (status & QMC_STATUS_OVFL)
        return 2;

    /*连读 6 字节: 01=X_LSB 02=X_MSB 03=Y_LSB 04=Y_MSB 05=Z_LSB 06=Z_MSB
     *16位补码, 低字节在前 (手册 9.2.1 节)
     */
    if (!QMC_ReadRegs(QMC_REG_DATA, buf, 6))
        return 0;

    *x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    *y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    *z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

    return 1;
}

void QMC5883P_Suspend(void)
{
    /*MODE=00 进挂起 (手册 6.2.4 节), 寄存器配置保持不变,
     *下次进指南针页重新写 CTRL1 即可唤醒
     */
    QMC_WriteReg(QMC_REG_CTRL1, QMC_MODE_SUSPEND);
}

/*══════════════════════════════════════════════════════════════
 *  诊断工具 (模块不在线时定位问题用)
 *════════════════════════════════════════════════════════════*/

uint8_t QMC5883P_ScanBus(uint8_t *found, uint8_t max)
{
    uint8_t addr;
    uint8_t count = 0;

    /*遍历合法 7 位地址空间 0x03~0x77 (0x00~0x02 和 0x78~0x7F 保留)
     *探测方式 = 只发地址字节看有无 ACK, 不碰任何寄存器, 对设备无副作用
     *本总线上必然能看到 0x3C (OLED 正在显示) ——
     *若连 0x3C 都扫不到, 说明本驱动自身时序有问题而非接线问题
     */
    for (addr = 0x03; addr <= 0x77; addr++)
    {
        if (count >= max)
            break;                          /*缓冲区满, 提前结束*/

        QMC_I2C_Start();
        if (QMC_I2C_SendByte((uint8_t)(addr << 1)))
            found[count++] = addr;          /*有 ACK = 该地址有设备*/
        QMC_I2C_Stop();
    }
    return count;
}

uint8_t QMC5883P_ReadID(void)
{
    uint8_t id = 0xFF;                      /*0xFF = 无应答时的哨兵值*/

    /*QMC_ReadRegs 失败时不写 buf, id 保持 0xFF*/
    QMC_ReadRegs(QMC_REG_CHIP_ID, &id, 1);
    return id;
}
