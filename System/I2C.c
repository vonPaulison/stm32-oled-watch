/**
 * ============================================================
 *  I2C.c — 软件 I2C 总线模块 (通用版) 实现
 *
 *  定位:   项目的第三套独立软件 I2C, 供以后新增的传感器/驱动
 *          模块使用 (接口见 I2C.h)。OLED.c 和 QMC5883P.c 各有
 *          自己的一套, 不经过本模块
 *
 *  时序来源: 从 QMC5883P.c 已上板验证过的软件 I2C 原样移植
 *          (开漏写1释放、ACK 检查、IDR 读回), 仅把引脚和从机
 *          地址参数化。~100kHz 保守时序已在同一对引脚 (PB8/PB9)
 *          上随 QMC5883P 验证可靠
 *
 *  总线特性:
 *    - 开漏输出: 写1 = 释放总线靠上拉拉高, 写0 = 主动拉低。
 *      绝不能改成推挽输出, 否则会和从机输出的低电平短路
 *    - 上拉电阻在模块板上 (OLED/QMC 模块均自带), MCU 侧不管
 *    - 与现有两个驱动一样只在主循环里使用 (TIM2 中断不碰
 *      I2C), 不同驱动对总线的使用不嵌套即无竞争
 * ============================================================
 */

#include "stm32f10x.h"      /*STM32 标准外设库: GPIO/RCC*/
#include "I2C.h"

/*══════════════════════════════════════════════════════════════
 *  硬件连接与总线参数 (换引脚只改这一组宏)
 *══════════════════════════════════════════════════════════════*/
/*与 OLED/QMC 完全相同的两条总线 (板上有上拉电阻)*/
#define I2C_SCL_PORT    GPIOB
#define I2C_SCL_PIN     GPIO_Pin_8      /*PB8 = SCL*/
#define I2C_SDA_PORT    GPIOB
#define I2C_SDA_PIN     GPIO_Pin_9      /*PB9 = SDA*/
#define I2C_GPIO_CLK    RCC_APB2Periph_GPIOB

/*I2C 时序延时计数
 *72MHz 下取 40 实测 SCL 压在 ~100kHz (标准模式), 与 QMC5883P
 *驱动同量级。总线不稳或换慢设备就调大; 想提速可调小,
 *但别低于 ~10 (GPIO 直翻会超出 400kHz 快速模式规格)
 */
#define I2C_DELAY_COUNT 40

/*7位地址左移1位拼 R/W 位:
 *  写 = (Addr<<1)|0    读 = (Addr<<1)|1
 */
#define I2C_ADDR_W(Addr)    ((uint8_t)((Addr) << 1))
#define I2C_ADDR_R(Addr)    ((uint8_t)(((Addr) << 1) | 0x01))

/*══════════════════════════════════════════════════════════════
 *  底层位操作 (开漏总线, 写1=释放靠上拉拉高, 写0=拉低)
 *══════════════════════════════════════════════════════════════*/

/**
 * I2C 时序小延时
 * GPIO 翻转在 72MHz 下会超出芯片 I2C 规格, 用短忙等把 SCL
 * 压回 ~100kHz (标准模式)
 * ── 为什么要压这么低 ──
 * 数据手册对标准模式从机的规定: 时钟下降沿后最长 3.45us 才
 * 更新 ACK/数据 (tVD;ACK / tVD;DAT 上限), 太快的时钟会采样到
 * "还没就位"的电平 → 误判 NACK/读错位
 * volatile 防止被编译器优化掉; 用 32 位计数, 调大宏也不会溢出
 */
static void I2C_Delay(void)
{
    volatile uint32_t t = I2C_DELAY_COUNT;
    while (t--);
}

/**
 * 释放/拉高 SCL (开漏模式下写1 = 释放总线, 由上拉电阻拉高)
 */
static void I2C_SCL_1(void)
{
    GPIO_SetBits(I2C_SCL_PORT, I2C_SCL_PIN);
    I2C_Delay();
}

/**
 * 拉低 SCL
 */
static void I2C_SCL_0(void)
{
    GPIO_ResetBits(I2C_SCL_PORT, I2C_SCL_PIN);
    I2C_Delay();
}

/**
 * 释放/拉高 SDA
 */
static void I2C_SDA_1(void)
{
    GPIO_SetBits(I2C_SDA_PORT, I2C_SDA_PIN);
    I2C_Delay();
}

/**
 * 拉低 SDA
 */
static void I2C_SDA_0(void)
{
    GPIO_ResetBits(I2C_SDA_PORT, I2C_SDA_PIN);
    I2C_Delay();
}

/**
 * 读 SDA 当前电平
 * 开漏模式下先写1释放总线, IDR 就是真实线电平
 * (被从机拉低=0, 上拉拉高=1)
 * @return 1=高电平, 0=低电平
 */
static uint8_t I2C_SDA_Read(void)
{
    return (GPIO_ReadInputDataBit(I2C_SDA_PORT, I2C_SDA_PIN) == Bit_SET) ? 1 : 0;
}

/*══════════════════════════════════════════════════════════════
 *  总线恢复 (I2C_BusInit 调用)
 *══════════════════════════════════════════════════════════════*/

/**
 * 解救被从机钳死的总线
 * 场景: MCU 复位/掉电重启时从机正发到一半, 它会一直钳住 SDA
 * 等不存在的时钟, 之后所有 Start 都发不出来 (Start 要求 SDA
 * 能由高变低)。标准解法: 手动补 9 个 SCL 时钟让它把当前字节
 * 发完, SDA 自然释放, 再补一个 Stop 让总线回到空闲态
 */
static void I2C_BusRecover(void)
{
    uint8_t i;

    if (I2C_SDA_Read())
        return;                 /*SDA 已被上拉释放, 总线没有卡死*/

    for (i = 0; i < 9; i++)     /*最多 9 个时钟 (一个字节+ACK)*/
    {
        I2C_SCL_1();
        I2C_SCL_0();
        if (I2C_SDA_Read())
            break;              /*从机发完了, SDA 释放*/
    }

    /*补一个 Stop: SCL 高电平期间 SDA 由低变高*/
    I2C_SDA_0();
    I2C_SCL_1();
    I2C_SDA_1();
}

/*══════════════════════════════════════════════════════════════
 *  位级原语公共函数 (接口见 I2C.h)
 *══════════════════════════════════════════════════════════════*/

void I2C_BusInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(I2C_GPIO_CLK, ENABLE);

    /*SCL/SDA 配成开漏输出, 速度 50MHz —— 与 OLED_GPIO_Init
     *完全相同的配置, 两边谁先初始化都不冲突
     */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = I2C_SCL_PIN;
    GPIO_Init(I2C_SCL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
    GPIO_Init(I2C_SDA_PORT, &GPIO_InitStructure);

    /*先释放两条总线, 再检查有没有从机把 SDA 钳死*/
    I2C_SCL_1();
    I2C_SDA_1();
    I2C_BusRecover();
}

void I2C_Start(void)
{
    I2C_SDA_1();    /*先释放 SDA, 保证可以从高变低*/
    I2C_SCL_1();    /*释放 SCL, 保证处于高电平*/
    I2C_SDA_0();    /*SCL 高电平期间拉低 SDA = 起始*/
    I2C_SCL_0();    /*拉低 SCL 占住总线, 方便后续时序拼接*/
}

void I2C_Stop(void)
{
    I2C_SDA_0();    /*确保 SDA 为低*/
    I2C_SCL_1();    /*释放 SCL 到高电平*/
    I2C_SDA_1();    /*SCL 高电平期间释放 SDA = 终止*/
}

uint8_t I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8; i++)
    {
        /*高位在前, 逐位放到 SDA 上 (0x80>>i 依次取出 bit7~bit0)*/
        if (Byte & (0x80 >> i))
            I2C_SDA_1();
        else
            I2C_SDA_0();

        I2C_SCL_1();    /*从机在 SCL 高电平期间采样 SDA*/
        I2C_SCL_0();    /*拉低准备发下一位*/
    }

    /*第9个时钟: 检查应答*/
    I2C_SDA_1();        /*释放 SDA, 交给从机驱动*/
    I2C_SCL_1();        /*第9个时钟高电平*/
    I2C_Delay();        /*从机置 ACK 最长需 3.45us (标准模式),
                         *不补这刀会采样过早, 把"在路上"的 ACK 误判成 NACK*/
    ack = !I2C_SDA_Read();  /*从机拉低 SDA = ACK*/
    I2C_SCL_0();

    return ack;
}

uint8_t I2C_ReceiveByte(uint8_t Ack)
{
    uint8_t i;
    uint8_t Byte = 0;

    I2C_SDA_1();        /*释放 SDA, 交给从机驱动*/
    for (i = 0; i < 8; i++)
    {
        I2C_SCL_1();                            /*SCL 高电平期间数据有效*/
        Byte <<= 1;                             /*左移腾出最低位*/
        if (I2C_SDA_Read())
            Byte |= 0x01;                       /*采样到高电平, 该位置1*/
        I2C_SCL_0();
    }

    /*第9个时钟: 主机回应答
     *ACK  = 拉低 SDA, 告诉从机"继续发"
     *NACK = 释放 SDA, 告诉从机"读完了" (I2C 规范要求最后一个字节 NACK)
     */
    if (Ack)
        I2C_SDA_0();
    else
        I2C_SDA_1();
    I2C_SCL_1();
    I2C_SCL_0();
    I2C_SDA_1();        /*释放 SDA, 方便后面发 Stop*/

    return Byte;
}

/*══════════════════════════════════════════════════════════════
 *  寄存器级便捷函数 (接口见 I2C.h)
 *══════════════════════════════════════════════════════════════*/

uint8_t I2C_WriteReg(uint8_t Addr, uint8_t Reg, uint8_t Val)
{
    uint8_t ok;

    I2C_Start();
    ok  = I2C_SendByte(I2C_ADDR_W(Addr));
    ok &= I2C_SendByte(Reg);
    ok &= I2C_SendByte(Val);
    I2C_Stop();

    return ok;
}

uint8_t I2C_ReadRegs(uint8_t Addr, uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    uint8_t i;
    uint8_t ok;

    I2C_Start();
    ok = I2C_SendByte(I2C_ADDR_W(Addr));
    ok &= I2C_SendByte(Reg);

    I2C_Start();            /*Repeated Start, 切换到读方向*/
    ok &= I2C_SendByte(I2C_ADDR_R(Addr));
    if (!ok)
    {
        I2C_Stop();         /*地址阶段就被 NACK, 直接收场*/
        return 0;
    }

    for (i = 0; i < Len; i++)
    {
        /*最后一个字节回 NACK, 其余回 ACK*/
        Buf[i] = I2C_ReceiveByte((i < (Len - 1)) ? 1 : 0);
    }
    I2C_Stop();

    return 1;
}

uint8_t I2C_ReadReg(uint8_t Addr, uint8_t Reg, uint8_t *Val)
{
    return I2C_ReadRegs(Addr, Reg, Val, 1);
}

/*══════════════════════════════════════════════════════════════
 *  总线诊断 (接口见 I2C.h)
 *══════════════════════════════════════════════════════════════*/

uint8_t I2C_IsDeviceOnline(uint8_t Addr)
{
    uint8_t ack;

    I2C_Start();
    ack = I2C_SendByte(I2C_ADDR_W(Addr));
    I2C_Stop();

    return ack;
}

uint8_t I2C_ScanBus(uint8_t *Found, uint8_t Max)
{
    uint8_t addr;
    uint8_t count = 0;

    /*遍历合法 7 位地址空间 0x03~0x77 (0x00~0x02 和 0x78~0x7F 保留)
     *探测方式 = 只发地址字节看有无 ACK, 不碰任何寄存器, 对设备无副作用
     *本总线必然能看到 0x3C (OLED) 和 0x2C (QMC5883P) ——
     *若连 0x3C 都扫不到, 说明本模块自身时序有问题而非接线问题
     */
    for (addr = 0x03; addr <= 0x77; addr++)
    {
        if (count >= Max)
            break;                          /*缓冲区满, 提前结束*/

        if (I2C_IsDeviceOnline(addr))
            Found[count++] = addr;          /*有 ACK = 该地址有设备*/
    }
    return count;
}
