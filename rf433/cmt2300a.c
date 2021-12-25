/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Bright.Chen
 * @Date: 2021-06-17 00:14:18
 * @LastEditors: Bright.Chen
 * @LastEditTime: 2021-06-17 04:47:20
 */

/**
 * cmt2300a 模组
 * 
 * 
 * 
 * 
 * 
 * 
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/of_device.h>

#include "cmt2300a.h"
#include "cmt2300a_params.h"

#define CMT2300A_CNT 1
#define CMT2300A_NAME "cmt2300a"

#define CMT2300A_GPIO_H (1)
#define CMT2300A_GPIO_L (0)

typedef enum
{
    CMT2300A_CMD_SET_RECV = 0, //  切换接收模式
    CMT2300A_CMD_SEND = 1,         //  发送数据
    CMT2300A_CMD_CHANEL,         //  选择信道
    CMT2300A_CMD_GETSRRI,         //  选择信道

} CMT2300A_CMD_t;

struct cmt2300a_dev
{
    struct device *dev;     /* 设备 	 */
    struct device_node *nd; /* 设备节点 */
    int major;              /* 主设备号	*/
    dev_t devid;            /* 设备号	*/
    struct cdev cdev;       /* cdev		*/
    struct class *class;    /* 类 		*/

    int cmt_spi_csb;  //  csb
    int cmt_spi_fcsb; //  fcsb
    int cmt_spi_sclk; //  sclk
    int cmt_spi_sdio; //  sdio
    int cmt_irq;      //  模组中断
    int cmt_irq_tx;      //  模组中断

    int is_open;  //  Device is open only once.
    int is_debug; //  log开关
};

static struct cmt2300a_dev cmt2300adev;

struct rf433_cmt2300a_gpio_t
{
    int *gpio;
    char *dts_name;
    int init_stat;
    int dir; //  0 - 输入    1 - 输出
};

/*******************************************
 *      3线模拟SPI interface
 *******************************************/
void cmt_spi3_csb_out(void);
void cmt_spi3_fcsb_out(void);
void cmt_spi3_sclk_out(void);
void cmt_spi3_sdio_out(void);
void cmt_spi3_sdio_in(void);

void cmt_spi3_csb_1(void);
void cmt_spi3_csb_0(void);
void cmt_spi3_fcsb_1(void);
void cmt_spi3_fcsb_0(void);
void cmt_spi3_sclk_1(void);
void cmt_spi3_sclk_0(void);
void cmt_spi3_sdio_1(void);
void cmt_spi3_sdio_0(void);
unsigned char cmt_spi3_sdio_read(void);

void cmt_spi3_delay(void);
void cmt_spi3_delay_us(void);
void cmt_spi3_init(void);
void cmt_spi3_send(unsigned char data8);
unsigned char cmt_spi3_recv(void);
void cmt_spi3_write(unsigned char addr, unsigned char dat);
void cmt_spi3_read(unsigned char addr, unsigned char *p_dat);
void cmt_spi3_write_fifo(const unsigned char *p_buf, int len);
void cmt_spi3_read_fifo(unsigned char *p_buf, int len);

/*******************************************
 *      CMT2300A Hal Interface
 *******************************************/
unsigned char CMT2300A_ReadReg(unsigned char addr);
void CMT2300A_WriteReg(unsigned char addr, unsigned char dat);
void CMT2300A_ReadFifo(unsigned char buf[], int len);
void CMT2300A_WriteFifo(const unsigned char buf[], int len);

/*******************************************
 *      CMT2300A Control Interface
 *******************************************/
void CMT2300A_DelayMs(unsigned int ms);
void CMT2300A_SoftReset(void);
unsigned char CMT2300A_GetChipStatus(void);
bool CMT2300A_AutoSwitchStatus(unsigned char nGoCmd);
bool CMT2300A_GoSleep(void);
bool CMT2300A_GoStby(void);
bool CMT2300A_GoTFS(void);
bool CMT2300A_GoRFS(void);
bool CMT2300A_GoTx(void);
bool CMT2300A_GoRx(void);
void CMT2300A_ConfigGpio(unsigned char nGpioSel);
void CMT2300A_ConfigInterrupt(unsigned char nInt1Sel, unsigned char nInt2Sel);
void CMT2300A_SetInterruptPolar(bool bActiveHigh);
void CMT2300A_SetFifoThreshold(unsigned char nFifoThreshold);
void CMT2300A_EnableAntennaSwitch(unsigned char nMode);
void CMT2300A_EnableInterrupt(unsigned char nEnable);
void CMT2300A_EnableRxFifoAutoClear(bool bEnable);
void CMT2300A_EnableFifoMerge(bool bEnable);
void CMT2300A_EnableReadFifo(void);
void CMT2300A_EnableWriteFifo(void);
void CMT2300A_RestoreFifo(void);
unsigned char CMT2300A_ClearTxFifo(void);
unsigned char CMT2300A_ClearRxFifo(void);
unsigned char CMT2300A_ClearInterruptFlags(void);
void CMT2300A_ConfigTxDin(unsigned char nDinSel);
void CMT2300A_EnableTxDin(bool bEnable);
void CMT2300A_EnableTxDinInvert(bool bEnable);
bool CMT2300A_IsExist(void);
unsigned char CMT2300A_GetRssiCode(void);
int CMT2300A_GetRssiDBm(void);
void CMT2300A_SetFrequencyChannel(unsigned char nChann);
void CMT2300A_SetFrequencyStep(unsigned char nOffset);
void CMT2300A_SetPayloadLength(int nLength);
void CMT2300A_EnableLfosc(bool bEnable);
void CMT2300A_EnableLfoscOutput(bool bEnable);
void CMT2300A_EnableAfc(bool bEnable);
void CMT2300A_SetAfcOvfTh(unsigned char afcOvfTh);
void CMT2300A_Init(void);
bool CMT2300A_ConfigRegBank(unsigned char base_addr, const unsigned char bank[], unsigned char len);

/*******************************************
 *      RF433 Interface
 *******************************************/
void RF_Init(void);
void RF_Config(void);
void RF433_Set_RX_Mode(void);
unsigned char Radio_Recv_FixedLen(unsigned char pBuf[], unsigned char len);
void Radio_Send_FixedLen(const unsigned char pBuf[], unsigned char len);
/**********************************************************************************
***********************************************************************************
**
**                      RF433 Interface    [Start]
**
***********************************************************************************
**********************************************************************************/
void RF_Init(void)
{
    unsigned char tmp = 0;

    CMT2300A_Init();

    /* Config registers */
    CMT2300A_ConfigRegBank(CMT2300A_CMT_BANK_ADDR, g_cmt2300aCmtBank, CMT2300A_CMT_BANK_SIZE);
    CMT2300A_ConfigRegBank(CMT2300A_SYSTEM_BANK_ADDR, g_cmt2300aSystemBank, CMT2300A_SYSTEM_BANK_SIZE);
    CMT2300A_ConfigRegBank(CMT2300A_FREQUENCY_BANK_ADDR, g_cmt2300aFrequencyBank, CMT2300A_FREQUENCY_BANK_SIZE);
    CMT2300A_ConfigRegBank(CMT2300A_DATA_RATE_BANK_ADDR, g_cmt2300aDataRateBank, CMT2300A_DATA_RATE_BANK_SIZE);
    CMT2300A_ConfigRegBank(CMT2300A_BASEBAND_BANK_ADDR, g_cmt2300aBasebandBank, CMT2300A_BASEBAND_BANK_SIZE);
    CMT2300A_ConfigRegBank(CMT2300A_TX_BANK_ADDR, g_cmt2300aTxBank, CMT2300A_TX_BANK_SIZE);

    // xosc_aac_code[2:0] = 2
    tmp = (~0x07) & CMT2300A_ReadReg(CMT2300A_CUS_CMT10);
    CMT2300A_WriteReg(CMT2300A_CUS_CMT10, tmp | 0x02);

    RF_Config();
}

void RF_Config(void)
{
#ifdef ENABLE_ANTENNA_SWITCH
    unsigned char nInt2Sel;
    /* If you enable antenna switch, GPIO1/GPIO2 will output RX_ACTIVE/TX_ACTIVE,
       and it can't output INT1/INT2 via GPIO1/GPIO2 */
    CMT2300A_EnableAntennaSwitch(0);

    /* Config GPIOs */
    CMT2300A_ConfigGpio(CMT2300A_GPIO3_SEL_INT2); /* INT2 > GPIO3 */
    /* Config interrupt */
    nInt2Sel = CMT2300A_INT_SEL_PKT_DONE; /* Config INT2 */
    nInt2Sel &= CMT2300A_MASK_INT2_SEL;
    nInt2Sel |= (~CMT2300A_MASK_INT2_SEL) & CMT2300A_ReadReg(CMT2300A_CUS_INT2_CTL);
    CMT2300A_WriteReg(CMT2300A_CUS_INT2_CTL, nInt2Sel);
#else
    
#if 0
   CMT2300A_ConfigGpio(CMT2300A_GPIO1_SEL_INT1); //  只设置 GPIO1到Int1中断

    CMT2300A_ConfigInterrupt(CMT2300A_INT_SEL_PKT_DONE, /* GPIO1 > PKT_DONE */
                             CMT2300A_INT_SEL_SYNC_OK   /* GPIO2 > SYNC_OK*/
    );
#else
    

	CMT2300A_ConfigGpio(CMT2300A_GPIO1_SEL_INT1|CMT2300A_GPIO3_SEL_INT2|CMT2300A_GPIO2_SEL_DOUT); // 
	
						  

    CMT2300A_ConfigInterrupt(CMT2300A_INT_SEL_PKT_DONE,
							 CMT2300A_INT_SEL_TX_DONE);  /* GPIO3 > PKT_DONE GPIO1_C3*/ 
							 
							 
#endif

#endif


#if 0
    /* Enable interrupt */
    CMT2300A_EnableInterrupt(CMT2300A_MASK_PKT_DONE_EN);
#else
	    CMT2300A_EnableInterrupt(CMT2300A_MASK_PKT_DONE_EN |CMT2300A_MASK_TX_DONE_EN);
#endif


    /* Disable low frequency OSC calibration */
    CMT2300A_EnableLfosc(false);

    /* Use a single 64-byte FIFO for either Tx or Rx */
    //CMT2300A_EnableFifoMerge(TRUE);

    //CMT2300A_SetFifoThreshold(16); // FIFO_TH

    /* This is optional, only needed when using Rx fast frequency hopping */
    /* See AN142 and AN197 for details */
    //CMT2300A_SetAfcOvfTh(0x27);

    /* Go to sleep for configuration to take effect */
	CMT2300A_SetFrequencyStep(200);

    CMT2300A_GoSleep();
}

/**
 * Description      :       将模式切换为接收模式
 * Para             :       None
 * Return           :       None
 **/
void RF433_Set_RX_Mode(void)
{
    msleep(1);
    CMT2300A_GoStby();
    /* Must clear FIFO after enable SPI to read or write the FIFO */
    CMT2300A_EnableReadFifo();
    CMT2300A_ClearInterruptFlags();
    CMT2300A_ClearRxFifo();
	CMT2300A_ConfigInterrupt(CMT2300A_INT_SEL_PKT_DONE, /* GPIO1 > PKT_DONE */
                             CMT2300A_INT_SEL_SYNC_OK);   /* GPIO2 > SYNC_OK*/
    CMT2300A_GoRx();
    return;
}

/**
 * Description      :       读取数据
 * Para             :       buf - 要穿出的数据
 *                          len - 要读取的数据长度
 * Return           :       0   - 接收数据成功
 *                          !0  - 无任何数据
 **/
unsigned char Radio_Recv_FixedLen(unsigned char pBuf[], unsigned char len)
{

    if (gpio_get_value(cmt2300adev.cmt_irq)) /* Read INT2, PKT_DONE  */
    {
        CMT2300A_GoStby();
        CMT2300A_ReadFifo(pBuf, len);
        CMT2300A_ClearRxFifo();
        CMT2300A_ClearInterruptFlags();
        CMT2300A_GoRx();
        return 0;
    }

    return 1;
}
/**
 * Description      :       发送数据
 * Para             :       buf - 要发送的数据
 *                          len - 要发送的数据长度
 * Return           :       None
 **/
void Radio_Send_FixedLen(const unsigned char pBuf[], unsigned char len)
{

   int timeout = 200;
   int ret = 0;
    msleep(1);
    //  切换为发送模式
    CMT2300A_GoStby();
	 CMT2300A_ConfigInterrupt(CMT2300A_INT_SEL_TX_DONE,CMT2300A_INT_SEL_TX_DONE);  /* GPIO3 > PKT_DONE GPIO1_C3*/ 
    CMT2300A_ClearInterruptFlags();
    CMT2300A_ClearTxFifo();
    CMT2300A_EnableWriteFifo();
    //  发送数据
    CMT2300A_WriteFifo(pBuf, len); // 写 TX_FIFO
    CMT2300A_GoTx();               // 启动发送

    while ((timeout--) && (ret == 0))
	{
		if (gpio_get_value(cmt2300adev.cmt_irq)) /* Read INT1, TX_DONE  */
		{
			ret = 1;
			//printk("RF433 send over\n");
		}
		else
		{
			msleep(1);
		}
	}
	if(ret == 1 )
    {
		CMT2300A_ClearInterruptFlags();
		CMT2300A_GoSleep();
	}
	else
    {
	//	printk("RF433 send timeout\n");
	}

    //msleep(10);
    //RF433_Set_RX_Mode();
    return;
}

/**********************************************************************************
***********************************************************************************
**
**                      RF433 Interface    [End]
**
***********************************************************************************
**********************************************************************************/

/**********************************************************************************
***********************************************************************************
**
**                      CMT2300A Control Interface    [Start]
**
***********************************************************************************
**********************************************************************************/

void CMT2300A_DelayMs(unsigned int ms)
{
    msleep(ms);
}

/*! ********************************************************
* @name    CMT2300A_SoftReset
* @desc    Soft reset.
* *********************************************************/
void CMT2300A_SoftReset(void)
{
    CMT2300A_WriteReg(0x7F, 0xFF);
}

/*! ********************************************************
* @name    CMT2300A_GetChipStatus
* @desc    Get the chip status.
* @return
*          CMT2300A_STA_PUP
*          CMT2300A_STA_SLEEP
*          CMT2300A_STA_STBY
*          CMT2300A_STA_RFS
*          CMT2300A_STA_TFS
*          CMT2300A_STA_RX
*          CMT2300A_STA_TX
*          CMT2300A_STA_EEPROM
*          CMT2300A_STA_ERROR
*          CMT2300A_STA_CAL
* *********************************************************/
unsigned char CMT2300A_GetChipStatus(void)
{
    return CMT2300A_ReadReg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
}

/*! ********************************************************
* @name    CMT2300A_AutoSwitchStatus
* @desc    Auto switch the chip status, and 10 ms as timeout.
* @param   nGoCmd: the chip next status
* @return  true or false
* *********************************************************/
bool CMT2300A_AutoSwitchStatus(unsigned char nGoCmd)
{
#ifdef ENABLE_AUTO_SWITCH_CHIP_STATUS
    int nBegTick = CMT2300A_GetTickCount();
    unsigned char nWaitStatus;

    switch (nGoCmd)
    {
    case CMT2300A_GO_SLEEP:
        nWaitStatus = CMT2300A_STA_SLEEP;
        break;
    case CMT2300A_GO_STBY:
        nWaitStatus = CMT2300A_STA_STBY;
        break;
    case CMT2300A_GO_TFS:
        nWaitStatus = CMT2300A_STA_TFS;
        break;
    case CMT2300A_GO_TX:
        nWaitStatus = CMT2300A_STA_TX;
        break;
    case CMT2300A_GO_RFS:
        nWaitStatus = CMT2300A_STA_RFS;
        break;
    case CMT2300A_GO_RX:
        nWaitStatus = CMT2300A_STA_RX;
        break;
    }

    CMT2300A_WriteReg(CMT2300A_CUS_MODE_CTL, nGoCmd);

    while (CMT2300A_GetTickCount() - nBegTick < 10)
    {
        CMT2300A_DelayUs(100);

        if (nWaitStatus == CMT2300A_GetChipStatus())
            return true;

        if (CMT2300A_GO_TX == nGoCmd)
        {
            CMT2300A_DelayUs(100);

            if (CMT2300A_MASK_TX_DONE_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_CLR1))
                return true;
        }

        if (CMT2300A_GO_RX == nGoCmd)
        {
            CMT2300A_DelayUs(100);

            if (CMT2300A_MASK_PKT_OK_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_FLAG))
                return true;
        }
    }

    return false;

#else
    CMT2300A_WriteReg(CMT2300A_CUS_MODE_CTL, nGoCmd);
    return true;
#endif
}

/*! ********************************************************
* @name    CMT2300A_GoSleep
* @desc    Entry SLEEP mode.
* @return  true or false
* *********************************************************/
bool CMT2300A_GoSleep(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_SLEEP);
}

/*! ********************************************************
* @name    CMT2300A_GoStby
* @desc    Entry Sleep mode.
* @return  true or false
* *********************************************************/
bool CMT2300A_GoStby(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_STBY);
}

/*! ********************************************************
* @name    CMT2300A_GoTFS
* @desc    Entry TFS mode.
* @return  true or false
* *********************************************************/
bool CMT2300A_GoTFS(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_TFS);
}

/*! ********************************************************
* @name    CMT2300A_GoRFS
* @desc    Entry RFS mode.
* @return  true or false
* *********************************************************/
bool CMT2300A_GoRFS(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_RFS);
}

/*! ********************************************************
* @name    CMT2300A_GoTx
* @desc    Entry Tx mode.
* @return  true or false
* *********************************************************/
bool CMT2300A_GoTx(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_TX);
}

/*! ********************************************************
* @name    CMT2300A_GoRx
* @desc    Entry Rx mode.
* @return  true or false
* *********************************************************/
bool CMT2300A_GoRx(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_RX);
}

/*! ********************************************************
* @name    CMT2300A_ConfigGpio
* @desc    Config GPIO pins mode.
* @param   nGpioSel: GPIO1_SEL | GPIO2_SEL | GPIO3_SEL | GPIO4_SEL
*          GPIO1_SEL:
*            CMT2300A_GPIO1_SEL_DOUT/DIN 
*            CMT2300A_GPIO1_SEL_INT1
*            CMT2300A_GPIO1_SEL_INT2 
*            CMT2300A_GPIO1_SEL_DCLK
*
*          GPIO2_SEL:
*            CMT2300A_GPIO2_SEL_INT1 
*            CMT2300A_GPIO2_SEL_INT2
*            CMT2300A_GPIO2_SEL_DOUT/DIN 
*            CMT2300A_GPIO2_SEL_DCLK
*
*          GPIO3_SEL:
*            CMT2300A_GPIO3_SEL_CLKO 
*            CMT2300A_GPIO3_SEL_DOUT/DIN
*            CMT2300A_GPIO3_SEL_INT2 
*            CMT2300A_GPIO3_SEL_DCLK
*
*          GPIO4_SEL:
*            CMT2300A_GPIO4_SEL_RSTIN 
*            CMT2300A_GPIO4_SEL_INT1
*            CMT2300A_GPIO4_SEL_DOUT 
*            CMT2300A_GPIO4_SEL_DCLK
* *********************************************************/
void CMT2300A_ConfigGpio(unsigned char nGpioSel)
{
    CMT2300A_WriteReg(CMT2300A_CUS_IO_SEL, nGpioSel);
}

/*! ********************************************************
* @name    CMT2300A_ConfigInterrupt
* @desc    Config interrupt on INT1 and INT2.
* @param   nInt1Sel, nInt2Sel
*            CMT2300A_INT_SEL_RX_ACTIVE
*            CMT2300A_INT_SEL_TX_ACTIVE
*            CMT2300A_INT_SEL_RSSI_VLD
*            CMT2300A_INT_SEL_PREAM_OK
*            CMT2300A_INT_SEL_SYNC_OK
*            CMT2300A_INT_SEL_NODE_OK
*            CMT2300A_INT_SEL_CRC_OK
*            CMT2300A_INT_SEL_PKT_OK
*            CMT2300A_INT_SEL_SL_TMO
*            CMT2300A_INT_SEL_RX_TMO
*            CMT2300A_INT_SEL_TX_DONE
*            CMT2300A_INT_SEL_RX_FIFO_NMTY
*            CMT2300A_INT_SEL_RX_FIFO_TH
*            CMT2300A_INT_SEL_RX_FIFO_FULL
*            CMT2300A_INT_SEL_RX_FIFO_WBYTE
*            CMT2300A_INT_SEL_RX_FIFO_OVF
*            CMT2300A_INT_SEL_TX_FIFO_NMTY
*            CMT2300A_INT_SEL_TX_FIFO_TH
*            CMT2300A_INT_SEL_TX_FIFO_FULL
*            CMT2300A_INT_SEL_STATE_IS_STBY
*            CMT2300A_INT_SEL_STATE_IS_FS
*            CMT2300A_INT_SEL_STATE_IS_RX
*            CMT2300A_INT_SEL_STATE_IS_TX
*            CMT2300A_INT_SEL_LED
*            CMT2300A_INT_SEL_TRX_ACTIVE
*            CMT2300A_INT_SEL_PKT_DONE
* *********************************************************/
void CMT2300A_ConfigInterrupt(unsigned char nInt1Sel, unsigned char nInt2Sel)
{
    nInt1Sel &= CMT2300A_MASK_INT1_SEL;
    nInt1Sel |= (~CMT2300A_MASK_INT1_SEL) & CMT2300A_ReadReg(CMT2300A_CUS_INT1_CTL);
    CMT2300A_WriteReg(CMT2300A_CUS_INT1_CTL, nInt1Sel);

    nInt2Sel &= CMT2300A_MASK_INT2_SEL;
    nInt2Sel |= (~CMT2300A_MASK_INT2_SEL) & CMT2300A_ReadReg(CMT2300A_CUS_INT2_CTL);
    CMT2300A_WriteReg(CMT2300A_CUS_INT2_CTL, nInt2Sel);
}

/*! ********************************************************
* @name    CMT2300A_SetInterruptPolar
* @desc    Set the polarity of the interrupt.
* @param   bEnable(true): active-high (default)
*          bEnable(false): active-low
* *********************************************************/
void CMT2300A_SetInterruptPolar(bool bActiveHigh)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_INT1_CTL);

    if (bActiveHigh)
        tmp &= ~CMT2300A_MASK_INT_POLAR;
    else
        tmp |= CMT2300A_MASK_INT_POLAR;

    CMT2300A_WriteReg(CMT2300A_CUS_INT1_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_SetFifoThreshold
* @desc    Set FIFO threshold.
* @param   nFifoThreshold
* *********************************************************/
void CMT2300A_SetFifoThreshold(unsigned char nFifoThreshold)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_PKT29);

    tmp &= ~CMT2300A_MASK_FIFO_TH;
    tmp |= nFifoThreshold & CMT2300A_MASK_FIFO_TH;

    CMT2300A_WriteReg(CMT2300A_CUS_PKT29, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableAntennaSwitch
* @desc    Enable antenna switch, output TX_ACTIVE/RX_ACTIVE
*          via GPIO1/GPIO2.
* @param   nMode 
*            0: RF_SWT1_EN=1, RF_SWT2_EN=0
*               GPIO1: RX_ACTIVE, GPIO2: TX_ACTIVE
*            1: RF_SWT1_EN=0, RF_SWT2_EN=1
*               GPIO1: RX_ACTIVE, GPIO2: ~RX_ACTIVE
* *********************************************************/
void CMT2300A_EnableAntennaSwitch(unsigned char nMode)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_INT1_CTL);

    if (0 == nMode)
    {
        tmp |= CMT2300A_MASK_RF_SWT1_EN;
        tmp &= ~CMT2300A_MASK_RF_SWT2_EN;
    }
    else if (1 == nMode)
    {
        tmp &= ~CMT2300A_MASK_RF_SWT1_EN;
        tmp |= CMT2300A_MASK_RF_SWT2_EN;
    }

    CMT2300A_WriteReg(CMT2300A_CUS_INT1_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableInterrupt
* @desc    Enable interrupt.
* @param   nEnable 
*            CMT2300A_MASK_SL_TMO_EN   |
*            CMT2300A_MASK_RX_TMO_EN   |
*            CMT2300A_MASK_TX_DONE_EN  |
*            CMT2300A_MASK_PREAM_OK_EN |
*            CMT2300A_MASK_SYNC_OK_EN  |
*            CMT2300A_MASK_NODE_OK_EN  |
*            CMT2300A_MASK_CRC_OK_EN   |
*            CMT2300A_MASK_PKT_DONE_EN
* *********************************************************/
void CMT2300A_EnableInterrupt(unsigned char nEnable)
{
    CMT2300A_WriteReg(CMT2300A_CUS_INT_EN, nEnable);
}

/*! ********************************************************
* @name    CMT2300A_EnableRxFifoAutoClear
* @desc    Auto clear Rx FIFO before entry Rx mode.
* @param   bEnable(true): Enable it(default)
*          bEnable(false): Disable it
* *********************************************************/
void CMT2300A_EnableRxFifoAutoClear(bool bEnable)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);

    if (bEnable)
        tmp &= ~CMT2300A_MASK_FIFO_AUTO_CLR_DIS;
    else
        tmp |= CMT2300A_MASK_FIFO_AUTO_CLR_DIS;

    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableFifoMerge
* @desc    Enable FIFO merge.
* @param   bEnable(true): use a single 64-byte FIFO for either Tx or Rx
*          bEnable(false): use a 32-byte FIFO for Tx and another 32-byte FIFO for Rx(default)
* *********************************************************/
void CMT2300A_EnableFifoMerge(bool bEnable)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);

    if (bEnable)
        tmp |= CMT2300A_MASK_FIFO_MERGE_EN;
    else
        tmp &= ~CMT2300A_MASK_FIFO_MERGE_EN;

    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableReadFifo
* @desc    Enable SPI to read the FIFO.
* *********************************************************/
void CMT2300A_EnableReadFifo(void)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);
    tmp &= ~CMT2300A_MASK_SPI_FIFO_RD_WR_SEL;
    tmp &= ~CMT2300A_MASK_FIFO_RX_TX_SEL;
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableWriteFifo
* @desc    Enable SPI to write the FIFO.
* *********************************************************/
void CMT2300A_EnableWriteFifo(void)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);
    tmp |= CMT2300A_MASK_SPI_FIFO_RD_WR_SEL;
    tmp |= CMT2300A_MASK_FIFO_RX_TX_SEL;
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_RestoreFifo
* @desc    Restore the FIFO.
* *********************************************************/
void CMT2300A_RestoreFifo(void)
{
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_RESTORE);
}

/*! ********************************************************
* @name    CMT2300A_ClearFifo
* @desc    Clear the Tx FIFO.
* @return  FIFO flags
*            CMT2300A_MASK_RX_FIFO_FULL_FLG |
*            CMT2300A_MASK_RX_FIFO_NMTY_FLG |
*            CMT2300A_MASK_RX_FIFO_TH_FLG   |
*            CMT2300A_MASK_RX_FIFO_OVF_FLG  |
*            CMT2300A_MASK_TX_FIFO_FULL_FLG |
*            CMT2300A_MASK_TX_FIFO_NMTY_FLG |
*            CMT2300A_MASK_TX_FIFO_TH_FLG
* *********************************************************/
unsigned char CMT2300A_ClearTxFifo(void)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_FLAG);
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_TX);
    return tmp;
}

/*! ********************************************************
* @name    CMT2300A_ClearFifo
* @desc    Clear the Rx FIFO.
* @return  FIFO flags
*            CMT2300A_MASK_RX_FIFO_FULL_FLG |
*            CMT2300A_MASK_RX_FIFO_NMTY_FLG |
*            CMT2300A_MASK_RX_FIFO_TH_FLG   |
*            CMT2300A_MASK_RX_FIFO_OVF_FLG  |
*            CMT2300A_MASK_TX_FIFO_FULL_FLG |
*            CMT2300A_MASK_TX_FIFO_NMTY_FLG |
*            CMT2300A_MASK_TX_FIFO_TH_FLG
* *********************************************************/
unsigned char CMT2300A_ClearRxFifo(void)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_FLAG);
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_RX);
    return tmp;
}

/*! ********************************************************
* @name    CMT2300A_ClearInterruptFlags
* @desc    Clear all interrupt flags.
* @return  Some interrupt flags
*            CMT2300A_MASK_SL_TMO_EN    |
*            CMT2300A_MASK_RX_TMO_EN    |
*            CMT2300A_MASK_TX_DONE_EN   |
*            CMT2300A_MASK_PREAM_OK_FLG |
*            CMT2300A_MASK_SYNC_OK_FLG  |
*            CMT2300A_MASK_NODE_OK_FLG  |
*            CMT2300A_MASK_CRC_OK_FLG   |
*            CMT2300A_MASK_PKT_OK_FLG
* *********************************************************/
unsigned char CMT2300A_ClearInterruptFlags(void)
{
    unsigned char nFlag1, nFlag2;
    unsigned char nClr1 = 0;
    unsigned char nClr2 = 0;
    unsigned char nRet = 0;
    unsigned char nIntPolar;

    nIntPolar = CMT2300A_ReadReg(CMT2300A_CUS_INT1_CTL);
    nIntPolar = (nIntPolar & CMT2300A_MASK_INT_POLAR) ? 1 : 0;

    nFlag1 = CMT2300A_ReadReg(CMT2300A_CUS_INT_FLAG);
    nFlag2 = CMT2300A_ReadReg(CMT2300A_CUS_INT_CLR1);

    if (nIntPolar)
    {
        /* Interrupt flag active-low */
        nFlag1 = ~nFlag1;
        nFlag2 = ~nFlag2;
    }

    if (CMT2300A_MASK_LBD_FLG & nFlag1)
    {
        nClr2 |= CMT2300A_MASK_LBD_CLR; /* Clear LBD_FLG */
    }

    if (CMT2300A_MASK_COL_ERR_FLG & nFlag1)
    {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR; /* Clear COL_ERR_FLG by PKT_DONE_CLR */
    }

    if (CMT2300A_MASK_PKT_ERR_FLG & nFlag1)
    {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR; /* Clear PKT_ERR_FLG by PKT_DONE_CLR */
    }

    if (CMT2300A_MASK_PREAM_OK_FLG & nFlag1)
    {
        nClr2 |= CMT2300A_MASK_PREAM_OK_CLR; /* Clear PREAM_OK_FLG */
        nRet |= CMT2300A_MASK_PREAM_OK_FLG;  /* Return PREAM_OK_FLG */
    }

    if (CMT2300A_MASK_SYNC_OK_FLG & nFlag1)
    {
        nClr2 |= CMT2300A_MASK_SYNC_OK_CLR; /* Clear SYNC_OK_FLG */
        nRet |= CMT2300A_MASK_SYNC_OK_FLG;  /* Return SYNC_OK_FLG */
    }

    if (CMT2300A_MASK_NODE_OK_FLG & nFlag1)
    {
        nClr2 |= CMT2300A_MASK_NODE_OK_CLR; /* Clear NODE_OK_FLG */
        nRet |= CMT2300A_MASK_NODE_OK_FLG;  /* Return NODE_OK_FLG */
    }

    if (CMT2300A_MASK_CRC_OK_FLG & nFlag1)
    {
        nClr2 |= CMT2300A_MASK_CRC_OK_CLR; /* Clear CRC_OK_FLG */
        nRet |= CMT2300A_MASK_CRC_OK_FLG;  /* Return CRC_OK_FLG */
    }

    if (CMT2300A_MASK_PKT_OK_FLG & nFlag1)
    {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR; /* Clear PKT_OK_FLG */
        nRet |= CMT2300A_MASK_PKT_OK_FLG;    /* Return PKT_OK_FLG */
    }

    if (CMT2300A_MASK_SL_TMO_FLG & nFlag2)
    {
        nClr1 |= CMT2300A_MASK_SL_TMO_CLR; /* Clear SL_TMO_FLG */
        nRet |= CMT2300A_MASK_SL_TMO_EN;   /* Return SL_TMO_FLG by SL_TMO_EN */
    }

    if (CMT2300A_MASK_RX_TMO_FLG & nFlag2)
    {
        nClr1 |= CMT2300A_MASK_RX_TMO_CLR; /* Clear RX_TMO_FLG */
        nRet |= CMT2300A_MASK_RX_TMO_EN;   /* Return RX_TMO_FLG by RX_TMO_EN */
    }

    if (CMT2300A_MASK_TX_DONE_FLG & nFlag2)
    {
        nClr1 |= CMT2300A_MASK_TX_DONE_CLR; /* Clear TX_DONE_FLG */
        nRet |= CMT2300A_MASK_TX_DONE_EN;   /* Return TX_DONE_FLG by TX_DONE_EN */
    }

    CMT2300A_WriteReg(CMT2300A_CUS_INT_CLR1, nClr1);
    CMT2300A_WriteReg(CMT2300A_CUS_INT_CLR2, nClr2);

    if (nIntPolar)
    {
        /* Interrupt flag active-low */
        nRet = ~nRet;
    }

    return nRet;
}

/*! ********************************************************
* @name    CMT2300A_ConfigTxDin
* @desc    Used to select whether to use GPIO1 or GPIO2 or GPIO3
*          as DIN in the direct mode. It only takes effect when 
*          call CMT2300A_EnableTxDin(true) in the direct mode.
* @param   nDinSel
*            CMT2300A_TX_DIN_SEL_GPIO1
*            CMT2300A_TX_DIN_SEL_GPIO2
*            CMT2300A_TX_DIN_SEL_GPIO3
* *********************************************************/
void CMT2300A_ConfigTxDin(unsigned char nDinSel)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);
    tmp &= ~CMT2300A_MASK_TX_DIN_SEL;
    tmp |= nDinSel;
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableTxDin
* @desc    Used to change GPIO1/GPIO2/GPIO3 between DOUT and DIN.
* @param   bEnable(true): used as DIN
*          bEnable(false): used as DOUT(default)
* *********************************************************/
void CMT2300A_EnableTxDin(bool bEnable)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);

    if (bEnable)
        tmp |= CMT2300A_MASK_TX_DIN_EN;
    else
        tmp &= ~CMT2300A_MASK_TX_DIN_EN;

    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableTxDinInvert
* @desc    Used to invert DIN data in direct mode.
* @param   bEnable(true): invert DIN
*          bEnable(false): not invert DIN(default)
* *********************************************************/
void CMT2300A_EnableTxDinInvert(bool bEnable)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_INT2_CTL);

    if (bEnable)
        tmp |= CMT2300A_MASK_TX_DIN_INV;
    else
        tmp &= ~CMT2300A_MASK_TX_DIN_INV;

    CMT2300A_WriteReg(CMT2300A_CUS_INT2_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_IsExist
* @desc    Chip indentify.
* @return  true: chip is exist, false: chip not found
* *********************************************************/
bool CMT2300A_IsExist(void)
{
    unsigned char back, dat;

    back = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
    CMT2300A_WriteReg(CMT2300A_CUS_PKT17, 0xAA);

    dat = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
    CMT2300A_WriteReg(CMT2300A_CUS_PKT17, back);

    if (0xAA == dat)
        return true;
    else
        return false;
}

/*! ********************************************************
* @name    CMT2300A_GetRssiCode
* @desc    Get RSSI code.
* @return  RSSI code
* *********************************************************/
unsigned char CMT2300A_GetRssiCode(void)
{
    return CMT2300A_ReadReg(CMT2300A_CUS_RSSI_CODE);
}

/*! ********************************************************
* @name    CMT2300A_GetRssiDBm
* @desc    Get RSSI dBm.
* @return  dBm
* *********************************************************/
int CMT2300A_GetRssiDBm(void)
{
    return (int)CMT2300A_ReadReg(CMT2300A_CUS_RSSI_DBM) - 128;
}

/*! ********************************************************
* @name    CMT2300A_SetFrequencyChannel
* @desc    This defines up to 255 frequency channel
*          for fast frequency hopping operation.
* @param   nChann: the frequency channel
* *********************************************************/
void CMT2300A_SetFrequencyChannel(unsigned char nChann)
{
	CMT2300A_GoStby();
	msleep(5);
        CMT2300A_WriteReg(CMT2300A_CUS_FREQ_CHNL, nChann);
	CMT2300A_GoSleep();
	msleep(5);

	msleep(5);
	CMT2300A_GoStby();
	msleep(5);
	CMT2300A_EnableReadFifo();
	CMT2300A_ClearInterruptFlags();
	CMT2300A_ClearRxFifo();
	CMT2300A_GoRx();
	msleep(1);

}

/*! ********************************************************
* @name    CMT2300A_SetFrequencyStep
* @desc    This defines the frequency channel step size 
*          for fast frequency hopping operation. 
*          One step size is 2.5 kHz.
* @param   nOffset: the frequency step
* *********************************************************/
void CMT2300A_SetFrequencyStep(unsigned char nOffset)
{
    CMT2300A_WriteReg(CMT2300A_CUS_FREQ_OFS, nOffset);
}

/*! ********************************************************
* @name    CMT2300A_SetPayloadLength
* @desc    Set payload length.
* @param   nLength
* *********************************************************/
void CMT2300A_SetPayloadLength(int nLength)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_PKT14);

    tmp &= ~CMT2300A_MASK_PAYLOAD_LENG_10_8;
    tmp |= (nLength >> 4) & CMT2300A_MASK_PAYLOAD_LENG_10_8;
    CMT2300A_WriteReg(CMT2300A_CUS_PKT14, tmp);

    tmp = nLength & CMT2300A_MASK_PAYLOAD_LENG_7_0;
    CMT2300A_WriteReg(CMT2300A_CUS_PKT15, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableLfosc
* @desc    If you need use sleep timer, you should enable LFOSC.
* @param   bEnable(true): Enable it(default)
*          bEnable(false): Disable it
* *********************************************************/
void CMT2300A_EnableLfosc(bool bEnable)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_SYS2);

    if (bEnable)
    {
        tmp |= CMT2300A_MASK_LFOSC_RECAL_EN;
        tmp |= CMT2300A_MASK_LFOSC_CAL1_EN;
        tmp |= CMT2300A_MASK_LFOSC_CAL2_EN;
    }
    else
    {
        tmp &= ~CMT2300A_MASK_LFOSC_RECAL_EN;
        tmp &= ~CMT2300A_MASK_LFOSC_CAL1_EN;
        tmp &= ~CMT2300A_MASK_LFOSC_CAL2_EN;
    }

    CMT2300A_WriteReg(CMT2300A_CUS_SYS2, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableLfoscOutput
* @desc    LFOSC clock is output via GPIO3.
* @param   bEnable(true): Enable it
*          bEnable(false): Disable it(default)
* *********************************************************/
void CMT2300A_EnableLfoscOutput(bool bEnable)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_INT2_CTL);

    if (bEnable)
        tmp |= CMT2300A_MASK_LFOSC_OUT_EN;
    else
        tmp &= ~CMT2300A_MASK_LFOSC_OUT_EN;

    CMT2300A_WriteReg(CMT2300A_CUS_INT2_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableAfc
* @desc    AFC enable or disanble.
* @param   bEnable(true): Enable it
*          bEnable(false): Disable it(default)
* *********************************************************/
void CMT2300A_EnableAfc(bool bEnable)
{
    unsigned char tmp = CMT2300A_ReadReg(CMT2300A_CUS_FSK5);

    if (bEnable)
        tmp |= 0x10;
    else
        tmp &= ~0x10;

    CMT2300A_WriteReg(CMT2300A_CUS_FSK5, tmp);
}

/*! ********************************************************
* @name    CMT2300A_SetAfcOvfTh
* @desc    This is optional, only needed when using Rx fast frequency hopping.
* @param   afcOvfTh: AFC_OVF_TH see AN142 and AN197 for details.
* *********************************************************/
void CMT2300A_SetAfcOvfTh(unsigned char afcOvfTh)
{
    CMT2300A_WriteReg(CMT2300A_CUS_FSK4, afcOvfTh);
}

/*! ********************************************************
* @name    CMT2300A_Init
* @desc    Initialize chip status.
* *********************************************************/
void CMT2300A_Init(void)
{
    unsigned char tmp;

    CMT2300A_SoftReset();
    CMT2300A_DelayMs(20);

    CMT2300A_GoStby();

    tmp = CMT2300A_ReadReg(CMT2300A_CUS_MODE_STA);
    tmp |= CMT2300A_MASK_CFG_RETAIN;  /* Enable CFG_RETAIN */
    tmp &= ~CMT2300A_MASK_RSTN_IN_EN; /* Disable RSTN_IN */
    CMT2300A_WriteReg(CMT2300A_CUS_MODE_STA, tmp);

    tmp = CMT2300A_ReadReg(CMT2300A_CUS_EN_CTL);
    tmp |= CMT2300A_MASK_LOCKING_EN; /* Enable LOCKING_EN */
    CMT2300A_WriteReg(CMT2300A_CUS_EN_CTL, tmp);

    CMT2300A_EnableLfosc(false); /* Diable LFOSC */

    CMT2300A_ClearInterruptFlags();
}

/*! ********************************************************
* @name    CMT2300A_ConfigRegBank
* @desc    Config one register bank.
* *********************************************************/
bool CMT2300A_ConfigRegBank(unsigned char base_addr, const unsigned char bank[], unsigned char len)
{
    unsigned char i;
    for (i = 0; i < len; i++)
        CMT2300A_WriteReg(i + base_addr, bank[i]);

    return true;
}

/**********************************************************************************
***********************************************************************************
**
**                      CMT2300A Control Interface    [End]
**
***********************************************************************************
**********************************************************************************/

/**********************************************************************************
***********************************************************************************
**
**                      CMT2300A Hal Interface    [Start]
**
***********************************************************************************
**********************************************************************************/

/*! ********************************************************
* @name    CMT2300A_ReadReg
* @desc    Read the CMT2300A register at the specified address.
* @param   addr: register address
* @return  Register value
* *********************************************************/
unsigned char CMT2300A_ReadReg(unsigned char addr)
{
    unsigned char dat = 0xFF;
    cmt_spi3_read(addr, &dat);

    return dat;
}

/*! ********************************************************
* @name    CMT2300A_WriteReg
* @desc    Write the CMT2300A register at the specified address.
* @param   addr: register address
*          dat: register value
* *********************************************************/
void CMT2300A_WriteReg(unsigned char addr, unsigned char dat)
{
    cmt_spi3_write(addr, dat);
}

/*! ********************************************************
* @name    CMT2300A_ReadFifo
* @desc    Reads the contents of the CMT2300A FIFO.
* @param   buf: buffer where to copy the FIFO read data
*          len: number of bytes to be read from the FIFO
* *********************************************************/
void CMT2300A_ReadFifo(unsigned char buf[], int len)
{
    cmt_spi3_read_fifo(buf, len);
}

/*! ********************************************************
* @name    CMT2300A_WriteFifo
* @desc    Writes the buffer contents to the CMT2300A FIFO.
* @param   buf: buffer containing data to be put on the FIFO
*          len: number of bytes to be written to the FIFO
* *********************************************************/
void CMT2300A_WriteFifo(const unsigned char buf[], int len)
{
    cmt_spi3_write_fifo(buf, len);
}

/**********************************************************************************
***********************************************************************************
**
**                      CMT2300A Hal Interface    [End]
**
***********************************************************************************
**********************************************************************************/

/**********************************************************************************
***********************************************************************************
**
**                      GPIO 模拟 3线SPI    [Start]
**
***********************************************************************************
**********************************************************************************/

//  设置CMT相关引脚的输入输出方向
void cmt_spi3_csb_out(void)
{
    gpio_direction_output(cmt2300adev.cmt_spi_csb, CMT2300A_GPIO_H);
}
void cmt_spi3_fcsb_out(void)
{
    gpio_direction_output(cmt2300adev.cmt_spi_fcsb, CMT2300A_GPIO_H);
}
void cmt_spi3_sclk_out(void)
{
    gpio_direction_output(cmt2300adev.cmt_spi_sclk, CMT2300A_GPIO_H);
}
void cmt_spi3_sdio_out(void)
{
    gpio_direction_output(cmt2300adev.cmt_spi_sdio, CMT2300A_GPIO_H);
}
void cmt_spi3_sdio_in(void)
{
    gpio_direction_input(cmt2300adev.cmt_spi_sdio);
}

//  设置gpio输出状态
void cmt_spi3_csb_1(void)
{
    gpio_set_value(cmt2300adev.cmt_spi_csb, CMT2300A_GPIO_H);
}
void cmt_spi3_csb_0(void)
{
    gpio_set_value(cmt2300adev.cmt_spi_csb, CMT2300A_GPIO_L);
}

void cmt_spi3_fcsb_1(void)
{
    gpio_set_value(cmt2300adev.cmt_spi_fcsb, CMT2300A_GPIO_H);
}
void cmt_spi3_fcsb_0(void)
{
    gpio_set_value(cmt2300adev.cmt_spi_fcsb, CMT2300A_GPIO_L);
}

void cmt_spi3_sclk_1(void)
{
    gpio_set_value(cmt2300adev.cmt_spi_sclk, CMT2300A_GPIO_H);
}
void cmt_spi3_sclk_0(void)
{
    gpio_set_value(cmt2300adev.cmt_spi_sclk, CMT2300A_GPIO_L);
}

void cmt_spi3_sdio_1(void)
{
    gpio_set_value(cmt2300adev.cmt_spi_sdio, CMT2300A_GPIO_H);
}
void cmt_spi3_sdio_0(void)
{
    gpio_set_value(cmt2300adev.cmt_spi_sdio, CMT2300A_GPIO_L);
}
//  获取sdio状态
unsigned char cmt_spi3_sdio_read(void)
{
    return gpio_get_value(cmt2300adev.cmt_spi_sdio);
}

//  延时函数
void cmt_spi3_delay(void)
{
    udelay(1);
}

void cmt_spi3_delay_us(void)
{
    udelay(1);
}

void cmt_spi3_init(void)
{
    cmt_spi3_csb_1();
    cmt_spi3_csb_out();
    cmt_spi3_csb_1(); /* CSB has an internal pull-up resistor */

    cmt_spi3_sclk_0();
    cmt_spi3_sclk_out();
    cmt_spi3_sclk_0(); /* SCLK has an internal pull-down resistor */

    cmt_spi3_sdio_1();
    cmt_spi3_sdio_out();
    cmt_spi3_sdio_1();

    cmt_spi3_fcsb_1();
    cmt_spi3_fcsb_out();
    cmt_spi3_fcsb_1(); /* FCSB has an internal pull-up resistor */

    cmt_spi3_delay();
}

void cmt_spi3_send(unsigned char data8)
{
    unsigned char i;

    for (i = 0; i < 8; i++)
    {
        cmt_spi3_sclk_0();

        /* Send byte on the rising edge of SCLK */
        if (data8 & 0x80)
            cmt_spi3_sdio_1();
        else
            cmt_spi3_sdio_0();

        cmt_spi3_delay();

        data8 <<= 1;
        cmt_spi3_sclk_1();
        cmt_spi3_delay();
    }
}

unsigned char cmt_spi3_recv(void)
{
    unsigned char i;
    unsigned char data8 = 0xFF;

    for (i = 0; i < 8; i++)
    {
        cmt_spi3_sclk_0();
        cmt_spi3_delay();
        data8 <<= 1;

        cmt_spi3_sclk_1();

        /* Read byte on the rising edge of SCLK */
        if (cmt_spi3_sdio_read())
            data8 |= 0x01;
        else
            data8 &= ~0x01;

        cmt_spi3_delay();
    }

    return data8;
}

void cmt_spi3_write(unsigned char addr, unsigned char dat)
{
    cmt_spi3_sdio_1();
    cmt_spi3_sdio_out();

    cmt_spi3_sclk_0();
    cmt_spi3_sclk_out();
    cmt_spi3_sclk_0();

    cmt_spi3_fcsb_1();
    cmt_spi3_fcsb_out();
    cmt_spi3_fcsb_1();

    cmt_spi3_csb_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    /* r/w = 0 */
    cmt_spi3_send(addr & 0x7F);

    cmt_spi3_send(dat);

    cmt_spi3_sclk_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    cmt_spi3_csb_1();

    cmt_spi3_sdio_1();
    cmt_spi3_sdio_in();

    cmt_spi3_fcsb_1();
}

void cmt_spi3_read(unsigned char addr, unsigned char *p_dat)
{
    cmt_spi3_sdio_1();
    cmt_spi3_sdio_out();

    cmt_spi3_sclk_0();
    cmt_spi3_sclk_out();
    cmt_spi3_sclk_0();

    cmt_spi3_fcsb_1();
    cmt_spi3_fcsb_out();
    cmt_spi3_fcsb_1();

    cmt_spi3_csb_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    /* r/w = 1 */
    cmt_spi3_send(addr | 0x80);

    /* Must set SDIO to input before the falling edge of SCLK */
    cmt_spi3_sdio_in();

    *p_dat = cmt_spi3_recv();

    cmt_spi3_sclk_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    cmt_spi3_csb_1();

    cmt_spi3_sdio_1();
    cmt_spi3_sdio_in();

    cmt_spi3_fcsb_1();
}

void cmt_spi3_write_fifo(const unsigned char *p_buf, int len)
{
    int i;

    cmt_spi3_fcsb_1();
    cmt_spi3_fcsb_out();
    cmt_spi3_fcsb_1();

    cmt_spi3_csb_1();
    cmt_spi3_csb_out();
    cmt_spi3_csb_1();

    cmt_spi3_sclk_0();
    cmt_spi3_sclk_out();
    cmt_spi3_sclk_0();

    cmt_spi3_sdio_out();

    for (i = 0; i < len; i++)
    {
        cmt_spi3_fcsb_0();

        /* > 1 SCLK cycle */
        cmt_spi3_delay();
        cmt_spi3_delay();

        cmt_spi3_send(p_buf[i]);

        cmt_spi3_sclk_0();

        /* > 2 us */
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();

        cmt_spi3_fcsb_1();

        /* > 4 us */
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
    }

    cmt_spi3_sdio_in();

    cmt_spi3_fcsb_1();
}

void cmt_spi3_read_fifo(unsigned char *p_buf, int len)
{
    int i;

    cmt_spi3_fcsb_1();
    cmt_spi3_fcsb_out();
    cmt_spi3_fcsb_1();

    cmt_spi3_csb_1();
    cmt_spi3_csb_out();
    cmt_spi3_csb_1();

    cmt_spi3_sclk_0();
    cmt_spi3_sclk_out();
    cmt_spi3_sclk_0();

    cmt_spi3_sdio_in();

    for (i = 0; i < len; i++)
    {
        cmt_spi3_fcsb_0();

        /* > 1 SCLK cycle */
        cmt_spi3_delay();
        cmt_spi3_delay();

        p_buf[i] = cmt_spi3_recv();

        cmt_spi3_sclk_0();

        /* > 2 us */
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();

        cmt_spi3_fcsb_1();

        /* > 4 us */
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
        cmt_spi3_delay_us();
    }

    cmt_spi3_sdio_in();

    cmt_spi3_fcsb_1();
}

/**********************************************************************************
***********************************************************************************
**
**                      GPIO 模拟 3线SPI    [End]
**
***********************************************************************************
**********************************************************************************/

/************************************************************************
 * Description:     GPIO初始化
 * Param:           priv    驱动句柄
 * Return:          0 - 初始化成功
 *                  !0 - 初始化失败
 * Author:          Bright.Chen
 ************************************************************************/
static int rf433_cmt2300a_gpio_init(struct cmt2300a_dev *priv)
{
    int ret = 0;
    int i = 0;
    struct rf433_cmt2300a_gpio_t gpio_info[6];

    gpio_info[0].gpio = &priv->cmt_spi_csb;
    gpio_info[0].dts_name = "cmt-spi-csb";
    gpio_info[0].init_stat = 1;
    gpio_info[0].dir = 1;

    gpio_info[1].gpio = &priv->cmt_spi_fcsb;
    gpio_info[1].dts_name = "cmt-spi-fcsb";
    gpio_info[1].init_stat = 1;
    gpio_info[1].dir = 1;

    gpio_info[2].gpio = &priv->cmt_spi_sclk;
    gpio_info[2].dts_name = "cmt-spi-sclk";
    gpio_info[2].init_stat = 1;
    gpio_info[2].dir = 1;

    gpio_info[3].gpio = &priv->cmt_spi_sdio;
    gpio_info[3].dts_name = "cmt-spi-sdio";
    gpio_info[3].init_stat = 1;
    gpio_info[3].dir = 1;

    gpio_info[4].gpio = &priv->cmt_irq;
    gpio_info[4].dts_name = "cmt-irq";
    gpio_info[4].init_stat = 1;
    gpio_info[4].dir = 0;

    gpio_info[5].gpio = &priv->cmt_irq_tx;
    gpio_info[5].dts_name = "cmt-irq-tx";
    gpio_info[5].init_stat = 1;
    gpio_info[5].dir = 0;

    for (i = 0; i < 6; i++)
    {
        *(gpio_info[i].gpio) = of_get_named_gpio(priv->nd, gpio_info[i].dts_name, 0);
        if (*(gpio_info[i].gpio) < 0)
        {
            printk("[RF433] get gpio failed: %s\n", gpio_info[i].dts_name);
        }
        else
        {
            printk("[RF433] get %s success\n", gpio_info[i].dts_name);
        }
        if (!gpio_is_valid(*(gpio_info[i].gpio)))
        {
            printk("[RF433] %s Invalid gpio pin\n", gpio_info[i].dts_name);
            return -EINVAL;
        }
        ret = gpio_request(*(gpio_info[i].gpio), dev_name(priv->dev));
        if (ret)
        {
            printk("[RF433] %s Failed to request: %d\n", gpio_info[i].dts_name, ret);
            return -EINVAL;
        }
        //  设置输入输出模式
        if (gpio_info[i].dir == 0)
        {
            //  输入
            ret = gpio_direction_input(*(gpio_info[i].gpio));
        }
        else
        {
            //  输出
            ret = gpio_direction_output(*(gpio_info[i].gpio), gpio_info[i].init_stat);
        }

        if (ret)
        {
            printk("[RF433] %s set output Err: %d\n", gpio_info[i].dts_name, ret);
            return -EINVAL;
        }
        else
        {
            printk("[RF433]%s set output success\n", gpio_info[i].dts_name);
        }
    }

    return 0;
}

static int rf433_cmt2300a_open(struct inode *inode, struct file *filp)
{
    printk("[RF433] cmt2300a is open.\n");

    RF_Init();
    RF433_Set_RX_Mode();

    return 0;
}

static ssize_t rf433_cmt2300a_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
    int ret = 0;
    long err;
    char recv_buf[128] = {0};
    ret = Radio_Recv_FixedLen(recv_buf, 32);
    if (ret == 0)
    {
        err = copy_to_user(buf, recv_buf, 32);
    }

    return ret == 0 ? 32 : 0;
}


static long rf433_cmt2300a_rf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{   
    int ret = 0;
    int value=0;
    int rssiCode=0;
	 void __user *argp = (void __user *)arg;
	switch (cmd) {
        case CMT2300A_CMD_GETSRRI:
		//printk("CMT2300A_GetRssiDBm =%d \n",CMT2300A_GetRssiDBm());
                //return put_user(CMT2300A_GetRssiDBm(), (int __user *)arg);
                 rssiCode=CMT2300A_GetRssiCode();
		 value = CMT2300A_GetRssiDBm();
		printk("CMT2300A_GetRssiDBm =%d rssiCode=%d \n",value,rssiCode);
                if (copy_to_user(argp, &value, sizeof(int)))
                        return -EFAULT;
                break;

        default:
                return -1;
        }


    return ret;
}


static ssize_t rf433_cmt2300a_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *off)
{
    unsigned char *tmp_buf = NULL;
    long err = 0;
    tmp_buf = kzalloc(cnt + 1, GFP_KERNEL); /* 申请内存 */
    if (tmp_buf == NULL)
    {
        printk("ziven qie 0 case\n");
	return -1;
    }
    if(cnt !=0)
    {
 	   err = copy_from_user(tmp_buf, buf, cnt);
    }
    else
    {
	
 	  err = copy_from_user(tmp_buf, buf, 1);
    }
    switch (tmp_buf[0])
    {
		case CMT2300A_CMD_SET_RECV:
			RF433_Set_RX_Mode();
			break;
		case CMT2300A_CMD_SEND:
			Radio_Send_FixedLen(tmp_buf + 1, cnt - 1);
			break;
		case CMT2300A_CMD_CHANEL:
			//CMT2300A_SetFrequencyChannel(tmp_buf[1]);
			CMT2300A_SetFrequencyChannel(cnt);
			//printk("gusd channel %d\n", tmp_buf[1]);
			printk("gusd channel %d\n", cnt);
			break;
		default:
			break;
    }

    kfree(tmp_buf);

    return 0;
}

static int rf433_cmt2300a_release(struct inode *inode, struct file *filp)
{
    printk("[RF433] cmt2300a is release.\n");
    return 0;
}

static const struct of_device_id rf433_cmt2300a_dt_ids[] = {
    {
        .compatible = "rf433-cmt2300a",
    },
    {}};
MODULE_DEVICE_TABLE(of, rf433_cmt2300a_dt_ids);

/* 设备操作函数 */
static struct file_operations rf433_cmt2300a_fops = {
    .owner = THIS_MODULE,
    .open = rf433_cmt2300a_open,
    .write = rf433_cmt2300a_write,
    .read = rf433_cmt2300a_read,
    .unlocked_ioctl=rf433_cmt2300a_rf_ioctl,
    .release = rf433_cmt2300a_release,
};

static int rf433_cmt2300a_probe(struct platform_device *pdev)
{

    // int ret = 0;
    struct cmt2300a_dev *priv = &cmt2300adev;

    if (NULL == pdev)
    {
        printk("[RF433] pdev is NULL\n");
        return -EINVAL;
    }

    if (NULL == pdev->dev.of_node)
    {
        printk("[RF433] of_node is NULL\n");
        return -EINVAL;
    }
    else
    {
        printk("[RF433] of_node isn't NULL\n");
    }

    priv->nd = pdev->dev.of_node;
    /* 1、设置设备号 */
    if (priv->major)
    {
        priv->devid = MKDEV(priv->major, 0);
        register_chrdev_region(priv->devid, CMT2300A_CNT, CMT2300A_NAME);
    }
    else
    {
        alloc_chrdev_region(&priv->devid, 0, CMT2300A_CNT, CMT2300A_NAME);
        priv->major = MAJOR(priv->devid);
    }

    /* 2、注册设备      */
    cdev_init(&priv->cdev, &rf433_cmt2300a_fops);
    cdev_add(&priv->cdev, priv->devid, CMT2300A_CNT);

    /* 3、创建类      */
    priv->class = class_create(THIS_MODULE, CMT2300A_NAME);
    if (IS_ERR(priv->class))
    {
        return PTR_ERR(priv->class);
    }

    /* 4、创建设备 */
    priv->dev = device_create(priv->class, NULL, priv->devid, NULL, CMT2300A_NAME);
    if (IS_ERR(priv->dev))
    {
        return PTR_ERR(priv->dev);
    }

    //  gpio init
    rf433_cmt2300a_gpio_init(priv);
    printk("[RF433] cmt2300a driver init success.\n");
    return 0;
}

static int rf433_cmt2300a_remove(struct platform_device *pdev)
{
    // struct cmt2300a_dev *priv = platform_get_drvdata(pdev);
    struct cmt2300a_dev *priv = &cmt2300adev;
    //  free gpio
    gpio_free(priv->cmt_spi_csb);
    gpio_free(priv->cmt_spi_fcsb);
    gpio_free(priv->cmt_spi_sclk);
    gpio_free(priv->cmt_spi_sdio);
    gpio_free(priv->cmt_irq);
    gpio_free(priv->cmt_irq_tx);

    cdev_del(&priv->cdev);                               /*  删除cdev */
    unregister_chrdev_region(priv->devid, CMT2300A_CNT); /* 注销设备号 */
    device_destroy(priv->class, priv->devid);
    class_destroy(priv->class);

    return 0;
}

static struct platform_driver rf433_cmt2300a_driver = {
    .driver = {
        .name = "rf433-cmt2300a",
        .owner = THIS_MODULE,
        .of_match_table = rf433_cmt2300a_dt_ids,
    },
    .probe = rf433_cmt2300a_probe,
    .remove = rf433_cmt2300a_remove,
};

/*
 * @description	: 驱动模块加载函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init rf433_cmt2300a_init(void)
{
    return platform_driver_register(&rf433_cmt2300a_driver);
}

/*
 * @description	: 驱动模块卸载函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit rf433_cmt2300a_exit(void)
{
    platform_driver_unregister(&rf433_cmt2300a_driver);
}

module_init(rf433_cmt2300a_init);
module_exit(rf433_cmt2300a_exit);

// module_platform_driver(rf433_cmt2300a_driver);

MODULE_AUTHOR("BrightChen");
MODULE_DESCRIPTION("3irobot RF433 cmt2300a driver");
MODULE_LICENSE("GPL v2");

