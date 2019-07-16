//
// Created by Ilia.Motornyi on 16/07/2019.
//
#include "main.h"
/**
 * @defgroup  XNUCLEO53L1A1_I2CExpanders I2C expender mapping
 * ## I2C Expanders address and i/o ##
 * Digit 1 2 3 and 4 below are numbered left to right "1234" on see on display
 * ### Expander 0 ###
 * U21 A[2-0]= 001 => i2c address[7..1] 0x43 ([7..0] 0x86 )
 * @li Digit#1  gpio 0 to 6
 * @li Digit#2  gpio 7 to 13
 * @li xshut_l  gpio 14
 * @li xshut_r  gpio 15
 *
 * ### Expander 1 ###
 * U19 A[2-0]= 000 => i2c address[7..1] 0x42 ([7..0] 0x84 )
 * @li Digit#3  gpio 0 to 6
 * @li Digit#4  gpio 7 to 13
 * @li PB1      gpio 14
 * @li xshut    gpio 15
 *
 * @note The 0/1  assignment is "digit" order logical don't look for any sense as  per device address or part number
 * @{
 */
/**
 * STMPE1600  i2c Expender register read
 * @param I2cExpAddr Expender address
 * @param index      register index
 * @param data       read data buffer
 * @param n_data     number of byte to read
 * @return           of if ok else i2c I/O operation status
 */
static int _ExpanderRd(int I2cExpAddr, int index, uint8_t *data, int n_data) {

    int status;
    uint8_t RegAddr;
    RegAddr = index;
    XNUCLEO53L1A1_GetI2cBus();
    status = HAL_I2C_Master_Transmit(&hi2c1, I2cExpAddr, &RegAddr, 1, 100);
    if (!status) {
        status = HAL_I2C_Master_Receive(&hi2c1, I2cExpAddr, data, n_data, n_data * 100);
    }
    XNUCLEO53L1A1_PutI2cBus();
    return status;
}


/**
 * STMPE1600 i2c Expender register write
 * @param I2cExpAddr Expender address
 * @param index      register index
 * @param data       data buffer
 * @param n_data     number of byte to write
 * @return           of if ok else i2c I/O operation status
 */
static int _ExpanderWR(int I2cExpAddr, int index, uint8_t *data, int n_data) {

    int status;
    uint8_t RegAddr[0x10];
    RegAddr[0] = index;
    memcpy(RegAddr + 1, data, n_data);
    XNUCLEO53L1A1_GetI2cBus();
    status = HAL_I2C_Master_Transmit(&hi2c1, I2cExpAddr, RegAddr, n_data + 1, 100);
    XNUCLEO53L1A1_PutI2cBus();
    return status;
}
/**
 * Expander 0 i2c address[7..0] format
 */
#define I2cExpAddr0 ((int)(0x43*2))
/**
 * Expander 1 i2c address[7..0] format
 */
#define I2cExpAddr1 ((int)(0x42*2))
/** @} XNUCLEO53L1A1_I2CExpanders*/
/**
 * GPIO monitor pin state register
 * 16 bit register LSB at lowest offset (little endian)
 */
#define GPMR    0x10
/**
 * STMPE1600 GPIO set pin state register
 * 16 bit register LSB at lowest offset (little endian)
 */
#define GPSR    0x12
/**
 * STMPE1600 GPIO set pin direction register
 * 16 bit register LSB at lowest offset
 */
#define GPDR    0x14
/**
 * cache the full set of expanded GPIO values to avoid i2c reading
 */
static union CurIOVal_u {
    uint8_t bytes[4];   /*!<  4 bytes array i/o view */
    uint32_t u32;       /*!<  single dword i/o view */
}
/** cache the extended IO values */
        CurIOVal;



int XNUCLEO53L1A1_ResetId(int DevNo, int state) {
    int status;
    switch (DevNo) {
        case 'c' :
            CurIOVal.bytes[3] &= ~0x80u; /* bit 15 expender 1  => byte #3 */
            if (state)
                CurIOVal.bytes[3] |= 0x80u; /* bit 15 expender 1  => byte #3 */
            status = _ExpanderWR(I2cExpAddr1, GPSR + 1, &CurIOVal.bytes[3], 1);
            break;
        case 'l' :
            CurIOVal.bytes[1] &= ~0x40u; /* bit 14 expender 0 => byte #1*/
            if (state)
                CurIOVal.bytes[1] |= 0x40u; /* bit 14 expender 0 => byte #1*/
            status = _ExpanderWR(I2cExpAddr0, GPSR + 1, &CurIOVal.bytes[1], 1);
            break;
        case 'r' :
            CurIOVal.bytes[1] &= ~0x80u; /* bit 15 expender 0  => byte #1 */
            if (state)
                CurIOVal.bytes[1] |= 0x80u; /* bit 15 expender 0 => byte #1*/
            status = _ExpanderWR(I2cExpAddr0, GPSR + 1, &CurIOVal.bytes[1], 1);
            break;
        default:
            printf("Invalid DevNo %d", DevNo);
            status = -1;
            goto done;
    }
//error with valid id
    if (status) {
        printf("expander i/o error for DevNo %d state %d ", DevNo, state);
    }
    done:
    return status;
}

/**
 * Set all i2c expended gpio in one go
 * @return i/o operation status
 */
static int _ExpandersSetAllIO(void) {
    int status;
    status = _ExpanderWR(I2cExpAddr0, GPSR, &CurIOVal.bytes[0], 2);
    if (status) {
        goto done_err;
    }
    status = _ExpanderWR(I2cExpAddr1, GPSR, &CurIOVal.bytes[2], 2);
    done_err:
    return status;
}

int XNUCLEO53L1A1_Init(void) {
    int status;
    uint8_t ExpanderData[2];

    status = _ExpanderRd(I2cExpAddr0, 0, ExpanderData, 2);
    if (status != 0 || ExpanderData[0] != 0x00 || ExpanderData[1] != 0x16) {
        printf("I2C Expander @0x%02X not detected", (int) I2cExpAddr0);
        goto done_err;

    }
    status = _ExpanderRd(I2cExpAddr1, 0, ExpanderData, 2);
    if (status != 0 || ExpanderData[0] != 0x00 || ExpanderData[1] != 0x16) {
        printf("I2C Expander @0x%02X not detected", (int) I2cExpAddr1);
        goto done_err;
    }

    CurIOVal.u32 = 0x0;
    /* setup expender   i/o direction  all output but exp1 bit 14*/
    ExpanderData[0] = 0xFF;
    ExpanderData[1] = 0xFF;
    status = _ExpanderWR(I2cExpAddr0, GPDR, ExpanderData, 2);
    if (status) {
        printf("Set Expander @0x%02X DR", I2cExpAddr0);
        goto done_err;
    }
    ExpanderData[0] = 0xFF;
    ExpanderData[1] = 0xBF; // all but bit 14-15 that is pb1 and xhurt
    status = _ExpanderWR(I2cExpAddr1, GPDR, ExpanderData, 2);
    if (status) {
        printf("Set Expander @0x%02X DR", I2cExpAddr1);
        goto done_err;
    }
    /* shut down all segment and all device */
    CurIOVal.u32 = 0x7F + (0x7Fu << 7u) + (0x7Fu << 16u) + (0x7Fu << (16u + 7u));
    status = _ExpandersSetAllIO();
    if (status) {
        printf("Set initial i/o ");
    }

    done_err:
    return status;
}

/**
 * @defgroup VL53L1A1_GPIO1_MAP    VL53L1A1 Sensor to MCU interrupt mapping
 *
 * # GPIO mapping ##
 * Various options can be used to map the 3 VL53L0X interrupt lines to MCU. By default, the expansion board is configured in
 * Center on-board vl53l0x mode which means only the center device can be used in interrupt mode. To use left and/or right devices
 * in interrupt mode, it is necessary to adapt the board configuration as described below.
 * ## One interrupt line shared by all sensors ##
 * All VL53L0x GPIO1 pins (open collector outputs) are connected together on the expansion board
 * and level shifter input drives single shared interrupt line to MCU.\n
 * Solder options to operate in this mode:
 * @li U7 and U8 are fitted  (connect GPIO all together before level shifter)
 * @li U10, U11, U15 and U18 are not soldered. (disconnect all level shifted option to arduino connector)
 * @li U14/U17 to select  PA4/PC1  => EXTI4_15_IRQn / EXTI0_1_IRQn (final selection option)
 *
 * see @a #VL53L1A1_GPIO1_C_OPTION for interrupt line selection
 *
 * @note requires @a #VL53L1A1_GPIO1_SHARED to be set (to 1)
 *
 * ##  One interrupt per sensor  ##
 * To use one interrupt per device :\n
 * @li Do not define @a #VL53L1A1_GPIO1_SHARED  or set to 0
 * @li U7 and U8 must not be soldered (disconnect L and R GPIO from C before level shifter)
 *
 * ### Center on-board vl53l0x  ###
 * @li U14 (fitted by default)  CN8#3    PA4
 * @li U17 (*option)         	CN8#5    PC1
 *
 * see @ref VL53L1A1_GPIO1_C_OPTION
 *
 * ### Left satellite ###
 * @li U10 (not fitted by default)  CN5#2    PC7
 * @li U11 (*option)         		CN5#1    PA9
 *
 * see @a #VL53L1A1_GPIO1_L_OPTION
 *
 * ### Right satellite ###
 * @li U18 (not fitted by default)  CN9#3    PA10
 * @li U15 (*option)       			CN9#5    PB5

 *
 * see @a #VL53L1A1_GPIO1_R_OPTION
 *
 * ### Interrupt vectors F401 and L476 ###
 * @li Center PA4 / PC1  => Extit4 / Exti 1
 * @li Left PC7 / PA9 => Extit9_5 / Extit9_5
 * @li Right PA10 / PB5 => Extit15_10 / Extit9_5
 *
 * @warning When selecting alternate option for both sensor L/R interrupt line will shared same vector
 *
 * ### L053 ###
 * @li Center PA4 / PC1  => EXTI4_15_IRQn / EXTI0_1_IRQn
 * @li Left PC7 / PA9  => EXTI4_15_IRQn / EXTI4_15_IRQn
 * @li Right PA10 /  PB5 => EXTI4_15_IRQn / EXTI4_15_IRQn
 *
 * @warning R and L have shared vector use, and will also share C vector if alternate option is selected for C
 */
