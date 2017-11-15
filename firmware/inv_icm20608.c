/*
 $License:
    Copyright (C) 2015 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_icm20608.c
 *      @brief      An I2C-based driver for Invensense 6-axis ICM20608
 *      @details    This driver currently works for the following devices:
 *                  ICM20608
 */
#include <math.h>
#include <cyu3dma.h>
#include "include/inv_icm20608.h"
#include "include/debug.h"

/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */
#if defined EMPL_TARGET_CYPRESS
#include "include/i2c.h"

#define i2c_write   Sensors_I2C_WriteReg
#define i2c_read    Sensors_I2C_ReadReg
#define delay_ms    mdelay
#define get_ms      get_tick_count
#define log_i       sensor_info
#define log_e       sensor_err
#define min(a, b)   ((a < b) ? a : b)
#else
#error  Gyro driver is missing the system layer implementations.
#endif

#if !defined ICM20608
#error  Which gyro are you using? Define ICM20608 in your compiler options.
#endif

static int set_int_enable(unsigned char enable);

/* Hardware registers needed by driver. */
struct sensor_reg_s {
  unsigned char who_am_i;
  unsigned char rate_div;
  unsigned char lpf;
  unsigned char prod_id;
  unsigned char xg_offs_usr;
  unsigned char yg_offs_usr;
  unsigned char zg_offs_usr;
  unsigned char user_ctrl;
  unsigned char fifo_en;
  unsigned char gyro_cfg;
  unsigned char accel_cfg;
  unsigned char accel_cfg2;
  unsigned char lp_mode_cfg;
  unsigned char motion_thr;
  unsigned char motion_dur;
  unsigned char fifo_count_h;
  unsigned char fifo_r_w;
  unsigned char raw_gyro;
  unsigned char raw_accel;
  unsigned char temp;
  unsigned char int_enable;
  unsigned char dmp_int_status;
  unsigned char int_status;
  unsigned char accel_intel;
  unsigned char pwr_mgmt_1;
  unsigned char pwr_mgmt_2;
  unsigned char int_pin_cfg;
  unsigned char mem_r_w;
  unsigned char xa_offset;
  unsigned char ya_offset;
  unsigned char za_offset;
  unsigned char i2c_mst;
  unsigned char bank_sel;
  unsigned char mem_start_addr;
  unsigned char prgm_start_h;
  unsigned char fifo_wm_th;
  unsigned char signal_reset;
  unsigned char st_gyro;
  unsigned char st_accel;
};

/* Information specific to a particular device. */
struct hw_s {
  unsigned char addr;
  unsigned short max_fifo;
  unsigned char num_reg;
  unsigned short temp_sens;
  short temp_offset;
  unsigned short bank_size;
};

/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct motion_int_cache_s {
  unsigned short gyro_fsr;
  unsigned char accel_fsr;
  unsigned short gyro_lpf;
  unsigned short accel_lpf;
  unsigned short sample_rate;
  unsigned char sensors_on;
  unsigned char fifo_sensors;
};

struct chip_cfg_s {
  /* Matches gyro_cfg >> 3 & 0x03 */
  unsigned char gyro_fsr;
  /* Matches accel_cfg >> 3 & 0x03 */
  unsigned char accel_fsr;
  /* Matches lp_mode_cfg >> 4 & 0x07 */
  unsigned char gyro_avgf;
  /* Matches accel_cfg2 >> 4 & 0x03 */
  unsigned char accel_avgf;
  /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
  unsigned char sensors;
  /* Matches config register. */
  unsigned char gyro_lpf;
  unsigned char accel_lpf;
  unsigned char clk_src;
  /* Sample rate, NOT rate divider. */
  unsigned short sample_rate;
  /* Matches fifo_en register. */
  unsigned char fifo_enable;
  /* Matches int enable register. */
  unsigned char int_enable;
  /* 1 if devices on auxiliary I2C bus appear on the primary. */
  unsigned char bypass_mode;
  /* 1 if device in low-power accel-only mode. */
  unsigned char lp_accel_mode;
  /* 1 if interrupts are only triggered on motion events. */
  unsigned char int_motion_only;
  struct motion_int_cache_s cache;
  /* 1 for active low interrupts. */
  unsigned char active_low_int;
  /* 1 for latched interrupts. */
  unsigned char latched_int;
};

/* Information for self-test. */
struct test_s {
  unsigned long gyro_sens;
  unsigned long accel_sens;
  unsigned char reg_rate_div;
  unsigned char reg_lpf;
  unsigned char reg_gyro_fsr;
  unsigned char reg_accel_fsr;
  unsigned short wait_ms;
  unsigned char packet_thresh;
  float min_dps;
  float max_dps;
  float max_gyro_var;
  float min_g;
  float max_g;
  float max_accel_var;
  float max_g_offset;
  unsigned short sample_wait_ms;
};

/* Gyro driver state variables. */
struct gyro_state_s {
  const struct sensor_reg_s *reg;
  const struct hw_s *hw;
  struct chip_cfg_s chip_cfg;
  const struct test_s *test;
};

/* Gyro Filter configurations. */
enum lpf_e {
  INV_GYRO_FILTER_250Hz = 0,
  INV_GYRO_FILTER_176HZ,
  INV_GYRO_FILTER_92HZ,
  INV_GYRO_FILTER_41HZ,
  INV_GYRO_FILTER_20HZ,
  INV_GYRO_FILTER_10HZ,
  INV_GYRO_FILTER_5HZ,
  INV_GYRO_FILTER_3200HZ,
  NUM_GYRO_FILTER
};

/* Accel Filter configurations. */
enum lpf_a_e {
  INV_ACCEL_FILTER_218Hz = 0,
  INV_ACCEL_FILTER_218HZ,
  INV_ACCEL_FILTER_99HZ,
  INV_ACCEL_FILTER_45HZ,
  INV_ACCEL_FILTER_21HZ,
  INV_ACCEL_FILTER_10HZ,
  INV_ACCEL_FILTER_5HZ,
  INV_ACCEL_FILTER_420HZ,
  NUM_ACCEL_FILTER
};

/* Full scale ranges. */
enum gyro_fsr_e {
  INV_FSR_250DPS = 0,
  INV_FSR_500DPS,
  INV_FSR_1000DPS,
  INV_FSR_2000DPS,
  NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e {
  INV_FSR_2G = 0,
  INV_FSR_4G,
  INV_FSR_8G,
  INV_FSR_16G,
  NUM_ACCEL_FSR
};

/* Accel Averaging Filters. */
enum accel_avgf_e {
  INV_ACCEL_4X_AVG = 0,
  INV_ACCEL_8X_AVG,
  INV_ACCEL_16X_AVG,
  INV_ACCEL_32X_AVG,
  NUM_ACCEL_AVG
};

/* Clock sources. */
enum clock_sel_e {
  INV_CLK_INTERNAL = 0,
  INV_CLK_PLL,
  NUM_CLK
};

/* Low-power accel wakeup rates. */
enum lp_accel_rate_e {
  INV_LPA_0_24HZ,
  INV_LPA_0_49HZ,
  INV_LPA_0_98HZ,
  INV_LPA_1_95HZ,
  INV_LPA_3_91HZ,
  INV_LPA_7_81HZ,
  INV_LPA_15_63HZ,
  INV_LPA_31_25HZ,
  INV_LPA_62_5HZ,
  INV_LPA_125HZ,
  INV_LPA_250HZ,
  INV_LPA_500HZ
};

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)
#define BIT_ACCL_FC_B       (0x08)
#define BIT_ACCL_DEC2       (0x30)
#define BIT_GYRO_FC_B       (0x01)
#define BIT_G_AVGCFG        (0x70)
#define WHOAMI_20608        (0xAF)
#define BIT_GYRO_CYCLE      (0x80)


const struct sensor_reg_s reg = {
  .who_am_i       = 0x75,
  .rate_div       = 0x19,
  .lpf            = 0x1A,
  .prod_id        = 0x0C,
  .xg_offs_usr    = 0x13,
  .yg_offs_usr    = 0x15,
  .zg_offs_usr    = 0x17,
  .user_ctrl      = 0x6A,
  .fifo_en        = 0x23,
  .gyro_cfg       = 0x1B,
  .accel_cfg      = 0x1C,
  .accel_cfg2     = 0x1D,
  .lp_mode_cfg    = 0x1E,
  .motion_thr     = 0x1F,
  .motion_dur     = 0x20,
  .fifo_count_h   = 0x72,
  .fifo_r_w       = 0x74,
  .raw_gyro       = 0x43,
  .raw_accel      = 0x3B,
  .temp           = 0x41,
  .int_enable     = 0x38,
  .dmp_int_status = 0x39,
  .int_status     = 0x3A,
  .accel_intel    = 0x69,
  .pwr_mgmt_1     = 0x6B,
  .pwr_mgmt_2     = 0x6C,
  .int_pin_cfg    = 0x37,
  .mem_r_w        = 0x6F,
  .xa_offset      = 0x77,
  .ya_offset      = 0x7A,
  .za_offset      = 0x7D,
  .i2c_mst        = 0x24,
  .bank_sel       = 0x6D,
  .mem_start_addr = 0x6E,
  .prgm_start_h   = 0x70,
  .fifo_wm_th     = 0x61,
  .signal_reset   = 0x68,
  .st_gyro        = 0x00,
  .st_accel       = 0x0D,
};

// 0xD0;  0xD2
const struct hw_s hw = {
  .addr           = 0xD0,
// .addr           = 0xD2,
  .max_fifo       = 1024,
  .num_reg        = 128,
  .temp_sens      = 321,
  .temp_offset    = 0,
  .bank_size      = 256
};

const struct test_s test = {
  .gyro_sens      = 32768 / 250,
  .accel_sens     = 32768 / 2,  // FSR = +-2G = 16384 LSB/G
  .reg_rate_div   = 0,      // 1kHz.
  .reg_lpf        = 2,      // 92Hz low pass filter
  .reg_gyro_fsr   = 0,      // 250dps
  .reg_accel_fsr  = 0x0,    // Accel FSR setting = 2g.
  .wait_ms        = 200,    // 200ms stabilization time
  .packet_thresh  = 200,    // 200 samples
  .min_dps        = 20.f,   // 20 dps for Gyro Criteria C
  .max_dps        = 60.f,   // Must exceed 60 dps threshold for Gyro Criteria B
  .max_gyro_var   = .5f,    // Must exceed +50% variation for Gyro Criteria A
  .min_g          = .225f,  // Accel must exceed Min 225 mg for Criteria B
  .max_g          = .675f,  // Accel cannot exceed Max 675 mg for Criteria B
  .max_accel_var  = .5f,    // Accel must be within 50% variation for Criteria A
  .max_g_offset   = .5f,    // 500 mg for Accel Criteria C
  .sample_wait_ms = 10      // 10ms sample time wait
};

static struct gyro_state_s st = {
  .reg = &reg,
  .hw = &hw,
  .test = &test
};

#define MAX_PACKET_LENGTH (12)
#define HWST_MAX_PACKET_LENGTH (512)

/**
*  @brief      Enable/disable data ready interrupt.
*  @param[in]  enable      1 to enable interrupt.
*  @return     0 if successful.
*/
static int set_int_enable(unsigned char enable) {
  unsigned char tmp;

  if (!st.chip_cfg.sensors)
    return -1;
  if (enable && st.chip_cfg.int_enable)
    return 0;
  if (enable)
    tmp = BIT_DATA_RDY_EN;
  else
    tmp = 0x00;
  if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
    return -1;
  st.chip_cfg.int_enable = tmp;

  return 0;
}
/**
*  @brief      Check WHOAMI to confirm ICM20608
*  @return     1 if correct. 0 if incorrect. -1 if i2c error
*/
int icm_check_whoami(void) {
  unsigned char data;

  if (i2c_read(st.hw->addr, st.reg->who_am_i, 1, &data)) {
    return -1;
  }

  if (data != WHOAMI_20608) {
    log_e("WHOAMI error: %#5x\r\n", &data);
    return 0;
  }

  return 1;
}

/**
*  @brief      Check WHOAMI to confirm ICM20608
*  @return     1 if correct. 0 if incorrect. -1 if i2c error
*/

unsigned char result_whoami;
unsigned char icm_check_whoami_debug(void) {
  if (i2c_read(st.hw->addr, st.reg->who_am_i, 1, &result_whoami) == 0) {
    sensor_dbg("icm_check_whoami_debug result_whoami:%d\r\n", result_whoami);
    return result_whoami;
  }
  return result_whoami;
}

/**
*  @brief      Register dump for testing.
*  @return     0 if successful.
*/
int icm_reg_dump(void) {
  unsigned char ii;
  unsigned char data;

  for (ii = 0; ii < st.hw->num_reg; ii++) {
    if (ii == st.reg->fifo_r_w || ii == st.reg->mem_r_w)
      continue;
    if (i2c_read(st.hw->addr, ii, 1, &data))
      return -1;
    log_i("%#5x: %#5x\r\n", ii, data);
  }
  return 0;
}

/**
 *  @brief      Read from a single register.
 *  NOTE: The memory and FIFO read/write registers cannot be accessed.
 *  @param[in]  reg     Register address.
 *  @param[out] data    Register data.
 *  @return     0 if successful.
 */
int icm_read_reg(unsigned char reg, unsigned char *data) {
  if (reg == st.reg->fifo_r_w || reg == st.reg->mem_r_w)
    return -1;
  if (reg >= st.hw->num_reg)
    return -1;
  return i2c_read(st.hw->addr, reg, 1, data);
}

int icm_write_reg(unsigned char reg, unsigned char data) {
  if (reg == st.reg->fifo_r_w || reg == st.reg->mem_r_w)
    return -1;
  if (reg >= st.hw->num_reg)
    return -1;
  return i2c_write(st.hw->addr, reg, 2, &data);
}

/**
 *  @brief      Initialize hardware.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 2000DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 50Hz\n
 *  FIFO rate: 50Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @return     0 if successful.
 */
int icm_init(void) {
  unsigned char data[6];

  /* Reset device. */
  data[0] = BIT_RESET;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
    return -1;
  delay_ms(100);

  /* Wake up chip. */
  data[0] = 0x00;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
    return -1;

  /* ICM20608 has no DMP therefore no need to share memory */
  data[0] = BIT_FIFO_SIZE_4096;
  if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, data))
    return -1;

  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg.sensors = 0xFF;
  st.chip_cfg.gyro_fsr = 0xFF;
  st.chip_cfg.accel_fsr = 0xFF;
  st.chip_cfg.gyro_lpf = 0xFF;
  st.chip_cfg.accel_lpf = 0xFF;
  st.chip_cfg.sample_rate = 0xFFFF;
  st.chip_cfg.fifo_enable = 0xFF;
  st.chip_cfg.bypass_mode = 0xFF;
  /* icm_set_sensors always preserves this setting. */
  st.chip_cfg.clk_src = INV_CLK_PLL;
  /* Handled in next call to icm_set_bypass. */
  st.chip_cfg.active_low_int = 1;
  st.chip_cfg.latched_int = 0;
  st.chip_cfg.int_motion_only = 0;
  st.chip_cfg.lp_accel_mode = 0;
  memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));

  if (icm_set_gyro_fsr(2000))
    return -1;
  if (icm_set_accel_fsr(2))
    return -1;
  if (icm_set_gyro_lpf(50))
    return -1;
  if (icm_set_accel_lpf(50))
    return -1;

  if (icm_set_sample_rate(50))
    return -1;

  /* Push both gyro and accel data into the FIFO. */
  if (icm_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
    return -1;

  if (icm_set_bypass(0))
    return -1;

  /* TODO(renyi): why we need this step: turn off -> delay -> turn on ?*/
  icm_set_sensors(0);

  CyU3PThreadSleep(100);

  icm_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  sensor_dbg("icm_init ok \r\n");
  return 0;
}

/**
 *  @brief      Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n ICM20645: .98, 1.95, 3.91, 7.81, 15.63, 31.25, 62.5, 125, 250, 500Hz
 *  \n If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
 *                          accel mode.
 *  @return     0 if successful.
 */
int icm_lp_accel_mode(unsigned short rate) {
  unsigned char tmp[2];

  if (rate) {
    if (rate > 640)
      return -1;

    if (!st.chip_cfg.lp_accel_mode) {
      /* Store current settings for later. */
      icm_get_gyro_fsr(&st.chip_cfg.cache.gyro_fsr);
      icm_get_accel_fsr(&st.chip_cfg.cache.accel_fsr);
      icm_get_gyro_lpf(&st.chip_cfg.cache.gyro_lpf);
      icm_get_accel_lpf(&st.chip_cfg.cache.accel_lpf);
      icm_get_sample_rate(&st.chip_cfg.cache.sample_rate);
      st.chip_cfg.cache.sensors_on = st.chip_cfg.sensors;
      icm_get_fifo_config(&st.chip_cfg.cache.fifo_sensors);
    }

    /* For LP accel, we automatically configure the hardware to produce latched
     * interrupts. In LP accel mode, the hardware cycles into sleep mode before
     * it gets a chance to deassert the interrupt pin; therefore, we shift this
     * responsibility over to the MCU.
     *
     * Any register read will clear the interrupt.
     */
    /* Set wake frequency. */
    if (rate == 1)
      tmp[0] = INV_LPA_0_98HZ;
    else if (rate == 2)
      tmp[0] = INV_LPA_1_95HZ;
    else if (rate <= 5)
      tmp[0] = INV_LPA_3_91HZ;
    else if (rate <= 10)
      tmp[0] = INV_LPA_7_81HZ;
    else if (rate <= 20)
      tmp[0] = INV_LPA_15_63HZ;
    else if (rate <= 40)
      tmp[0] = INV_LPA_31_25HZ;
    else if (rate <= 80)
      tmp[0] = INV_LPA_62_5HZ;
    else if (rate <= 160)
      tmp[0] = INV_LPA_125HZ;
    else if (rate <= 320)
      tmp[0] = INV_LPA_250HZ;
    else
      tmp[0] = INV_LPA_500HZ;
    if (i2c_write(st.hw->addr, st.reg->lp_mode_cfg, 1, tmp))
      goto lp_restore;
    /* Enable LP ACCEL mode, ACCEL_FCHOICE_B=1*/
    if (i2c_read(st.hw->addr, st.reg->accel_cfg2, 1, tmp))
      goto lp_restore;

    tmp[0] = BIT_FIFO_SIZE_4096 | BIT_ACCL_FC_B | tmp[0];
    if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, tmp))
      goto lp_restore;

    tmp[0] = BIT_LPA_CYCLE;
    tmp[1] = BIT_STBY_XYZG;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, tmp))
      goto lp_restore;

    st.chip_cfg.sensors = INV_XYZ_ACCEL;
    st.chip_cfg.clk_src = 0;
    st.chip_cfg.lp_accel_mode = 1;
    icm_configure_fifo(0);
    return 0;
  }
lp_restore:
  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg.gyro_fsr = 0xFF;
  st.chip_cfg.accel_fsr = 0xFF;
  st.chip_cfg.gyro_lpf = 0xFF;
  st.chip_cfg.accel_lpf = 0xFF;
  st.chip_cfg.sample_rate = 0xFFFF;
  st.chip_cfg.sensors = 0xFF;
  st.chip_cfg.fifo_enable = 0xFF;
  st.chip_cfg.clk_src = INV_CLK_PLL;
  icm_set_sensors(st.chip_cfg.cache.sensors_on);
  icm_set_gyro_fsr(st.chip_cfg.cache.gyro_fsr);
  icm_set_accel_fsr(st.chip_cfg.cache.accel_fsr);
  icm_set_gyro_lpf(st.chip_cfg.cache.gyro_lpf);
  icm_set_accel_lpf(st.chip_cfg.cache.accel_lpf);
  icm_set_sample_rate(st.chip_cfg.cache.sample_rate);
  icm_configure_fifo(st.chip_cfg.cache.fifo_sensors);
  return 0;
}

/**
 *  @brief      Sets the sensor into 6-axis low power mode
 *  @param      gyro_avg        gyro average filters
 *  @param      enable   enable or diable
 *  @return     0 if successful.
 */
int icm_lp_6axis_mode(int gyro_avg, unsigned char enable) {
  unsigned char tmp;

  if (icm_set_gyro_avgf(gyro_avg))
    return -1;

  if (i2c_read(st.hw->addr, st.reg->lp_mode_cfg, 1, &tmp))
    return -1;

  if (enable) {
    tmp |= BIT_GYRO_CYCLE;
  } else {
    tmp &= ~BIT_GYRO_CYCLE;
  }

  if (i2c_write(st.hw->addr, st.reg->lp_mode_cfg, 1, &tmp))
    return -1;

  return 0;
}
/**
 *  @brief      Read raw IMU data[gyro accel temperature] directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int icm_get_sensor_reg(unsigned char *data, unsigned long *timestamp) {
  if (!(st.chip_cfg.sensors & INV_XYZ_GYRO))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_accel, 14, (uint8_t *)data))
    return -1;

  if (timestamp)
    get_ms(timestamp);

  return 0;
}
/**
 *  @brief      Read raw gyro data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int icm_get_gyro_reg(short *data, unsigned long *timestamp) {
  unsigned char tmp[6];

  if (!(st.chip_cfg.sensors & INV_XYZ_GYRO))
    return -1;
#if I2C_RW_SINGLE
  if (i2c_read(st.hw->addr, st.reg->raw_gyro, 1, (uint8_t *)&tmp[0]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_gyro + 1, 1, (uint8_t *)&tmp[1]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_gyro + 2, 1, (uint8_t *)&tmp[2]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_gyro + 3, 1, (uint8_t *)&tmp[3]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_gyro + 4, 1, (uint8_t *)&tmp[4]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_gyro + 5, 1, (uint8_t *)&tmp[5]))
    return -1;
#else
  if (i2c_read(st.hw->addr, st.reg->raw_gyro, 6, tmp))
     return -1;
#endif
  data[0] = (tmp[0] << 8) | tmp[1];
  data[1] = (tmp[2] << 8) | tmp[3];
  data[2] = (tmp[4] << 8) | tmp[5];

  if (timestamp)
    get_ms(timestamp);

  return 0;
}

/**
 *  @brief      Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int icm_get_accel_reg(short *data, unsigned long *timestamp) {
  unsigned char tmp[6];

  if (!(st.chip_cfg.sensors & INV_XYZ_ACCEL))
    return -1;

#if I2C_RW_SINGLE
  if (i2c_read(st.hw->addr, st.reg->raw_accel, 1, (uint8_t *)&tmp[0]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_accel + 1, 1, (uint8_t *)&tmp[1]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_accel + 2, 1, (uint8_t *)&tmp[2]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_accel + 3, 1, (uint8_t *)&tmp[3]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_accel + 4, 1, (uint8_t *)&tmp[4]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->raw_accel + 5, 1, (uint8_t *)&tmp[5]))
    return -1;
#else
  if (i2c_read(st.hw->addr, st.reg->raw_accel, 6, tmp))
      return -1;
#endif
  data[0] = (tmp[0] << 8) | tmp[1];
  data[1] = (tmp[2] << 8) | tmp[3];
  data[2] = (tmp[4] << 8) | tmp[5];

  if (timestamp)
    get_ms(timestamp);

  // sensor_dbg("icm_get_accel_reg, tmp is:%d\r\n" ,tmp);
  return 0;
}

/**
 *  @brief      Read temperature data directly from the registers.
 *  @param[out] data        Data in q16 format.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int icm_get_temperature(long *data, unsigned long *timestamp) {
  unsigned char tmp[2];
  short raw;

  if (!(st.chip_cfg.sensors))
    return -1;

  if (i2c_read(st.hw->addr, st.reg->temp, 2, tmp))
    return -1;
  raw = (tmp[0] << 8) | tmp[1];
  if (timestamp)
    get_ms(timestamp);

  data[0] = (long)((35 + ((raw - (float)st.hw->temp_offset) / st.hw->temp_sens)) * 65536L);
#if 0
  sensor_dbg("icm_get_temperature tmp:[%d] data:[%d]\r\n", tmp, data[0]);
#endif
  return 0;
}

/**
 *  @brief      Read biases to the accel bias 20608 registers.
 *  This function reads from the ICM20608 accel offset cancellations registers.
 *  The format are G in +-8G format. The register is initialized with OTP
 *  factory trim values.
 *  @param[in]  accel_bias  returned structure with the accel bias
 *  @return     0 if successful.
 */
int icm_read_accel_bias(long *accel_bias) {
  unsigned char data[6];
  if (i2c_read(st.hw->addr, st.reg->xa_offset, 2, &data[0]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->ya_offset, 2, &data[2]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->za_offset, 2, &data[4]))
    return -1;
  accel_bias[0] = ((long)data[0] << 8) | data[1];
  accel_bias[1] = ((long)data[2] << 8) | data[3];
  accel_bias[2] = ((long)data[4] << 8) | data[5];
  return 0;
}

int icm_read_gyro_bias(long *gyro_bias) {
  unsigned char data[6];
  if (i2c_read(st.hw->addr, st.reg->xg_offs_usr, 2, &data[0]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->yg_offs_usr, 2, &data[2]))
    return -1;
  if (i2c_read(st.hw->addr, st.reg->zg_offs_usr, 2, &data[4]))
    return -1;
  gyro_bias[0] = ((long)data[0] << 8) | data[1];
  gyro_bias[1] = ((long)data[2] << 8) | data[3];
  gyro_bias[2] = ((long)data[4] << 8) | data[5];
  return 0;
}

/**
 *  @brief      Push biases to the gyro bias 20608 registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-1000dps format.
 *  @param[in]  gyro_bias  New biases.
 *  @return     0 if successful.
 */
int icm_set_gyro_bias_reg(long *gyro_bias) {
  unsigned char data[6] = {0, 0, 0, 0, 0, 0};
  int i = 0;
  for (i = 0; i < 3; i++) {
    gyro_bias[i] = (-gyro_bias[i]);
  }
  data[0] = (gyro_bias[0] >> 8) & 0xff;
  data[1] = (gyro_bias[0]) & 0xff;
  data[2] = (gyro_bias[1] >> 8) & 0xff;
  data[3] = (gyro_bias[1]) & 0xff;
  data[4] = (gyro_bias[2] >> 8) & 0xff;
  data[5] = (gyro_bias[2]) & 0xff;
  if (i2c_write(st.hw->addr, st.reg->xg_offs_usr, 2, &data[0]))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->yg_offs_usr, 2, &data[2]))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->zg_offs_usr, 2, &data[4]))
    return -1;
  return 0;
}

/**
 *  @brief      Push biases to the accel bias 20608 registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-16G format.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
int icm_set_accel_bias_reg(const long *accel_bias) {
  unsigned char data[6] = {0, 0, 0, 0, 0, 0};
  long accel_reg_bias[3] = {0, 0, 0};

  if (icm_read_accel_bias(accel_reg_bias))
    return -1;

  // Preserve bit 0 of factory value (for temperature compensation)
  accel_reg_bias[0] -= (accel_bias[0] & ~1);
  accel_reg_bias[1] -= (accel_bias[1] & ~1);
  accel_reg_bias[2] -= (accel_bias[2] & ~1);

  data[0] = (accel_reg_bias[0] >> 8) & 0xff;
  data[1] = (accel_reg_bias[0]) & 0xff;
  data[2] = (accel_reg_bias[1] >> 8) & 0xff;
  data[3] = (accel_reg_bias[1]) & 0xff;
  data[4] = (accel_reg_bias[2] >> 8) & 0xff;
  data[5] = (accel_reg_bias[2]) & 0xff;

  if (i2c_write(st.hw->addr, st.reg->xa_offset, 2, &data[0]))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->ya_offset, 2, &data[2]))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->za_offset, 2, &data[4]))
    return -1;

  return 0;
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int icm_reset_fifo(void) {
  unsigned char data;

  if (!(st.chip_cfg.sensors))
    return -1;

  data = 0;
  if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
    return -1;

  data = BIT_FIFO_RST;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
    return -1;
  if (st.chip_cfg.bypass_mode || !(st.chip_cfg.sensors & INV_XYZ_COMPASS))
    data = BIT_FIFO_EN;
  else
    data = BIT_FIFO_EN | BIT_AUX_IF_EN;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
    return -1;
  delay_ms(50);
  if (st.chip_cfg.int_enable)
    data = BIT_DATA_RDY_EN;
  else
    data = 0;
  if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &st.chip_cfg.fifo_enable))
    return -1;

  return 0;
}

/**
 *  @brief  Get current averaging filter settings for gyro low power mode
 *  @return 0 if successful.
 */
int icm_get_gyro_avgf(unsigned short *avgf) {
  switch (st.chip_cfg.gyro_avgf) {
  case INV_GYRO_1X_AVG:
    avgf[0] = 1;
    break;
  case INV_GYRO_2X_AVG:
    avgf[0] = 2;
    break;
  case INV_GYRO_8X_AVG:
    avgf[0] = 8;
    break;
  case INV_GYRO_16X_AVG:
    avgf[0] = 16;
    break;
  case INV_GYRO_32X_AVG:
    avgf[0] = 32;
    break;
  case INV_GYRO_64X_AVG:
    avgf[0] = 64;
    break;
  case INV_GYRO_128X_AVG:
    avgf[0] = 128;
    break;
  default:
    avgf[0] = 0;
    break;
  }
  return 0;
}

/**
 *  @brief  Sets the averaging filter for gyro low power mode
 *  @return 0 if successful.
 */
int icm_set_gyro_avgf(unsigned short avgf) {
  unsigned char data, tmp;

  if (!(st.chip_cfg.sensors))
    return -1;

  if (i2c_read(st.hw->addr, st.reg->lp_mode_cfg, 1, &tmp))
    return -1;

  tmp &= ~BIT_G_AVGCFG;

  switch (avgf) {
  case INV_GYRO_1X_AVG:
    data = INV_GYRO_1X_AVG << 4;
    break;
  case INV_GYRO_2X_AVG:
    data = INV_GYRO_2X_AVG << 4;
    break;
  case INV_GYRO_8X_AVG:
    data = INV_GYRO_8X_AVG << 4;
    break;
  case INV_GYRO_16X_AVG:
    data = INV_GYRO_16X_AVG << 4;
    break;
  case INV_GYRO_32X_AVG:
    data = INV_GYRO_32X_AVG << 4;
    break;
  case INV_GYRO_64X_AVG:
    data = INV_GYRO_64X_AVG << 4;
    break;
  case INV_GYRO_128X_AVG:
    data = INV_GYRO_128X_AVG << 4;
    break;
  default:
    return -1;
  }

  if (st.chip_cfg.gyro_avgf == (data >> 4))
    return 0;
  tmp = data | tmp;
  if (i2c_write(st.hw->addr, st.reg->lp_mode_cfg, 1, &tmp))
    return -1;
  st.chip_cfg.gyro_avgf = data >> 4;

  return 0;
}

/**
 *  @brief  Get current averaging filter settings for accel low power mode
 *  @return 0 if successful.
 */
int icm_get_accel_avgf(unsigned short *avgf) {
  switch (st.chip_cfg.accel_avgf) {
  case INV_ACCEL_4X_AVG:
    avgf[0] = 4;
    break;
  case INV_ACCEL_8X_AVG:
    avgf[0] = 8;
    break;
  case INV_ACCEL_16X_AVG:
    avgf[0] = 16;
    break;
  case INV_ACCEL_32X_AVG:
    avgf[0] = 32;
    break;
  default:
    avgf[0] = 0;
    break;
  }
  return 0;
}

/**
 *  @brief  Sets the averaging filter for accel low power mode
 *  @return 0 if successful.
 */
int icm_set_accel_avgf(unsigned short avgf) {
  unsigned char data, tmp;

  if (!(st.chip_cfg.sensors))
    return -1;

  if (i2c_read(st.hw->addr, st.reg->accel_cfg2, 1, &tmp))
    return -1;

  tmp &= ~BIT_ACCL_DEC2;

  switch (avgf) {
  case INV_ACCEL_4X_AVG:
    data = INV_ACCEL_4X_AVG << 4;
    break;
  case INV_ACCEL_8X_AVG:
    data = INV_ACCEL_8X_AVG << 4;
    break;
  case INV_ACCEL_16X_AVG:
    data = INV_ACCEL_16X_AVG << 4;
    break;
  case INV_ACCEL_32X_AVG:
    data = INV_ACCEL_32X_AVG << 4;
    break;
  default:
    return -1;
  }

  if (st.chip_cfg.accel_avgf == (data >> 4))
    return 0;
  tmp = data | tmp;
  if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, &tmp))
    return -1;
  st.chip_cfg.accel_avgf = data >> 4;

  return 0;
}

/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int icm_get_gyro_fsr(unsigned short *fsr) {
  switch (st.chip_cfg.gyro_fsr) {
  case INV_FSR_250DPS:
    fsr[0] = 250;
    break;
  case INV_FSR_500DPS:
    fsr[0] = 500;
    break;
  case INV_FSR_1000DPS:
    fsr[0] = 1000;
    break;
  case INV_FSR_2000DPS:
    fsr[0] = 2000;
    break;
  default:
    fsr[0] = 0;
    break;
  }
  return 0;
}

/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int icm_set_gyro_fsr(unsigned short fsr) {
  unsigned char data, tmp;

  if (!(st.chip_cfg.sensors))
    return -1;

  if (i2c_read(st.hw->addr, st.reg->gyro_cfg, 1, &tmp))
    return -1;

  tmp &= BIT_GYRO_FC_B;

  switch (fsr) {
  case 250:
    data = INV_FSR_250DPS << 3;
    break;
  case 500:
    data = INV_FSR_500DPS << 3;
    break;
  case 1000:
    data = INV_FSR_1000DPS << 3;
    break;
  case 2000:
    data = INV_FSR_2000DPS << 3;
    break;
  default:
    return -1;
  }

  if (st.chip_cfg.gyro_fsr == (data >> 3))
    return 0;
  tmp = data | tmp;
  if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &tmp))
    return -1;
  st.chip_cfg.gyro_fsr = data >> 3;
  return 0;
}

/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int icm_get_accel_fsr(unsigned char *fsr) {
  switch (st.chip_cfg.accel_fsr) {
  case INV_FSR_2G:
    fsr[0] = 2;
    break;
  case INV_FSR_4G:
    fsr[0] = 4;
    break;
  case INV_FSR_8G:
    fsr[0] = 8;
    break;
  case INV_FSR_16G:
    fsr[0] = 16;
    break;
  default:
    return -1;
  }

  return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int icm_set_accel_fsr(unsigned char fsr) {
  unsigned char data;

  if (!(st.chip_cfg.sensors))
    return -1;

  switch (fsr) {
  case 2:
    data = INV_FSR_2G << 3;
    break;
  case 4:
    data = INV_FSR_4G << 3;
    break;
  case 8:
    data = INV_FSR_8G << 3;
    break;
  case 16:
    data = INV_FSR_16G << 3;
    break;
  default:
    return -1;
  }

  if (st.chip_cfg.accel_fsr == (data >> 3))
    return 0;
  if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, &data))
    return -1;
  st.chip_cfg.accel_fsr = data >> 3;
  return 0;
}

/**
 *  @brief      Get the current gyro DLPF setting.
 *  @param[out] lpf Current gyro LPF setting.
 *  0 if successful.
 */
int icm_get_gyro_lpf(unsigned short *lpf) {
  switch (st.chip_cfg.gyro_lpf) {
  case INV_GYRO_FILTER_250Hz:
    lpf[0] = 250;
    break;
  case INV_GYRO_FILTER_176HZ:
    lpf[0] = 176;
    break;
  case INV_GYRO_FILTER_92HZ:
    lpf[0] = 92;
    break;
  case INV_GYRO_FILTER_41HZ:
    lpf[0] = 41;
    break;
  case INV_GYRO_FILTER_20HZ:
    lpf[0] = 20;
    break;
  case INV_GYRO_FILTER_10HZ:
    lpf[0] = 10;
    break;
  case INV_GYRO_FILTER_5HZ:
    lpf[0] = 5;
    break;
  case INV_GYRO_FILTER_3200HZ:
    lpf[0] = 3200;
    break;
  default:
    lpf[0] = 0;
    break;
  }
  return 0;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 250, 176, 92, 41, 20, 10, 5, and 0
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int icm_set_gyro_lpf(unsigned short lpf) {
  unsigned char data, tmp;

  if (!(st.chip_cfg.sensors))
    return -1;

  if (lpf == 0) {
    // set bit FCHOICE for gyro = 1 to bypass lpf
    if (i2c_read(st.hw->addr, st.reg->gyro_cfg, 1, &tmp))
      return -1;
    tmp |= BIT_GYRO_FC_B;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &tmp))
      return -1;
    return 0;
  } else {
    if (i2c_read(st.hw->addr, st.reg->gyro_cfg, 1, &tmp))
      return -1;
    tmp &= ~BIT_GYRO_FC_B;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &tmp))
      return -1;
  }

  if (lpf >= 250)
    data = INV_GYRO_FILTER_250Hz;
  else if (lpf >= 176)
    data = INV_GYRO_FILTER_176HZ;
  else if (lpf >= 92)
    data = INV_GYRO_FILTER_92HZ;
  else if (lpf >= 41)
    data = INV_GYRO_FILTER_41HZ;
  else if (lpf >= 20)
    data = INV_GYRO_FILTER_20HZ;
  else if (lpf >= 10)
    data = INV_GYRO_FILTER_10HZ;
  else if (lpf >= 5)
    data = INV_GYRO_FILTER_5HZ;

  if (st.chip_cfg.gyro_lpf == data)
    return 0;
  if (i2c_write(st.hw->addr, st.reg->lpf, 1, &data))
    return -1;

  st.chip_cfg.gyro_lpf = data;
  return 0;
}

/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
int icm_get_accel_lpf(unsigned short *lpf) {
  switch (st.chip_cfg.accel_lpf) {
  case INV_ACCEL_FILTER_218Hz:
    lpf[0] = 218;
    break;
  case INV_ACCEL_FILTER_99HZ:
    lpf[0] = 99;
    break;
  case INV_ACCEL_FILTER_45HZ:
    lpf[0] = 45;
    break;
  case INV_ACCEL_FILTER_21HZ:
    lpf[0] = 21;
    break;
  case INV_ACCEL_FILTER_10HZ:
    lpf[0] = 10;
    break;
  case INV_ACCEL_FILTER_5HZ:
    lpf[0] = 5;
    break;
  case INV_ACCEL_FILTER_420HZ:
    lpf[0] = 420;
    break;
  default:
    lpf[0] = 0;
    break;
  }
  return 0;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 218, 99, 45, 21, 10, 5, 0
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int icm_set_accel_lpf(unsigned short lpf) {
  unsigned char data = 0;
  unsigned char tmp;

  if (!(st.chip_cfg.sensors))
    return -1;

  if (lpf == 0) {
    // set bit FCHOICE for accel = 1 to bypass lpf
    if (i2c_read(st.hw->addr, st.reg->accel_cfg2, 1, &tmp))
      return -1;
    tmp |= BIT_ACCL_FC_B;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, &tmp))
      return -1;
    return 0;
  } else {
    if (i2c_read(st.hw->addr, st.reg->accel_cfg2, 1, &tmp))
      return -1;
    tmp &= ~(BIT_ACCL_FC_B | BITS_LPF);
  }

  if (lpf >= 218)
    data = INV_ACCEL_FILTER_218Hz;
  else if (lpf >= 188)
    data = INV_ACCEL_FILTER_99HZ;
  else if (lpf >= 42)
    data = INV_ACCEL_FILTER_45HZ;
  else if (lpf >= 20)
    data = INV_ACCEL_FILTER_21HZ;
  else if (lpf >= 10)
    data = INV_ACCEL_FILTER_10HZ;
  else if (lpf >= 5)
    data = INV_ACCEL_FILTER_5HZ;

  if (st.chip_cfg.accel_lpf == data)
    return 0;

  tmp |= data;

  if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, &tmp))
    return -1;

  st.chip_cfg.accel_lpf = data;
  return 0;
}

/**
 *  @brief      Get the current watermark threshold setting.
 *  @param[out] threshold
 *  0 if successful.
 */
int icm_get_watermark(unsigned char *threshold) {
  if (i2c_read(st.hw->addr, st.reg->fifo_wm_th, 1, threshold))
    return -1;

  return 0;
}

/**
 *  @brief      Set watermark level threshold.
 *  Watermark supports only maximum 255 bytes
 *  @param[in]  threshold
 *  @return     0 if successful.
 */
int icm_set_watermark(unsigned char threshold) {
  if (i2c_write(st.hw->addr, st.reg->fifo_wm_th, 1, &threshold))
    return -1;

  return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int icm_get_sample_rate(unsigned short *rate) {
  rate[0] = st.chip_cfg.sample_rate;
  return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int icm_set_sample_rate(unsigned short rate) {
  unsigned char data;

  if (!(st.chip_cfg.sensors))
    return -1;

  if (st.chip_cfg.lp_accel_mode) {
    if (rate && (rate <= 40)) {
      /* Just stay in low-power accel mode. */
      icm_lp_accel_mode(rate);
      return 0;
    }
    /* Requested rate exceeds the allowed frequencies in LP accel mode,
     * switch back to full-power mode.
     */
    icm_lp_accel_mode(0);
  }
  if (rate < 4)
    rate = 4;
  else if (rate > 1000)
    rate = 1000;

  data = 1000 / rate - 1;
  if (i2c_write(st.hw->addr, st.reg->rate_div, 1, &data))
    return -1;

  st.chip_cfg.sample_rate = 1000 / (1 + data);

  /* Automatically set LPF to 1/2 sampling rate. */
  icm_set_gyro_lpf(st.chip_cfg.sample_rate >> 1);
  icm_set_accel_lpf(st.chip_cfg.sample_rate >> 1);

  return 0;
}

/**
 *  @brief      Get gyro sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to dps.
 *  @return     0 if successful.
 */
int icm_get_gyro_sens(float *sens) {
  switch (st.chip_cfg.gyro_fsr) {
  case INV_FSR_250DPS:
    sens[0] = 131.f;
    break;
  case INV_FSR_500DPS:
    sens[0] = 65.5f;
    break;
  case INV_FSR_1000DPS:
    sens[0] = 32.8f;
    break;
  case INV_FSR_2000DPS:
    sens[0] = 16.4f;
    break;
  default:
    return -1;
  }
  return 0;
}

/**
 *  @brief      Get accel sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to g's.
 *  @return     0 if successful.
 */
int icm_get_accel_sens(unsigned short *sens) {
  switch (st.chip_cfg.accel_fsr) {
  case INV_FSR_2G:
    sens[0] = 16384;
    break;
  case INV_FSR_4G:
    sens[0] = 8192;
    break;
  case INV_FSR_8G:
    sens[0] = 4096;
    break;
  case INV_FSR_16G:
    sens[0] = 2048;
    break;
  default:
    return -1;
  }

  return 0;
}

/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
int icm_get_fifo_config(unsigned char *sensors) {
  sensors[0] = st.chip_cfg.fifo_enable;
  return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int icm_configure_fifo(unsigned char sensors) {
  unsigned char prev;
  int result = 0;

  if (!(st.chip_cfg.sensors))
    return -1;
  prev = st.chip_cfg.fifo_enable;
  st.chip_cfg.fifo_enable = sensors & st.chip_cfg.sensors;
  if (st.chip_cfg.fifo_enable != sensors)
    /* You're not getting what you asked for. Some sensors are
     * asleep.
     */
    result = -1;
  else
    result = 0;
  if (sensors || st.chip_cfg.lp_accel_mode)
    set_int_enable(1);
  else
    set_int_enable(0);
  if (sensors) {
    if (icm_reset_fifo()) {
      st.chip_cfg.fifo_enable = prev;
      return -1;
    }
  }

  return result;
}

/**
 *  @brief      Get current power state.
 *  @param[in]  power_on    1 if turned on, 0 if suspended.
 *  @return     0 if successful.
 */
int icm_get_power_state(unsigned char *power_on) {
  if (st.chip_cfg.sensors)
    power_on[0] = 1;
  else
    power_on[0] = 0;
  return 0;
}

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int icm_set_sensors(unsigned char sensors) {
  unsigned char data;

  if (sensors & INV_XYZ_GYRO)
    data = INV_CLK_PLL;
  else if (sensors)
    data = 0;
  else
    data = BIT_SLEEP;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data)) {
    st.chip_cfg.sensors = 0;
    return -1;
  }
  st.chip_cfg.clk_src = data & ~BIT_SLEEP;

  data = 0;
  if (!(sensors & INV_X_GYRO))
    data |= BIT_STBY_XG;
  if (!(sensors & INV_Y_GYRO))
    data |= BIT_STBY_YG;
  if (!(sensors & INV_Z_GYRO))
    data |= BIT_STBY_ZG;
  if (!(sensors & INV_XYZ_ACCEL))
    data |= BIT_STBY_XYZA;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_2, 1, &data)) {
    st.chip_cfg.sensors = 0;
    return -1;
  }

  if (sensors && (sensors != INV_XYZ_ACCEL))
    /* Latched interrupts only used in LP accel mode. */
    icm_set_int_latched(0);

  st.chip_cfg.sensors = sensors;
  st.chip_cfg.lp_accel_mode = 0;
  delay_ms(50);
  // sensor_dbg("icm_set_sensors ok \r\n");
  return 0;
}

/**
 *  @brief      Read the MPU interrupt status registers.
 *  @param[out] status  Mask of interrupt bits.
 *  @return     0 if successful.
 */
int icm_get_int_status(short *status) {
  unsigned char tmp[2];
  if (!st.chip_cfg.sensors)
    return -1;
  if (i2c_read(st.hw->addr, st.reg->dmp_int_status, 2, tmp))
    return -1;
  status[0] = (tmp[0] << 8) | tmp[1];
  return 0;
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int icm_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
                  unsigned char *sensors, unsigned char *more) {
  /* Assumes maximum packet size is gyro (6) + accel (6). */
  unsigned char data[MAX_PACKET_LENGTH];
  unsigned char packet_size = 0;
  unsigned short fifo_count, index = 0;

  sensors[0] = 0;
  if (!st.chip_cfg.sensors)
    return -1;
  if (!st.chip_cfg.fifo_enable)
    return -1;

  if (st.chip_cfg.fifo_enable & INV_X_GYRO)
    packet_size += 2;
  if (st.chip_cfg.fifo_enable & INV_Y_GYRO)
    packet_size += 2;
  if (st.chip_cfg.fifo_enable & INV_Z_GYRO)
    packet_size += 2;
  if (st.chip_cfg.fifo_enable & INV_XYZ_ACCEL)
    packet_size += 6;

  if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
    return -1;
  fifo_count = (data[0] << 8) | data[1];
  if (fifo_count < packet_size)
    return 0;
  // log_i("FIFO count: %hd\n", fifo_count);
  if (fifo_count > (st.hw->max_fifo >> 1)) {
    /* FIFO is 50% full, better check overflow bit. */
    if (i2c_read(st.hw->addr, st.reg->int_status, 1, data))
      return -1;
    if (data[0] & BIT_FIFO_OVERFLOW) {
      icm_reset_fifo();
      return -2;
    }
  }
  get_ms((unsigned long*)timestamp);

  if (i2c_read(st.hw->addr, st.reg->fifo_r_w, packet_size, data))
    return -1;
  more[0] = fifo_count / packet_size - 1;
  sensors[0] = 0;

  if ((index != packet_size) && (st.chip_cfg.fifo_enable & INV_XYZ_ACCEL)) {
    accel[0] = (data[index + 0] << 8) | data[index + 1];
    accel[1] = (data[index + 2] << 8) | data[index + 3];
    accel[2] = (data[index + 4] << 8) | data[index + 5];
    sensors[0] |= INV_XYZ_ACCEL;
    index += 6;
  }
  if ((index != packet_size) && (st.chip_cfg.fifo_enable & INV_X_GYRO)) {
    gyro[0] = (data[index + 0] << 8) | data[index + 1];
    sensors[0] |= INV_X_GYRO;
    index += 2;
  }
  if ((index != packet_size) && (st.chip_cfg.fifo_enable & INV_Y_GYRO)) {
    gyro[1] = (data[index + 0] << 8) | data[index + 1];
    sensors[0] |= INV_Y_GYRO;
    index += 2;
  }
  if ((index != packet_size) && (st.chip_cfg.fifo_enable & INV_Z_GYRO)) {
    gyro[2] = (data[index + 0] << 8) | data[index + 1];
    sensors[0] |= INV_Z_GYRO;
    index += 2;
  }

  return 0;
}

/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int icm_set_bypass(unsigned char bypass_on) {
  unsigned char tmp;

  if (st.chip_cfg.bypass_mode == bypass_on)
    return 0;

  if (bypass_on) {
    if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
      return -1;
    tmp &= ~BIT_AUX_IF_EN;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
      return -1;
    delay_ms(3);
    tmp = BIT_BYPASS_EN;
    if (st.chip_cfg.active_low_int)
      tmp |= BIT_ACTL;
    if (st.chip_cfg.latched_int)
      tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
    if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
      return -1;
  } else {
    /* Enable I2C master mode if compass is being used. */
    if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
      return -1;
    if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
      tmp |= BIT_AUX_IF_EN;
    else
      tmp &= ~BIT_AUX_IF_EN;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
      return -1;
    delay_ms(3);
    if (st.chip_cfg.active_low_int)
      tmp = BIT_ACTL;
    else
      tmp = 0;
    if (st.chip_cfg.latched_int)
      tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
    if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
      return -1;
  }
  st.chip_cfg.bypass_mode = bypass_on;
  return 0;
}

/**
 *  @brief      Set interrupt level.
 *  @param[in]  active_low  1 for active low, 0 for active high.
 *  @return     0 if successful.
 */
int icm_set_int_level(unsigned char active_low) {
  st.chip_cfg.active_low_int = active_low;
  return 0;
}

/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int icm_set_int_latched(unsigned char enable) {
  unsigned char tmp;
  if (st.chip_cfg.latched_int == enable)
    return 0;

  if (enable)
    tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
  else
    tmp = 0;
  if (st.chip_cfg.bypass_mode)
    tmp |= BIT_BYPASS_EN;
  if (st.chip_cfg.active_low_int)
    tmp |= BIT_ACTL;
  if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
    return -1;
  st.chip_cfg.latched_int = enable;
  return 0;
}

#define REG_20608_XG_ST_DATA     0x0
#define REG_20608_XA_ST_DATA     0xD
static const unsigned short icm_20608_st_tb[256] = {
      2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,  // 7
      2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,  // 15
      3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,  // 23
      3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,  // 31
      3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,  // 39
      3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,  // 47
      4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,  // 55
      4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,  // 63
      4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,  // 71
      5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,  // 79
      5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,  // 87
      6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,  // 95
      6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,  // 103
      7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,  // 111
      7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,  // 119
      8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
      9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
      10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
      10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
      11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
      12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
      13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
      15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
      16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
      17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
      19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
      20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
      22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
      24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
      26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
      28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
      30903, 31212, 31524, 31839, 32157, 32479, 32804, 33132
    };
static int accel_20608_self_test(long *bias_regular, long *bias_st, int debug) {
  int i, result = 0, otp_value_zero = 0;
  float accel_st_al_min, accel_st_al_max;
  float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], accel_offset_max;
  unsigned char regs[3];
  if (i2c_read(st.hw->addr, REG_20608_XA_ST_DATA, 3, regs)) {
    if (debug)
      log_i("Reading OTP Register Error.\n");
    return 0x07;
  }
  if (debug)
    log_i("Accel OTP:%d, %d, %d\n", regs[0], regs[1], regs[2]);
  for (i = 0; i < 3; i++) {
    if (regs[i] != 0) {
      ct_shift_prod[i] = icm_20608_st_tb[regs[i] - 1];
      ct_shift_prod[i] *= 65536.f;
      ct_shift_prod[i] /= test.accel_sens;
    } else {
      ct_shift_prod[i] = 0;
      otp_value_zero = 1;
    }
  }
  if (otp_value_zero == 0) {
    if (debug)
      log_i("ACCEL:CRITERIA A\n");
    for (i = 0; i < 3; i++) {
      st_shift_cust[i] = bias_st[i] - bias_regular[i];
      if (debug) {
        log_i("Bias_Shift=%7.4f, Bias_Reg=%7.4f, Bias_HWST=%7.4f\r\n",
              st_shift_cust[i] / 1.f, bias_regular[i] / 1.f,
              bias_st[i] / 1.f);
        log_i("OTP value: %7.4f\r\n", ct_shift_prod[i] / 1.f);
      }

      st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i] - 1.f;

      if (debug)
        log_i("ratio=%7.4f, threshold=%7.4f\r\n", st_shift_ratio[i] / 1.f,
              test.max_accel_var / 1.f);

      if (fabs(st_shift_ratio[i]) > test.max_accel_var) {
        if (debug)
          log_i("ACCEL Fail Axis = %d\n", i);
        result |= 1 << i;  // Error condition
      }
    }
  } else {
    /* Self Test Pass/Fail Criteria B */
    accel_st_al_min = test.min_g * 65536.f;
    accel_st_al_max = test.max_g * 65536.f;

    if (debug) {
      log_i("ACCEL:CRITERIA B\r\n");
      log_i("Min MG: %7.4f\r\n", accel_st_al_min / 1.f);
      log_i("Max MG: %7.4f\r\n", accel_st_al_max / 1.f);
    }

    for (i = 0; i < 3; i++) {
      st_shift_cust[i] = bias_st[i] - bias_regular[i];

      if (debug)
        log_i("Bias_shift=%7.4f, st=%7.4f, reg=%7.4f\n", st_shift_cust[i] / 1.f,
              bias_st[i] / 1.f, bias_regular[i] / 1.f);
      if (st_shift_cust[i] < accel_st_al_min || st_shift_cust[i] > accel_st_al_max) {
        if (debug)
          log_i("Accel FAIL axis:%d <= 225mg or >= 675mg\n", i);
        result |= 1 << i;  // Error condition
      }
    }
  }

  if (result == 0) {
    /* Self Test Pass/Fail Criteria C */
    accel_offset_max = test.max_g_offset * 65536.f;
    if (debug)
      log_i("Accel:CRITERIA C: bias less than %7.4f\n", accel_offset_max / 1.f);
    for (i = 0; i < 3; i++) {
      if (fabs(bias_regular[i]) > accel_offset_max) {
        if (debug)
          log_i("FAILED: Accel axis:%d = %ld > 500mg\n", i, bias_regular[i]);
        result |= 1 << i;  // Error condition
      }
    }
  }

  return result;
}

static int gyro_20608_self_test(long *bias_regular, long *bias_st, int debug) {
  int i, result = 0, otp_value_zero = 0;
  float gyro_st_al_max;
  float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], gyro_offset_max;
  unsigned char regs[3];

  if (i2c_read(st.hw->addr, REG_20608_XG_ST_DATA, 3, regs)) {
    if (debug)
      log_i("Reading OTP Register Error.\n");
    return 0x07;
  }

  if (debug)
    log_i("Gyro OTP:%d, %d, %d\r\n", regs[0], regs[1], regs[2]);

  for (i = 0; i < 3; i++) {
    if (regs[i] != 0) {
      ct_shift_prod[i] = icm_20608_st_tb[regs[i] - 1];
      ct_shift_prod[i] *= 65536.f;
      ct_shift_prod[i] /= test.gyro_sens;
    } else {
      ct_shift_prod[i] = 0;
      otp_value_zero = 1;
    }
  }

  if (otp_value_zero == 0) {
    if (debug)
      log_i("GYRO:CRITERIA A\n");
    /* Self Test Pass/Fail Criteria A */
    for (i = 0; i < 3; i++) {
      st_shift_cust[i] = bias_st[i] - bias_regular[i];

      if (debug) {
        log_i("Bias_Shift=%7.4f, Bias_Reg=%7.4f, Bias_HWST=%7.4f\r\n",
              st_shift_cust[i] / 1.f, bias_regular[i] / 1.f,
              bias_st[i] / 1.f);
        log_i("OTP value: %7.4f\r\n", ct_shift_prod[i] / 1.f);
      }

      st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i];

      if (debug)
        log_i("ratio=%7.4f, threshold=%7.4f\r\n", st_shift_ratio[i] / 1.f,
              test.max_gyro_var / 1.f);

      if (fabs(st_shift_ratio[i]) < test.max_gyro_var) {
        if (debug)
          log_i("Gyro Fail Axis = %d\n", i);
        result |= 1 << i;
      }
    }
  } else {
    /* Self Test Pass/Fail Criteria B */
    gyro_st_al_max = test.max_dps * 65536.f;

    if (debug) {
      log_i("GYRO:CRITERIA B\r\n");
      log_i("Max DPS: %7.4f\r\n", gyro_st_al_max / 1.f);
    }

    for (i = 0; i < 3; i++) {
      st_shift_cust[i] = bias_st[i] - bias_regular[i];

      if (debug)
        log_i("Bias_shift=%7.4f, st=%7.4f, reg=%7.4f\n", st_shift_cust[i] / 1.f,
               bias_st[i] / 1.f, bias_regular[i] / 1.f);
      if (st_shift_cust[i] < gyro_st_al_max) {
        if (debug)
          log_i("GYRO FAIL axis:%d greater than 60dps\n", i);
        result |= 1 << i;
      }
    }
  }

  if (result == 0) {
    /* Self Test Pass/Fail Criteria C */
    gyro_offset_max = test.min_dps * 65536.f;
    if (debug)
      log_i("Gyro:CRITERIA C: bias less than %7.4f\n", gyro_offset_max / 1.f);
    for (i = 0; i < 3; i++) {
      if (fabs(bias_regular[i]) > gyro_offset_max) {
        if (debug)
          log_i("FAILED: Gyro axis:%d = %ld > 20dps\n", i, bias_regular[i]);
        result |= 1 << i;
      }
    }
  }
  return result;
}

static int get_st_20608_biases(long *gyro, long *accel, unsigned char hw_test, int debug) {
  unsigned char data[HWST_MAX_PACKET_LENGTH];
  unsigned char packet_count, ii;
  unsigned short fifo_count;
  int s = 0, read_size = 0, ind;

  data[0] = 0x01;
  data[1] = 0;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
    return -1;
  delay_ms(200);
  data[0] = 0;
  if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
    return -1;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
    return -1;
  data[0] = BIT_FIFO_RST | BIT_DMP_RST;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
    return -1;
  delay_ms(15);
  data[0] = st.test->reg_lpf;
  if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
    return -1;
  data[0] = st.test->reg_rate_div;
  if (i2c_write(st.hw->addr, st.reg->rate_div, 1, data))
    return -1;
  if (hw_test)
    data[0] = st.test->reg_gyro_fsr | 0xE0;
  else
    data[0] = st.test->reg_gyro_fsr;
  if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, data))
    return -1;

  if (hw_test)
    data[0] = st.test->reg_accel_fsr | 0xE0;
  else
    data[0] = test.reg_accel_fsr;
  if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
    return -1;

  delay_ms(test.wait_ms);  // wait 200ms for sensors to stabilize

  /* Enable FIFO */
  data[0] = BIT_FIFO_EN;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
    return -1;
  data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
    return -1;

  // initialize the bias return values
  gyro[0] = gyro[1] = gyro[2] = 0;
  accel[0] = accel[1] = accel[2] = 0;

  if (debug)
    log_i("Starting Bias Loop Reads\n");

  // start reading samples
  while (s < test.packet_thresh) {
    delay_ms(test.sample_wait_ms);  // wait 10ms to fill FIFO
    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
      return -1;
    fifo_count = (data[0] << 8) | data[1];
    packet_count = fifo_count / MAX_PACKET_LENGTH;
    if ((test.packet_thresh - s) < packet_count)
      packet_count = test.packet_thresh - s;
    read_size = packet_count * MAX_PACKET_LENGTH;

    // burst read from FIFO
    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, read_size, data))
      return -1;
    ind = 0;
    for (ii = 0; ii < packet_count; ii++) {
      short accel_cur[3], gyro_cur[3];
      accel_cur[0] = ((short)data[ind + 0] << 8) | data[ind + 1];
      accel_cur[1] = ((short)data[ind + 2] << 8) | data[ind + 3];
      accel_cur[2] = ((short)data[ind + 4] << 8) | data[ind + 5];
      accel[0] += (long)accel_cur[0];
      accel[1] += (long)accel_cur[1];
      accel[2] += (long)accel_cur[2];
      gyro_cur[0] = (((short)data[ind + 6] << 8) | data[ind + 7]);
      gyro_cur[1] = (((short)data[ind + 8] << 8) | data[ind + 9]);
      gyro_cur[2] = (((short)data[ind + 10] << 8) | data[ind + 11]);
      gyro[0] += (long)gyro_cur[0];
      gyro[1] += (long)gyro_cur[1];
      gyro[2] += (long)gyro_cur[2];
      ind += MAX_PACKET_LENGTH;
    }
    s += packet_count;
  }

  if (debug)
    log_i("Samples: %d\n", s);

  // stop FIFO
  data[0] = 0;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
    return -1;

  gyro[0] = (long)(((long long)gyro[0] << 16) / test.gyro_sens / s);
  gyro[1] = (long)(((long long)gyro[1] << 16) / test.gyro_sens / s);
  gyro[2] = (long)(((long long)gyro[2] << 16) / test.gyro_sens / s);
  accel[0] = (long)(((long long)accel[0] << 16) / test.accel_sens / s);
  accel[1] = (long)(((long long)accel[1] << 16) / test.accel_sens / s);
  accel[2] = (long)(((long long)accel[2] << 16) / test.accel_sens / s);
  /* remove gravity from bias calculation */
  if (accel[2] > 0L)
    accel[2] -= 65536L;
  else
    accel[2] += 65536L;

  if (debug) {
    log_i("Accel offset data HWST bit=%d: %7.4f %7.4f %7.4f\r\n", hw_test, accel[0] / 65536.f,
           accel[1] / 65536.f, accel[2] / 65536.f);
    log_i("Gyro offset data HWST bit=%d: %7.4f %7.4f %7.4f\r\n", hw_test, gyro[0] / 65536.f,
           gyro[1] / 65536.f, gyro[2] / 65536.f);
  }

  return 0;
}
/**
 *  @brief      Trigger gyro/accel self-test for ICM20608
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Gyro.
 *  \n Bit 1:   Accel.
 *
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @param[in]  debug       Debug flag used to print out more detailed logs.
 *  @return     Result mask (see above).
 */
int icm_get_sensor_data(long *gyro, long *accel, unsigned char debug) {
  int ii;
  int result = 0;

  unsigned char data[HWST_MAX_PACKET_LENGTH];
  unsigned char packet_count;
  unsigned short fifo_count;
  int s = 0, read_size = 0, ind;

  /* Enable FIFO */
  data[0] = BIT_FIFO_EN;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
    return -1;

  // initialize the bias return values
  gyro[0] = gyro[1] = gyro[2] = 0;
  accel[0] = accel[1] = accel[2] = 0;

  // start reading samples
  while (s < test.packet_thresh) {
    // wait 10ms to fill FIFO
    CyU3PThreadSleep(10);
    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
      return -1;

    fifo_count = (data[0] << 8) | data[1];
    packet_count = fifo_count / MAX_PACKET_LENGTH;

    if ((test.packet_thresh - s) < packet_count)
      packet_count = test.packet_thresh - s;
    read_size = packet_count * MAX_PACKET_LENGTH;

    // burst read from FIFO
    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, read_size, data))
      return -1;
    ind = 0;
    for (ii = 0; ii < packet_count; ii++) {
      short accel_cur[3], gyro_cur[3];
      accel_cur[0] = ((short)data[ind + 0] << 8) | data[ind + 1];
      accel_cur[1] = ((short)data[ind + 2] << 8) | data[ind + 3];
      accel_cur[2] = ((short)data[ind + 4] << 8) | data[ind + 5];
      accel[0] += (long)accel_cur[0];
      accel[1] += (long)accel_cur[1];
      accel[2] += (long)accel_cur[2];
      gyro_cur[0] = (((short)data[ind + 6] << 8) | data[ind + 7]);
      gyro_cur[1] = (((short)data[ind + 8] << 8) | data[ind + 9]);
      gyro_cur[2] = (((short)data[ind + 10] << 8) | data[ind + 11]);
      gyro[0] += (long)gyro_cur[0];
      gyro[1] += (long)gyro_cur[1];
      gyro[2] += (long)gyro_cur[2];
      ind += MAX_PACKET_LENGTH;
    }
    s += packet_count;
  }

  // stop FIFO
  data[0] = 0;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
    return -1;

  gyro[0] = (long)(((long long)gyro[0] << 16) / test.gyro_sens / s);
  gyro[1] = (long)(((long long)gyro[1] << 16) / test.gyro_sens / s);
  gyro[2] = (long)(((long long)gyro[2] << 16) / test.gyro_sens / s);
  accel[0] = (long)(((long long)accel[0] << 16) / test.accel_sens / s);
  accel[1] = (long)(((long long)accel[1] << 16) / test.accel_sens / s);
  accel[2] = (long)(((long long)accel[2] << 16) / test.accel_sens / s);

  /* since this is set to +/- 2G */
  if (accel[2] > 0L)
    accel[2] -= 65536L;
  else
    accel[2] += 65536L;

  return result;
}

int icm_burst_read_data(unsigned char* sensor_data) {
  int result = 0;

  unsigned char data[HWST_MAX_PACKET_LENGTH];
  int read_size = 0;

  /* Enable FIFO */
  data[0] = BIT_FIFO_EN;
  if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
    return -1;

  read_size = MAX_PACKET_LENGTH;

  i2c_read(st.hw->addr, st.reg->fifo_r_w, read_size, data);

  memcpy(sensor_data, data, read_size);

  // stop FIFO
  data[0] = 0;
  if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
    return -1;

  return result;
}

/**
 *  @brief      Trigger gyro/accel self-test for ICM20608
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Gyro.
 *  \n Bit 1:   Accel.
 *
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @param[in]  debug       Debug flag used to print out more detailed logs.
 *  @return     Result mask (see above).
 */
int icm_run_self_test(long *gyro, long *accel, unsigned char debug) {
  const unsigned char tries = 2;
  long gyro_st[3], accel_st[3];
  unsigned char accel_result = 0, gyro_result = 0;
  int ii;

  int result;
  unsigned char accel_fsr, fifo_sensors, sensors_on;
  unsigned short gyro_fsr, sample_rate, gyro_lpf, accel_lpf;

  if (debug)
    log_i("Starting ICM20608 HWST!\r\n");

  /* Get initial settings. */
  icm_get_gyro_fsr(&gyro_fsr);
  icm_get_accel_fsr(&accel_fsr);
  icm_get_gyro_lpf(&gyro_lpf);
  icm_get_accel_lpf(&accel_lpf);
  icm_get_sample_rate(&sample_rate);
  sensors_on = st.chip_cfg.sensors;
  icm_get_fifo_config(&fifo_sensors);

  sensor_dbg("sensors_on is%d ,sample_rate is %d\r\n", sensors_on, sample_rate);
  if (debug)
    log_i("Retrieving Biases\r\n");

  for (ii = 0; ii < tries; ii++)
    if (!get_st_20608_biases(gyro, accel, 0, debug))
      break;
  if (ii == tries) {
    /* If we reach this point, we most likely encountered an I2C error.
     * We'll just report an error for all three sensors.
     */
    if (debug)
      log_i("Retrieving Biases Error - possible I2C error\n");

    result = 0;
    goto restore;
  }

  if (debug)
    log_i("Retrieving ST Biases\n");

  for (ii = 0; ii < tries; ii++)
    if (!get_st_20608_biases(gyro_st, accel_st, 1, debug))
      break;
  if (ii == tries) {
    if (debug)
      log_i("Retrieving ST Biases Error - possible I2C error\n");

    /* Again, probably an I2C error. */
    result = 0;
    goto restore;
  }
#if 1
  sensor_dbg("Starting accel_20608_self_test... \r\n");
  accel_result = accel_20608_self_test(accel, accel_st, debug);
  if (debug) {
    log_i("Accel Self Test Results: %d\n", accel_result);
  }
#else
  sensor_dbg("Skipping accel_20608_self_test...\r\n");
  icm_check_whoami_debug();
#endif

  sensor_dbg("Starting gyro_20608_self_test...\r\n");
  gyro_result = gyro_20608_self_test(gyro, gyro_st, debug);
  if (debug) {
    log_i("Gyro Self Test Results: %d\n", gyro_result);
  }

  result = 0;
  if (!gyro_result)
    result |= 0x01;
  if (!accel_result)
    result |= 0x02;

  sensor_dbg("Exiting HWST...result is:%d\r\n", result);

restore:
  if (debug)
    log_i("Exiting HWST\n");
  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg.gyro_fsr = 0xFF;
  st.chip_cfg.accel_fsr = 0xFF;
  st.chip_cfg.gyro_lpf = 0xFF;
  st.chip_cfg.accel_lpf = 0xFF;
  st.chip_cfg.sample_rate = 0xFFFF;
  st.chip_cfg.sensors = 0xFF;
  st.chip_cfg.fifo_enable = 0xFF;
  st.chip_cfg.clk_src = INV_CLK_PLL;
  icm_set_gyro_fsr(gyro_fsr);
  icm_set_accel_fsr(accel_fsr);
  icm_set_gyro_lpf(gyro_lpf);
  icm_set_accel_lpf(accel_lpf);
  icm_set_sample_rate(sample_rate);
  icm_set_sensors(sensors_on);
  icm_configure_fifo(fifo_sensors);

  sensor_dbg("Exiting HWST \r\n");
  return result;
}


int icm_run_self_test_debug(long *gyro, long *accel, unsigned char debug) {
  const unsigned char tries = 2;
  long gyro_st[3], accel_st[3];
  unsigned char accel_result, gyro_result;
  int ii;

  int result = 3;
  unsigned char accel_fsr, fifo_sensors, sensors_on;
  unsigned short gyro_fsr, sample_rate, gyro_lpf, accel_lpf;

  if (debug)
    log_i("Starting ICM20608 HWST!\r\n");

  /* Get initial settings. */
  icm_get_gyro_fsr(&gyro_fsr);
  icm_get_accel_fsr(&accel_fsr);
  icm_get_gyro_lpf(&gyro_lpf);
  icm_get_accel_lpf(&accel_lpf);
  icm_get_sample_rate(&sample_rate);

  sensors_on = st.chip_cfg.sensors;

  icm_get_fifo_config(&fifo_sensors);

  if (debug)
    log_i("Retrieving Biases\r\n");

  for (ii = 0; ii < tries; ii++) {
    if (!get_st_20608_biases(gyro, accel, 0, debug)) {
      break;
    }
  }

  if (ii == tries) {
    /* If we reach this point, we most likely encountered an I2C error.
     * We'll just report an error for all three sensors.
     */

    if (debug)
      log_i("Retrieving Biases Error - possible I2C error\n");

    result = 0;
    goto restore;
  }

  if (debug)
    log_i("Retrieving ST Biases\n");

  for (ii = 0; ii < tries; ii++) {
    if (!get_st_20608_biases(gyro_st, accel_st, 1, debug))
      break;
  }

  if (ii == tries) {
    if (debug)
      log_i("Retrieving ST Biases Error - possible I2C error\n");

    /* Again, probably an I2C error. */
    result = 0;
    goto restore;
  }

  accel_result = accel_20608_self_test(accel, accel_st, debug);
  if (debug)
    log_i("Accel Self Test Results: %d\n", accel_result);

  gyro_result = gyro_20608_self_test(gyro, gyro_st, debug);
  if (debug)
    log_i("Gyro Self Test Results: %d\n", gyro_result);

  result = 0;
  if (!gyro_result)
    result |= 0x01;
  if (!accel_result)
    result |= 0x02;

restore:
  if (debug)
    log_i("Exiting HWST\n");
  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg.gyro_fsr = 0xFF;
  st.chip_cfg.accel_fsr = 0xFF;
  st.chip_cfg.gyro_lpf = 0xFF;
  st.chip_cfg.accel_lpf = 0xFF;
  st.chip_cfg.sample_rate = 0xFFFF;
  st.chip_cfg.sensors = 0xFF;
  st.chip_cfg.fifo_enable = 0xFF;
  st.chip_cfg.clk_src = INV_CLK_PLL;
  icm_set_gyro_fsr(gyro_fsr);
  icm_set_accel_fsr(accel_fsr);
  icm_set_gyro_lpf(gyro_lpf);
  icm_set_accel_lpf(accel_lpf);
  icm_set_sample_rate(sample_rate);
  icm_set_sensors(sensors_on);
  icm_configure_fifo(fifo_sensors);

  return result;
}

/**
 *  @brief      Enters LP accel motion interrupt mode.
 *
 *  \n The hardware motion threshold can be between 4mg and 1020mg in 4mg
 *  increments.
 *
 *  \n ICM20645 Low-power accel mode supports the following frequencies
 *  \n up to 500Hz. Exact frequencies are
 *  \n .24Hz, .49Hz, .98Hz, 1.95Hz, 3.91Hz, 7.81Hz, 15.63Hz, 31.25Hz, 62.5Hz
 *  \n 125Hz, 250Hz, 500Hz
 *  \n API will match the input freq to the closest match
 *
 *  \n\n NOTES:
 *  \n The driver will round down @e thresh to the nearest supported value if
 *  an unsupported threshold is selected.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e lpa_freq.
 *  \n To disable this mode, set @e lpa_freq to zero. The driver will restore
 *  the previous configuration.
 *
 *  @param[in]  thresh      Motion threshold in mg.
 *  @param[in]  time        Duration in milliseconds that the accel data must
 *                          exceed @e thresh before motion is reported.
 *  @param[in]  lpa_freq    Minimum sampling rate, or zero to disable.
 *  @return     0 if successful.
 */
int icm_lp_motion_interrupt(unsigned short thresh, unsigned char time,
                            unsigned short lpa_freq) {
  unsigned char data[3], temp[1];;
  if (lpa_freq) {
    unsigned char thresh_hw;

    /* 1LSb = 4mg. */
    if (thresh > 1020)
      thresh_hw = 255;
    else if (thresh < 4)
      thresh_hw = 1;
    else
      thresh_hw = thresh >> 2;
    if (!time)
      /* Minimum duration must be 1ms. */
      time = 1;

    if (lpa_freq > 640)
      /* At this point, the chip has not been re-configured, so the
       * function can safely exit.
       */
      return -1;

    if (!st.chip_cfg.int_motion_only) {
      /* Store current settings for later. */
      icm_get_gyro_fsr(&st.chip_cfg.cache.gyro_fsr);
      icm_get_accel_fsr(&st.chip_cfg.cache.accel_fsr);
      icm_get_gyro_lpf(&st.chip_cfg.cache.gyro_lpf);
      icm_get_accel_lpf(&st.chip_cfg.cache.accel_lpf);
      icm_get_sample_rate(&st.chip_cfg.cache.sample_rate);
      st.chip_cfg.cache.sensors_on = st.chip_cfg.sensors;
      icm_get_fifo_config(&st.chip_cfg.cache.fifo_sensors);
    }

    /* Disable hardware interrupts. */
    set_int_enable(0);

    /* Enter full-power accel-only mode, no FIFO/DMP. */
    data[0] = 0;
    data[1] = 0;
    data[2] = BIT_STBY_XYZG;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 3, data))
      goto lp_int_restore;

    /* Set motion threshold. */
    data[0] = thresh_hw;
    if (i2c_write(st.hw->addr, st.reg->motion_thr, 1, data))
      goto lp_int_restore;

    /* Set wake frequency. */
    if (lpa_freq == 1)
      data[0] = INV_LPA_0_98HZ;
    else if (lpa_freq == 2)
      data[0] = INV_LPA_1_95HZ;
    else if (lpa_freq <= 5)
      data[0] = INV_LPA_3_91HZ;
    else if (lpa_freq <= 10)
      data[0] = INV_LPA_7_81HZ;
    else if (lpa_freq <= 20)
      data[0] = INV_LPA_15_63HZ;
    else if (lpa_freq <= 40)
      data[0] = INV_LPA_31_25HZ;
    else if (lpa_freq <= 80)
      data[0] = INV_LPA_62_5HZ;
    else if (lpa_freq <= 160)
      data[0] = INV_LPA_125HZ;
    else if (lpa_freq <= 320)
      data[0] = INV_LPA_250HZ;
    else
      data[0] = INV_LPA_500HZ;
    if (i2c_write(st.hw->addr, st.reg->lp_mode_cfg, 1, data))
      goto lp_int_restore;

    /* Enable motion interrupt (ICM20608 version). */
    data[0] = BITS_WOM_EN;
    if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data))
      goto lp_int_restore;
    /* Enable LP ACCEL mode, ACCEL_FCHOICE_B=1*/
    if (i2c_read(st.hw->addr, st.reg->accel_cfg2, 1, temp))
      return -1;

    /*Bypass accel DLPF. */
    data[0] = BIT_ACCL_FC_B | temp[0];
    if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, data))
      goto lp_int_restore;

    /* Enable interrupt. */
    data[0] = BIT_MOT_INT_EN;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
      goto lp_int_restore;

    /* Enable cycle mode. */
    data[0] = BIT_LPA_CYCLE;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
      goto lp_int_restore;
    st.chip_cfg.int_motion_only = 1;
    return 0;
  } else {
    /* Don't "restore" the previous state if no state has been saved. */
    unsigned int ii;
    char *cache_ptr = (char*)&st.chip_cfg.cache;
    for (ii = 0; ii < sizeof(st.chip_cfg.cache); ii++) {
      if (cache_ptr[ii] != 0)
        goto lp_int_restore;
    }
    /* If we reach this point, motion interrupt mode hasn't been used yet. */
    return -1;
  }
lp_int_restore:
  /* Set to invalid values to ensure no I2C writes are skipped. */
  st.chip_cfg.gyro_fsr = 0xFF;
  st.chip_cfg.accel_fsr = 0xFF;
  st.chip_cfg.gyro_lpf = 0xFF;
  st.chip_cfg.accel_lpf = 0xFF;
  st.chip_cfg.sample_rate = 0xFFFF;
  st.chip_cfg.sensors = 0xFF;
  st.chip_cfg.fifo_enable = 0xFF;
  st.chip_cfg.clk_src = INV_CLK_PLL;
  icm_set_sensors(st.chip_cfg.cache.sensors_on);
  icm_set_gyro_fsr(st.chip_cfg.cache.gyro_fsr);
  icm_set_accel_fsr(st.chip_cfg.cache.accel_fsr);
  icm_set_gyro_lpf(st.chip_cfg.cache.gyro_lpf);
  icm_set_accel_lpf(st.chip_cfg.cache.accel_lpf);
  icm_set_sample_rate(st.chip_cfg.cache.sample_rate);
  icm_configure_fifo(st.chip_cfg.cache.fifo_sensors);

  /* Disable motion interrupt. */
  data[0] = 0;
  if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data))
    goto lp_int_restore;

  st.chip_cfg.int_motion_only = 0;
  return 0;
}
