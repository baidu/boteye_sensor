/*
## Cypress USB 3.0 Platform source file
## ===========================
##
##  Copyright Cypress Semiconductor Corporation, 2010-2011,
##  All Rights Reserved
##  UNPUBLISHED, LICENSED SOFTWARE.
##
##  CONFIDENTIAL AND PROPRIETARY INFORMATION
##  WHICH IS THE PROPERTY OF CYPRESS.
##
##  Use of this file is governed
##  by the license agreement included in the file
##
##     <install>/license/license.txt
##
##  where <install> is the Cypress software
##  installation root directory path.
##
## ===========================
*/

#include <cyu3usb.h>
#include <cyu3error.h>
#include <cyu3spi.h>
#include "include/debug.h"
#include "include/extension_unit.h"
#include "include/sensor_v034_raw.h"
#include "include/fx3_bsp.h"
#include "include/uvc.h"
#include "include/xp_sensor_firmware_version.h"
#include "include/inv_icm20608.h"

uint16_t glSpiPageSize = 0x100;  /* SPI Page size to be used for transfers. */
/* Give a timeout value of 5s for any flash programming. */
#define CY_FX_FLASH_PROG_TIMEOUT                (5000)
CyU3PDmaChannel glSpiTxHandle;   /* SPI Tx channel handle */
CyU3PDmaChannel glSpiRxHandle;   /* SPI Rx channel handle */
// uint8_t glEp0Buf[256] __attribute__ ((aligned (32)));
static uint8_t glEp0Buf[256];
static CyU3PReturnStatus_t CyFxFlashProgSpiWaitForStatus(void);

/* Array to hold sensor data */
volatile char last_imu[IMU_BURST_LEN] = {' '};

/**
 *  @brief      handle single imu read/write of extension unit request.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
void EU_Rqts_imu_rw(uint8_t bRequest) {
  #define CMD_IMU_RW_LEN 5
  #define CMD_IMU_RW_R 0
  #define CMD_IMU_RW_W 1

  static uint8_t regval;
  uint8_t Ep0Buffer[32];
  uint16_t readCount;
  uint16_t regaddr;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  switch (bRequest) {
  case CY_FX_USB_UVC_GET_CUR_REQ:
    Ep0Buffer[0] = 0;
    Ep0Buffer[1] = 0;
    Ep0Buffer[2] = 0;
    Ep0Buffer[3] = 0;
    Ep0Buffer[4] = regval;
    CyU3PUsbSendEP0Data(CMD_IMU_RW_LEN, (uint8_t *)Ep0Buffer);
    // sensor_dbg("EU request IMU read reg val: 0x%x\r\n", regval);
    break;
  case CY_FX_USB_UVC_SET_CUR_REQ:
    apiRetStatus = CyU3PUsbGetEP0Data(CMD_IMU_RW_LEN, Ep0Buffer, &readCount);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      sensor_err("CyU3 get Ep0 data failed\r\n");
      CyFxAppErrorHandler(apiRetStatus);
    }
    regaddr = (Ep0Buffer[1] << 8) + Ep0Buffer[2];
    // sensor_dbg("regaddr: 0x%x\r\n", regaddr);
    if (Ep0Buffer[0] == CMD_IMU_RW_R) {
      icm_read_reg(regaddr, &regval);
    } else if (Ep0Buffer[0] == CMD_IMU_RW_W) {
      regval = (Ep0Buffer[3] << 8) + Ep0Buffer[4];
      icm_write_reg(regaddr, regval);
      sensor_info("warning: icm reg set write addr: 0x%x, val: 0x%x\r\n", regaddr, regval);
    }
    break;
  case CY_FX_USB_UVC_GET_LEN_REQ:
    Ep0Buffer[0] = CMD_IMU_RW_LEN;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    // sensor_dbg("EU request IMU_RW get len: 0x%x\r\n", Ep0Buffer[0]);
    break;
  case CY_FX_USB_UVC_GET_INFO_REQ:
    // sensor_dbg("EU request IMU_RW get info\r\n");
    Ep0Buffer[0] = 3;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    break;
  default:
    sensor_dbg("unknown reg rw cmd: 0x%x\r\n", bRequest);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/**
 *  @brief      handle burst imu read of extension unit request.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
void EU_Rqts_imu_burst(uint8_t bRequest) {
  uint8_t Ep0Buffer[32];
  #ifndef IMU_LOOP_SAMPLE
  int status;
  uint8_t raw_IMU_data[14];
  #endif

  switch (bRequest) {
  case CY_FX_USB_UVC_GET_CUR_REQ:
    // sensor_dbg("EU request IMU burst get cur xu\r\n");
    #ifndef IMU_LOOP_SAMPLE
    status = icm_get_sensor_reg(raw_IMU_data, 0);
    if (status != CY_U3P_SUCCESS) {
        sensor_err("get icm data err\r\n");
    }
    int i = 0;
    for (i = 0; i < 6; ++i) {
      last_imu[i + 0] = (char)(raw_IMU_data[i]);
      last_imu[i + 6] = (char)(raw_IMU_data[i + 8]);
    }
    uint32_t t = CyU3PGetTime();
    last_imu[12] = t >> 24;
    last_imu[13] = t >> 16;
    last_imu[14] = t >> 8;
    last_imu[15] = t >> 0;
    #endif
    CyU3PUsbSendEP0Data(IMU_BURST_LEN, (uint8_t *)last_imu);
    break;
  case CY_FX_USB_UVC_SET_CUR_REQ:
    sensor_dbg("EU request IMU burst set cur xu\r\n");
    break;
  case CY_FX_USB_UVC_GET_LEN_REQ:
    Ep0Buffer[0] = IMU_BURST_LEN;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    sensor_dbg("EU request IMU burst get len: 0x%x\r\n", Ep0Buffer[0]);
    break;
  case CY_FX_USB_UVC_GET_INFO_REQ:
    sensor_dbg("EU request IMU burst get info\r\n");
    Ep0Buffer[0] = 3;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    break;
  default:
    sensor_dbg("unknown reg burst cmd: 0x%x\r\n", bRequest);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/**
 *  @brief      handle spi flash action of extension unit request.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
void EU_Rqts_spi_flash(uint8_t bRequest) {
  uint8_t Ep0Buffer[32];
  uint16_t readCount;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  switch (bRequest) {
  case CY_FX_USB_UVC_GET_CUR_REQ:
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_SET_CUR_REQ:
    apiRetStatus = CyU3PUsbGetEP0Data(1, Ep0Buffer, &readCount);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      sensor_err("CyU3 get Ep0 data failed\r\n");
      CyFxAppErrorHandler(apiRetStatus);
    }

    if (Ep0Buffer[0] == 'E') {
      sensor_dbg("erase flash\r\n");
      CyFxFlashProgEraseSector(CyTrue, 0, glEp0Buf);
      // CyU3PThreadSleep(10000);
    }
    if (Ep0Buffer[0] == 'R') {
      sensor_dbg("read flash\r\n");
      /* replace this with SPI  API to erase the flash */
      CyU3PMemSet (glEp0Buf, 0, sizeof (glEp0Buf));
      apiRetStatus = CyFxFlashProgSpiTransfer(1, sizeof (glEp0Buf), glEp0Buf, CyTrue);
    }
    if (Ep0Buffer[0] == 'W') {
      sensor_dbg("write flash\r\n");
      /* replace this with SPI  API to erase the flash */
      CyU3PMemSet(glEp0Buf, 100, sizeof (glEp0Buf));
      apiRetStatus = CyFxFlashProgSpiTransfer(1, sizeof (glEp0Buf), glEp0Buf, CyFalse);
    }
    break;
  case CY_FX_USB_UVC_GET_LEN_REQ:
    Ep0Buffer[0] = 1;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_INFO_REQ:
    Ep0Buffer[0] = 3;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    break;
  default:
    sensor_err("unknown flash cmd: 0x%x\r\n", bRequest);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/**
 *  @brief      handle camera chip register of extension unit request.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
void EU_Rqts_cam_reg(uint8_t bRequest) {
  static uint16_t read_reg_addr = 0;
  uint8_t Ep0Buffer[32] = {0};
  uint8_t HighAddr;
  uint8_t LowAddr;
  uint8_t HighData;
  uint8_t LowData;
  uint16_t readval;
  uint16_t readCount;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  switch (bRequest) {
  case CY_FX_USB_UVC_GET_CUR_REQ:
    // sensor_dbg("EU request camera reg get cur\r\n");
    Ep0Buffer[0] = read_reg_addr >> 8;
    Ep0Buffer[1] = read_reg_addr & 0xFF;

    readval = V034_RegisterRead(Ep0Buffer[0], Ep0Buffer[1]);

    Ep0Buffer[2] = readval >> 8;
    Ep0Buffer[3] = readval & 0xFF;
    CyU3PUsbSendEP0Data(4, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_SET_CUR_REQ:
    // sensor_dbg("EU request camera reg set cur\r\n");
    apiRetStatus = CyU3PUsbGetEP0Data(1, Ep0Buffer, &readCount);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      sensor_err("CyU3 get Ep0 data failed\r\n");
      CyFxAppErrorHandler(apiRetStatus);
    }
   /*
    * We use the MSB of Ep0Buffer[0] to mark what Ep0Buffer really stores:
    * 1) The reg_addr and reg_val for a write operation
    * 2) 0nly the reg_addr for the following read operation
    */
    if (!(Ep0Buffer[0] & 0x80)) {
      HighAddr = Ep0Buffer[0];
      LowAddr  = Ep0Buffer[1];
      HighData = Ep0Buffer[2];
      LowData  = Ep0Buffer[3];
      V034_RegisterWrite(HighAddr, LowAddr, HighData, LowData);
      CyU3PThreadSleep(1);
    } else {
      read_reg_addr = Ep0Buffer[0] << 8| Ep0Buffer[1];
    }
    break;
  case CY_FX_USB_UVC_GET_LEN_REQ:
    sensor_dbg("EU request camera reg get len\r\n");
    Ep0Buffer[0] = 4;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_INFO_REQ:
    sensor_dbg("EU request camera reg get info\r\n");
    Ep0Buffer[0] = 3;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    break;
  default:
    sensor_err("unknown cam reg cmd: 0x%x\r\n", bRequest);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/**
 *  @brief      handle soft version information read of extension unit request.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
void EU_Rqts_soft_version(uint8_t bRequest) {
  uint8_t Ep0Buffer[32];
  uint16_t readCount;
  char *version_info = FIRMWARE_VERSION;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  switch (bRequest) {
  case CY_FX_USB_UVC_GET_CUR_REQ:
    CyU3PUsbSendEP0Data(strlen(version_info), (uint8_t *)version_info);
    sensor_dbg("EU software version get cur request\r\n");
    break;
  case CY_FX_USB_UVC_SET_CUR_REQ:
    apiRetStatus = CyU3PUsbGetEP0Data(1, Ep0Buffer, &readCount);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      sensor_err("CyU3 get Ep0 data failed\r\n");
      CyFxAppErrorHandler(apiRetStatus);
    }
    sensor_dbg("EU software version set cur request\r\n");
    break;
  case CY_FX_USB_UVC_GET_LEN_REQ:
    Ep0Buffer[0] = strlen(version_info);
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    sensor_dbg("EU software version get len:%d request\r\n", Ep0Buffer[0]);
    break;
  case CY_FX_USB_UVC_GET_INFO_REQ:
    Ep0Buffer[0] = 3;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    sensor_dbg("EU software version get info request\r\n");
    break;
  default:
    sensor_err("unknown software version cmd: 0x%x\r\n", bRequest);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/**
 *  @brief      handle hard version information read of extension unit request.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
void EU_Rqts_hard_version(uint8_t bRequest) {
  uint8_t Ep0Buffer[32];
  uint16_t readCount;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  switch (bRequest) {
  case CY_FX_USB_UVC_GET_CUR_REQ:
    Ep0Buffer[0] = hardware_version_num >> 24;
    Ep0Buffer[1] = hardware_version_num >> 16;
    Ep0Buffer[2] = hardware_version_num >> 8;
    Ep0Buffer[3] = hardware_version_num >> 0;
    CyU3PUsbSendEP0Data(4, Ep0Buffer);
    sensor_dbg("EU hardware version get cur request\r\n");
    break;
  case CY_FX_USB_UVC_SET_CUR_REQ:
    apiRetStatus = CyU3PUsbGetEP0Data(1, Ep0Buffer, &readCount);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      sensor_err("CyU3 get Ep0 data failed\r\n");
      CyFxAppErrorHandler(apiRetStatus);
    }
    sensor_dbg("EU hardware version set cur request\r\n");
    break;
  case CY_FX_USB_UVC_GET_LEN_REQ:
    Ep0Buffer[0] = 4;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    sensor_dbg("EU hardware version get len:%d request\r\n", Ep0Buffer[0]);
    break;
  case CY_FX_USB_UVC_GET_INFO_REQ:
    Ep0Buffer[0] = 3;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    sensor_dbg("EU hardware version get info request\r\n");
    break;
  default:
    sensor_err("unknown hardware version cmd: 0x%x\r\n", bRequest);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}
/**
 *  @brief      control firmware flag.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
void EU_Rqts_firmware_flag(uint8_t bRequest) {
  uint8_t Ep0Buffer[32];
  uint16_t readCount;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  switch (bRequest) {
  case CY_FX_USB_UVC_GET_CUR_REQ:
    sensor_dbg("EU firmware flag get firmware_ctrl_flag: 0x%x\r\n", firmware_ctrl_flag);
    Ep0Buffer[0] = firmware_ctrl_flag >> 24;
    Ep0Buffer[1] = firmware_ctrl_flag >> 16;
    Ep0Buffer[2] = firmware_ctrl_flag >> 8;
    Ep0Buffer[3] = firmware_ctrl_flag >> 0;
    CyU3PUsbSendEP0Data(4, Ep0Buffer);
    break;
  case CY_FX_USB_UVC_SET_CUR_REQ:
    apiRetStatus = CyU3PUsbGetEP0Data(4, Ep0Buffer, &readCount);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      sensor_err("CyU3 get Ep0 data failed\r\n");
      CyFxAppErrorHandler(apiRetStatus);
    }
    firmware_ctrl_flag = Ep0Buffer[0] << 24 | Ep0Buffer[1] << 16 | Ep0Buffer[2] << 8 | Ep0Buffer[3];
    debug_level = firmware_ctrl_flag & (DEBUG_DBG_BIT | DEBUG_INFO_BIT |DEBUG_DUMP_BIT);
    sensor_dbg("EU firmware flag set firmware_ctrl_flag: 0x%x\r\n", firmware_ctrl_flag);
    break;
  case CY_FX_USB_UVC_GET_LEN_REQ:
    Ep0Buffer[0] = 4;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    sensor_dbg("EU firmware flag get len:%d request\r\n", Ep0Buffer[0]);
    break;
  case CY_FX_USB_UVC_GET_INFO_REQ:
    Ep0Buffer[0] = 3;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    sensor_dbg("EU firmware flag get info request\r\n");
    break;
  default:
    sensor_err("unknown firmware_flag cmd: 0x%x\r\n", bRequest);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/* SPI initialization for flash programmer application. */
CyU3PReturnStatus_t CyFxFlashProgSpiInit(uint16_t pageLen) {
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
  CyU3PSpiConfig_t spiConfig;
  CyU3PDmaChannelConfig_t dmaConfig;

  /* Start the SPI module and configure the master. */
  status = CyU3PSpiInit();
  if (status != CY_U3P_SUCCESS) {
    sensor_err("CyU3 spi module int failed!\r\n");
    return status;
  }

  /* Start the SPI master block. Run the SPI clock at 8MHz
   * and configure the word length to 8 bits. Also configure
   * the slave select using FW. */
  CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
  spiConfig.isLsbFirst = CyFalse;
  spiConfig.cpol       = CyTrue;
  spiConfig.ssnPol     = CyFalse;
  spiConfig.cpha       = CyTrue;
  spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
  spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
  spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
  spiConfig.clock      = 8000000;
  spiConfig.wordLen    = 8;

  status = CyU3PSpiSetConfig(&spiConfig, NULL);
  if (status != CY_U3P_SUCCESS) {
    return status;
  }

  /* Create the DMA channels for SPI write and read. */
  CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
  dmaConfig.size           = pageLen;
  /* No buffers need to be allocated as this channel
   * will be used only in override mode. */
  dmaConfig.count          = 0;
  dmaConfig.prodAvailCount = 0;
  dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
  dmaConfig.prodHeader     = 0;
  dmaConfig.prodFooter     = 0;
  dmaConfig.consHeader     = 0;
  dmaConfig.notification   = 0;
  dmaConfig.cb             = NULL;

  /* Channel to write to SPI flash. */
  dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
  dmaConfig.consSckId = CY_U3P_LPP_SOCKET_SPI_CONS;
  status = CyU3PDmaChannelCreate(&glSpiTxHandle, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);
  if (status != CY_U3P_SUCCESS) {
    return status;
  }

  /* Channel to read from SPI flash. */
  dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_SPI_PROD;
  dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
  status = CyU3PDmaChannelCreate(&glSpiRxHandle, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);
  if (status == CY_U3P_SUCCESS) {
    glSpiPageSize = pageLen;
  }

  return status;
}

/* Wait for the status response from the SPI flash. */
static CyU3PReturnStatus_t CyFxFlashProgSpiWaitForStatus(void) {
  uint8_t buf[2], rd_buf[2];
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  /* Wait for status response from SPI flash device. */
  do {
    buf[0] = 0x06;  /* Write enable command. */

    CyU3PSpiSetSsnLine(CyFalse);
    status = CyU3PSpiTransmitWords(buf, 1);
    CyU3PSpiSetSsnLine(CyTrue);
    if (status != CY_U3P_SUCCESS) {
      sensor_err("SPI WR_ENABLE command failed\n\r");
      return status;
    }

    buf[0] = 0x05;  /* Read status command */

    CyU3PSpiSetSsnLine(CyFalse);
    status = CyU3PSpiTransmitWords(buf, 1);
    if (status != CY_U3P_SUCCESS) {
      sensor_err("SPI READ_STATUS command failed\n\r");
      CyU3PSpiSetSsnLine(CyTrue);
      return status;
    }

    status = CyU3PSpiReceiveWords(rd_buf, 2);
    CyU3PSpiSetSsnLine(CyTrue);
    if (status != CY_U3P_SUCCESS) {
      sensor_err("SPI status read failed\n\r");
      return status;
    }
  } while ((rd_buf[0] & 1) || (!(rd_buf[0] & 0x2)));

  return CY_U3P_SUCCESS;
}

/* SPI read / write for programmer application. */
CyU3PReturnStatus_t CyFxFlashProgSpiTransfer(uint16_t pageAddress, uint16_t  byteCount,
                                             uint8_t  *buffer, CyBool_t  isRead) {
  CyU3PDmaBuffer_t buf_p;
  uint8_t location[4];
  uint32_t byteAddress = 0;
  uint16_t pageCount = (byteCount / glSpiPageSize);
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  if (byteCount == 0) {
    return CY_U3P_SUCCESS;
  }
  if ((byteCount % glSpiPageSize) != 0) {
    pageCount++;
  }

  buf_p.buffer = buffer;
  buf_p.status = 0;

  byteAddress  = pageAddress * glSpiPageSize;
  sensor_dbg("SPI access - addr: 0x%x, size: 0x%x, pages: 0x%x.\r\n",
            byteAddress, byteCount, pageCount);

  while (pageCount != 0) {
    location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
    location[2] = (byteAddress >> 8) & 0xFF;
    location[3] = byteAddress & 0xFF;               /* LS byte */

    if (isRead) {
      location[0] = 0x03; /* Read command. */

      buf_p.size  = glSpiPageSize;
      buf_p.count = glSpiPageSize;

      status = CyFxFlashProgSpiWaitForStatus();
      if (status != CY_U3P_SUCCESS)
        return status;

      CyU3PSpiSetSsnLine(CyFalse);
      status = CyU3PSpiTransmitWords(location, 4);
      if (status != CY_U3P_SUCCESS) {
        sensor_err("SPI READ command failed\r\n");
        CyU3PSpiSetSsnLine(CyTrue);
        return status;
      }

      CyU3PSpiSetBlockXfer(0, glSpiPageSize);

      status = CyU3PDmaChannelSetupRecvBuffer(&glSpiRxHandle, &buf_p);
      if (status != CY_U3P_SUCCESS) {
        CyU3PSpiSetSsnLine(CyTrue);
        return status;
      }
      status = CyU3PDmaChannelWaitForCompletion(&glSpiRxHandle, CY_FX_FLASH_PROG_TIMEOUT);
      if (status != CY_U3P_SUCCESS) {
        CyU3PSpiSetSsnLine(CyTrue);
        return status;
      }

      CyU3PSpiSetSsnLine(CyTrue);
      CyU3PSpiDisableBlockXfer(CyFalse, CyTrue);
    } else { /* Write */
      location[0] = 0x02; /* Write command */

      buf_p.size  = glSpiPageSize;
      buf_p.count = glSpiPageSize;

      status = CyFxFlashProgSpiWaitForStatus();
      if (status != CY_U3P_SUCCESS)
        return status;

      CyU3PSpiSetSsnLine(CyFalse);
      status = CyU3PSpiTransmitWords(location, 4);
      if (status != CY_U3P_SUCCESS) {
        sensor_err("SPI WRITE command failed\r\n");
        CyU3PSpiSetSsnLine(CyTrue);
        return status;
      }

      CyU3PSpiSetBlockXfer(glSpiPageSize, 0);

      status = CyU3PDmaChannelSetupSendBuffer(&glSpiTxHandle, &buf_p);
      if (status != CY_U3P_SUCCESS) {
        CyU3PSpiSetSsnLine(CyTrue);
        return status;
      }
      status = CyU3PDmaChannelWaitForCompletion(&glSpiTxHandle,
               CY_FX_FLASH_PROG_TIMEOUT);
      if (status != CY_U3P_SUCCESS) {
        CyU3PSpiSetSsnLine(CyTrue);
        return status;
      }

      CyU3PSpiSetSsnLine(CyTrue);
      CyU3PSpiDisableBlockXfer(CyTrue, CyFalse);
    }

    /* Update the parameters */
    byteAddress  += glSpiPageSize;
    buf_p.buffer += glSpiPageSize;
    pageCount--;

    CyU3PThreadSleep(10);
  }
  return CY_U3P_SUCCESS;
}

/* Function to erase SPI flash sectors. */
CyU3PReturnStatus_t CyFxFlashProgEraseSector(CyBool_t isErase, uint8_t sector, uint8_t *wip) {
  uint32_t temp = 0;
  uint8_t  location[4], rdBuf[2];
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  if ((!isErase) && (wip == NULL)) {
    return CY_U3P_ERROR_BAD_ARGUMENT;
  }

  location[0] = 0x06;  /* Write enable. */

  CyU3PSpiSetSsnLine(CyFalse);
  status = CyU3PSpiTransmitWords(location, 1);
  CyU3PSpiSetSsnLine(CyTrue);
  if (status != CY_U3P_SUCCESS)
    return status;

  if (isErase) {
    location[0] = 0xD8; /* Sector erase. */
    temp        = sector * 0x10000;
    location[1] = (temp >> 16) & 0xFF;
    location[2] = (temp >> 8) & 0xFF;
    location[3] = temp & 0xFF;

    CyU3PSpiSetSsnLine(CyFalse);
    status = CyU3PSpiTransmitWords(location, 4);
    CyU3PSpiSetSsnLine(CyTrue);
  } else {
    location[0] = 0x05; /* Read status */

    CyU3PSpiSetSsnLine(CyFalse);
    status = CyU3PSpiTransmitWords(location, 1);
    if (status != CY_U3P_SUCCESS) {
      CyU3PSpiSetSsnLine(CyTrue);
      return status;
    }

    status = CyU3PSpiReceiveWords(rdBuf, 2);
    CyU3PSpiSetSsnLine(CyTrue);
    *wip = rdBuf[0] & 0x1;
  }

  return status;
}
