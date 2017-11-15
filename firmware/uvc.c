/*
 ## Cypress FX3 Camera Kit Source file (uvc.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
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
#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include <cyu3gpif.h>
#include <cyu3i2c.h>
#include <cyu3gpio.h>
#include <cyu3pib.h>
#include <cyu3utils.h>

#include "include/uvc.h"
#include "include/i2c.h"
#include "include/camera_ptzcontrol.h"
#include "include/cyfxgpif2config.h"
#include "include/inv_icm20608.h"
#include "include/LI_USB30_SENSORs.h"
/*This is for LI-CAM-MT9V034 V1.3*/
#include "include/sensor_v034_raw.h"
#include "include/debug.h"
#include "include/xp_sensor_firmware_version.h"
#include "include/extension_unit.h"
#include "include/cyfxuvcdscr.h"
#include "include/fx3_bsp.h"
#include "include/kfifo.h"

/* debug_level :control debug log messages print level
 * 0 bit set: show debug level log
 * 1 bit set: show messages level log
 * 2 bit set: show buff messages logging */
int debug_level = 0;
uint32_t firmware_ctrl_flag;
/******************************************************************************
                                         Global Variables
 *****************************************************************************/
static int res_switch = 0;
static CyU3PThread   uvcAppThread;                      /* UVC video streaming thread. */
static CyU3PThread   uvcAppEP0Thread;                   /* UVC control request handling thread. */
static CyU3PThread   Datahandle_Thread;                 /* IMU or other Data handle thread. */
static CyU3PEvent    glFxUVCEvent;                      /* Event group used to signal threads. */
CyU3PDmaMultiChannel glChHandleUVCStream;               /* DMA multi-channel handle. */

/* Current UVC control request fields. See USB specification for definition */
uint8_t  bmReqType, bRequest;
uint16_t wValue, wIndex, wLength;
/* Whether USB connection is active */
CyBool_t        isUsbConnected = CyFalse;
/* Current USB connection speed */
CyU3PUSBSpeed_t usbSpeed = CY_U3P_NOT_CONNECTED;
/* Whether a CLEAR_FEATURE (stop streaming) request has been received */
CyBool_t        clearFeatureRqtReceived = CyFalse;
/* Whether USB host has started streaming data */
CyBool_t        streamingStarted = CyFalse;
/* Whether the application is active or not */
CyBool_t        glIsApplnActive  = CyFalse;

#ifdef BACKFLOW_DETECT
/* Whether buffer overflow error is detected */
uint8_t back_flow_detected = 0;
#endif
/* Whether end of frame (FV) signal has been hit. */
static volatile CyBool_t hitFV = CyFalse;
/* Whether the GPIF init function has been called. */
static volatile CyBool_t gpif_initialized = CyFalse;
/* Count of buffers received and committed during the current video frame. */
static volatile uint16_t prodCount = 0, consCount = 0;
static volatile uint16_t underrunCnt = 0;

/* IMU Header are prefixed at the top of each frame as timestamp */
volatile char glIMUHeader[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                                 'A', 'B', 'C', 'D', 'E', 'F'};

#define IMU_POOL_LEN     640 * 2 - 4 - 17
static volatile CyBool_t addIMU = CyFalse;
static volatile CyBool_t readyIMU = CyFalse;
static uint8_t IMU_pool_buf[IMU_POOL_LEN] =  {0};
struct __kfifo  IMU_kfifo;

/* UVC Probe Control Settings for a USB 3.0 connection. */
uint8_t glProbeCtrl[CY_FX_UVC_MAX_PROBE_SETTING] = {
    /* bmHint : no hit */
    0x00, 0x00,
    /* Use 1st Video format index */
    0x01,
    /* Use 1st Video frame index */
    0x01,
    /* Desired frame interval in the unit of 100ns: 30 fps */
    0x0A, 0x8B, 0x02, 0x00,    // 0x15, 0x16, 0x05, 0x00,  /* 60ps */
    /* Key frame rate in key frame/video frame units */
    0x00, 0x00,
    /* PFrame rate in PFrame / key frame units */
    0x00, 0x00,
    /* Compression quality control */
    0x00, 0x00,
    /* Window size for average bit rate */
    0x00, 0x00,
    /* Internal video streaming i/f latency in ms */
    0x00, 0x00,
    /* Max video frame size in bytes */
    0x00, 0x48, 0x3F, 0x00,
    /* No. of bytes device can rx in single payload = 16 KB */
    0x00, 0x40, 0x00, 0x00
  };

/* UVC Probe Control Setting for a USB 2.0 connection. */
uint8_t glProbeCtrl20[CY_FX_UVC_MAX_PROBE_SETTING] = {
    /* bmHint : no hit */
    0x00, 0x00,
    /* Use 1st Video format index */
    0x01,
    /* Use 1st Video frame index */
    0x01,
    /* Desired frame interval in the unit of 100ns: 15 fps */
    0x2A, 0x2C, 0x0A, 0x00,
    /* Key frame rate in key frame/video frame units */
    0x00, 0x00,
    /* PFrame rate in PFrame / key frame units */
    0x00, 0x00,
    /* Compression quality control */
    0x00, 0x00,
    /* Window size for average bit rate */
    0x00, 0x00,
    /* Internal video streaming i/f latency in ms */
    0x00, 0x00,
    /* Max video frame size in bytes */
    0x00, 0x60, 0x09, 0x00,
    /* No. of bytes device can rx in single payload = 16 KB */
    0x00, 0x40, 0x00, 0x00
  };

/** \brief Video Probe Commit Control
 *
 * This array is filled out when the host sends down the SET_CUR request
 */
static uint8_t glCommitCtrl[CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED];

/* UVC Header to be prefixed at the top of each 16 KB video data buffer. */
uint8_t volatile glUVCHeader[CY_FX_UVC_MAX_HEADER] = {
    /* Header Length */
    0x0C,
    /* Bit field header field */
    0x8C,
    /* Presentation time stamp field */
    0x00, 0x00, 0x00, 0x00,
    /* Source clock reference field */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
/******************************************************************************
                                         Static Function
 *****************************************************************************/
static void usb_set_desc(void);

/**
 *  @brief      Add the IMU packet header to the top of the specified DMA buffer.
 *  @param[in]  buffer_p    Buffer pointer.
 *  @return     no return.
 */
void CyFxUVCAddHeader_IMU(uint8_t *buffer_p) {
  uint32_t imu_fifo_len = 0;
  uint32_t imu_num = 0;
  if (!readyIMU) {
    CyU3PMemCopy(buffer_p, (uint8_t *)glIMUHeader, sizeof (glIMUHeader));
    return;
  }

  addIMU = CyFalse;
  /* IMU Burst from image data byte operation
  ------------------------------------------------------------------------------------------
  |Byte location|image timestamp|IMU flag|IMU num|Single IMU Data 0| ... |Single IMU Data N|
  ------------------------------------------------------------------------------------------
  |   Byte num  |      16       |    1   |   4   |        17       | ... |        17       |
  ------------------------------------------------------------------------------------------  
  IMU flag:0 -> this version of fimware don't support IMU burst from image.
           1 -> this version of frimwre support. 
  */
  /*NOTE: we remain last_imu at first 17 bytes to old vesion of tracking */
  CyU3PMemCopy(buffer_p, (uint8_t *)last_imu, sizeof (last_imu));
  /* update imu burst embed image flags */
  if (firmware_ctrl_flag & IMU_FROM_IMAGE_BIT) {
    *(buffer_p + 16) = 1;
    CyU3PMutexGet(&(IMU_kfifo.lock), CYU3P_WAIT_FOREVER);
    imu_fifo_len = kfifo_used(&IMU_kfifo);
    if (imu_fifo_len >= IMU_BURST_LEN) {
      imu_num = imu_fifo_len / IMU_BURST_LEN;
      CyU3PMemCopy(buffer_p + IMU_BURST_LEN, (uint8_t *)(&imu_num), 4);
      kfifo_out(&IMU_kfifo, (void *)(buffer_p + IMU_BURST_LEN + 4), imu_fifo_len);
    }
    CyU3PMutexPut(&(IMU_kfifo.lock));
  }
  return;
}

/**
 *  @brief      Add the UVC packet header to the top of the specified DMA buffer.
 *  @param[in]  buffer_p    Buffer pointer.
 *  @param[in]  frameInd    EOF or normal frame indication
 *  @return     no return.
 */
void CyFxUVCAddHeader(uint8_t *buffer_p, uint8_t frameInd) {
  /* Copy header to buffer */
  CyU3PMemCopy(buffer_p, (uint8_t *)glUVCHeader, CY_FX_UVC_MAX_HEADER);

  /* The EOF flag needs to be set if this is the last packet for this video frame. */
  if (frameInd & CY_FX_UVC_HEADER_EOF) {
    buffer_p[1] |= CY_FX_UVC_HEADER_EOF;
  }
}

/* This function performs the operations for a Video Streaming Abort.
   This is called every time there is a USB reset, suspend or disconnect event.
 */
static void CyFxUVCApplnAbortHandler(void) {
  uint32_t flag;
  if (CyU3PEventGet(&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, \
                    &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS) {
    sensor_dbg("video streaming abort!\r\n ");
    /* Clear the Video Stream Request Event */
    CyU3PEventSet(&glFxUVCEvent, ~(CY_FX_UVC_STREAM_EVENT), CYU3P_EVENT_AND);

    /* Set Video Stream Abort Event */
    CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_OR);
  }
}

/**
 *  @brief      This is the Callback function to handle the USB Events.
 *  @param[in]  evtype    Event type.
 *  @param[in]  evdata    Event data.
 *  @return     no return.
 */
static void UVC_USBEvent_CB(CyU3PUsbEventType_t evtype, uint16_t evdata) {
  switch (evtype) {
  /* USB Connect event */
  case CY_U3P_USB_EVENT_CONNECT:
    sensor_dbg("USB %s connect event detected\r\n", evdata == 1 ? "3.0" : "2.0");
    break;

  /* USB Set Configuration event. evData provides the configuration number selected by the host */
  case CY_U3P_USB_EVENT_SETCONF:
    glIsApplnActive = CyTrue;
    sensor_dbg("USB set Configuration evdata:0x%x\r\n", evdata);
    break;

  /* USB Reset event. evData indicates whether 3.0 or 2.0 connection*/
  case CY_U3P_USB_EVENT_RESET:
    sensor_dbg("USB %s Reset Detect\r\n", evdata == 1 ? "3.0" : "2.0");
    CyU3PGpifDisable(CyTrue);
    gpif_initialized = 0;
    streamingStarted = CyFalse;
    CyFxUVCApplnAbortHandler();
    break;

  /* USB Suspend event for both USB 2.0 and 3.0 connections. evData is not used */
  case CY_U3P_USB_EVENT_SUSPEND:
    sensor_dbg("USB SUSPEND event detected\r\n");
    CyU3PGpifDisable(CyTrue);
    gpif_initialized = 0;
    streamingStarted = CyFalse;
    CyFxUVCApplnAbortHandler();
    break;

  /* USB Resume event */
  case CY_U3P_USB_EVENT_RESUME:
    sensor_dbg("USB RESUME event detected\r\n");
    break;

  /* USB Disconnect event. The evData is not used */
  case CY_U3P_USB_EVENT_DISCONNECT:
    sensor_dbg("USB disconnect event detected\r\n");
    CyU3PGpifDisable(CyTrue);
    gpif_initialized = 0;
    isUsbConnected   = CyFalse;
    streamingStarted = CyFalse;
    CyFxUVCApplnAbortHandler();
    if (glIsApplnActive) {
      /* enable USB LPM to receive a USB connect or reset event every time*/
      CyU3PUsbLPMEnable();
      glIsApplnActive = CyFalse;
    }
    break;

  /* Indicates a data underrun error has been detected on one of the USB endpoints.
   * The event data will provide the endpoint number. */
  case CY_U3P_USB_EVENT_EP_UNDERRUN:
    underrunCnt++;
    sensor_err("EP Underrun on %d, totalcnt:%d\r\n", evdata, underrunCnt);
    break;

  /* USB Set Interface event. The evData parameter provides the interface number and the selected
   * alternate setting values.
   * Bits  7 - 0: LSB of wValue field from setup request.
   * Bits 15 - 8: LSB of wIndex field from setup request. */
  case CY_U3P_USB_EVENT_SETINTF:
    sensor_dbg("USB Set Interface event detected\r\n");
    break;

  /* Event notifying completion of the status phase of a control request. This event is only generated
   * for control requests that are handled by the user's application; and not for requests that are
   * handled internally by the USB driver. */
  case CY_U3P_USB_EVENT_EP0_STAT_CPLT:
    // sensor_dbg("USB EP0 STAT CPLT event detected\r\n");
    break;

  /* USB Speed event */
  case CY_U3P_USB_EVENT_SPEED:
    sensor_dbg("USB Speed event detected\r\n");
    break;

  default:
    break;
  }
}
/**
 *  @brief      Callback to handle the USB Setup Requests and UVC Class events.
 *  @param[in]  setupdat0    SETUP Data 0.
 *  @param[in]  setupdat1    SETUP Data 1.
 *  @return     no return.
 */
static CyBool_t UVC_USBSetup_CB(uint32_t setupdat0, uint32_t setupdat1) {
  CyBool_t uvcHandleReq = CyFalse;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  /* Obtain Request Type and Request */
  bmReqType = (uint8_t)(setupdat0 & CY_FX_USB_SETUP_REQ_TYPE_MASK);
  bRequest  = (uint8_t)((setupdat0 & CY_FX_USB_SETUP_REQ_MASK) >> 8);
  wValue    = (uint16_t)((setupdat0 & CY_FX_USB_SETUP_VALUE_MASK) >> 16);
  wIndex    = (uint16_t)(setupdat1 & CY_FX_USB_SETUP_INDEX_MASK);
  wLength   = (uint16_t)((setupdat1 & CY_FX_USB_SETUP_LENGTH_MASK) >> 16);

  /* Check for UVC Class Requests */
  switch (bmReqType) {
  case CY_FX_USB_UVC_GET_REQ_TYPE:
  case CY_FX_USB_UVC_SET_REQ_TYPE:
    /* UVC Specific requests are handled in the EP0 thread. */
    switch (wIndex & 0xFF) {
    case CY_FX_UVC_CONTROL_INTERFACE: {
      uvcHandleReq = CyTrue;
      status = CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT,
                              CYU3P_EVENT_OR);
      if (status != CY_U3P_SUCCESS) {
        sensor_err("Set CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT Failed %x\n", status);
        CyU3PUsbStall(0, CyTrue, CyFalse);
      }
    }
    break;

    case CY_FX_UVC_STREAM_INTERFACE: {
      uvcHandleReq = CyTrue;
      status = CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT,
                              CYU3P_EVENT_OR);
      if (status != CY_U3P_SUCCESS) {
        /* Error handling */
        sensor_err("Set CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT Failed %x\n", status);
        CyU3PUsbStall(0, CyTrue, CyFalse);
      }
    }
    break;

    default:
      break;
    }
    break;

  case CY_FX_USB_SET_INTF_REQ_TYPE:
    if (bRequest == CY_FX_USB_SET_INTERFACE_REQ) {
      /* MAC OS sends Set Interface Alternate Setting 0 command after
       * stopping to stream. This application needs to stop streaming. */
      if ((wIndex == CY_FX_UVC_STREAM_INTERFACE) && (wValue == 0)) {
        /* Stop GPIF state machine to stop data transfers through FX3 */
        sensor_dbg("Alternate setting 0..\r\n");
        CyU3PGpifDisable(CyTrue);
        gpif_initialized = 0;
        streamingStarted = CyFalse;
        /* Place the EP in NAK mode before cleaning up the pipe. */
        CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyTrue);
        CyU3PBusyWait(100);

        /* Reset and flush the endpoint pipe. */
        CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
        CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
        CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
        CyU3PBusyWait(100);

        /* Clear the stall condition and sequence numbers. */
        CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);
        uvcHandleReq = CyTrue;
        /* Complete Control request handshake */
        CyU3PUsbAckSetup();
        /* Indicate stop streaming to main thread */
        clearFeatureRqtReceived = CyTrue;
        CyFxUVCApplnAbortHandler();
      }
    }
    break;

  case CY_U3P_USB_TARGET_ENDPT:
    if (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE) {
      if (wIndex == CY_FX_EP_BULK_VIDEO) {
        /* Windows OS sends Clear Feature Request after it stops streaming,
         * however MAC OS sends clear feature request right after it sends a
         * Commit -> SET_CUR request. Hence, stop streaming only of streaming
         * has started. */
        if (streamingStarted == CyTrue) {
          sensor_info("Clear feature request detected..\r\n");
          /*free kfifo */
          kfifo_free(&IMU_kfifo);

          /* Disable the GPIF state machine. */
          CyU3PGpifDisable(CyTrue);
          gpif_initialized = 0;
          streamingStarted = CyFalse;

          /* Place the EP in NAK mode before cleaning up the pipe. */
          CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyTrue);
          CyU3PBusyWait(100);

          /* Reset and flush the endpoint pipe. */
          CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
          CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
          CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
          CyU3PBusyWait(100);

          /* Clear the stall condition and sequence numbers. */
          CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);

          uvcHandleReq = CyTrue;
          /* Complete Control request handshake */
          CyU3PUsbAckSetup();
          /* Indicate stop streaming to main thread */
          clearFeatureRqtReceived = CyTrue;
          CyFxUVCApplnAbortHandler();
        } else {
          uvcHandleReq = CyTrue;
          CyU3PUsbAckSetup();
        }
      }
    }
    break;
  default:
    break;
  }
  /* Return status of request handling to the USB driver */
  return uvcHandleReq;
}

/* DMA callback providing notification when each buffer has been sent out to the USB host.
 * This is used to track whether all of the data has been sent out.
 */
void CyFxUvcApplnDmaCallback(CyU3PDmaMultiChannel *multiChHandle, CyU3PDmaCbType_t type,
                              CyU3PDmaCBInput_t *input) {
  if (type == CY_U3P_DMA_CB_CONS_EVENT) {
    consCount++;
    streamingStarted = CyTrue;
  }
}

/**
 *  @brief      .
 *  This function is called from the GPIF callback when we have reached the end of a video frame.
 *  The DMA buffer containing the last part of the frame may not have been committed, and need to
 *  be manually wrapped up. This function uses the current GPIF state ID to identify the socket on
 *  which this last buffer is pending, and then uses the CyU3PDmaMultiChannelSetWrapUp function
 *  to commit the buffer.
 *  @param[in]  handle     Handle to DMA channel.
 *  @param[in]  stateId    Current GPIF state ID.
 *  @return     no return.
 */
static uint8_t CyFxUvcAppCommitEOF(CyU3PDmaMultiChannel *handle, uint8_t stateId) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  uint8_t socket = 0xFF;      /*  Invalid value. */

  /* Verify that the current state is a terminal state for the GPIF state machine. */
  switch (stateId) {
  case FULL_BUF_IN_SCK0:
  case FULL_BUF_IN_SCK1:
    /* Buffer is already full and would have been committed. Do nothing. */
    break;
  case PARTIAL_BUF_IN_SCK0:
    socket = 0;
    break;
  case PARTIAL_BUF_IN_SCK1:
    socket = 1;
    break;
  default:
    /* Unexpected current state. Return error. */
    return 1;
  }

  if (socket != 0xFF) {
    /* We have a partial buffer. Commit the buffer manually. The Wrap Up API,
     * here, helps produce a partially filled buffer on the producer side. This
     * action will cause CyU3PDmaMultiChannelGetBuffer API in the UVC_AppThread_Entry
     * function to succeed one more time with less than full producer buffer count */
    apiRetStatus = CyU3PDmaMultiChannelSetWrapUp(handle, socket);
    if (apiRetStatus != CY_U3P_SUCCESS) {
      sensor_err("Channel Set WrapUp failed, Error code = %d\r\n", apiRetStatus);
      CyFxAppErrorHandler(apiRetStatus);
    }
  }

  return 0;
}

/* GpifCB callback function is invoked when FV triggers GPIF interrupt */
void CyFxGpifCB(CyU3PGpifEventType event, uint8_t currentState) {
  if (event == CYU3P_GPIF_EVT_SM_INTERRUPT) {
    // sensor_dbg("CYU3P_GPIF_EVT_SM_INTERRUPT...\r\n");
    hitFV = CyTrue;
    // sensor_info("a frame Transfer prodCount:%d consCount:%d\r\n", prodCount, consCount);
    if (CyFxUvcAppCommitEOF(&glChHandleUVCStream, currentState) != CY_U3P_SUCCESS)
      sensor_err("Commit EOF failed!\n");
  }
}

/* This function initializes the Debug Module for the UVC Application */
static void CyFxUVCApplnDebugInit(void) {
  CyU3PUartConfig_t uartConfig;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  /* Initialize the UART for printing debug messages */
  apiRetStatus = CyU3PUartInit();
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("CyU3 uart module init error\r\n");
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Set UART Configuration */
  uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
  uartConfig.stopBit  = CY_U3P_UART_ONE_STOP_BIT;
  uartConfig.parity   = CY_U3P_UART_NO_PARITY;
  uartConfig.txEnable = CyTrue;
  uartConfig.rxEnable = CyFalse;
  uartConfig.flowCtrl = CyFalse;
  uartConfig.isDma    = CyTrue;

  /* Set the UART configuration */
  apiRetStatus = CyU3PUartSetConfig(&uartConfig, NULL);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("CyU3 set uart config error\r\n");
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Set the UART transfer */
  apiRetStatus = CyU3PUartTxSetBlockXfer(0xFFFFFFFF);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("CyU3 set uart blockl xfer error\r\n");
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Initialize the debug application */
  apiRetStatus = CyU3PDebugInit(CY_U3P_LPP_SOCKET_UART_CONS, 8);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("CyU3 debug module init error\r\n");
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Disable the debug print header. */
  CyU3PDebugPreamble(CyFalse);
}

#ifdef BACKFLOW_DETECT
static void CyFxUvcAppPibCallback(CyU3PPibIntrType cbType, uint16_t cbArg) {
  if ((cbType == CYU3P_PIB_INTR_ERROR) && ((cbArg == 0x1005) || (cbArg == 0x1006))) {
    if (!back_flow_detected) {
      sensor_err("Backflow detected...\r\n");
      back_flow_detected = 1;
    }
  }
}
#endif

/* This function initializes the USB Module, creates event group,
   sets the enumeration descriptors, configures the Endpoints and
   configures the DMA module for the UVC Application */
static void CyFxUVCApplnInit(void) {
  CyU3PDmaMultiChannelConfig_t dmaMultiConfig;
  CyU3PEpConfig_t              endPointConfig;
  CyU3PReturnStatus_t          apiRetStatus;
  CyU3PPibClock_t              pibclock;
  CyU3PReturnStatus_t          status = CY_U3P_SUCCESS;

  /* Create UVC event group */
  apiRetStatus = CyU3PEventCreate(&glFxUVCEvent);
  if (apiRetStatus != 0) {
    sensor_err("UVC Create Event failed, Error Code = %d\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

#ifdef UVC_PTZ_SUPPORT
  CyFxUvcAppPTZInit();
#endif

  isUsbConnected = CyFalse;
  clearFeatureRqtReceived = CyFalse;

  /* Initialize FX3 GPIO module. */
  fx3_gpio_module_init();

  /* Initialize the P-port. */
  pibclock.clkDiv      = 2;
  pibclock.clkSrc      = CY_U3P_SYS_CLK;
  pibclock.isDllEnable = CyFalse;
  pibclock.isHalfDiv   = CyFalse;

  apiRetStatus = CyU3PPibInit(CyTrue, &pibclock);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("PIB Function Failed to Start, Error Code = %d\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Setup the Callback to Handle the GPIF INTR event */
  CyU3PGpifRegisterCallback(CyFxGpifCB);

#ifdef BACKFLOW_DETECT
  back_flow_detected = 0;
  CyU3PPibRegisterCallback(CyFxUvcAppPibCallback, CYU3P_PIB_INTR_ERROR);
#endif
  sensor_gpio_init();

  /* Initialize the application. */
  status = CyFx_I2cInit();
  if (status != CY_U3P_SUCCESS) {
    sensor_err("I2C bus init failure!\r\n");
  }
  /* Initialize the SPI interface for flash of page size 256 bytes. */
  status = CyFxFlashProgSpiInit(0x100);
  if (status != CY_U3P_SUCCESS) {
    return;
  }
  // Initialize the INV sensor
  status = icm_init();
  if (status != CY_U3P_SUCCESS) {
    sensor_err("icm init failure!\r\n");
    return;
  }

  icm_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  /* Push both gyro and accel data into the FIFO. */
  icm_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  CyU3PThreadSleep(100);
  readyIMU = CyTrue;
  sensor_dbg("IMU is ready!\r\n");
  /* Initialize v034/v024 senosr */
  V034_sensor_init();

  /* USB initialization. */
  apiRetStatus = CyU3PUsbStart();
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("USB Function Failed to Start, Error Code = %d\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Setup the Callback to Handle the USB Setup Requests */
  CyU3PUsbRegisterSetupCallback(UVC_USBSetup_CB, CyFalse);

  /* Setup the Callback to Handle the USB Events */
  CyU3PUsbRegisterEventCallback(UVC_USBEvent_CB);

  /* Register a callback to handle LPM requests from the USB 3.0 host. */
  // CyU3PUsbRegisterLPMRequestCallback(CyFxUSBUARTAppLPMRqtCB);
  usb_set_desc();

  /* Configure the video streaming endpoint. */
  endPointConfig.enable   = 1;
  endPointConfig.epType   = CY_U3P_USB_EP_BULK;
  endPointConfig.pcktSize = CY_FX_EP_BULK_VIDEO_PKT_SIZE;
  endPointConfig.isoPkts  = 1;
  endPointConfig.burstLen = 16;
  endPointConfig.streams  = 0;
  apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_BULK_VIDEO, &endPointConfig);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error Handling */
    sensor_err("USB Set Endpoint config failed, Error Code = %d\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Configure the status interrupt endpoint.
     Note: This endpoint is not being used by the application as of now. This can be used in case
     UVC device needs to notify the host about any error conditions. A MANUAL_OUT DMA channel
     can be associated with this endpoint and used to send these data packets.
  */
  endPointConfig.enable   = 1;
  endPointConfig.epType   = CY_U3P_USB_EP_INTR;
  endPointConfig.pcktSize = 64;
  endPointConfig.isoPkts  = 0;
  endPointConfig.streams  = 0;
  endPointConfig.burstLen = 1;
  apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONTROL_STATUS, &endPointConfig);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error Handling */
    sensor_err("USB Set Endpoint config failed, Error Code = %d\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Create a DMA Manual channel for sending the video data to the USB host. */
  dmaMultiConfig.size           = CY_FX_UVC_STREAM_BUF_SIZE;
  dmaMultiConfig.count          = CY_FX_UVC_STREAM_BUF_COUNT;
  dmaMultiConfig.validSckCount  = 2;
  dmaMultiConfig.prodSckId[0]   = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_0;
  dmaMultiConfig.prodSckId[1]   = (CyU3PDmaSocketId_t)CY_U3P_PIB_SOCKET_1;
  dmaMultiConfig.consSckId[0]   = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_CONS_0
                                  | CY_FX_EP_VIDEO_CONS_SOCKET);
  dmaMultiConfig.prodAvailCount = 0;
  dmaMultiConfig.prodHeader     = 12; /* 12 byte UVC header to be added. */
  dmaMultiConfig.prodFooter     = 4;  /* 4 byte footer to compensate for the 12 byte header. */
  dmaMultiConfig.consHeader     = 0;
  dmaMultiConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
  dmaMultiConfig.notification   = CY_U3P_DMA_CB_CONS_EVENT;
  dmaMultiConfig.cb             = CyFxUvcApplnDmaCallback;
  apiRetStatus = CyU3PDmaMultiChannelCreate(&glChHandleUVCStream,
                 CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE, &dmaMultiConfig);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error handling */
    sensor_err("DMA Channel Creation Failed, Error Code = %d\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Enable USB connection from the FX3 device, preferably at USB 3.0 speed. */
  apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    sensor_err("USB Connect failed, Error Code = %d\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }
  CyU3PUsbLPMDisable();
}
static void usb_set_desc(void) {
  /* Register the USB device descriptors with the driver. */
  CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSBDeviceDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSBDeviceDscrSS);

  /* BOS and Device qualifier descriptors. */
  CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);

  /* Configuration descriptors. */
  CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);

  /* String Descriptors */
  CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);

  /* update hardware vesion info to usb USB Product Dscr */
  update_hard_version_dscr();
  CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);

  /* update firmware vesion info to usb Serial Number descriptor */
  update_soft_version_dscr();
  CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 3, (uint8_t *)CyFxUSBSerialNumberDscr);
}
/*
 * Load the GPIF configuration on the GPIF-II engine. This operation is performed
 * whenever a new video streaming session is started.
 */
static void CyFxUvcAppGpifInit(void) {
  CyU3PReturnStatus_t apiRetStatus;

  apiRetStatus = CyU3PGpifLoad((CyU3PGpifConfig_t *) &CyFxGpifConfig);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error Handling */
    sensor_err("Loading GPIF Configuration failed, Error Code = %d\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }

  /* Start the state machine from the designated start state. */
  apiRetStatus = CyU3PGpifSMStart(START, ALPHA_START);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    /* Error Handling */
    sensor_err("Starting GPIF state machine failed, Error Code = %d\r\n", apiRetStatus);
    CyFxAppErrorHandler(apiRetStatus);
  }
  sensor_dbg("GPIF Init done\r\n");
}

/*
 * Entry function for the UVC Application Thread
 */
void UVC_AppThread_Entry(uint32_t input) {
  CyU3PDmaBuffer_t    produced_buffer;
  CyU3PReturnStatus_t apiRetStatus;
  uint32_t flag;
#ifdef DEBUG_PRINT_FRAME_COUNT
  uint32_t frameCnt = 0;
#endif

  /* Initialize the Uart Debug Module */
  CyFxUVCApplnDebugInit();
  sensor_dbg("Uart Debug Module init succeed, compiled at [%s] %s\r\n", __TIME__, __DATE__);

  /* Initialize the UVC Application */
  CyFxUVCApplnInit();

  sensor_dbg("start uvc app thread\r\n");
  sensor_info("check firmware_ctrl_flag:0x%x\r\n", firmware_ctrl_flag);
/*
   This thread continually checks whether video streaming is enabled, and commits video data if so.

   The CY_FX_UVC_STREAM_EVENT and CY_FX_UVC_STREAM_ABORT_EVENT event flags are monitored by this
   thread. The CY_FX_UVC_STREAM_EVENT event flag is enabled when the USB host sends a COMMIT control
   request to the video streaming interface, and stays ON as long as video streaming is enabled.

   The CY_FX_UVC_STREAM_ABORT_EVENT event indicates that we need to abort the video streaming. This
   only happens when we receive a CLEAR_FEATURE request indicating that streaming is to be stopped,
   or when we have a critical error in the data path. In both of these cases, the CY_FX_UVC_STREAM_EVENT
   event flag will be cleared before the CY_FX_UVC_STREAM_ABORT_EVENT event flag is enabled.

   This sequence ensures that we do not get stuck in a loop where we are trying to send data instead
   of handling the abort request.
 */
  for (;;) {
    /* Waiting for the Video Stream Event */
    if (CyU3PEventGet(&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, &flag,
                      CYU3P_NO_WAIT) == CY_U3P_SUCCESS) {
      /* Check if we have a buffer ready to go. */
      // sensor_dbg("CY_FX_UVC_STREAM_EVENT check buffer\r\n");
      apiRetStatus = CyU3PDmaMultiChannelGetBuffer(&glChHandleUVCStream, &produced_buffer,
                                                     CYU3P_NO_WAIT);
      if (apiRetStatus == CY_U3P_SUCCESS) {
        // sensor_dbg("CY_FX_UVC_STREAM_EVENT got buffer\r\n");
        if (produced_buffer.count == CY_FX_UVC_BUF_FULL_SIZE) {
          if (addIMU)
            CyFxUVCAddHeader_IMU(produced_buffer.buffer);
          CyFxUVCAddHeader(produced_buffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_FRAME);
        } else {
          /*
           * If we have a partial buffer, this is guaranteed to be
           * the end of the video frame for uncompressed images.
           */
          CyFxUVCAddHeader(produced_buffer.buffer - CY_FX_UVC_MAX_HEADER, CY_FX_UVC_HEADER_EOF);
          // sensor_info("a frame Transfer prodCount:%d consCount:%d\r\n", prodCount, consCount);
        }

        /* Commit the updated DMA buffer to the USB endpoint. */
        prodCount++;
        // sensor_dbg("CY_FX_UVC_STREAM_EVENT send buffer now \r\n");
        apiRetStatus = CyU3PDmaMultiChannelCommitBuffer(&glChHandleUVCStream,
                       produced_buffer.count + CY_FX_UVC_MAX_HEADER, 0);
        if (apiRetStatus != CY_U3P_SUCCESS) {
          prodCount--;
          sensor_err("Error in multichannelcommitbuffer: Code = %d, size = %x, dmaDone %x\r\n",
                    apiRetStatus, produced_buffer.count, prodCount - consCount);
        }
      }

      /* If we have the end of frame signal and all of the committed data has been read by the USB host;
               we can reset the DMA channel and prepare for the next video frame. */
      if ((hitFV) && (prodCount == consCount)) {
        prodCount = 0;
        consCount = 0;
        hitFV     = CyFalse;
        addIMU    = CyTrue;
#ifdef BACKFLOW_DETECT
        back_flow_detected = 0;
#endif
        // sensor_dbg("<hitFV && (prodCount == consCount)>\r\n");
        /* Toggle UVC header FRAME ID bit */
        glUVCHeader[1] ^= CY_FX_UVC_HEADER_FRAME_ID;
        /* Reset the DMA channel. */
        // sensor_dbg("<Reset the DMA channel>\r\n");
        apiRetStatus = CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
        if (apiRetStatus != CY_U3P_SUCCESS) {
          sensor_err("DMA Channel Reset Failed, Error Code = %d\r\n", apiRetStatus);
          CyFxAppErrorHandler(apiRetStatus);
        }

        /* Start Channel Immediately */
        apiRetStatus = CyU3PDmaMultiChannelSetXfer(&glChHandleUVCStream, 0, 0);
        if (apiRetStatus != CY_U3P_SUCCESS) {
          sensor_dbg("DMA Channel Set Transfer Failed, Error Code = %d\r\n", apiRetStatus);
          CyFxAppErrorHandler(apiRetStatus);
        }
        /* Jump to the start state of the GPIF state machine. 257 is used as an
                   arbitrary invalid state (> 255) number. */
        CyU3PGpifSMSwitch(257, 0, 257, 0, 2);
      }
    } else {
      // Sid. Force the two counters in sync
      // o.w. re-play doesn't work
      prodCount = 0;
      consCount = 0;
      /* If we have a stream abort request pending. */
      if (CyU3PEventGet(&glFxUVCEvent, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_AND_CLEAR,
                        &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS) {
        hitFV = CyFalse;

        IMU_kfifo.kfifo_flag &= ~KFIFO_IS_START;
        sensor_dbg("got CY_FX_UVC_STREAM_ABORT_EVENT \r\n");
        V034_stream_stop(SENSOR_ADDR_WR);

        if (!clearFeatureRqtReceived) {
          apiRetStatus = CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
          if (apiRetStatus != CY_U3P_SUCCESS) {
            CyFxAppErrorHandler(apiRetStatus);
          }

          /* Flush the Endpoint memory */
          CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
        }

        clearFeatureRqtReceived = CyFalse;
      } else {
        /* We are essentially idle at this point. 
           Wait for the reception of a start streaming request. */
        CyU3PEventGet(&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_AND, &flag,
                      CYU3P_WAIT_FOREVER);
        sensor_dbg("got CY_FX_UVC_STREAM_EVENT idle?\r\n");
        /* Set DMA Channel transfer size, first producer socket */
        apiRetStatus = CyU3PDmaMultiChannelSetXfer(&glChHandleUVCStream, 0, 0);
        /* apiRetStatus will be CY_U3P_ERROR_ALREADY_STARTED occasionally. It is known bug but
         * don't make effect image Transfer*/
        if (apiRetStatus == CY_U3P_ERROR_ALREADY_STARTED) {
          sensor_err("DMA Channel Set Transfer Failed and aleardy started\r\n");
        } else if (apiRetStatus != CY_U3P_SUCCESS && \
                 apiRetStatus != CY_U3P_ERROR_ALREADY_STARTED) {
          sensor_err("DMA Channel Set Transfer Failed, Error Code = %d\r\n", apiRetStatus);
          CyFxAppErrorHandler(apiRetStatus);
        }

        /* Initialize gpif configuration and waveform descriptors */
        if (gpif_initialized == CyFalse) {
          CyFxUvcAppGpifInit();
          gpif_initialized = CyTrue;
        } else {
          /* Jump to the start state of the GPIF state machine. 257 is used as an
             arbitrary invalid state (> 255) number. */
          sensor_dbg("<Jump to the start state of the GPIF>\r\n");
          CyU3PGpifSMSwitch(257, 0, 257, 0, 2);
        }
        IMU_kfifo.kfifo_flag |= KFIFO_IS_START;
      }
    }
    /* Allow other ready threads to run before proceeding. */
    CyU3PThreadRelinquish();
  }
}

/**
 *  @brief      handle Processing Unit brightness control.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
static void PU_brightness_control(uint8_t bRequest) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  uint16_t readCount;
  uint8_t Ep0Buffer[32];

  switch (bRequest) {
  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 2 byte. */
    Ep0Buffer[0] = 2;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
    Ep0Buffer[0] = V034_SensorGetBrightness();
    Ep0Buffer[1] = 0;
    sensor_info("Warning: get brightness: %d\r\n", Ep0Buffer[0]);
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
    Ep0Buffer[0] = 0;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
    Ep0Buffer[0] = 255;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
    Ep0Buffer[0] = 1;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    break;
  /* Both GET and SET requests are supported, auto modes not supported */
  case CY_FX_USB_UVC_GET_INFO_REQ:
    Ep0Buffer[0] = 3;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
    // sensor_dbg("GET_DEF brightness\r\n");
    Ep0Buffer[0] = 55;
    Ep0Buffer[1] = 0;
    CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
    apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                                       Ep0Buffer, &readCount);
    if (apiRetStatus == CY_U3P_SUCCESS) {
      // sensor_dbg("set brightness: %d, readcount: %d\r\n", Ep0Buffer[0], readCount);
      V034_SensorSetBrightness(Ep0Buffer[0]);
    }
    sensor_info("Warning: set brightness: %d\r\n", Ep0Buffer[0]);
    break;
  default:
    sensor_err("Processing Unit Brightness Control bRequest Error, bRequest = 0x%x\r\n", bRequest);
    CyFxAppErrorHandler(apiRetStatus);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
    }
}

/**
 *  @brief      handle Processing Unit gain control.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
static void PU_gain_control(uint8_t bRequest) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  uint16_t readCount;
  uint8_t Ep0Buffer[32];

    switch (bRequest) {
    case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of gain data = 2 byte. */
      Ep0Buffer[0] = 2;
      CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
      Ep0Buffer[0] = V034_SensorGetGain();
      Ep0Buffer[1] = 0;
      sensor_info("Warning: get gain: %d\r\n", Ep0Buffer[0]);
      CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum gain = 0. */
      Ep0Buffer[0] = 0;
      Ep0Buffer[1] = 0;
      CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum gain = 255. */
      Ep0Buffer[0] = 255;
      Ep0Buffer[1] = 0;
      CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
      Ep0Buffer[0] = 1;
      Ep0Buffer[1] = 0;
      CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_INFO_REQ: /* auto modes not supported */
      Ep0Buffer[0] = 3;
      CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_DEF_REQ: /* Default gain value = 55. */
      // sensor_dbg("GET_DEF gain\r\n");
      Ep0Buffer[0] = 55;
      Ep0Buffer[1] = 0;
      CyU3PUsbSendEP0Data(2, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_SET_CUR_REQ: /* Update gain value. */
      apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                                         Ep0Buffer, &readCount);
      if (apiRetStatus == CY_U3P_SUCCESS) {
        V034_SensorSetGain(Ep0Buffer[0]);
      }
      sensor_info("Warning: set gain: %d\r\n", Ep0Buffer[0]);
      break;
    default:
      sensor_err("Processing Unit Gain Control bRequest Error, bRequest = 0x%x\r\n", bRequest);
      CyFxAppErrorHandler(apiRetStatus);
      CyU3PUsbStall(0, CyTrue, CyFalse);
      break;
    }
}

/*
 * Handler for control requests addressed to the Processing Unit.
 */
static void Handle_ProcessingUnit_Rqts(void) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  sensor_dbg("handle processing unit rqts: %x\r\n", wValue);
  switch (wValue) {
  case CY_FX_UVC_PU_BRIGHTNESS_CONTROL:
    PU_brightness_control(bRequest);
    break;
  case CY_FX_UVC_PU_GAIN_CONTROL:
    PU_gain_control(bRequest);
    break;
  default:
    sensor_err("Processing Unit wValue Error, wValue = 0x%x\r\n", wValue);
    CyFxAppErrorHandler(apiRetStatus);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}
/**
 *  @brief      handle Processing Unit brightness control.
 *  @param[out] bRequest    bRequst value of uvc.
 *  @return     0 if successful.
 */
static void CT_exposure_time_control(uint8_t bRequest) {
  uint16_t readCount;
  uint16_t exposure_time;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  uint8_t Ep0Buffer[32];

  switch (bRequest) {
  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of exposure data = 4 byte. */
    Ep0Buffer[0] = 4;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current exposure value. */
    exposure_time = V034_SensorGetExposuretime();
    sensor_info("Warning: get exposure: %d\r\n", exposure_time);
    Ep0Buffer[0] = exposure_time & 0x00FF;
    Ep0Buffer[1] = exposure_time >> 8;
    Ep0Buffer[2] = 0;
    Ep0Buffer[3] = 0;
    CyU3PUsbSendEP0Data(4, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum exposure = 0. */
    Ep0Buffer[0] = 0;
    Ep0Buffer[1] = 0;
    Ep0Buffer[2] = 0;
    Ep0Buffer[3] = 0;
    CyU3PUsbSendEP0Data(4, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum exposure = 255. */
    Ep0Buffer[0] = 255;
    Ep0Buffer[1] = 0;
    Ep0Buffer[2] = 0;
    Ep0Buffer[3] = 0;
    CyU3PUsbSendEP0Data(4, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
    Ep0Buffer[0] = 1;
    Ep0Buffer[1] = 0;
    Ep0Buffer[2] = 0;
    Ep0Buffer[3] = 0;
    CyU3PUsbSendEP0Data(4, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_INFO_REQ: /* auto modes not supported */
    Ep0Buffer[0] = 3;
    CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default exposure value = 55. */
    // sensor_dbg("GET_DEF exposure\r\n");
    Ep0Buffer[0] = 55;
    Ep0Buffer[1] = 0;
    Ep0Buffer[2] = 0;
    Ep0Buffer[3] = 0;
    CyU3PUsbSendEP0Data(4, (uint8_t *)Ep0Buffer);
    break;
  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update exposure value. */
    apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED, Ep0Buffer, &readCount);
    if (apiRetStatus == CY_U3P_SUCCESS) {
      exposure_time = Ep0Buffer[0] + (Ep0Buffer[1] << 8);
      V034_SensorSetExposuretime(exposure_time);
      sensor_info("Warning: set exposure: %d, readcount: %d\r\n", exposure_time, readCount);
    }
    break;
  default:
    sensor_err("Camera Terminal bRequest Error, bRequest = 0x%x\r\n", bRequest);
    CyFxAppErrorHandler(apiRetStatus);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}
/*
 * Handler for control requests addressed to the UVC Camera Terminal unit.
 */
static void Handle_CameraTerminal_Rqts(void) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  switch (wValue) {
  case CY_FX_UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
    CT_exposure_time_control(bRequest);
    break;
  default:
    sensor_err("Camera Terminal wValue Error, wValue = 0x%x\r\n", wValue);
    CyFxAppErrorHandler(apiRetStatus);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/*
 * Handler for UVC Interface control requests.
 */
static void Handle_InterfaceCtrl_Rqts(void) {
  uint8_t Ep0Buffer[32];

  sensor_dbg("UVC Handle Interface Ctrl Rqts \r\n");
  switch (wValue) {
  case VC_REQUEST_ERROR_CODE_CONTROL:
    switch (bRequest) {
    case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
      Ep0Buffer[0] = 1;
      CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
      Ep0Buffer[0] = 0x06;   // invalid control
      CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_INFO_REQ: /* Only GET requests is supported */
      Ep0Buffer[0] = 1;
      CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
      break;
    default:
      CyU3PUsbStall(0, CyTrue, CyFalse);
      break;
    }
  case VC_SET_SEL_U1_U2_TEST:  // add for SET_SEL Test,USB30CV
    // can not stall
    sensor_dbg("SET SEL Test....\n");
    break;
  default:
    /* No requests supported as of now. Just stall EP0 to fail the request. */
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/*
 * Handler for control requests addressed to the Extension Unit.
 */
static void Handle_ExtensionUnit_Rqts(void) {
  switch (wValue) {
  case CY_FX_UVC_XU_REG_RW:
    EU_Rqts_imu_rw(bRequest);
    break;
  case CY_FX_UVC_XU_REG_BURST:
    EU_Rqts_imu_burst(bRequest);
    break;
  case CY_FX_UVC_XU_SPI_FLASH:
    EU_Rqts_spi_flash(bRequest);
    break;
  case CY_FX_UVC_XU_CAM_REG:
    EU_Rqts_cam_reg(bRequest);
    break;
  case CY_FX_UVC_XU_SVER_RW:
    EU_Rqts_soft_version(bRequest);
  case CY_FX_UVC_XU_HVER_RW:
    EU_Rqts_hard_version(bRequest);
  case CY_FX_UVC_XU_FLAG_RW:
    EU_Rqts_firmware_flag(bRequest);
    break;
  default:
    sensor_err("invalid extension cmd: 0x%x\r\n", wValue);
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/*
 * Handler for the video streaming control requests.
 */
static void Handle_VideoStreaming_Rqts(void) {
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  uint16_t status = 0;
  uint16_t readCount;
  uint8_t Ep0Buffer[32];

  switch (wValue) {
  case CY_FX_UVC_PROBE_CTRL:
    // bad for linux
    sensor_dbg("wValue = CY_FX_UVC_PROBE_CTRL\r\n");
    switch (bRequest) {
    case CY_FX_USB_UVC_GET_INFO_REQ:
      Ep0Buffer[0] = 3;                /* GET/SET requests are supported. */
      sensor_dbg("bRequest = CY_FX_USB_UVC_GET_INFO_REQ\r\n");
      CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_LEN_REQ:
      Ep0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
      sensor_dbg("bRequest = CY_FX_UVC_MAX_PROBE_SETTING\r\n");
      CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_CUR_REQ:
    case CY_FX_USB_UVC_GET_MIN_REQ:
    case CY_FX_USB_UVC_GET_MAX_REQ:
    case CY_FX_USB_UVC_GET_DEF_REQ: /* There is only one setting per USB speed. */
      if (usbSpeed == CY_U3P_SUPER_SPEED) {
        // bad for linux
        sensor_dbg("usbSpeed = CY_U3P_SUPER_SPEED \r\n");
        CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
      } else {
        // bad for linux
        sensor_dbg("usbSpeed != CY_U3P_SUPER_SPEED \r\n");
        CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
      }
      break;
    case CY_FX_USB_UVC_SET_CUR_REQ:
      sensor_dbg("bRequest = CY_FX_USB_UVC_SET_CUR_REQ \r\n");
      apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                                         glCommitCtrl, &readCount);
      if (apiRetStatus == CY_U3P_SUCCESS) {
        if (usbSpeed == CY_U3P_SUPER_SPEED) {
        /* Copy the relevant settings from the host provided data 
           into the active data structure. */
          glProbeCtrl[2] = glCommitCtrl[2];
          glProbeCtrl[3] = glCommitCtrl[3];
          glProbeCtrl[4] = glCommitCtrl[4];
          glProbeCtrl[5] = glCommitCtrl[5];
          glProbeCtrl[6] = glCommitCtrl[6];
          glProbeCtrl[7] = glCommitCtrl[7];
        }
      }
      break;
    default:
      CyU3PUsbStall(0, CyTrue, CyFalse);
      break;
    }
    break;

  case CY_FX_UVC_COMMIT_CTRL:
    sensor_dbg("wValue = CY_FX_UVC_COMMIT_CTRL\r\n");
    switch (bRequest) {
    case CY_FX_USB_UVC_GET_INFO_REQ:
      sensor_dbg("bRequest = CY_FX_UVC_COMMIT_CTRL\r\n");
      Ep0Buffer[0] = 3;                        /* GET/SET requests are supported. */
      CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_LEN_REQ:
      sensor_dbg("bRequest = CY_FX_USB_UVC_GET_LEN_REQ\r\n");
      Ep0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
      CyU3PUsbSendEP0Data(1, (uint8_t *)Ep0Buffer);
      break;
    case CY_FX_USB_UVC_GET_CUR_REQ:
      if (usbSpeed == CY_U3P_SUPER_SPEED) {
        sensor_dbg("usbSpeed = CY_U3P_SUPER_SPEED\r\n");
        CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl);
      } else {
        sensor_dbg("usbSpeed != CY_U3P_SUPER_SPEED \r\n");
        CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t *)glProbeCtrl20);
      }
      break;
    case CY_FX_USB_UVC_SET_CUR_REQ:
      /* The host has selected the parameters for the video stream. Check the desired
         resolution settings, configure the sensor and start the video stream. */
      apiRetStatus = CyU3PUsbGetEP0Data(CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
                                        glCommitCtrl, &readCount);
      sensor_dbg("bRequest = CY_FX_USB_UVC_SET_CUR_REQ\r\n");
      if (apiRetStatus == CY_U3P_SUCCESS) {
        if (usbSpeed == CY_U3P_SUPER_SPEED) {
          switch (glProbeCtrl[3]) {
          case 1:
            sensor_dbg(" <start stream - default VGA> \r\n");
            V034_stream_start(SENSOR_ADDR_WR);
            break;
          case 2:
            break;
          default:
            break;
          }

          sensor_dbg("Res switch index: %x,Switched %d times\r\n", glProbeCtrl[3], res_switch);
          res_switch++;
        } else {
          sensor_dbg("<<start stream - error reset>>\r\n");
          V034_stream_start(SENSOR_ADDR_WR);
        }
        // could merge into the setting above
        // V034_SensorChangeConfig();

        status = kfifo_init(&IMU_kfifo, (void *)IMU_pool_buf, IMU_POOL_LEN);
        if (status)
          sensor_err("IMU kfifo init error!\r\n");
        /* We can start streaming video now. */
        sensor_dbg(" <start stream - now> \r\n");

        apiRetStatus = CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_OR);
        if (apiRetStatus != CY_U3P_SUCCESS) {
          sensor_err("Set CY_FX_UVC_STREAM_EVENT failed %x\n", apiRetStatus);
        }
      }
      break;

    default:
      CyU3PUsbStall(0, CyTrue, CyFalse);
      break;
    }
    break;

  default:
    CyU3PUsbStall(0, CyTrue, CyFalse);
    break;
  }
}

/*
 * Entry function for the UVC control request processing thread.
 */
void UVC_EP0Thread_Entry(uint32_t input) {
  uint32_t eventMask = CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT | CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT;
  uint32_t eventFlag;

  sensor_dbg("UVC control request proceeding thread\r\n");
  for (;;) {
    /* Wait for a Video control or streaming related request on the control endpoint. */
    if (CyU3PEventGet(&glFxUVCEvent, eventMask, CYU3P_EVENT_OR_CLEAR, &eventFlag,
                      CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS) {
      /* If this is the first request received, query the connection speed. */
      if (!isUsbConnected) {
        usbSpeed = CyU3PUsbGetSpeed();
        if (usbSpeed != CY_U3P_NOT_CONNECTED) {
          isUsbConnected = CyTrue;
          sensor_dbg("<_connected_>\r\n");
        }
      }

      if (eventFlag & CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT) {
        switch ((wIndex >> 8)) {
        case CY_FX_UVC_PROCESSING_UNIT_ID:
          // sensor_dbg("UVC Handle Processing Unit Rqts\r\n");
          Handle_ProcessingUnit_Rqts();
          break;

        case CY_FX_UVC_CAMERA_TERMINAL_ID:
          // sensro_dbg("CY_FX_UVC_CAMERA_TERMINAL_ID\r\n");
          Handle_CameraTerminal_Rqts();
          break;

        case CY_FX_UVC_INTERFACE_CTRL:
          sensor_dbg("<CY_FX_UVC_INTERFACE_CTRL>\r\n");
          Handle_InterfaceCtrl_Rqts();
          break;

        case CY_FX_UVC_EXTENSION_UNIT_ID:
          // sensor_dbg("<CY_FX_UVC_EXTENSION_UNIT_ID>\r\n");
          Handle_ExtensionUnit_Rqts();
          break;

        default:
          /* Unsupported request. Fail by stalling the control endpoint. */
          sensor_err("<<Unsupported request>>\r\n");
          CyU3PUsbStall(0, CyTrue, CyFalse);
          break;
        }
      }

      if (eventFlag & CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT) {
        if (wIndex != CY_FX_UVC_STREAM_INTERFACE) {
          sensor_err("<stream_error>\r\n");
          CyU3PUsbStall(0, CyTrue, CyFalse);
        } else {
          sensor_dbg("<stream_request:>\r\n");
          Handle_VideoStreaming_Rqts();
        }
      }
    }
    /* Allow other ready threads to run. */
    CyU3PThreadRelinquish();
  }
}

/*
 * Entry function for the Data handle thread.
 */
void Data_handle_Thread_Entry(uint32_t input) {
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
  int count = 0;

  sensor_dbg("start Data handle thread!\r\n");
  for (;;) {
    ++count;
    if (glIsApplnActive && readyIMU == CyTrue && (IMU_kfifo.kfifo_flag & KFIFO_IS_START)) {
#ifdef IMU_LOOP_SAMPLE
      uint8_t raw_IMU_data[14];
      status = icm_get_sensor_reg(raw_IMU_data, 0);
      if (status != CY_U3P_SUCCESS) {
          sensor_err("get icm data err\r\n");
      }
      int i = 0;
      for (i = 0; i < 6; ++i) {
        /* 0~5byte: gyro; 6~12btye:accel */
        last_imu[i + 0] = (char)(raw_IMU_data[i + 8]);
        last_imu[i + 6] = (char)(raw_IMU_data[i]);
      }
      uint32_t t = CyU3PGetTime();
      last_imu[12] = t >> 24;
      last_imu[13] = t >> 16;
      last_imu[14] = t >> 8;
      last_imu[15] = t >> 0;
      /* show this version can get burst imu from image. */
      last_imu[16] = 0;
      if (firmware_ctrl_flag & IMU_FROM_IMAGE_BIT) {
        CyU3PMutexGet(&(IMU_kfifo.lock), CYU3P_WAIT_FOREVER);
        if (kfifo_unused(&IMU_kfifo) >= sizeof(last_imu)) {
          kfifo_in(&IMU_kfifo, (void *)last_imu, sizeof(last_imu));
        } else {
          sensor_err("IMU Pool is full ,Please check frame rate\r\n");
        }
        CyU3PMutexPut(&(IMU_kfifo.lock));
      }
#endif
      CyU3PThreadSleep(1);
    } else {
      /* No active data transfer. Sleep for a small amount of time. */
      CyU3PThreadSleep(100);
    }
    /* Allow other ready threads to run before proceeding. */
    CyU3PThreadRelinquish();
  }
}

/*
 * This function is called by the FX3 framework once the ThreadX RTOS has started up.
 * The application specific threads and other OS resources are created and initialized here.
 */
void CyFxApplicationDefine(void) {
  void *ptr1, *ptr2, *ptr3;
  uint32_t retThrdCreate;

  /* Allocate the memory for the thread stacks. */
  ptr1 = CyU3PMemAlloc(UVC_APP_THREAD_STACK);
  ptr2 = CyU3PMemAlloc(UVC_APP_THREAD_STACK);
  ptr3 = CyU3PMemAlloc(UVC_APP_THREAD_STACK);
  if ((ptr1 == 0) || (ptr2 == 0) || (ptr3 == 0))
    goto fatalErrorHandler;

  /* Create the UVC application thread. */
  retThrdCreate = CyU3PThreadCreate(&uvcAppThread,
                                     "UVC App Thread",
                                     UVC_AppThread_Entry,
                                     0,
                                     ptr1,
                                     UVC_APP_THREAD_STACK,
                                     UVC_APP_THREAD_PRIORITY,
                                     UVC_APP_THREAD_PRIORITY,
                                     CYU3P_NO_TIME_SLICE,
                                     CYU3P_AUTO_START);
  if (retThrdCreate != 0) {
    goto fatalErrorHandler;
  }

  /* Create the control request handling thread. */
  retThrdCreate = CyU3PThreadCreate(&uvcAppEP0Thread,
                                     "UVC App EP0 Thread",
                                     UVC_EP0Thread_Entry,
                                     0,
                                     ptr2,
                                     UVC_APP_EP0_THREAD_STACK,
                                     UVC_APP_EP0_THREAD_PRIORITY,
                                     UVC_APP_EP0_THREAD_PRIORITY,
                                     CYU3P_NO_TIME_SLICE,
                                     CYU3P_AUTO_START);
  if (retThrdCreate != 0) {
    goto fatalErrorHandler;
  }

  retThrdCreate = CyU3PThreadCreate(&Datahandle_Thread,
                                     "Data handle Thread",
                                     Data_handle_Thread_Entry,
                                     0,
                                     ptr3,
                                     UVC_APP_EP0_THREAD_STACK,
                                     UVC_APP_DATA_THREAD_PRIORITY,
                                     UVC_APP_DATA_THREAD_PRIORITY,
                                     CYU3P_NO_TIME_SLICE,
                                     CYU3P_AUTO_START);
  if (retThrdCreate != 0) {
    goto fatalErrorHandler;
  }

  return;

fatalErrorHandler:
  /* Add custom recovery or debug actions here */
  /* Loop indefinitely */
  while (1);
}

/* Main entry point for the C code. We perform device initialization and start
 * the ThreadX RTOS here. */
int main(void) {
  CyU3PReturnStatus_t apiRetStatus;

  firmware_ctrl_flag = firmware_ctrl_flag | DEBUG_DBG_BIT;
  firmware_ctrl_flag = firmware_ctrl_flag | DEBUG_INFO_BIT;
  firmware_ctrl_flag = firmware_ctrl_flag | DEBUG_DUMP_BIT;
  firmware_ctrl_flag = firmware_ctrl_flag | IMU_FROM_IMAGE_BIT;

  debug_level = firmware_ctrl_flag & (DEBUG_DBG_BIT | DEBUG_INFO_BIT |DEBUG_DUMP_BIT);
  /* Initialize the device */
  apiRetStatus = CyU3PDeviceInit(0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    goto handle_fatal_error;
  }

  /* Turn on instruction cache to improve firmware performance.
     Use Release build to improve it further */
  apiRetStatus = CyU3PDeviceCacheControl(CyTrue, CyFalse, CyFalse);

  /* IO matrix configure */
  fx3_IO_matrix_config();

  /* This is a non returnable call for initializing the RTOS kernel */
  CyU3PKernelEntry();
  /* Dummy return to make the compiler happy */
  return 0;

handle_fatal_error:
  /* Cannot recover from this error. */
  while (1);
}
