/*
 ## Cypress FX3 Camera Kit header file (camera_ptzcontrol.h)
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

/*
 * This file defines the variables and functions used to control and query the Pan, Tilt
 * and Zoom controls for this UVC camera function.
 */
#ifndef FIRMWARE_INCLUDE_CAMERA_PTZCONTROL_H_
#define FIRMWARE_INCLUDE_CAMERA_PTZCONTROL_H_
#include "include/uvc.h"
#ifdef UVC_PTZ_SUPPORT
/* Minimum Lobjective value for the sensor lens. */
#define wObjectiveFocalLengthMin                (uint16_t)(1)
/* Maximum Lobjective value for the sensor lens. */
#define wObjectiveFocalLengthMax                (uint16_t)(10)
/* Locular value for the sensor lens. */
#define wOcularFocalLength                      (uint16_t)(1)
/* Default zoom setting that we start with. */
#define ZOOM_DEFAULT                            (uint16_t)(5)
/* Minimum supported zoom value. */
#define CyFxUvcAppGetMinimumZoom()              (wObjectiveFocalLengthMin)
/* Maximum supported zoom value. */
#define CyFxUvcAppGetMaximumZoom()              (wObjectiveFocalLengthMax)
/* Zoom resolution is one unit. */
#define CyFxUvcAppGetZoomResolution()           ((uint16_t)1)
/* Default zoom setting. */
#define CyFxUvcAppGetDefaultZoom()              ((uint16_t)ZOOM_DEFAULT)
/* Minimum value for Pan and Tilt controls. */
#define PANTILT_MIN                             (int32_t)(-648000)
/* Maximum value for Pan and Tilt controls. */
#define PANTILT_MAX                             (int32_t)(648000)
/* Minimum pan value. */
#define CyFxUvcAppGetMinimumPan()               (PANTILT_MIN)
/* Maximum pan value. */
#define CyFxUvcAppGetMaximumPan()               (PANTILT_MAX)
/* Resolution for pan setting. */
#define CyFxUvcAppGetPanResolution()            ((int32_t)1)
/* Default pan setting. */
#define CyFxUvcAppGetDefaultPan()               ((int32_t)0)
/* Minimum tilt value. */
#define CyFxUvcAppGetMinimumTilt()              (PANTILT_MIN)
/* Maximum tilt value. */
#define CyFxUvcAppGetMaximumTilt()              (PANTILT_MAX)
/* Resolution for tilt setting. */
#define CyFxUvcAppGetTiltResolution()           ((int32_t)1)
/* Default tilt setting. */
#define CyFxUvcAppGetDefaultTilt()              ((int32_t)0)

/* Function    : CyFxUvcAppPTZInit
   Description : Initialize the Pan, Tilt and Zoom settings for the camera.
   Parameters  : None
 */
extern void CyFxUvcAppPTZInit(void);

/* Function    : CyFxUvcAppGetCurrentZoom
   Description : Get the current zoom setting for the sensor.
   Parameters  : None
 */
extern uint16_t CyFxUvcAppGetCurrentZoom(void);


/* Function    : CyFxUvcAppGetCurrentPan
   Description : Get the current pan setting for the camera.
   Parameters  : None
 */
extern int32_t CyFxUvcAppGetCurrentPan(void);

/* Function    : CyFxUvcAppGetCurrentTilt
   Description : Get the current tilt setting for the camera.
   Parameters  : None
 */
extern int32_t CyFxUvcAppGetCurrentTilt(void);

/* Function    : CyFxUvcAppModifyPan
   Description : Stub function that can be filled in to implement a camera PAN control.
   Parameters  :
                 panValue - PAN control value selected by the host application.
 */
extern void CyFxUvcAppModifyPan(int32_t panValue);

/* Function    : CyFxUvcAppModifyTilt
   Description : Stub function that can be filled in to implement a camera TILT control.
   Parameters  :
                 tiltValue - TILT control value selected by the host application.
 */
extern void CyFxUvcAppModifyTilt(int32_t tiltValue);

/* Function    : CyFxUvcAppModifyZoom
   Description : Stub function that can be filled in to implement a camera ZOOM control.
   Parameters  :
                 zoomValue - ZOOM control value selected by the host application.
 */
extern void CyFxUvcAppModifyZoom(uint16_t zoomValue);
#endif
#endif  // FIRMWARE_INCLUDE_CAMERA_PTZCONTROL_H_
