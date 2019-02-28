/**
 * @file lv_ex_porting.h
 *
 */

#ifndef LV_TUTORIAL_PORTING_H
#define LV_TUTORIAL_PORTING_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/


#include "lv_Conf.h"   
#include "../lvgl/lvgl.h"

#include "BSP_LCD.h"
#include "BSP_Touch.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lv_porting(void);

/**********************
 *      MACROS
 **********************/

#endif /*USE_LV_TUTORIALS*/

#ifdef __cplusplus
} /* extern "C" */


#endif /*LV_TUTORIAL_PORTING_H*/
