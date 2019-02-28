/**
 * @file lv_tutorial_themes.h
 *
 */

#ifndef LV_TUTORIAL_THEMES_H
#define LV_TUTORIAL_THEMES_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../lv_conf.h"


#define USE_LV_TUTORIALS 1

#if USE_LV_TUTORIALS
    
#include "../lvgl/lvgl.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lv_tutorial_themes(void);

/**********************
 *      MACROS
 **********************/

#endif /*USE_LV_TUTORIALS*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_TUTORIAL_THEMES_H*/
