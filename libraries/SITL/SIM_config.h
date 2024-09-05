#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS_config.h>

#ifndef HAL_SIM_ADSB_ENABLED
#define HAL_SIM_ADSB_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED
#define HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef HAL_SIM_PS_RPLIDARA1_ENABLED
#define HAL_SIM_PS_RPLIDARA1_ENABLED HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED
#endif

#ifndef HAL_SIM_PS_RPLIDARA2_ENABLED
#define HAL_SIM_PS_RPLIDARA2_ENABLED HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED
#endif

#ifndef AP_SIM_IS31FL3195_ENABLED
#define AP_SIM_IS31FL3195_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_LED_N_ENABLED
#define AP_SIM_LED_N_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL) && defined(WITH_SITL_RGBLED)
#endif

#ifndef AP_SIM_GPIO_LED_1_ENABLED
#define AP_SIM_GPIO_LED_1_ENABLED AP_SIM_LED_N_ENABLED && 0
#endif

#ifndef AP_SIM_GPIO_LED_2_ENABLED
#define AP_SIM_GPIO_LED_2_ENABLED AP_SIM_LED_N_ENABLED && 0
#endif

#ifndef AP_SIM_GPIO_LED_3_ENABLED
#define AP_SIM_GPIO_LED_3_ENABLED AP_SIM_LED_N_ENABLED && 0
#endif

#ifndef AP_SIM_GPIO_LED_RGB_ENABLED
#define AP_SIM_GPIO_LED_RGB_ENABLED AP_SIM_LED_N_ENABLED && 0
#endif

#ifndef AP_SIM_LOWEHEISER_ENABLED
#define AP_SIM_LOWEHEISER_ENABLED AP_SIM_ENABLED && HAL_MAVLINK_BINDINGS_ENABLED
#endif

#ifndef AP_SIM_SHIP_ENABLED
#define AP_SIM_SHIP_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_SLUNGPAYLOAD_ENABLED
#define AP_SIM_SLUNGPAYLOAD_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_TSYS03_ENABLED
#define AP_SIM_TSYS03_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_ADSB_SAGETECH_MXS_ENABLED
#define AP_SIM_ADSB_SAGETECH_MXS_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_SERIALDEVICE_CORRUPTION_ENABLED
#define AP_SIM_SERIALDEVICE_CORRUPTION_ENABLED 0
#endif

#ifndef HAL_SIM_GPS_ENABLED
#define HAL_SIM_GPS_ENABLED AP_SIM_ENABLED
#endif

#ifndef AP_SIM_GPS_BACKEND_DEFAULT_ENABLED
#define AP_SIM_GPS_BACKEND_DEFAULT_ENABLED AP_SIM_ENABLED
#endif

#ifndef AP_SIM_GPS_FILE_ENABLED
// really need to use AP_FileSystem for this.
#define AP_SIM_GPS_FILE_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#endif

#ifndef AP_SIM_GPS_SBF_ENABLED
#define AP_SIM_GPS_SBF_ENABLED AP_SIM_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_SIM_GPS_TRIMBLE_ENABLED
#define AP_SIM_GPS_TRIMBLE_ENABLED AP_SIM_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_SIM_GPS_MSP_ENABLED
#define AP_SIM_GPS_MSP_ENABLED AP_SIM_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_SIM_GPS_NMEA_ENABLED
#define AP_SIM_GPS_NMEA_ENABLED AP_SIM_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_SIM_GPS_NOVA_ENABLED
#define AP_SIM_GPS_NOVA_ENABLED AP_SIM_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_SIM_GPS_SBP2_ENABLED
#define AP_SIM_GPS_SBP2_ENABLED AP_SIM_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_SIM_GPS_SBP_ENABLED
#define AP_SIM_GPS_SBP_ENABLED AP_SIM_GPS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_SIM_GPS_UBLOX_ENABLED
#define AP_SIM_GPS_UBLOX_ENABLED AP_SIM_GPS_BACKEND_DEFAULT_ENABLED
#endif


// simulated compass support; currently only in SITL, not SimOnHW:
#ifndef AP_SIM_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_SIM_COMPASS_BACKEND_DEFAULT_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_COMPASS_QMC5883L_ENABLED
#define AP_SIM_COMPASS_QMC5883L_ENABLED AP_SIM_COMPASS_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_SIM_STRATOBLIMP_ENABLED
#define AP_SIM_STRATOBLIMP_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_SA_GD2000_ENABLED
#define AP_SIM_SA_GD2000_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_GLIDER_ENABLED
#define AP_SIM_GLIDER_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SIM_SOLOGIMBAL_ENABLED
#define AP_SIM_SOLOGIMBAL_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL && HAL_MAVLINK_BINDINGS_ENABLED)
#endif

#ifndef AP_SIM_GIMBAL_ENABLED
#define AP_SIM_GIMBAL_ENABLED (AP_SIM_SOLOGIMBAL_ENABLED)
#endif

