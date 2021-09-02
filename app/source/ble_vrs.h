/**
 * @file       ble_vrs.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      VRS (BLE Vibration Service)
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BLE_VIBRATION_SERVICE_H
#define __BLE_VIBRATION_SERVICE_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

/* Public defines ----------------------------------------------------- */
#define BLE_UUID_VRS_SERVICE (0x3234) /**< The UUID of the Vibration Service. */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Vibration Service event type
 */
typedef enum
{
  BLE_VRS_EVT_NOTIFICATION_ENABLED, /**< Vibration value notification enabled event. */
  BLE_VRS_EVT_NOTIFICATION_DISABLED /**< Vibration value notification disabled event. */
} 
ble_vrs_evt_type_t;

/**
 * @brief Vibration Service event.
 */
typedef struct
{
  ble_vrs_evt_type_t evt_type;     /**< Type of event. */
  uint16_t           conn_handle;  /**< Connection handle. */
}
ble_vrs_evt_t;

/* Forward declaration of the ble_vrs_t type. */
typedef struct ble_vrs_s ble_vrs_t;

/* Vibration Service event handler type. */
typedef void (* ble_vrs_evt_handler_t) (ble_vrs_t * p_vrs, ble_vrs_evt_t * p_evt);

/**
 * @brief Nordic Vibration Service initialization structure.
 */
typedef struct
{
  ble_vrs_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Vibration Service. */
  bool                   support_notification;           /**< TRUE if notification of Vibration measurement is supported. */
  ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Vibration characteristic */
  uint8_t                initial_vibration;              /**< Initial Vibration */
  security_req_t         bl_rd_sec;                      /**< Security requirement for reading the BL characteristic value. */
  security_req_t         bl_cccd_wr_sec;                 /**< Security requirement for writing the BL characteristic CCCD. */
  security_req_t         bl_report_rd_sec;               /**< Security requirement for reading the BL characteristic descriptor. */
}
ble_vrs_init_t;

/**
 * @brief Nordic Vibration Service structure.
 */
struct ble_vrs_s
{
  uint8_t                  uuid_type;                 /**< UUID type for Vibration Service Base UUID. */
  ble_vrs_evt_handler_t    evt_handler;               /**< Event handler to be called for handling events in the Vibration Service. */
  uint16_t                 service_handle;            /**< Handle of Vibration Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t vibration_handles;         /**< Handles related to the Vibration characteristic. */
  uint16_t                 report_ref_handle;         /**< Handle of the Report Reference descriptor. */
  float                    vibration_last;            /**< Last Vibration measurement passed to the Vibration Service. */
  bool                     is_notification_supported; /**< TRUE if notification of Vibration is supported. */
};

/* Public macros ------------------------------------------------------ */
/**
 * @brief  Macro for defining a ble_nus instance.
 *
 * @param[in]     _name  Name of the instance.
 *
 * @attention     None
 *
 * @return        None
 */
#define BLE_VRS_DEF(_name)                        \
static ble_vrs_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,               \
                     BLE_HRS_BLE_OBSERVER_PRIO,   \
                     ble_vrs_on_ble_evt, &_name)

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief                     Function for initializing the Nordic Vibration Service.
 *
 * @param[in]     p_vrs_init  Information needed to initialize the service.
 * 
 * @param[out]    p_vrs       Nordic Vibration Service structure. This structure must be supplied
 *                            by the application. It is initialized by this function and will
 *                            later be used to identify this particular service instance.
 *
 * @attention     None
 *
 * @return
 * - NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * - NRF_ERROR_NULL If either of the pointers p_vrs or p_vrs_init is NULL.
 */
uint32_t ble_vrs_init(ble_vrs_t *p_vrs, ble_vrs_init_t const *p_vrs_init);

/**
 * @brief                        Function for updating the Vibration level.
 *
 * @param[in]     p_bas          Vibration Service structure.
 * @param[in]     vibration      New Vibration measurement value
 * @param[in]     conn_handle    Connection handle.
 * 
 * @attention     None
 *
 * @return        None
 */
ret_code_t ble_vrs_vibration_update(ble_vrs_t *p_vrs,
                                    float     vibration,
                                    uint16_t  conn_handle);

/**
 * @brief                     Function for handling the Nordic Vibration Service's BLE events.
 *
 * @param[in]     p_ble_evt   Event received from the SoftDevice.
 * @param[in]     p_context   Nordic Vibration Service structure.
 * 
 * @attention     None
 *
 * @return        None
 */
void ble_vrs_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

#endif // __BLE_VIBRATION_SERVICE_H

/* End of file -------------------------------------------------------- */
