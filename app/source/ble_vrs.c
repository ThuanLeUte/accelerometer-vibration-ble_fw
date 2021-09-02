/**
 * @file       ble_vrs.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Thuan Le
 * @brief      BRS (BLE Vibration Service)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "sdk_common.h"
#include "ble.h"
#include "ble_vrs.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "damos_ram.h"

/* Private defines ---------------------------------------------------- */
#define BLE_UUID_VRS_ANGLE_LEVEL_CHARACTERISTIC  0x3235

#define VRS_BASE_UUID                                                                                \
  {                                                                                                  \
    {                                                                                                \
      0x41, 0xEE, 0x68, 0x3A, 0x99, 0x0F, 0x0E, 0x72, 0x85, 0x49, 0x8D, 0xB3, 0x00, 0x00, 0x00, 0x00 \
    }                                                                                                \
  } /**< Used vendor specific UUID. */

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void on_connect(ble_vrs_t *p_vrs, ble_evt_t const *p_ble_evt);
static void on_write(ble_vrs_t *p_vrs, ble_evt_t const *p_ble_evt);
static ret_code_t vibration_level_char_add(ble_vrs_t *p_vrs,
                                          const ble_vrs_init_t *p_vrs_init);
static ret_code_t vibration_notification_send(ble_gatts_hvx_params_t *const p_hvx_params,
                                              uint16_t conn_handle);

/* Function definitions ----------------------------------------------- */
uint32_t ble_vrs_init(ble_vrs_t *p_vrs, ble_vrs_init_t const *p_vrs_init)
{
  ret_code_t err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t vrs_base_uuid = VRS_BASE_UUID;

  VERIFY_PARAM_NOT_NULL(p_vrs);
  VERIFY_PARAM_NOT_NULL(p_vrs_init);

  // Initialize the service structure.
  p_vrs->evt_handler               = p_vrs_init->evt_handler;
  p_vrs->is_notification_supported = p_vrs_init->support_notification;
  p_vrs->vibration_last            = 0;

  // Add a custom base UUID.
  err_code = sd_ble_uuid_vs_add(&vrs_base_uuid, &p_vrs->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_vrs->uuid_type;
  ble_uuid.uuid = BLE_UUID_VRS_SERVICE;

  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                      &ble_uuid,
                                      &p_vrs->service_handle);
  VERIFY_SUCCESS(err_code);

  // Add the Vibration Characteristic.
  err_code = vibration_level_char_add(p_vrs, p_vrs_init);
  return err_code;
}
     
ret_code_t ble_vrs_vibration_update(ble_vrs_t *p_vrs,
                                    float     vibration,
                                    uint16_t  conn_handle)
{
  VERIFY_PARAM_NOT_NULL(p_vrs);

  ret_code_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  if (vibration != p_vrs->vibration_last)
  {
    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = 4;
    gatts_value.offset  = 0;
    gatts_value.p_value =(uint8_t *) &vibration;

    // Update database.
    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                      p_vrs->vibration_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
      NRF_LOG_INFO( "Vibration has been updated: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(vibration));

      // Save new Vibration value.
      p_vrs->vibration_last = vibration;
    }
    else
    {
      NRF_LOG_DEBUG("Error during Vibration update: 0x%08X", err_code)

      return err_code;
    }

    // Send value if connected and notifying.
    if (p_vrs->is_notification_supported)
    {
      ble_gatts_hvx_params_t hvx_params;

      memset(&hvx_params, 0, sizeof(hvx_params));

      hvx_params.handle = p_vrs->vibration_handles.value_handle;
      hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
      hvx_params.offset = gatts_value.offset;
      hvx_params.p_len  = &gatts_value.len;
      hvx_params.p_data = gatts_value.p_value;

      if (conn_handle == BLE_CONN_HANDLE_ALL)
      {
        ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

        // Try sending notifications to all valid connection handles.
        for (uint32_t i = 0; i < conn_handles.len; i++)
        {
          if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
          {
            if (err_code == NRF_SUCCESS)
            {
              err_code = vibration_notification_send(&hvx_params,
                                                     conn_handles.conn_handles[i]);
            }
            else
            {
              // Preserve the first non-zero error code
              UNUSED_RETURN_VALUE(vibration_notification_send(&hvx_params,
                                                              conn_handles.conn_handles[i]));
            }
          }
        }
      }
      else
      {
        err_code = vibration_notification_send(&hvx_params, conn_handle);
      }
    }
    else
    {
      err_code = NRF_ERROR_INVALID_STATE;
    }
  }

  return err_code;
}

void ble_vrs_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
  if ((p_context == NULL) || (p_ble_evt == NULL))
  {
    return;
  }

  ble_vrs_t *p_vrs = (ble_vrs_t *)p_context;

  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
    on_connect(p_vrs, p_ble_evt);
    break;

  case BLE_GATTS_EVT_WRITE:
    on_write(p_vrs, p_ble_evt);
    break;

  default:
    break;
  }
}

/* Private function definitions --------------------------------------- */
/**
 * @brief         Function for adding the Vibration characteristic.
 *
 * @param[in]     p_vrs         Vibration Service structure.
 * @param[in]     p_vrs_init    Information needed to initialize the service.
 *
 * @attention     None
 *
 * @return        None
 */
static ret_code_t vibration_level_char_add(ble_vrs_t *p_vrs, const ble_vrs_init_t *p_vrs_init)
{
  ret_code_t              err_code;
  ble_add_char_params_t   add_char_params;
  ble_add_descr_params_t  add_descr_params;
  uint8_t                 initial_vibration;
  uint8_t                 init_len;
  uint8_t                 encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];

  // Add Vibration characteristic
  initial_vibration = p_vrs_init->initial_vibration;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid                     = BLE_UUID_VRS_ANGLE_LEVEL_CHARACTERISTIC;
  add_char_params.max_len                  = sizeof(float);
  add_char_params.init_len                 = sizeof(float);
  add_char_params.p_init_value             = &initial_vibration;
  add_char_params.char_props.notify        = p_vrs->is_notification_supported;
  add_char_params.char_props.read          = 1;
  add_char_params.char_props.write         = 1;
  add_char_params.char_props.write_wo_resp = 1;
  add_char_params.cccd_write_access        = p_vrs_init->bl_cccd_wr_sec;
  add_char_params.read_access              = p_vrs_init->bl_rd_sec;
  add_char_params.write_access             = p_vrs_init->bl_rd_sec;

  err_code = characteristic_add(p_vrs->service_handle,
                                &add_char_params,
                                &(p_vrs->vibration_handles));
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  if (p_vrs_init->p_report_ref != NULL)
  {
    // Add Report Reference descriptor
    init_len = ble_srv_report_ref_encode(encoded_report_ref, p_vrs_init->p_report_ref);

    memset(&add_descr_params, 0, sizeof(add_descr_params));
    add_descr_params.uuid        = BLE_UUID_REPORT_REF_DESCR;
    add_descr_params.read_access = p_vrs_init->bl_report_rd_sec;
    add_descr_params.init_len    = init_len;
    add_descr_params.max_len     = add_descr_params.init_len;
    add_descr_params.p_value     = encoded_report_ref;

    err_code = descriptor_add(p_vrs->vibration_handles.value_handle,
                              &add_descr_params,
                              &p_vrs->report_ref_handle);
    return err_code;
  }
  else
  {
    p_vrs->report_ref_handle = BLE_GATT_HANDLE_INVALID;
  }

  return NRF_SUCCESS;
}

/**
 * @brief         Function for sending notifications with the Vibration Level characteristic.
 *
 * @param[in]     p_hvx_params Pointer to structure with notification data.
 * @param[in]     conn_handle  Connection handle.
 *
 * @attention     None
 *
 * @return        NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t vibration_notification_send(ble_gatts_hvx_params_t *const p_hvx_params,
                                                 uint16_t conn_handle)
{
  ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);
  if (err_code == NRF_SUCCESS)
  {
    NRF_LOG_INFO("Vibration notification has been sent using conn_handle: 0x%04X", conn_handle);
  }
  else
  {
    NRF_LOG_DEBUG("Error: 0x%08X while sending notification with conn_handle: 0x%04X",
                  err_code,
                  conn_handle);
  }
  return err_code;
}

/**
 * @brief         Function for handling the Connect event.
 *
 * @param[in]     p_vrs       Vibration Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void on_connect(ble_vrs_t *p_vrs, ble_evt_t const *p_ble_evt)
{

}

/**
 * @brief         Function for handling the Write event.
 *
 * @param[in]     p_vrs       Vibration Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void on_write(ble_vrs_t *p_vrs, ble_evt_t const *p_ble_evt)
{
  ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  g_degree_vibration_level = p_evt_write->data[0];

  if (g_degree_vibration_level > 10)
    g_degree_vibration_level = 10;
  else if (g_degree_vibration_level == 0)
    g_degree_vibration_level = 1;

  NRF_LOG_INFO("on_write: %d", g_degree_vibration_level);
}

/* End of file -------------------------------------------------------- */
