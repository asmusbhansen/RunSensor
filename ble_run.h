#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "nrf_gpio.h"

/**@brief   Macro for defining a ble_run instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_RUN_DEF(_name)                                                                          \
static ble_run_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_run_on_ble_evt, &_name)


//8efcc191-6ee9-4c70-8615-3456c364d7e6 //Original 128bit IID from uuidgenerator.net


#define RUN_SERVICE_UUID_BASE         {0xe6, 0xd7, 0x64, 0xc3, 0x56, 0x34, 0x15, 0x86, \
                                          0x70, 0x4c, 0xe9, 0x6e, 0x91, 0xc1, 0xfc, 0x8e}

#define RUN_SERVICE_UUID              0x0000
#define XY_VALUE_CHAR_UUID            0x0001
#define YZ_VALUE_CHAR_UUID            0x0001



typedef enum
{
    BLE_RUN_EVT_NOTIFICATION_ENABLED,                             /**< Run value notification enabled event. */
    BLE_RUN_EVT_NOTIFICATION_DISABLED,                            /**< Run value notification disabled event. */
    BLE_RUN_EVT_DISCONNECTED,
    BLE_RUN_EVT_CONNECTED
} ble_run_evt_type_t;

/**@brief Run Service event. */
typedef struct
{
    ble_run_evt_type_t evt_type;                                  /**< Type of event. */
} ble_run_evt_t;

// Forward declaration of the ble_run_t type.
typedef struct ble_run_s ble_run_t;

/**@brief Run Service event handler type. */
/** this defines ble_run_evt_handlet_t as a pointer to a function taking argument p_run and p_evt while returning void **/
typedef void (*ble_run_evt_handler_t) (ble_run_t * p_run, ble_run_evt_t * p_evt);

/**@brief Run Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_run_evt_handler_t         evt_handler;                /**< Event handler to be called for handling events in the Run Service. */
    uint8_t                       initial_xz_value;           /**< Initial xz value */
    ble_srv_cccd_security_mode_t  xz_value_char_attr_md;      /**< Initial security level for XZ characteristics attribute */
    //New val
    uint8_t                       initial_yz_value;           /**< Initial yz value */
    ble_srv_cccd_security_mode_t  yz_value_char_attr_md;      /**< Initial security level for YZ characteristics attribute */
    //\New val
} ble_run_init_t;


/**@brief Run Service structure. This contains various status information for the service. */
struct ble_run_s
{
    ble_run_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Run Service. */
    uint16_t                      service_handle;                 /**< Handle of Run Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      xz_value_handles;               /**< Handles related to the Run Value characteristic. */
    //New val
    ble_gatts_char_handles_t      yz_value_handles;               /**< Handles related to the Custom Value characteristic. */
    //\New val
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

/**@brief Function for initializing the Run Service.
 *
 * @param[out]  p_run       Run Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_run_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_run_init(ble_run_t * p_run, const ble_run_init_t * p_run_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 * @param[in]   p_context  Run Service structure.
 */
void ble_run_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for updating the Run value.
 *
 * @details The application calls this function when the Run value should be updated. If
 *          notification has been enabled, the Run value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_run          Run Service structure.
 * @param[in]   Run value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_run_xz_value_update(ble_run_t * p_run, uint8_t xz_value);


