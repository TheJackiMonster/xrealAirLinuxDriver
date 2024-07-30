#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifndef __cplusplus
#include <stdint.h>
#else
#include <cstdint>
#endif

#include "hid_ids.h"

const uint16_t xreal_vendor_id = 0x3318;
const uint16_t xreal_product_ids[NUM_SUPPORTED_PRODUCTS] = {
    0x0424, // XREAL Air
    0x0428, // XREAL Air 2
    0x0432, // XREAL Air 2 Pro
    0x0426  // XREAL Air 2 Ultra
};

const int xreal_imu_interface_ids[NUM_SUPPORTED_PRODUCTS] = {
    3, // XREAL Air
    3, // XREAL Air 2
    3, // XREAL Air 2 Pro
    2  // XREAL Air 2 Ultra
};

const int xreal_mcu_interface_ids[NUM_SUPPORTED_PRODUCTS] = {
    4,  // XREAL Air
    4,  // XREAL Air 2
    4,  // XREAL Air 2 Pro
    -1  // TODO - XREAL Air 2 Ultra MCU support via interface 0
};

int xreal_imu_interface_id(uint16_t product_id) {
    for (int i = 0; i < NUM_SUPPORTED_PRODUCTS; i++) {
        if (xreal_product_ids[i] == product_id) {
            return xreal_imu_interface_ids[i];
        }
    }

    return -1;
}

int xreal_mcu_interface_id(uint16_t product_id) {
    for (int i = 0; i < NUM_SUPPORTED_PRODUCTS; i++) {
        if (xreal_product_ids[i] == product_id) {
            return xreal_mcu_interface_ids[i];
        }
    }

    return -1;
}