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
    0x0428 // XREAL Air 2
};

bool is_xreal_product_id(uint16_t product_id) {
    for (int i = 0; i < NUM_SUPPORTED_PRODUCTS; i++) {
        if (xreal_product_ids[i] == product_id) {
            return true;
        }
    }
    return false;
}