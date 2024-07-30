#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifndef __cplusplus
#include <stdint.h>
#else
#include <cstdint>
#endif

#define NUM_SUPPORTED_PRODUCTS 4

extern const uint16_t xreal_vendor_id;
extern const uint16_t xreal_product_ids[NUM_SUPPORTED_PRODUCTS];
int xreal_imu_interface_id(uint16_t product_id);
int xreal_mcu_interface_id(uint16_t product_id);