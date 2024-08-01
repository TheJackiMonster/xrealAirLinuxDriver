#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifndef __cplusplus
#include <stdint.h>
#else
#include <cstdint>
#endif

#define NUM_SUPPORTED_PRODUCTS 3

extern const uint16_t xreal_vendor_id;
extern const uint16_t xreal_product_ids[NUM_SUPPORTED_PRODUCTS];

bool is_xreal_product_id(uint16_t product_id);