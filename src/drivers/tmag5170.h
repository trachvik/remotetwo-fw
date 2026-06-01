#ifndef TMAG5170_H_
#define TMAG5170_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>

/**
 * @brief TMAG5170 device structure
 */
struct tmag5170_device {
        const struct device *dev;
};

/**
 * @brief Initialize TMAG5170 encoder (looks up DT node tmag5170)
 *
 * @param dev Pointer to tmag5170_device structure
 * @return 0 on success, negative errno code on failure
 */
int tmag5170_init(struct tmag5170_device *dev);

/**
 * @brief Read angle in degrees (0-360)
 *
 * @param dev Pointer to tmag5170_device structure
 * @param angle_deg Pointer to store angle in degrees
 * @return 0 on success, negative errno code on failure
 */
int tmag5170_read_angle_deg(struct tmag5170_device *dev, float *angle_deg);

/**
 * @brief Read angle in radians (0-2π)
 *
 * @param dev Pointer to tmag5170_device structure
 * @param angle_rad Pointer to store angle in radians
 * @return 0 on success, negative errno code on failure
 */
int tmag5170_read_angle_rad(struct tmag5170_device *dev, float *angle_rad);

#endif /* TMAG5170_H_ */
