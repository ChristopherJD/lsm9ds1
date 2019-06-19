#ifndef LSM9DS1_PRIVATE_H_
#define LSM9DS1_PRIVATE_H_

#ifdef __cplusplus
extern "C"
{
#endif

lsm9ds1_status_t lsm9ds1_select_sub_device(lsm9ds1_device_t *self, lsm9ds1_sub_device_t sub_device);

#ifdef __cplusplus
}
#endif

#endif /* LSM9DS1_PRIVATE_H_ */