/*for Hall sensor common header file*/
#ifndef __LINUX_HALL_SENSOR_H
#define __LINUX_HALL_SENSOR_H

#include <linux/notifier.h>

#define HALL_POLE_BIT  1
#define HALL_POLE_N    0
#define HALL_POLE_S    1
#define HALL_FAR       0
#define HALL_NEAR      1

extern int hallsensor_register_notifier(struct notifier_block *nb);
extern int hallsensor_unregister_notifier(struct notifier_block *nb);
extern int hallsensor_notifier_call_chain(unsigned long val, void *v);

#endif
