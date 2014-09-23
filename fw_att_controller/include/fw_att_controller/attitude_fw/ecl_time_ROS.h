/*
 * ecl_time_ROS.h
 *
 *  Created on: Sep 22, 2014
 *      Author: roman
 */

#ifndef ECL_TIME_ROS_H_
#define ECL_TIME_ROS_H_

#include <ros/ros.h>
#include <stdint.h>

uint64_t ecl_elapsed_time(uint64_t *lastrun);
uint64_t ecl_absolute_time();


#endif /* ECL_TIME_ROS_H_ */
