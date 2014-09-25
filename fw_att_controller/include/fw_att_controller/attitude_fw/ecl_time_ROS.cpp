#include <ros/ros.h>
#include <stdint.h>

uint64_t ecl_elapsed_time(uint64_t *lastrun)
{
	return (ros::Time::now().toNSec()/1e3 - *lastrun) ;	//maybe should guard against negative values
}

uint64_t ecl_absolute_time()
{
	return (ros::Time::now().toNSec()/1e3);
}
