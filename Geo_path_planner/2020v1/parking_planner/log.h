#ifndef _PLANNING_LOG_H_
#define _PLANNING_LOG_H_


#include <iostream>
#include <glog/logging.h>
#include <glog/raw_logging.h>

#define ADEBUG VLOG(4) << "[DEBUG] "
#define AINFO LOG(INFO)
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AFATAL LOG(FATAL)



#endif