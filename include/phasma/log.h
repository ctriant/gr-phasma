/*
 * log.h
 *
 *  Created on: Sep 5, 2016
 *      Author: ctriant
 */

#ifndef INCLUDE_PHASMA_LOG_H_
#define INCLUDE_PHASMA_LOG_H_

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/syscall.h>

#define PHASMA_MESSAGES 1
#define PHASMA_DEBUG_MESSAGES 0

#if PHASMA_MESSAGES
#define PHASMA_LOG_CLASS_INFO(CLASS_DEBUG_ENABLE, M, ...) 				\
	do { 										\
		if(CLASS_DEBUG_ENABLE) { 						\
				fprintf(stderr, "[INFO]: " M " \n", ##__VA_ARGS__); 	\
		}									\
	} while(0)

#else
#define PHASMA_LOG_CLASS_INFO(CLASS, M, ...)
#endif

#if PHASMA_MESSAGES
#define PHASMA_LOG_INFO(M, ...) 							\
		fprintf(stderr, "[INFO]: " M " \n", ##__VA_ARGS__)

#else
#define PHASMA_LOG_INFO(M, ...)
#endif

#define PHASMA_ERROR(M, ...) 							\
	fprintf(stderr, "[ERROR] %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define PHASMA_WARN(M, ...) 								\
	fprintf(stderr, "[WARNING] %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#if PHASMA_DEBUG_MESSAGES
#define PHASMA_DEBUG(M, ...) 							\
	fprintf(stderr, "[DEBUG]: " M "\n", ##__VA_ARGS__)
#else
#define PHASMA_DEBUG(M, ...)
#endif



#endif /* INCLUDE_PHASMA_LOG_H_ */
