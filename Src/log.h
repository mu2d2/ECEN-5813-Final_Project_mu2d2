/**
 ******************************************************************************
 * @file           : log.h
 * @author         : Muthuu SVS
 * @brief          : abstraction away from printf
 * Reference PES 10 F091RC and GPIO pt2.pdf
 ******************************************************************************
 */

#ifndef LOG_H
#define LOG_H

#include <stdio.h>

//#define DEBUG
#ifdef DEBUG
#	define LOG(format, ...) printf(format, ##__VA_ARGS__)
#else
#	define LOG(format, ...) ((void)0)
#endif


#endif /*LOG_H*/
