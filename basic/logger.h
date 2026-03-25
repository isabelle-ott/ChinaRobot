#ifndef LOGGER_H
#define LOGGER_H
#include "bsp.h"
void _logger_print_init(void);
/* ���ĺ궨����������Կ�����Ӧ�Ĵ�ӡ */

void xxx_print_log(const char *str);
void xxx_print_log_f(const char *format, ...);

void logger_restart_RX(void);

char *logger_getToIDLE(void);

char *logger_get(void);

void logger_itCall(uint16_t Size, BaseType_t *pxHigherPriorityTaskWoken);


// print ==��ʱ��ӡ==
#if 1
#define print(str) xxx_print_log(str "\n")
#define print_f(format, ...) xxx_print_log_f(format "\n", ##__VA_ARGS__)
#else
#define print(str)
#define print_f(format, ...)
#endif

// log_error ==��ӡ����==
#if 1
#define log_error(str) xxx_print_log("\x1b[91;107m[err]" str "\n")
#define log_error_f(format, ...) xxx_print_log_f("\x1b[91;107m[err]" format "\n", ##__VA_ARGS__)
#else
#define log_error(str)
#define log_error_f(format, ...)
#endif

// log_info ==������ӡ==
#if 1
#define log_info(str) xxx_print_log(str "\n")
#define log_info_f(format, ...) xxx_print_log_f(format "\n", ##__VA_ARGS__)
#else
#define log_info(str)
#define log_info_f(format, ...)
#endif

#endif
