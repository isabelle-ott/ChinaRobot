/**
 * @file debouncer.c
 * @brief 通用消抖处理器，用于处理按键等“01传感器”的数据抖动
 * @author TanES (tan_163mail@163.com)
 */
#include "debouncer.h"
typedef struct DEBOUNCER_param_struct DEBOUNCER_prarm_t;
typedef struct DEBOUNCER_struct DEBOUNCER_t;
/* 消抖器的配置参数结构体，可以被多个消抖器共用 */
struct DEBOUNCER_param_struct
{
	uint8_t Rising_debounce;	 // 是否对上升沿消抖
	uint8_t Descending_debounce; // 是否对下降沿消抖
	uint32_t debounce_time;		 // 消抖时间。对于要消抖的沿，只有持续超过这个时间的稳定状态
};

/* 消抖器结构体，需要为每个传感器定义一个消抖器 */
struct DEBOUNCER_struct
{
	DEBOUNCER_prarm_t *param;		   // 配置参数
	uint8_t raw_data;				   // 原始数据
	uint8_t debounced_data;			   // 消抖之后的数据
	uint8_t is_debounceData_change;	   // 消抖数据切换的标志位
	uint32_t last_read_tick;		   // 上一次采集raw_data的时刻
	uint32_t last_rawData_bounce_tick; // raw_data的最后一次跳变的时刻
};

void debounce_init(DEBOUNCER_t *debounce, DEBOUNCER_prarm_t *debounce_param, uint8_t state, uint32_t tick)
{
	debounce->param = debounce_param;
	debounce->raw_data = state;
	debounce->debounced_data = state;
	debounce->is_debounceData_change = 0;
	debounce->last_read_tick = tick;
	debounce->last_rawData_bounce_tick = tick;
}

/**
 * @brief 对数据进行消抖处理
 * @param debounce 消抖器
 * @param data 采集的数据(非零值会被归并为1)
 * @param tick 时间戳
 */
uint8_t debounce_process(DEBOUNCER_t *debounce, uint8_t data, uint32_t tick)
{
	data = !!data;
	uint8_t is_refresh_debouncedData = 0;

	/* raw_data 发生跳变时，重置计时 */
	if (data != debounce->raw_data)
	{
		debounce->raw_data = data;
		debounce->last_rawData_bounce_tick = tick;
		/* 当前跳变沿不需要消抖时，需要把当前采集值更新到稳定值 */
		is_refresh_debouncedData = ((data == 1 && (!debounce->param->Rising_debounce)) ||
									(data == 0 && (!debounce->param->Descending_debounce)));
	}
	/* 如果距离上次跳变的时间超过了消抖时间，需要把当前采集值更新到稳定值 */
	is_refresh_debouncedData |= (tick >= debounce->param->debounce_time + debounce->last_rawData_bounce_tick);
	/* 如果距离上次采集的时间超过了消抖时间，需要把当前采集值更新到稳定值 */
	is_refresh_debouncedData |= (tick >= debounce->param->debounce_time + debounce->last_read_tick);

	/* 稳定值与采集值不同，才有必要更新稳定值 */
	if (is_refresh_debouncedData && (data != debounce->debounced_data))
	{
		debounce->debounced_data = data;
		debounce->is_debounceData_change = 1;
	}
	else
		debounce->is_debounceData_change = 0;

	debounce->last_read_tick = tick;
	return debounce->debounced_data;
}
