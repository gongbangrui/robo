#ifndef IR_DETECT_EXEC_H
#define IR_DETECT_EXEC_H

#include "cross_exec.h"

extern const cross_executor_t ir_detect_exec;

/* 查询前红外是否满足触发条件 (电平由 IR_TRIGGER_LEVEL 配置) */
uint8_t ir_detect_triggered(void);

#endif
