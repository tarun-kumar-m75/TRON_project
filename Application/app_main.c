#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "main.h"

uint32_t value = 0;
LOCAL void task_1(INT stacd, void *exinf);	// task execution function
LOCAL ID	tskid_1;			// Task ID number
LOCAL T_CTSK ctsk_1 = {				// Task creation information
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= task_1,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL void task_2(INT stacd, void *exinf);	// task execution function
LOCAL ID	tskid_2;			// Task ID number
LOCAL T_CTSK ctsk_2 = {				// Task creation information
	.itskpri	= 10,
	.stksz		= 1024,
	.task		= task_2,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL void task_1(INT stacd, void *exinf){

	while(1){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		value = HAL_ADC_GetValue(&hadc1);
		tk_dly_tsk(100);
	}
}

LOCAL void task_2(INT stacd, void *exinf){
	while(1){
		tm_printf((UB*)"adc value : ");
		tm_printf((UB*)"%d\n",value);
		tk_dly_tsk(50);
	}
}

EXPORT INT usermain(void)
{
	/* Create & Start Tasks */
	tm_putstring((UB*)"Start User-main program.\n");

	tskid_1 = tk_cre_tsk(&ctsk_1);
	tk_sta_tsk(tskid_1, 0);

	tskid_2 = tk_cre_tsk(&ctsk_2);
	tk_sta_tsk(tskid_2, 0);

	tk_slp_tsk(TMO_FEVR);

	return 0;
}
