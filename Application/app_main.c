#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include<rf_task.h>
#include<ultrasonic_task.h>



/* usermain関数 */
EXPORT INT usermain(void)
{
	tskid_1 = tk_cre_tsk(&ctsk_1);
	tskid_2 = tk_cre_tsk(&ctsk_2);
	tm_printf((UB*)"Task ID = %d\r\n", tskid_1);
	tk_sta_tsk(tskid_1, 0);
	tk_slp_tsk(TMO_FEVR);
/*	tm_printf((UB*)"Task ID = %d\r\n", tskid_2);
	tk_sta_tsk(tskid_2, 0);
	tk_slp_tsk(TMO_FEVR);*/
	return 0;
}
