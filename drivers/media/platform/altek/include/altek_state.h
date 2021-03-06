/*
 * File: altek_state.h
 * Description: FSM
 *
 * Copyright 2019-2030  Altek Semiconductor Corporation
 */


/*
 * this file was generated by NunniFSMGen
 */

/*
 * This file is part of al6100.
 *
 * al6100 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2, as published by
 * the Free Software Foundation.
 *
 * al6100 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTIBILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License version 2 for
 * more details.
 *
 * You should have received a copy of the General Public License version 2
 * along with al6100. If not, see https://www.gnu.org/licenses/gpl-2.0.html.
 */


#ifndef altek_state_H
#define altek_state_H

#include "altek_statefsm.h"

#ifdef __cplusplus
extern "C"
{
#endif		/* __cplusplus */


int state_open(void *o);
int error_sequence(void *o);
int redundant_sequence(void *o);
int state_enter_hwpt(void *o);
int state_write_basic_code(void *o);
int state_close(void *o);
int state_leave_hwpt(void *o);
int state_scenario_chg(void *o);
int state_strm_on_off(void *o);
int state_enter_cp(void *o);
int state_set_qp(void *o);
int state_stop(void *o);
int state_leave_cp(void *o);
int state_leave_cp_standy(void *o);


#ifdef __cplusplus
}
#endif		/* __cplusplus */


#endif
