#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\build_cli2\\sketch\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\esp32_camera\\sensors\\private_include\\ov7670.h"
#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\esp32_camera\\sensors\\private_include\\ov7670.h"
/*
 * This file is part of the OpenMV project.
 * author: Juan Schiavoni <juanjoseschiavoni@hotmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV7670 driver.
 *
 */
#ifndef __OV7670_H__
#define __OV7670_H__
#include "sensor.h"

/**
 * @brief Detect sensor pid
 *
 * @param slv_addr SCCB address
 * @param id Detection result
 * @return
 *     0:       Can't detect this sensor
 *     Nonzero: This sensor has been detected
 */
int ov7670_detect(int slv_addr, sensor_id_t *id);

/**
 * @brief initialize sensor function pointers
 *
 * @param sensor pointer of sensor
 * @return
 *      Always 0
 */
int ov7670_init(sensor_t *sensor);

#endif // __OV7670_H__
