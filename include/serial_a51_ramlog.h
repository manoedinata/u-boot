/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SERIAL_A51_RAMLOG_H
#define __SERIAL_A51_RAMLOG_H

/**
 * a51_ramlog_since_boot() - Everything this u-boot has written to the ramlog
 *
 * Return: malloc()ed, NUL-terminated copy (caller frees), or NULL if the
 * ramlog console is not in use or out of memory
 */
char *a51_ramlog_since_boot(void);

#endif
