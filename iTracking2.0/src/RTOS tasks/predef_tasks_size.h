/*------------------------------------------------------------------------------------------
 * INTRODUCE
 * Brief: This is header file contain all tasks memory allocating definitions.
 * Author: Duy Vinh To - Team DinhViSo
 * Detail:
 -------------------------------------------------------------------------------------------*/
#ifndef TASKMEMORY_H_
#define TASKMEMORY_H_

/*------------------------------------------------------------------------------------------
 * DEFINES
 -------------------------------------------------------------------------------------------*/
#define TSKMAIN_SIZE		( 512 * 3 )
#define TSKLOG_SIZE			( 256 ) //4096
#define TSKRECTD_SIZE		( 512 )//4096
#define TSKNETCOMM_SIZE		( 512 * 2 )
#define TSKNETCOMMIR_SIZE	( 256 * 2 )
#define TSKBKPTRKDAT2FL_SIZE (512 * 2)
#define TSKBKPTD_SIZE		( 1024 * 1 + 512 )
#define TSKCMD_SIZE			( 256 )
#define TSKSENS_SIZE		( 256 )
#define TSKRFIDREADER_SIZE	( 256 )
#define TSKCAM_SIZE	( 256 )

#endif // TASKMEMORY_H_
