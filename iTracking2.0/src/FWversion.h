/*
 * FWversion.h
 *
 *  Created on: Mar 9, 2017
 *      Author: dv198
 */

#ifndef FWVERSION_H_
#define FWVERSION_H_

/*-----------------------------------------------------------------------------
 * MACRO
 ------------------------------------------------------------------------------*/
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
/*-----------------------------------------------------------------------------
 * DEFINE
 ------------------------------------------------------------------------------*/
#define FW_BUILTDATE		__DATE__ " at " __TIME__
#define FW_MAINVERSION		2
#define FW_SUBVERSION		0
#define FW_OEM_BRANCHNAME	"DVS"
#define FW_VERSION 			STR(FW_MAINVERSION) "." STR(FW_SUBVERSION) "_" FW_OEM_BRANCHNAME

#endif /* FWVERSION_H_ */
