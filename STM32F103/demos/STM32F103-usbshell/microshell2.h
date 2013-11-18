//
//  microshell2.h
//  
//
//  Created by Marian Such on 8/5/13.
//
//

#ifndef _microshell2_h
#define _microshell2_h

#include <stdbool.h>

#define chprint(...) chprintf((BaseChannel *)&STDOUT_SD, __VA_ARGS__)

void start_shell(void);

typedef void (*shellcmd_t)(int argc, char * argv[]);


typedef struct {
	const char        *sc_name;
	shellcmd_t        sc_function;
//	bool              sc_expand;
//	char **           sc_keys;
} ShellCommand;


#endif
