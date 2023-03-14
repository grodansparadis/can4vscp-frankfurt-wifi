
#include <stdio.h>
#include <stdlib.h>

#define VSCP_MALLOC(s)   malloc(s)
#define VSCP_REMALLOC(s) remalloc(s)
#define VSCP_FREE(x)     free(x)
