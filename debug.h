#pragma once

/* define macros, if debug is enabled */

#ifdef DEBUG
#define LV_(s, args...) printf(s "\n", args)
#define LS_(s) printf(s "\n")
#define L__(s) printf(#s "\n")
#else
#define LV_(s, args...) do {} while(0)
#define LS_(s) do {} while(0)
#define L__(s) do {} while(0)
#endif
