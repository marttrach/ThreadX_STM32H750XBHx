#ifndef IOT_H
#define IOT_H
// Debugging utilities
#define IOT_LOG_NONE      0
#define IOT_LOG_DEBUG     1
#define IOT_LOG_INFO      2
#define IOT_LOG_WARNING   3
#define IOT_LOG_ERR       4
//Set debug level, 0 means no debug
#define IOT_DEBUG_LEVEL IOT_LOG_DEBUG

#if (IOT_DEBUG_LEVEL ==  0)
#define DEBUG_DUMP(...)  do{}while(0)
#else
#define DEBUG_DUMP(Lv, ...) do{if((Lv) >= IOT_DEBUG_LEVEL) printf(__VA_ARGS__);}while(0)
#endif
#endif
