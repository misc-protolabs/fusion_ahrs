#ifndef _SD_LOG_H_
#define _SD_LOG_H_

typedef struct {
  char buf[512];
  int len;
  unsigned long log_idx;
  bool sd_present;
  bool logging;
} sd_log_T;

extern sd_log_T sd_log;

extern void sd_log_init( void);
extern void sd_log_write( void);
extern void sd_log_close( void);
extern unsigned int sd_log_new( void);

#endif
