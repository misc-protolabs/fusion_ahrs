#include "SPI.h"
#include "FS.h"
#include "SD_MMC.h"

#include "sd_log.h"

File logfile;
sd_log_T sd_log;

unsigned int sd_log_new( void)
{
  unsigned int n = 0;
  unsigned int n_log = 0;
  char fname[32];
  int stat = SD_MMC.exists( "/logs");
  if( !stat) {
    stat = SD_MMC.mkdir( "/logs");
  }

  char hdr[128];
  int len;
  for( n=0; n<0xffff; n++)
  {
    sprintf( (char*)(&fname[0]), "/logs/%08x.csv", n);
    stat = SD_MMC.exists( (const char*)(&fname[0]));
    if( !stat)
    {
      logfile = SD_MMC.open( fname, FILE_WRITE);

      len = sprintf( hdr, "idx,dt,batt-v");
      logfile.write( (const uint8_t*)( hdr), len);
      len = sprintf( hdr, ",acc-x,acc-y,acc-z");
      logfile.write( (const uint8_t*)( hdr), len);
      len = sprintf( hdr, ",gyro-x,gyro-y,gyro-z");
      logfile.write( (const uint8_t*)( hdr), len);
      len = sprintf( hdr, ",mag-x,mag-y,mag-z");
      logfile.write( (const uint8_t*)( hdr), len);
      len = sprintf( hdr, ",pitch,roll,yaw");
      logfile.write( (const uint8_t*)( hdr), len);
      len = sprintf( hdr, ",fe-acc-x,fe-acc-y,fe-acc-z");
      logfile.write( (const uint8_t*)( hdr), len);
      len = sprintf( hdr, ",fl-acc-x,fl-acc-y,fl-acc-z");
      logfile.write( (const uint8_t*)( hdr), len);

      len = sprintf( hdr, "\n");
      logfile.write( (const uint8_t*)( hdr), len);
      sd_log.log_idx = 0;
      n_log = n;
      n = 0xffff;
      Serial.printf( "SD - (%S) created.\n", fname);
      delay(500);
    }
  }
  sd_log.logging = 1;
  return n_log;
}

void sdmmc_init( void)
{
  sd_log.sd_present = 0;
  if(!SD_MMC.begin()){
      Serial.println("Card Mount Failed");
      return;
  }
  sd_log.sd_present = 1;
  sd_log.logging = 0;
  
  uint8_t cardType = SD_MMC.cardType();

  if(cardType == CARD_NONE){
      Serial.println("No SD_MMC card attached");
      return;
  }

  Serial.print("SD_MMC Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
  Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));

  unsigned int n_log = sd_log_new();
}

void sd_log_init( void) {
  Serial.println( "---sdmmc-init.");
  sdmmc_init();
}

void sd_log_write( void)
{
  if( sd_log.len > 0)
  {
    logfile.write( (const uint8_t *)( &sd_log.buf[0]), sd_log.len);
    sd_log.len = 0;
  }
}

void sd_log_close( void)
{
  logfile.close();
  sd_log.logging = 0;
}
