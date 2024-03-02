#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "lcd.h"

void app_main(void)
{
  lcd_init();
  while (true)
  {
    printf("Hello from app_main!\n");
    sleep(1);
  }
}
