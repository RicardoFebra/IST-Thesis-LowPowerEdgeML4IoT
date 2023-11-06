
//#include <stdio.h>
//#include "sdkconfig.h"
//#include "soc/soc_caps.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "esp_sleep.h"
//#include "esp_log.h"
#include "driver/rtc_io.h"

static void deep_sleep_task(void *args)
{
    // enter deep sleep
    esp_deep_sleep_start();
}

static void deep_sleep_register_rtc_timer_wakeup(void)
{
    const int wakeup_time_sec = 5;
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
}

//------------------------------------------------------------------------------------------
//                              Watch Dog
#include <esp_task_wdt.h>

bool timer_flag = 0;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

//--------------------------------------------------------------------------
//                      EEPROM

#include <Preferences.h>
Preferences preferences;