
//------------------------------------------------------------------------------------------
//                              ISTART LORA V2.0 - Init defenition

#define LEDG 8
#define LEDB 9
#define LEDR 10
#define BTN_BR 6    //Botão
#define BTN_BL SDA  //Botão
#define BTN_TR 13   //Botão
#define BTN_TL SCL  //Botão
#define BTN_M 12    //Botão

#define BAT_2_SENSOR_POWER 7  //High to have bat power in sensors

#define BUZZER 3

#define LDR_OR A1  //LDR orange
#define LDR_BL A2  //LDR blue
#define LDR_GR A3  //LDR green
#define LDR_BR A4  //LDR brown

#define ADC_AREF 2.33f

#define RFM_CS 11   //PA16
#define RFM_DIO0 5  //PA15
#define RFM_RST 2   //PA14

#define INT_BLACK_CABLE_MOLEX 6

#define MEDIR_CAP 3 //PA09
#define CAP_V A2
#define ADC_AREF 2.33f
#define BATVOLT_R1 100.0f
#define BATVOLT_R2 150.0f
#define BATVOLT_PIN CAP_V

const byte DIO1 = A5; //LoRa Pin

int reset_cause = 0; //0 - normal power on ; 1 - Reset requested by System ; 2 - Reset by Watchdog ; 3 - External reset requested ; 4 - Reset Brown Out 3.3V; 5 - Reset Brown Out 1.2V
volatile bool sleep_flag = true;

void MCU_Sleep() {

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  Serial.flush();
  Serial.end();
  // Disable USB
  USBDevice.detach();
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB

  // Disable systick interrupt
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
  //BARRIER INSTRUCTION - WASNT HERE BEFORE
  __DSB();
  // SAMD sleep
  __WFI();
  // Enable systick interrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

  Serial.begin(256000);
}

//------------------------------------------------------------------------------------------
//                              ISTART LORA V2.0 - Fuses

void setup_BOD33(){
    Serial.println(F("Fuse settings before:"));
    Serial.println((*(uint32_t*)NVMCTRL_USER), HEX);         // Display the current user word 0 fuse settings
    Serial.println((*(uint32_t*)(NVMCTRL_USER + 4)), HEX);   // Display the current user word 1 fuse settings
    uint32_t userWord0 = *((uint32_t*)NVMCTRL_USER);            // Read fuses for user word 0
    uint32_t userWord1 = *((uint32_t*)(NVMCTRL_USER + 4));      // Read fuses for user word 1
    NVMCTRL->CTRLB.bit.CACHEDIS = 1;                            // Disable the cache
    NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS / 2;               // Set the address
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_EAR |                // Erase the auxiliary user page row
                         NVMCTRL_CTRLA_CMDEX_KEY;
    while(!NVMCTRL->INTFLAG.bit.READY)                          // Wait for the NVM command to complete
    NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;                 // Clear the error flags
    NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS / 2;               // Set the address
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_PBC |                // Clear the page buffer
                         NVMCTRL_CTRLA_CMDEX_KEY;
    while(!NVMCTRL->INTFLAG.bit.READY)                          // Wait for the NVM command to complete
    NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;                 // Clear the error flags
    *((uint32_t*)NVMCTRL_USER) = userWord0 & ~FUSES_BOD33_EN_Msk;  // Disable the BOD33 enable fuse in user word 0
    *((uint32_t*)(NVMCTRL_USER + 4)) = userWord1;               // Copy back user word 1 unchanged
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_WAP  |               // Write to the user page
                         NVMCTRL_CTRLA_CMDEX_KEY;
    while(!NVMCTRL->INTFLAG.bit.READY)                          // Wait for the NVM command to complete
    NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;                 // Clear the error flags
    NVMCTRL->CTRLB.bit.CACHEDIS = 0;                            // Enable the cache
    Serial.println(F("Fuse settings after:"));
    Serial.println((*(uint32_t*)NVMCTRL_USER), HEX);         // Display the current user word 0 fuse settings
    Serial.println((*(uint32_t*)(NVMCTRL_USER + 4)), HEX);   // Display the current user word 1 fuse settings

}

void nvm_wait_states(){
     NVMCTRL->CTRLB.bit.RWS = 3; 
}

//------------------------------------------------------------------------------------------
//                              Watch Dog

#include <Sodaq_wdt.h>

//------------------------------------------------------------------------------------------
//                              Battery

float getBatteryVoltage(){
  digitalWrite(MEDIR_CAP, HIGH);
  sodaq_wdt_safe_delay(100); //Must be bellow wdt call
  uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(CAP_V));
  digitalWrite(MEDIR_CAP, LOW);
  sodaq_wdt_safe_delay(100);
  return (float)voltage;
}