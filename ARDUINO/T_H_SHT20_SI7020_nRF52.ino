// ########################################  Thermometer and hygrometer on nRF52840 ############################################### //
//  LED -     P0.05                                                                                                                 //
//  BUTTON -  P0.03                                                                                                                 //
//  SDA -     P0.30                                                                                                                 //
//  SCL -     P0.31                                                                                                                 //
//  RX -      P0.09                                                                                                                 //
//  TX -      P0.10                                                                                                                 //
//                                                                                                                                  //                                                                                                                              //
//  For external interrupts to work, you must add __attribute__ ((weak)) in the Winterrups.c file                                   //
//  before the void GPIOTE_IRQHandler (){} function.                                                                                //
//                                                                                                                                  //
//  File path - C:\Users!!!USER NAME!!!\AppData\Local\Arduino15\packages\sandeepmistry\hardware\nRF5\0.6.0\cores\nRF5               //
//                                                                                                                                  //
// ################################################################################################################################ //

//#define SHT20
#define SI7020

//#define TESTING
#define RF52840 // If you use a nRF52840, if you use nRF52832, then disable it


bool onoff = 1;
bool change;
bool chek_h = true;
bool check;
bool tch;
bool hch;
bool bch;
bool configMode;
bool button_flag;
bool nosleep;
bool flag_update_transport_param;
bool flag_sendRoute_parent;
bool flag_no_present;
bool flag_nogateway_mode;
bool flag_find_parent_process;
bool Ack_FP;

uint8_t cpNom;
uint8_t cpCount;
uint8_t timeSend;
uint8_t battSend;
uint8_t battery;
uint8_t old_battery;
uint8_t err_delivery_beat;
uint8_t problem_mode_count;

float temperatureSend;
float humiditySend;
int16_t temperature;
int16_t humidity;
float old_temperature;
float old_humidity;
float tempThreshold = 0.5; // порог сравнения в предыдущими показаниями температуры
float humThreshold = 3.0; // порог сравнения в предыдущими показаниями влажности
int16_t nRFRSSI;
int16_t old_nRFRSSI;
int16_t myid;
int16_t mypar;
int16_t old_mypar = -1;

uint16_t temp;
uint16_t batteryVoltage;
uint16_t minuteT = 60000;
uint16_t BATT_TIME;
uint16_t BATT_COUNT;

uint32_t lightMillisR;
uint32_t configMillis;
uint32_t previousMillis;
uint32_t time_start_ms;
uint32_t time_now_s;
uint32_t SLEEP_TIME;

float batteryVoltageF;


#ifdef SHT20
#include "DFRobot_SHT20.h"
DFRobot_SHT20    sensor; // https://github.com/DFRobot/DFRobot_SHT20
#endif

#ifdef SI7020
#include "Adafruit_Si7021.h" // https://github.com/adafruit/Adafruit_Si7021
Adafruit_Si7021 sensor = Adafruit_Si7021();
#endif

#define MY_DEBUG
#ifndef MY_DEBUG
#define MY_DISABLED_SERIAL
#endif
#define MY_RADIO_NRF5_ESB
#define MY_NRF5_ESB_PA_LEVEL (NRF5_PA_MAX)
int16_t mtwr;
#define MY_TRANSPORT_WAIT_READY_MS (mtwr)


//#define MY_PASSIVE_NODE
//#define MY_NODE_ID 200
//#define MY_PARENT_NODE_ID 0
//#define MY_PARENT_NODE_IS_STATIC
//#define MY_TRANSPORT_UPLINK_CHECK_DISABLED

#ifdef SHT20
#define SN "EFEKTA T&H SHT20"
#endif
#ifdef SI7020
#define SN "EFEKTA T&H SI7020"
#endif
#define SV "1.94"

#define TEMP_ID 1
#define HUM_ID 2
#define SIGNAL_Q_ID 100
#define BATTERY_VOLTAGE_ID 101
#define SET_TIME_SEND_ID 102
#define SET_BATT_SEND_ID 103


#include <MySensors.h>
MyMessage msgTemp(TEMP_ID, V_TEMP);
MyMessage msgHum(HUM_ID, V_HUM);
MyMessage sqMsg(SIGNAL_Q_ID, V_VAR1);
MyMessage bvMsg(BATTERY_VOLTAGE_ID, V_VAR1);
MyMessage setTimeSendMsg(SET_TIME_SEND_ID, V_VAR1);
MyMessage setBattSendMsg(SET_BATT_SEND_ID, V_VAR1);
#if defined(TESTING)
MyMessage tempMsg(254, V_VAR1);
#endif

// SDK PORT
uint32_t PIN_BUTTON_MASK;
volatile byte buttIntStatus = 0;
#define APP_GPIOTE_MAX_USERS 1
extern "C" {
#include "app_gpiote.h"
#include "nrf_gpio.h"
}
static app_gpiote_user_id_t m_gpiote_user_id;


void preHwInit() {
  pinMode(PIN_BUTTON, INPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, HIGH);
}


void before()
{
  //########################################## CONFIG MCU ###############################################


  //##############################################################
#ifdef RF52840

  if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
      ((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos))) {
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
    NRF_UICR->PSELRESET[0] = 18;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
    NRF_UICR->PSELRESET[1] = 18;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
    NVIC_SystemReset();
  }
#endif
  //################################################################

  NRF_POWER->DCDCEN = 1;
  NRF_SAADC ->ENABLE = 0;
  NRF_PWM0  ->ENABLE = 0;

  NRF_NFCT->TASKS_DISABLE = 1;
  NRF_NVMC->CONFIG = 1;
  NRF_UICR->NFCPINS = 0;
  NRF_NVMC->CONFIG = 0;
  NRF_PWM1  ->ENABLE = 0;
  NRF_PWM2  ->ENABLE = 0;
  NRF_TWIM1 ->ENABLE = 0;
  NRF_TWIS1 ->ENABLE = 0;

#ifndef MY_DEBUG
  NRF_UART0->ENABLE = 0;
#endif



  //########################################## INIT HAPPY ##############################################

  happy_init();

  //########################################## CONFIG PROG ###############################################

  timeSend = loadState(102);  // сохранение в памяти интервала отправки данных с сенсора sht/si, максимано 60 минут, если 0 то отправку не делает, только обновляет инфо на экране
  if (timeSend > 30) {
    timeSend = 30;
    saveState(102, timeSend);
  }
  //timeSend = 1; // для теста, 1 минута

  battSend = loadState(103);  // сохранение в памяти интервала отправки данных о заряде батареи и качества сигнала, максимано 24 часа, если 0 то отправку не делает, только обновляет инфо на экране
  if (battSend > 24) {
    battSend = 24;
    saveState(103, battSend);
  }
  //battSend = 1; // для теста, 1 час

  default_param();

  digitalWrite(BLUE_LED, LOW);
  wait(50);
  digitalWrite(BLUE_LED, HIGH);
}


void presentation()
{
  check = sendSketchInfo(SN, SV);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(30);
    check = sendSketchInfo(SN, SV);
    wait(30);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(TEMP_ID, S_TEMP, "Temperature");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(45);
    check = present(TEMP_ID, S_TEMP, "Temperature");
    wait(45);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(HUM_ID, S_HUM, "Humidity");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(60);
    check = present(HUM_ID, S_HUM, "Humidity");
    wait(60);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(75);
    check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
    wait(75);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(90);
    check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
    wait(90);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_TIME_SEND_ID, S_CUSTOM, "T&H SEND INTERVAL | Min");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(105);
    check = present(SET_TIME_SEND_ID, S_CUSTOM, "T&H SEND INTERVAL | Min");
    wait(105);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(120);
    check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
    wait(120);
    _transportSM.failedUplinkTransmissions = 0;
  }
  wait(100);
}


void setup() {
  config_Happy_node();
  readBatt();

  check = send(setTimeSendMsg.set(timeSend));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(135);
    check = send(setTimeSendMsg.set(timeSend));
    wait(135);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = send(setBattSendMsg.set(battSend));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(150);
    check = send(setBattSendMsg.set(battSend));
    wait(150);
    _transportSM.failedUplinkTransmissions = 0;
  }

#if defined(TESTING)
  check = send(tempMsg.set(SLEEP_TIME));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(300);
    check = send(tempMsg.set(SLEEP_TIME));
    wait(300);
    _transportSM.failedUplinkTransmissions = 0;
  }

  check = send(tempMsg.set(BATT_TIME));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(300);
    check = send(tempMsg.set(BATT_TIME));
    wait(300);
    _transportSM.failedUplinkTransmissions = 0;
  }
#endif

#ifdef RF52840
  NRF_RADIO->TXPOWER = 8;
  wait(10);
#endif

  interrupt_Init();
  wait(30);

#ifdef SHT20
  sensor.initSHT20();
#endif
#ifdef SI7020
  sensor.begin();
#endif
}



void loop() {
  if (flag_update_transport_param == true) {
    update_Happy_transport();
  }
  if (flag_sendRoute_parent == true) {
    present_only_parent();
  }
  if (isTransportReady() == true) {
    if (flag_nogateway_mode == false) {
      if (flag_find_parent_process == true) {
        find_parent_process();
      }

      if (configMode == 0) {
        if (buttIntStatus == PIN_BUTTON) {
          if (digitalRead(PIN_BUTTON) == 0 && button_flag == 0) {
            button_flag = 1;
            previousMillis = millis();
            ledsOff();
          }

          if (digitalRead(PIN_BUTTON) == 0 && button_flag == 1) {
            if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 3000)) {
              if (millis() - lightMillisR > 70) {
                lightMillisR = millis();
                onoff = !onoff;
                digitalWrite(BLUE_LED, onoff);
              }
            }
            if ((millis() - previousMillis > 3000) && (millis() - previousMillis <= 4000)) {
              ledsOff();
            }
            if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 7000)) {
              if (millis() - lightMillisR > 55) {
                lightMillisR = millis();
                onoff = !onoff;
                digitalWrite(BLUE_LED, onoff);
              }
            }
            if ((millis() - previousMillis > 7000) && (millis() - previousMillis <= 8000)) {
              ledsOff();
            }
            if ((millis() - previousMillis > 8000) && (millis() - previousMillis <= 11000)) {
              // ############################ BINDING MODE -- DISABLE ############################
              if (millis() - lightMillisR > 40) {
                lightMillisR = millis();
                onoff = !onoff;
                digitalWrite(BLUE_LED, onoff);
              }
            }
            if ((millis() - previousMillis > 11000) && (millis() - previousMillis <= 12000)) {
              ledsOff();
            }
            if ((millis() - previousMillis > 12000) && (millis() - previousMillis <= 15000)) {
              if (millis() - lightMillisR > 25) {
                lightMillisR = millis();
                onoff = !onoff;
                digitalWrite(BLUE_LED, onoff);
              }
            }
            if (millis() - previousMillis > 15000) {
              ledsOff();
            }
          }

          if (digitalRead(PIN_BUTTON) == 1 && button_flag == 1) {
            if (millis() - previousMillis <= 3000 && button_flag == 1)
            {
              ledsOff();
              button_flag = 0;
              buttIntStatus = 0;
              presentation();
              nosleep = 0;
            }
            if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 7000) && button_flag == 1)
            {
              ledsOff();
              configMode = 1;
              button_flag = 0;
              buttIntStatus = 0;
              NRF5_ESB_startListening();
              wait(30);
              configMillis = millis();
            }
            if ((millis() - previousMillis > 8000) && (millis() - previousMillis <= 11000) && button_flag == 1)
            {
              // ############################ BINDING MODE -- DISABLE ############################
              ledsOff();
              button_flag = 0;
              buttIntStatus = 0;
              nosleep = 0;
            }
            if ((millis() - previousMillis > 12000) && (millis() - previousMillis <= 15000) && button_flag == 1)
            {
              new_device();
            }
            if (((millis() - previousMillis > 3000) && (millis() - previousMillis <= 4000)) || ((millis() - previousMillis > 7000) && (millis() - previousMillis <= 8000)) || ((millis() - previousMillis > 11000) && (millis() - previousMillis <= 12000)) || ((millis() - previousMillis > 15000)) && button_flag == 1)
            {
              ledsOff();
              button_flag = 0;
              buttIntStatus = 0;
              nosleep = 0;
            }
          }
        } else {
          readData();
          if (change == 1) {
            sendData();
            change = 0;
          }
          nosleep = false;
        }
      } else {
        if (millis() - configMillis > 20000) {
          digitalWrite(BLUE_LED, LOW);
          wait(50);
          digitalWrite(BLUE_LED, HIGH);
          configMode = 0;
          button_flag = 0;
          buttIntStatus = 0;
          nosleep = 0;
        }
      }
    } else {
      if (buttIntStatus == PIN_BUTTON) {
        if (digitalRead(PIN_BUTTON) == 0 && button_flag == 0) {
          button_flag = 1;
          previousMillis = millis();
          ledsOff();
        }

        if (digitalRead(PIN_BUTTON) == 0 && button_flag == 1) {
          if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 3000)) {
            if (millis() - lightMillisR > 30) {
              lightMillisR = millis();
              onoff = !onoff;
              digitalWrite(BLUE_LED, onoff);
            }
          }
          if ((millis() - previousMillis > 3000) && (millis() - previousMillis <= 4000)) {
            ledsOff();
          }
           if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 7000)) {
            if (millis() - lightMillisR > 45) {
              lightMillisR = millis();
              onoff = !onoff;
              digitalWrite(BLUE_LED, onoff);
            }
          }
           if (millis() - previousMillis > 7000) {
            ledsOff();
          }
        }

        if (digitalRead(PIN_BUTTON) == 1 && button_flag == 1) {
          if (millis() - previousMillis <= 3000 && button_flag == 1)
          {
            ledsOff();
            button_flag = 0;
            buttIntStatus = 0;
            check_parent();
            nosleep = 0;
          }
          if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 7000) && button_flag == 1)
          {
           new_device();
          }
          if (((millis() - previousMillis > 3000) && (millis() - previousMillis <= 4000)) || ((millis() - previousMillis > 7000)) && button_flag == 1)
            {
              ledsOff();
              button_flag = 0;
              buttIntStatus = 0;
              nosleep = 0;
            }
        }
      } else {
        check_parent();
      }
    }
  }

  if (_transportSM.failureCounter > 0)
  {
    _transportConfig.parentNodeId = loadState(201);
    _transportConfig.nodeId = myid;
    _transportConfig.distanceGW = loadState(202);
    mypar = _transportConfig.parentNodeId;
    nosleep = false;
    err_delivery_beat = 6;
    happy_node_mode();
    gateway_fail();
  }

  if (nosleep == false) {
    sleep(SLEEP_TIME);
    nosleep = 1;
  }
}


void ledsOff() {
  digitalWrite(BLUE_LED, HIGH);
}



//########################################## BATTARY ###################################################

void readBatt() {
  if (BATT_TIME != 0) {
    batteryVoltage = hwCPUVoltage();
    battery = battery_level_in_percent(batteryVoltage);
    batteryVoltageF = (float)batteryVoltage / 1000.00;
    CORE_DEBUG(PSTR("battery voltage: %d\n"), batteryVoltage);
    CORE_DEBUG(PSTR("battery percentage: %d\n"), battery);
    if (battery != old_battery) {
      bch = true;
      old_battery = battery;
    }
  }
}


void batLevSend() {
  if (BATT_TIME != 0) {
    if (battery > 100) {
      battery = 100;
    }
    check = sendBatteryLevel(battery, 1);
    wait(500);
    if (!check) {
      _transportSM.failedUplinkTransmissions = 0;
      wait(50);
      check = sendBatteryLevel(battery, 1);
      wait(500);
    }
    if (check == true) {
      err_delivery_beat = 0;
      if (flag_nogateway_mode == true) {
        flag_nogateway_mode = false;
        CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
        err_delivery_beat = 0;
      }
      CORE_DEBUG(PSTR("MyS: SEND BATTERY LEVEL\n"));
      CORE_DEBUG(PSTR("MyS: BATTERY LEVEL %: %d\n"), battery);
    } else {
      _transportSM.failedUplinkTransmissions = 0;
      if (err_delivery_beat < 6) {
        err_delivery_beat++;
      }
      if (err_delivery_beat == 5) {
        if (flag_nogateway_mode == false) {
          gateway_fail();
          CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
        }
      }
    }
    check = send(bvMsg.set(batteryVoltageF, 2));
    if (!check) {
      _transportSM.failedUplinkTransmissions = 0;
      wait(100);
      check = send(bvMsg.set(batteryVoltageF, 2));
      _transportSM.failedUplinkTransmissions = 0;
      wait(100);
    } else {
      CORE_DEBUG(PSTR("MyS: SEND BATTERY VOLTAGE\n"));
    }
    if (bch == true) {
      lqSend();
    }
  }
}


void lqSend() {
  nRFRSSI = transportGetReceivingRSSI();
  nRFRSSI = map(nRFRSSI, -85, -40, 0, 100);
  if (nRFRSSI < 0) {
    nRFRSSI = 0;
  }
  if (nRFRSSI > 100) {
    nRFRSSI = 100;
  }
  if (nRFRSSI != old_nRFRSSI) {
    check = send(sqMsg.set(nRFRSSI));
    if (!check) {
      _transportSM.failedUplinkTransmissions = 0;
      wait(100);
      check = send(sqMsg.set(nRFRSSI));
      _transportSM.failedUplinkTransmissions = 0;
    } else {
      CORE_DEBUG(PSTR("MyS: SEND LINK QUALITY\n"));
      CORE_DEBUG(PSTR("MyS: LINK QUALITY %: %d\n"), nRFRSSI);
    }
    old_nRFRSSI = nRFRSSI;
  }
}


static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
  uint8_t battery_level;

  if (mvolts >= 3000)
  {
    battery_level = 100;
  }
  else if (mvolts > 2900)
  {
    battery_level = 100 - ((3000 - mvolts) * 58) / 100;
  }
  else if (mvolts > 2740)
  {
    battery_level = 42 - ((2900 - mvolts) * 24) / 160;
  }
  else if (mvolts > 2440)
  {
    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
  }
  else if (mvolts > 2100)
  {
    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
  }
  else
  {
    battery_level = 0;
  }

  return battery_level;
}


//####################################### SENSOR DATA ##################################################

void readData() {
  temperatureSend = sensor.readTemperature();
  temperature = round(temperatureSend);
  if (chek_h == true) {
    humiditySend = sensor.readHumidity();
    humidity = round(humiditySend);
    if (humidity < 0) {
      humiditySend = 0.0;
    }
    if (humidity > 100) {
      humiditySend = 100.0;
    }

    if (abs(humiditySend - old_humidity) >= humThreshold) {
      old_humidity = humiditySend;
      change = true;
      hch = true;
    }

    chek_h = false;
  } else {
    chek_h = true;
  }

  if (humidity < -40) {
    humiditySend = -40.0;
  }
  if (temperature > 125) {
    temperature = 125.0;
  }

  if (abs(temperatureSend - old_temperature) >= tempThreshold) {
    old_temperature = temperatureSend;
    change = true;
    tch = true;
  }

  BATT_COUNT++;
  CORE_DEBUG(PSTR("BATT_COUNT: %d\n"), BATT_COUNT);
  if (BATT_COUNT == BATT_TIME) {
    CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
    readBatt();
    BATT_COUNT = 0;
    change = true;
  }
}


void sendData() {
  if (flag_nogateway_mode == false) {
    if (tch == true) {
      check = send(msgTemp.set(temperatureSend, 2));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(50);
        check = send(msgTemp.set(temperatureSend, 2));
        if (check == false) {
          wait(150);
          _transportSM.failedUplinkTransmissions = 0;
          check = send(msgTemp.set(temperatureSend, 2));
          wait(150);
        }
      }
      tch = false;

      if (check == true) {
        err_delivery_beat = 0;
        if (flag_nogateway_mode == true) {
          flag_nogateway_mode = false;
          CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
          err_delivery_beat = 0;
        }
        CORE_DEBUG(PSTR("MyS: SEND BATTERY LEVEL\n"));
        CORE_DEBUG(PSTR("MyS: BATTERY LEVEL %: %d\n"), battery);
      } else {
        _transportSM.failedUplinkTransmissions = 0;
        if (err_delivery_beat < 6) {
          err_delivery_beat++;
        }
        if (err_delivery_beat == 5) {
          if (flag_nogateway_mode == false) {
            gateway_fail();
            CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
          }
        }
      }
    }

    if (hch == true) {
      check = send(msgHum.set(humiditySend, 2));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(50);
        check = send(msgHum.set(humidity, 0));
        if (check == false) {
          wait(150);
          _transportSM.failedUplinkTransmissions = 0;
          check = send(msgHum.set(humiditySend, 2));
          wait(150);
        }
      }
      hch = false;

      if (check == true) {
        err_delivery_beat = 0;
        if (flag_nogateway_mode == true) {
          flag_nogateway_mode = false;
          CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
          err_delivery_beat = 0;
        }
        CORE_DEBUG(PSTR("MyS: SEND BATTERY LEVEL\n"));
        CORE_DEBUG(PSTR("MyS: BATTERY LEVEL %: %d\n"), battery);
      } else {
        _transportSM.failedUplinkTransmissions = 0;
        if (err_delivery_beat < 6) {
          err_delivery_beat++;
        }
        if (err_delivery_beat == 5) {
          if (flag_nogateway_mode == false) {
            gateway_fail();
            CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
          }
        }
      }
    }

    if (bch == true) {
      batLevSend();
      bch = false;
    }
    digitalWrite(BLUE_LED, LOW);
    wait(50);
    digitalWrite(BLUE_LED, HIGH);
  } else {
    tch = false;
    hch = false;
    bch = false;
  }
}


void timeConf() {
  if (flag_nogateway_mode == false) {
    if (timeSend != 0) {
      SLEEP_TIME = timeSend * minuteT;
    } else {
      if (battSend != 0) {
        SLEEP_TIME = battSend * minuteT * 60;
      } else {
        SLEEP_TIME = minuteT * 60 * 24;
      }
    }
    if (battSend != 0) {
      if (timeSend != 0) {
        BATT_TIME = battSend * 60 / timeSend;
      } else {
        BATT_TIME = 1;
      }
    } else {
      BATT_TIME = 0;
    }
  } else {
    SLEEP_TIME = 30 * minuteT;
  }
}


//############################################## RECEIVE CONF ##################################################

void receive(const MyMessage & message)
{
  if (message.sensor == SET_TIME_SEND_ID) {
    if (message.type == V_VAR1) {
      timeSend = message.getByte();
      if (timeSend > 60) {
        timeSend = 60;
      }
      saveState(102, timeSend);
      wait(50);
      send(setTimeSendMsg.set(timeSend));
      wait(50);
      digitalWrite(BLUE_LED, LOW);
      wait(50);
      digitalWrite(BLUE_LED, HIGH);
      configMode = false;
      nosleep = false;
      change = true;
      timeConf();
    }
  }

  if (message.sensor == SET_BATT_SEND_ID) {
    if (message.type == V_VAR1) {
      battSend = message.getByte();
      if (battSend > 24) {
        battSend = 24;
      }
      saveState(103, battSend);
      wait(50);
      send(setBattSendMsg.set(battSend));
      wait(50);
      digitalWrite(BLUE_LED, LOW);
      wait(50);
      digitalWrite(BLUE_LED, HIGH);
      configMode = false;
      nosleep = false;
      change = true;
      timeConf();
    }
  }
}


//################################################ INTERRUPTS #################################################

void interrupt_Init() {
  //***
  //SET
  //NRF_GPIO_PIN_NOPULL
  //NRF_GPIO_PIN_PULLUP
  //NRF_GPIO_PIN_PULLDOWN
  //***
  nrf_gpio_cfg_input(PIN_BUTTON, NRF_GPIO_PIN_NOPULL);
  APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
  PIN_BUTTON_MASK = 1 << PIN_BUTTON;
  app_gpiote_user_register(&m_gpiote_user_id, PIN_BUTTON_MASK, PIN_BUTTON_MASK, gpiote_event_handler);
  app_gpiote_user_enable(m_gpiote_user_id);
  buttIntStatus = 0;
}


void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
  MY_HW_RTC->CC[0] = (MY_HW_RTC->COUNTER + 2);

  if (PIN_BUTTON_MASK & event_pins_high_to_low) {
    if (buttIntStatus == 0) {
      buttIntStatus = PIN_BUTTON;
    }
  }
}


//################################################ RESETS ########################################################

void new_device() {
  hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  saveState(200, 255);
  hwReboot();
}

void default_param() {
  old_temperature = 0;
  old_humidity = 0;
  old_battery = 255;
  old_nRFRSSI = -1;
}


//############################################## HAPPY MODE #####################################################

void happy_init() {
  //hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255); // ******************** checking the node config reset *************************

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 0) {
    hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  }
  if (loadState(200) == 0) {
    saveState(200, 255);
  }
  CORE_DEBUG(PSTR("EEPROM NODE ID: %d\n"), hwReadConfig(EEPROM_NODE_ID_ADDRESS));
  CORE_DEBUG(PSTR("USER MEMORY SECTOR NODE ID: %d\n"), loadState(200));

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 255) {
    mtwr = 0;
  } else {
    mtwr = 7000;
    no_present();
  }
  CORE_DEBUG(PSTR("MY_TRANSPORT_WAIT_MS: %d\n"), mtwr);
}


void config_Happy_node() {
  if (mtwr == 0) {
    myid = getNodeId();
    saveState(200, myid);
    mypar = _transportConfig.parentNodeId;
    old_mypar = mypar;
    saveState(201, mypar);
    saveState(202, _transportConfig.distanceGW);
  }
  if (mtwr != 0) {
    myid = getNodeId();
    if (myid != loadState(200)) {
      saveState(200, myid);
    }
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      if (mypar != loadState(201)) {
        saveState(201, mypar);
      }
      if (_transportConfig.distanceGW != loadState(202)) {
        saveState(202, _transportConfig.distanceGW);
      }
      present_only_parent();
    }
    if (isTransportReady() == false)
    {
      no_present();
      err_delivery_beat = 6;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(201);
      _transportConfig.distanceGW = loadState(202);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
  timeConf();
}


void check_parent() {
  _transportSM.findingParentNode = true;
  CORE_DEBUG(PSTR("MyS: SEND FIND PARENT REQUEST, WAIT RESPONSE\n"));
  _sendRoute(build(_msg, 255, NODE_SENSOR_ID, C_INTERNAL, 7).set(""));
  wait(700, C_INTERNAL, 8);
  if (_msg.sensor == 255) {
    if (mGetCommand(_msg) == C_INTERNAL) {
      if (_msg.type == 8) {
        Ack_FP = true;
        CORE_DEBUG(PSTR("MyS: PARENT RESPONSE FOUND\n"));
      }
    }
  }
  if (Ack_FP == true) {
    CORE_DEBUG(PSTR("MyS: FIND PARENT PROCESS\n"));
    Ack_FP = false;
    transportSwitchSM(stParent);
    flag_nogateway_mode = false;
    flag_find_parent_process = true;
    problem_mode_count = 0;
  } else {
    _transportSM.findingParentNode = false;
    CORE_DEBUG(PSTR("MyS: PARENT RESPONSE NOT FOUND\n"));
    _transportSM.failedUplinkTransmissions = 0;
    CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
    nosleep = false;
  }
}


void find_parent_process() {
  flag_update_transport_param = true;
  flag_find_parent_process = false;
  CORE_DEBUG(PSTR("MyS: STANDART TRANSPORT MODE IS RESTORED\n"));
  err_delivery_beat = 0;
#ifdef RF52840
  NRF_RADIO->TXPOWER = 8;
  wait(10);
#endif
  timeConf();
  default_param();
  nosleep = false;
}


void gateway_fail() {
  flag_nogateway_mode = true;
  flag_update_transport_param = false;
  timeConf();
}


void happy_node_mode() {
  _transportSM.findingParentNode = false;
  _transportSM.transportActive = true;
  _transportSM.uplinkOk = true;
  _transportSM.pingActive = false;
  _transportSM.failureCounter = 0u;
  _transportSM.uplinkOk = true;
  _transportSM.failureCounter = 0u;
  _transportSM.failedUplinkTransmissions = 0u;
  transportSwitchSM(stReady);
  CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
}


void present_only_parent() {
  if (old_mypar != mypar) {
    CORE_DEBUG(PSTR("MyS: SEND LITTLE PRESENT:) WITH PARENT ID\n"));
    if (_sendRoute(build(_msgTmp, 0, NODE_SENSOR_ID, C_INTERNAL, 6).set(mypar))) {
      flag_sendRoute_parent = false;
      old_mypar = mypar;
    } else {
      flag_sendRoute_parent = true;
    }
  }
}


void update_Happy_transport() {
  CORE_DEBUG(PSTR("MyS: UPDATE TRANSPORT CONFIGURATION\n"));
  mypar = _transportConfig.parentNodeId;
  if (mypar != loadState(201))
  {
    saveState(201, mypar);
  }
  if (_transportConfig.distanceGW != loadState(202))
  {
    saveState(202, _transportConfig.distanceGW);
  }
  present_only_parent();
  wait(50);
  nosleep = false;
  flag_update_transport_param = false;
}


void no_present() {
  _coreConfig.presentationSent = true;
  _coreConfig.nodeRegistered = true;
}
