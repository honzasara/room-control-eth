#include <PID_v1.h>

#include <NTPClient.h>

#include <ArduinoJson.h>

#include <SPI.h>
#include <PubSubClient.h>
#include <UIPEthernet.h>

#include <avr/wdt.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include <avr/pgmspace.h>
#include "font_alfa.h"

#include <avr/wdt.h>
#include <avr/boot.h>

#include <RTClib.h>
#include <saric_ds2482.h>
#include <ow.h>
#include <EEPROM.h>

#ifndef BV
#define BV(bit) (1<<(bit))
#endif
#ifndef cbi
#define cbi(reg,bit) reg &= ~(BV(bit))
#endif
#ifndef sbi
#define sbi(reg,bit) reg |= (BV(bit))
#endif


#define MAX_SVETEL  8
#define LIGHT_DEFAULT 255
#define LIGHT_FREE  255

#define TYPE_FUNCTION 0
#define TYPE_STATIC 1
#define TYPE_DYNAMIC 2
#define TYPE_FUNCTION_DYNAMIC 3

#define HW_ONEWIRE_MAXROMS 5
#define HW_ONEWIRE_MAXDEVICES 5
#define MAX_THERMOSTAT 3
#define MAX_TEMP 320
#define MIN_TEMP 160
#define AVAILABLE_PROGRAM 8
#define MAX_PROGRAM_INTERVAL 11

#define PROG_FREE 255

#define MAX_RTDS 5

#define TDS_MEMORY_MAP_TDS 0
#define TDS_MEMORY_MAP_RTDS 16

#define TERM_READY 1
#define TERM_STOP 0

#define TERM_STAV_STOP 0
#define TERM_STAV_BEZI 1
#define TERM_OK 2


#define TERM_MODE_OFF 0
#define TERM_MODE_MAX 1
#define TERM_MODE_PROG 2
#define TERM_MODE_MAN_HEAT 3
#define TERM_MODE_CLIMATE 4
#define TERM_MODE_MAN_COOL 5
#define TERM_MODE_FAN 6
#define TERM_MODE_ERR 255

#define POWER_OUTPUT_ERR 255
#define POWER_OUTPUT_OFF 254
#define POWER_OUTPUT_HEAT_OFF 0
#define POWER_OUTPUT_COOL_OFF 1
#define POWER_OUTPUT_FAN_OFF 2

#define POWER_OUTPUT_HEAT_MAX 10
#define POWER_OUTPUT_COOL_MAX 11
#define POWER_OUTPUT_FAN_MAX 12


#define set_default_values 90
#define my_sense 91
#define my_default_ring 92

#define my_jas_disp 96
#define eeprom_term_mode 97
#define my_jas_key 98
#define my_rs_id 99


#define eeprom_thermostat_0 100
/*
   char name 0..9
   float mezni 10..14
   uint8_t program 15
   uint8_t associate_tds 16
   uint8_t ready 17
   uint8_t mode 18
   uint8_t associate_output 19
*/
#define eeprom_thermostat_1 120
#define eeprom_thermostat_2 140


#define wire_know_rom_0 160
#define wire_know_rom_4 240



#define NRF_CE  29
#define NRF_CS  28
#define NRF_INT 30

#define tr1  14
#define tr2  15
#define tr3  18
#define tr4  21
#define tr5  20
#define tr6  19
#define DP   22

#define rs485 10
#define PWM_DISP 3 /// TIMER0
#define PWM_KEY 12 /// TIMER1
#define SENSE 13   /// TIMER1

#define SER1  24
#define SER2  27
#define SCLK  25
#define PCLK  26

#define LIGHT A7

/*
  #define NRF_CE 29
  #define NRF_CS 28
  #define NRF_IRQ 30 ///INT6
*/


#define ETH_RST 2
#define ETH_INT 22

#define LED 0

#define TL_OFF    0b10000000
#define TL_MAX    0b01000000
#define TL_PROG   0b00100000
#define TL_CLIMA  0b00001000
#define TL_OK     0b00000100
#define TL_DOWN   0b00000010
#define TL_UP     0b00000001

#define TL_OFF_I   7
#define TL_MAX_I   6
#define TL_PROG_I  5
#define TL_CLIMA_I  3
#define TL_OK_I    2
#define TL_DOWN_I  1
#define TL_UP_I    0

#define LED_OFF  0b10000000
#define LED_MAX 0b01000000
#define LED_PROG  0b00100000
#define LED_CLIMA 0b00010000
#define LED_OK   0b00001000
#define LED_DOWN 0b00000100
#define LED_UP   0b00000010
#define LED_ERR  0b00000001

#define LED_OFF_I  7
#define LED_MAX_I 6
#define LED_PROG_I  5
#define LED_CLIMA_I 4
#define LED_OK_I   3
#define LED_DOWN_I 2
#define LED_UP_I   1
#define LED_ERR_I  0

#define MAX_TEMP_BUFFER 32
#define UART_RECV_MAX_SIZE 64

#define MAX_ITEMS_ROOT_MENU 12
#define MAX_HISTORY 5



StaticJsonDocument<512> doc;

struct struct_ds2482
{
  uint8_t i2c_addr;
  uint8_t HWwirenum;
};

struct struct_1w_rom
{
  uint8_t rom[8];
  uint8_t assigned_ds2482;
  uint8_t used;
};

typedef struct struct_DDS18s20
{
  uint8_t used;
  uint8_t rom[8];
  uint8_t assigned_ds2482;
  int offset;
  char name[8];
  uint8_t period;
};

typedef struct struct_status_DDS18s20
{
  uint8_t tempL;
  uint8_t tempH;
  uint8_t CR;
  uint8_t CP;
  uint8_t CRC;
  int temp;
  int average_temp[10];
  uint8_t online;
  uint8_t period_now;
  uint8_t wait;
};

typedef void (*function)(uint8_t args, uint8_t *ret, char *tmp);


typedef struct ItemMenu
{
  char *name;
  function on_show;
  uint8_t type;
  uint8_t first;
  uint8_t last;
};

typedef struct RootMenu
{
  char *name;
  ItemMenu *Item[MAX_ITEMS_ROOT_MENU];
  uint8_t items;
  uint8_t current_item;
  uint8_t args1;
  uint8_t args2;
  uint8_t args3;
};

typedef struct Struct_RootHistory
{
  RootMenu *history[MAX_HISTORY];
  uint8_t menu_max;
};


typedef struct struct_my_device
{
  uint8_t mac[6];
  uint8_t myIP[4];
  uint8_t myMASK[4];
  uint8_t myDNS[4];
  uint8_t myGW[4];
  char nazev[10];
  uint8_t mqtt_server[4];
  uint16_t mqtt_port = 1883;
  char mqtt_user[20];
  char mqtt_key[20];
  uint8_t ntp_server[4];
};

#define device_nazev 260 /// 10
#define device_mac 270  /// 6
#define device_ip 276   /// 4
#define device_mask 280 /// 4
#define device_dns 284 /// 4
#define device_gw 288 /// 4
#define device_mqtt_server 292 /// 4
#define device_mqtt_port 296 /// 2
#define device_mqtt_user 298 /// 20
#define device_mqtt_key 318 /// 20
#define device_ntp_server 338 ///4
#define nexxxxxt 342

#define pid_konst_p1 446
#define pid_konst_i1 450
#define pid_konst_d1 454
#define pid_konst_p2 458
#define pid_konst_i2 462
#define pid_konst_d2 466
#define pid_konst_p3 470
#define pid_konst_i3 474
#define pid_konst_d3 478
#define nexxxxt 482


/// v promene thermostat_program_active_
/* 0 dany program neaktivni
   1 rezim pro topeni
   2 rezim pro klimatizaci
   3 rezim pro ventilator
*/
#define static_thermostat_program_0 500
#define thermostat_program_0_active 500

#define thermostat_interval_program_0_week_day 501
#define thermostat_interval_program_0_start_1 502
#define thermostat_interval_program_0_stop_1 503
#define thermostat_interval_program_0_free_2 504
#define thermostat_interval_program_0_threshold_low 505
#define thermostat_interval_program_0_threshold_high 506

#define thermostat_interval_program_10_week_day 561
#define thermostat_interval_program_10_start_10 562
#define thermostat_interval_program_10_stop_10 563
#define thermostat_interval_program_10_free_10 564
#define thermostat_interval_program_10_threshold_low 565
#define thermostat_interval_program_10_threshold_high 566
#define thermostat_program_0_name 567

#define static_thermostat_program_1 590
#define static_thermostat_program_2 680
#define static_thermostat_program_3 770
#define static_thermostat_program_4 860
#define static_thermostat_program_5 950
#define static_thermostat_program_6 1040
#define static_thermostat_program_7 1130
///
#define remote_tds_name0  1220
#define remote_tds_name1  1230
#define remote_tds_name2  1240
#define remote_tds_name3  1250
#define remote_tds_name4  1260

#define nrf_channel 1300
#define nrf_write_name 1301
#define nrf_read1_name 1306
#define nrf_read2_name 1311
#define nrf_read3_name 1312
#define nrf_read4_name 1313
#define nrf_read5_name 1314
#define nrf_power 1315

#define light_output_0 1316
#define light_output_7 1323

#define tds_period1 1324
#define tds_period4 1328

///define next 1329

//////////////////////////////////////////////////////////////
RTC_DS1307 rtc;
DateTime now;

EthernetClient ethClient;
EthernetUDP udpClient;
PubSubClient mqtt_client(ethClient);
NTPClient timeClient(udpClient);

RF24 radio(NRF_CE, NRF_CS);



struct_my_device device;

uint8_t therm_stav[MAX_THERMOSTAT];
struct_1w_rom w_rom[HW_ONEWIRE_MAXROMS];
struct_ds2482 ds2482_address[1];
struct_status_DDS18s20 status_tds18s20[HW_ONEWIRE_MAXROMS];
uint8_t Global_HWwirenum = 0;
uint8_t tmp_rom[8];
uint8_t key = 0;
uint8_t key_now = 0;
uint8_t key_press = 0;
uint8_t key_release = 0;
uint8_t key_press_cnt[8];
uint8_t led, led_old, led_blink, led_blink_old;
volatile uint8_t rsid;
uint8_t ppwm = 0;
uint16_t uptime = 0;
uint8_t term_mode = 0;
uint16_t timer3_counter;
uint8_t timer2_counter;
uint8_t display_pos;
uint8_t jas_disp, jas_key;
uint16_t statisice = 0;
uint16_t destisice = 0;
uint16_t  tisice = 0;
uint16_t  stovky = 0;
uint16_t  desitky = 0;
uint16_t  jednotky = 0;
uint8_t tecka = 0;
long int milis = 0;
long int milis_05s = 0;
long int milis_1s = 0;
long int milis_10s = 0;
long int milis_key = 0;
uint32_t load2 = 0;
uint32_t load_2 = 0;
uint16_t aktual_light = 0;
uint8_t delay_show_menu = 0;
uint8_t start_at = 0;
uint8_t re_at = 0;
uint8_t r_id = 0;
char uart_recv[UART_RECV_MAX_SIZE];
char mqtt_log[128];
uint8_t mqtt_log_cnt = 0;
uint8_t default_ring = 0;

uint8_t rs_device[32];

byte nrf_addresses[][6] = {"1Node", "2Node"};

uint8_t link_status, mqtt_status;



double PID_Input[3];
double PID_Output[3];
double PID_Setpoint[3];

float PID_p[3];
float PID_i[3];
float PID_d[3];


PID pid0(&PID_Input[0], &PID_Output[0], &PID_Setpoint[0], 2, 5, 1, DIRECT);
PID pid1(&PID_Input[1], &PID_Output[1], &PID_Setpoint[1], 2, 5, 1, DIRECT);
PID pid2(&PID_Input[2], &PID_Output[2], &PID_Setpoint[2], 2, 5, 1, DIRECT);



uint8_t use_tds = 0;
uint8_t use_prog = 0;
uint8_t use_light = 0;
int remote_tds[5];
int remote_tds_last_update[5];
uint8_t light_value[MAX_SVETEL];
uint8_t light_state[MAX_SVETEL];

/*
  const char at_term_light[] PROGMEM =  "term/light";
  const char at_term_brightness[] PROGMEM = "term/brightness";
  const char at_term_ring_threshold[] PROGMEM = "term/ring/threshold";
  const char at_device_ident[] PROGMEM = "device/ident";
  const char at_room_control_OK[] PROGMEM = "room-control OK";
  const char at_device_name[] PROGMEM = "device/name";
  const char at_1w_count[] PROGMEM = "1w/count";
  const char at_term_ring_output[] PROGMEM = "term/ring/output";
  const char at_term_ring_prog[] PROGMEM = "term/ring/prog";
*/

const char global_time_set[] PROGMEM = "global/time/set";
const char global_time_ntp[] PROGMEM = "global/time/ntp";
const char thermctl_header_in[] PROGMEM  = "/thermctl-in/";
const char thermctl_header_out[] PROGMEM  = "/thermctl-out/";

const char lightctl_header_in[] PROGMEM  = "/lightctl-in/";
const char lightctl_header_out[] PROGMEM  = "/lightctl-out/";

const char title_root_termostat[] PROGMEM = "Trmst";
const char title_item_menu_temp[] PROGMEM = "Teplota";
const char title_item_menu_time[] PROGMEM = "Cas";

const char title_term_climate[] PROGMEM = "-MIN- ";
const char title_term_man[] PROGMEM = "MAN$*$";
const char title_term_prog[] PROGMEM = "-PROG-";
const char title_term_max[] PROGMEM = "-MAX- ";
const char title_term_off[] PROGMEM = "-OFF- ";

const char title_setup_menu[] PROGMEM = "Setup";
const char title_error[] PROGMEM = "ERROR";
const char title_menu_back[] PROGMEM = "-ZPET-";


const char title_item_setup_jas[] PROGMEM = "-nJAS-";
const char title_item_select_prog[] PROGMEM = "-nPROG";
const char title_item_select_ring[] PROGMEM = "-nRING";
const char title_item_select_reset[] PROGMEM = "-RESET";

const char title_item_setup_jas_dynamic[] PROGMEM = "JAS $$";
const char title_item_setup_jas_auto[] PROGMEM = "JAS AT";

const char title_item_set_select_prog[] PROGMEM = "PROG $";
const char title_item_set_select_ring[] PROGMEM = "RING $";
const char title_item_set_select_osvetleni[] PROGMEM = "INT--$";

char led_display_text[8];


Struct_RootHistory RootHistory;
RootMenu rm;
RootMenu term_max;
RootMenu term_climate;
RootMenu term_off;
RootMenu term_prog;
RootMenu term_man;
RootMenu setup_menu;
ItemMenu item_show_temp, item_show_time;

RootMenu setup_menu_jas;
RootMenu setup_select_prog;
RootMenu setup_select_ring;
RootMenu setup_select_osvetleni;

ItemMenu item_set_jas,  item_select_prog, item_select_ring, item_select_reset, item_select_osvetleni, item_zpet;
ItemMenu item_set_jas_dyn, item_set_jas_auto;
ItemMenu item_set_man_temp;
ItemMenu item_set_select_prog;
ItemMenu item_set_select_ring;
ItemMenu item_set_select_osvetleni;













uint8_t light_get_output(uint8_t idx)
{
  return EEPROM.read(light_output_0 + idx);
}

void light_set_output(uint8_t idx, uint8_t output)
{
  EEPROM.write(light_output_0 + idx, output);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void(* resetFunc) (void) = 0; //declare reset function @ address 0


/****************************************************************************************************************************************************************************************************/
//// prime zobrazeni na displaji
void show_direct(char *tmp)
{
  jednotky = pgm_read_word_near(&SixteenAlfaNumeric[tmp[5]]);
  desitky = pgm_read_word_near(&SixteenAlfaNumeric[tmp[4]]);
  stovky = pgm_read_word_near(&SixteenAlfaNumeric[tmp[3]]);
  tisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[2]]);
  destisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[1]]);
  statisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[0]]);
}
///////////////////////////////////////////////////////////////////////////////////////////////////
void menu_history_init(RootMenu *rm)
{
  RootHistory.history[0] = rm;
  RootHistory.menu_max = 0;
}
///////////////////////////////////////
void menu_init(RootMenu *rm)
{
  for (uint8_t itm = 0; itm < MAX_ITEMS_ROOT_MENU; itm++) rm->Item[itm] = nullptr;
  rm->items = 0;
  rm->current_item = 0;
  rm->args3 = 0;
}
////////////////////////////////////
void menu_set_root_menu(RootMenu *rm)
{
  if (RootHistory.menu_max < MAX_HISTORY)
  {
    RootHistory.menu_max++;
    RootHistory.history[RootHistory.menu_max] = rm;
  }
}
//////////////////////////////////
void menu_back_root_menu(void)
{
  if (RootHistory.menu_max > 0 ) RootHistory.menu_max--;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_root_setname(RootMenu *rm, char *name)
{
  rm->name = name;
}
///////////////////////////////////////
bool menu_root_additem(RootMenu *rm, ItemMenu *item)
{
  if (rm->items < MAX_ITEMS_ROOT_MENU)
  {
    for (uint8_t itm = 0; itm < MAX_ITEMS_ROOT_MENU; itm++)
      if (rm->Item[itm] == nullptr)
      {
        rm->Item[itm] = item;
        rm->items++;
        return true;
      }
  }
  else
    return false;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool menu_root_deleteitem(RootMenu *rm, ItemMenu *item)
{
  if (rm->items > 0)
  {
    for (uint8_t itm = 0; itm < MAX_ITEMS_ROOT_MENU; itm++)
      if (rm->Item[itm] == item)
      {
        rm->Item[itm] = nullptr;
        rm->items--;
        return true;
      }
  }
  else
    return false;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_item_set_properties(ItemMenu *item, char *name, uint8_t type, function on_show, uint8_t first = 0, uint8_t last = 0)
{
  item->name = name;
  item->on_show = on_show;
  item->type = type;
  item->first = first;
  item->last = last;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_item_update_limits(ItemMenu *item, uint8_t first = 0, uint8_t last = 0)
{
  item->first = first;
  item->last = last;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_display(void)
{
  static char tmp[8];
  static char tmp2[5];
  RootMenu _rm;
  uint8_t t, r, i, p;
  _rm = *RootHistory.history[RootHistory.menu_max];
  tecka = 0;

  if (_rm.items == 0)
  {
    strcpy_P(tmp, _rm.name);
    show_direct(tmp);
    goto endfce;
  }

  if (_rm.Item[_rm.current_item]->type == TYPE_FUNCTION)
  {
    _rm.Item[_rm.current_item]->on_show(0, 0, 0);
    goto endfce;
  }

  if (_rm.Item[_rm.current_item]->type == TYPE_FUNCTION_DYNAMIC)
  {
    _rm.Item[_rm.current_item]->on_show(_rm.args3, &_rm.args2, NULL);
    goto endfce;
  }

  if (_rm.Item[_rm.current_item]->type == TYPE_STATIC)
  {
    strcpy_P(tmp, _rm.Item[_rm.current_item]->name);
    show_direct(tmp);
    goto endfce;
  }

  if (_rm.Item[_rm.current_item]->type == TYPE_DYNAMIC)
  {
    if (_rm.Item[_rm.current_item]->on_show != nullptr)
    {
      _rm.Item[_rm.current_item]->on_show(_rm.args3, &_rm.args2, tmp2);
    }
    else
    {
      itoa(_rm.args3, tmp2, 10);
    }
    /// zastupny znak $
    /// zastupny znak * je znak vcetne tecky
    /// nahrazovany text
    strcpy_P(tmp, _rm.Item[_rm.current_item]->name);
    /// text pro nahrazeni
    r = strlen(tmp2);
    t = 0;
    p = 0;
    tecka = 0;
    for (i = 0; i < strlen(tmp); i++)
      if (tmp[i] == '*')
      {
        tecka = tecka + (1 << (i));
        p++;
      }
    for (i = 0; i < strlen(tmp); i++)
      if ((tmp[i] == '$' || tmp[i] == '*')  && t < r)
      {
        tmp[i] = tmp2[t];
        t++;
      }
    for (; i < 6; i++)
    {
      tmp[i] = 32; ///' '
    }
    show_direct(tmp);
  }
endfce:;
  *RootHistory.history[RootHistory.menu_max] = _rm;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void get_current_menu(char *text)
{
  strcpy_P(text, RootHistory.history[RootHistory.menu_max]->name);
}

void get_current_item(char *tmp)
{
  tmp[0] = 0;
  RootMenu _rm;
  _rm = *RootHistory.history[RootHistory.menu_max];
  if (_rm.items > 0)
    strcpy_P(tmp, _rm.Item[_rm.current_item]->name);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_next(void)
{
  RootMenu *_rm;
  _rm = RootHistory.history[RootHistory.menu_max];

  uint8_t type = _rm->Item[_rm->current_item]->type;
  if (type == TYPE_STATIC || type == TYPE_FUNCTION)
  {
    if (_rm->current_item < _rm->items - 1)
      _rm->current_item++;
    else
      _rm->current_item = 0;
    _rm->args3 = _rm->Item[_rm->current_item]->first ;
  }

  if (type == TYPE_DYNAMIC || type == TYPE_FUNCTION_DYNAMIC)
  {
    if (_rm->args3 < _rm->Item[_rm->current_item]->last)
    {
      _rm->args3++;
    }
    else
    {
      _rm->args3 = _rm->Item[_rm->current_item]->first ;
      if (_rm->current_item < _rm->items - 1)
        _rm->current_item++;
      else
        _rm->current_item = 0;
    }
  }
  RootHistory.history[RootHistory.menu_max] = _rm;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_prev(void)
{
  RootMenu *_rm;
  _rm = RootHistory.history[RootHistory.menu_max];

  uint8_t type = _rm->Item[_rm->current_item]->type;
  if (type == TYPE_STATIC || type == TYPE_FUNCTION)
  {
    if (_rm->current_item > 0)
      _rm->current_item--;
    else
      _rm->current_item = _rm->items - 1;
    _rm->args3 = _rm->Item[_rm->current_item]->last;
  }
  if (type == TYPE_DYNAMIC || type == TYPE_FUNCTION_DYNAMIC)
  {
    if (_rm->args3 > _rm->Item[_rm->current_item]->first)
    {
      _rm->args3--;
    }
    else
    {
      _rm->args3 = _rm->Item[_rm->current_item]->last;
      if (_rm->current_item > 0)
        _rm->current_item--;
      else
        _rm->current_item = _rm->items - 1;
    }
  }
  RootHistory.history[RootHistory.menu_max] = _rm;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*************************************************************************************************************************************************************//
uint8_t keyboard_get_sense(void)
{
  uint8_t itmp = EEPROM.read(my_sense);
  return itmp;
}

void keyboard_set_sense(uint8_t sense)
{
  EEPROM.write(my_sense, sense);
}

void keyboard_apply_sense(void)
{
  uint8_t itmp = EEPROM.read(my_sense);
  analogWrite(SENSE, itmp);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// funkce pro zobrazeni nastavene teploty.
void set_man_term_temp(uint8_t cnt, uint8_t *ret_data, char *text)
{
  int tepl;
  tepl = 160 + (cnt * 5);
  itoa(tepl, text, 10);
}

void set_select_ring(uint8_t cnt, uint8_t *ret_data, char *text)
{
  itoa(cnt, text, 10);
}
/////////////////////////////////////////////////////////////////////
/// vraci list svetelnych kontroleru ///
void set_select_osvetleni(uint8_t cnt, uint8_t *ret_data, char *text)
{
  char tmp1[8];
  uint8_t light = 0;
  uint8_t findit = 0;
  char c[3];
  uint8_t fix = 0;

  for (uint8_t idx = 0; idx < MAX_SVETEL; idx++)
    if (light_get_output(idx) != LIGHT_FREE)
    {
      if (fix == cnt)
      {
        findit = 1;
        light = idx;
        break;
      }
      fix++;
    }
  if (findit == 1)
  {
    *ret_data = light;
    strcpy(tmp1, "SVETL");
    itoa(light, c, 10);
    strcat(tmp1, c);
  }
  if (findit != 1)
  {
    *ret_data = LIGHT_FREE;
    strcpy(tmp1, "INT-E-");
  }
  show_direct(tmp1);
}
////////////////////////////////////////////////////////////////////
/// vraci list aktivnich programu ///
void set_select_active_prog(uint8_t cnt, uint8_t *ret_data, char *text)
{
  uint8_t fix = 0;
  char tmp1[8];
  char c[3];
  uint8_t findit = 0;
  uint8_t prog = 0;
  for (uint8_t idx = 0; idx < AVAILABLE_PROGRAM; idx++)
    if (thermostat_program_get_active(idx) != 0)
    {
      if (fix == cnt)
      {
        findit = 1;
        prog = idx;
        break;
      }
      fix++;
    }
  if (findit == 1)
  {
    *ret_data = prog;
    strcpy(tmp1, "PROG ");
    itoa(prog, c, 10);
    strcat(tmp1, c);
  }
  if (findit != 1)
  {
    *ret_data = PROG_FREE;
    strcpy(tmp1, "PROG-E");
  }
  show_direct(tmp1);
}
////////////////////
/// reverzni funkce, index menu programu vraci na skutecny index programu
uint8_t rev_get_show_prog(uint8_t cnt)
{
  uint8_t index = 0;
  for (uint8_t idx = 0; idx < AVAILABLE_PROGRAM; idx++)
    if (thermostat_program_get_active(idx) != 0)
    {
      if (idx == cnt)
      {
        break;
      }
      index++;
    }
  return index;
}
////////////////////////
/// reverzni funkce, index menu svetlo vraci na skutecny index svetla
uint8_t rev_get_show_light(uint8_t cnt)
{
  uint8_t index = 0;
  for (uint8_t idx = 0; idx < MAX_SVETEL; idx++)
    if (light_get_output(idx) != LIGHT_FREE)
    {
      if (idx == cnt)
      {
        break;
      }
      index++;
    }
  return index;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//// vraci pocet pouzitych programu
uint8_t count_use_prog(void)
{
  uint8_t cnt = 0;
  for (uint8_t idx = 0; idx < AVAILABLE_PROGRAM; idx++)
  {
    if (thermostat_program_get_active(idx) != 0) cnt++;
  }
  return cnt;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//// vraci pocet pouzitych vzdalenych mqtt tds cidel
uint8_t count_use_rtds(void)
{
  uint8_t cnt = 0;
  uint8_t active = 0;
  for (uint8_t idx = 0; idx < MAX_RTDS; idx++)
  {
    remote_tds_get_active(idx, &active);
    if (active == 1) cnt++;
  }
  return cnt;
}
////////////////////////
/// vraci pocet svitidel k nastaveni
uint8_t count_use_light(void)
{
  uint8_t cnt = 0;
  for (uint8_t idx = 0; idx < MAX_SVETEL; idx++)
    if (light_get_output(idx) !=  LIGHT_FREE) cnt++;
  return cnt;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************************************************************************************/
///////////////////////////////////////// TDS funkce //////////////////////////////////////////////////////
/// vraci pocet alokovanych k pouziti 1w cidel
uint8_t count_use_tds(void)
{
  uint8_t cnt = 0;
  for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXROMS; idx++)
    if ( EEPROM.read(wire_know_rom_0 + (idx * 20)) == 1) cnt++;
  return cnt;
}
/// ziska strikturu tds
uint8_t get_tds18s20(uint8_t idx, struct_DDS18s20 *tds)
{
  uint8_t ret = 0;
  if (idx < HW_ONEWIRE_MAXROMS)
  {
    tds->used = EEPROM.read(wire_know_rom_0 + (idx * 20));
    for (uint8_t m = 0; m < 8; m++)
    {
      tds->rom[m] = EEPROM.read(wire_know_rom_0 + (idx * 20) + 1 + m);
      tds->name[m] = EEPROM.read(wire_know_rom_0 + (idx * 20) + 12 + m);
    }
    tds->assigned_ds2482 = EEPROM.read(wire_know_rom_0 + (idx * 20) + 9);
    tds->offset = (EEPROM.read(wire_know_rom_0 + (idx * 20) + 10) << 8) + EEPROM.read(wire_know_rom_0 + (idx * 20) + 11);
    tds->period = EEPROM.read(tds_period1 + idx);
    ret = 1;
  }
  return ret;
}
/// nastavi strukturu tds
void set_tds18s20(uint8_t idx, struct_DDS18s20 *tds)
{
  EEPROM.write(wire_know_rom_0 + (idx * 20), tds->used);
  for (uint8_t m = 0; m < 8; m++)
  {
    EEPROM.write(wire_know_rom_0 + (idx * 20) + 1 + m, tds->rom[m]);
    EEPROM.write(wire_know_rom_0 + (idx * 20) + 12 + m, tds->name[m]);
  }
  EEPROM.write(wire_know_rom_0 + (idx * 20) + 9, tds->assigned_ds2482 );
  EEPROM.write(wire_know_rom_0 + (idx * 20) + 10, (tds->offset >> 8) & 0xff);
  EEPROM.write(wire_know_rom_0 + (idx * 20) + 11, (tds->offset) & 0xff);
  EEPROM.write(tds_period1 + idx, tds->period);
}

//// ziska nazev tds cidla
void tds_get_name(uint8_t idx, char *name)
{
  struct_DDS18s20 tds;
  get_tds18s20(idx, &tds);
  strcpy(name, tds.name);
}
//// nastavi nazev k tds cidlu
void tds_set_name(uint8_t idx, char *name)
{
  struct_DDS18s20 tds;
  get_tds18s20(idx, &tds);
  strcpy(tds.name, name);
  set_tds18s20(idx, &tds);
}
//// funkce nastavi offset cidlu tds
void tds_set_offset(uint8_t idx, int offset)
{
  struct_DDS18s20 tds;
  get_tds18s20(idx, &tds);
  tds.offset = offset;
  set_tds18s20(idx, &tds);
}
//// funkce ziska offset cidlu tds
int tds_get_offset(uint8_t idx)
{
  struct_DDS18s20 tds;
  get_tds18s20(idx, &tds);
  return tds.offset;
}
///////
//// funkce nastavi periodu mereni
void tds_set_period(uint8_t idx, uint8_t period)
{
  struct_DDS18s20 tds;
  get_tds18s20(idx, &tds);
  tds.period = period;
  set_tds18s20(idx, &tds);
}
//// funkce ziska periodu mereni
int tds_get_period(uint8_t idx)
{
  struct_DDS18s20 tds;
  get_tds18s20(idx, &tds);
  return tds.period;
}

//// funkce vymaze associovane 1wire -> tds
void tds_set_clear(uint8_t idx)
{
  struct_DDS18s20 tds;
  get_tds18s20(idx, &tds);
  if (tds.used == 1)
  {
    tds.used = 0;
    strcpy(tds.name, "FREE");
    tds.offset = 0;
    for (uint8_t i = 0; i < 8; i++)
      tds.rom[i] = 0;
    tds.assigned_ds2482 = 0;
    set_tds18s20(idx, &tds);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/********************************************************************************************************/
///   NASTAVENI REMOTE TDS CIDEL /////////////////////////////////////////////////////////////////////////
/// ziska nazev topicu
void remote_tds_get_name(uint8_t idx, uint8_t *active, char *name)
{
  char t;
  for (uint8_t i = 0; i < 9; i++)
  {
    t = EEPROM.read(remote_tds_name0 + (10 * idx) + i);
    name[i] = t;
    if (t == 0) break;
  }
  *active = EEPROM.read(remote_tds_name0 + (10 * idx) + 9);
}
/// nastavi topic
void remote_tds_set_name(uint8_t idx, uint8_t active , char *name)
{
  char tmp2[64];
  char t;
  for (uint8_t i = 0; i < 9; i++)
  {
    t = name[i];
    EEPROM.write(remote_tds_name0 + (10 * idx) + i, t);
    if (t == 0) break;
  }
  EEPROM.write(remote_tds_name0 + (10 * idx) + 9, active);
}
/// je aktivni
void remote_tds_get_active(uint8_t idx, uint8_t *active)
{
  *active = EEPROM.read(remote_tds_name0 + (10 * idx) + 9);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*************************************************************************************************************************************************//
/// funkce pro obsluhu seriove rs485 linky
void new_parse_at(char *input, char *out1, char *out2)
{
  uint8_t count = 0;
  uint8_t q = 0;
  out1[0] = 0;
  out2[0] = 0;

  while ( (count < strlen(input)) && (input[count] != ',') && (q < MAX_TEMP_BUFFER - 1))
  {
    out1[q] = input[count];
    out1[q + 1] = 0;
    q++;
    count++;
  }

  count++;
  q = 0;
  while ((count < strlen(input)) && (q < MAX_TEMP_BUFFER - 1) )
  {
    out2[q] = input[count];
    out2[q + 1] = 0;
    q++;
    count++;
  }
}
//////////////////////////////////////////////////////////////////////////
void send_at(uint8_t id, char *cmd, char *args)
{
  char tmp1[MAX_TEMP_BUFFER];
  char tmp2[8];
  tmp1[0] = 0;
  tmp2[0] = 0;
  //digitalWrite(rs485, HIGH);
  strcpy(tmp1, "at+");
  itoa(id, tmp2, 10);
  strcat(tmp1, tmp2);
  strcat(tmp1, ",");
  strcat(tmp1, cmd);
  if (strlen(args) > 0)
  {
    strcat(tmp1, ",");
    strcat(tmp1, args);
  }
  strcat(tmp1, ";");
  Serial.println(tmp1);
  Serial.flush();
  //digitalWrite(rs485, LOW);
}
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void read_at(char *input)
{
  uint8_t c;

  while (Serial.available() > 0)
  {
    c = Serial.read();

    //////////
    if ( re_at > 0 && re_at < (UART_RECV_MAX_SIZE - 1) )
    {
      if (c == ';')
      {
        start_at = 255;
        re_at = 0;
        break;
        goto endloop;
      }
      input[re_at - 1] = c;
      input[re_at] = 0;
      re_at++;
    };
    //////////
    if (start_at == 2)
      if (c == '+')
        start_at = 3;
      else
        start_at = 0;
    //////////
    if (start_at == 1)
      if ( c == 't')
        start_at = 2;
      else
        start_at = 0;
    //////////
    if (start_at == 0)
      if (c == 'a')
        start_at = 1;
    //////////
    if (start_at == 3)
    {
      re_at = 1;
      start_at = 4;
    }
    //////////
endloop:;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///tato funkce vraci na zaklade vstupu uint8_t id cidla tds. prvne jedu lokalni tds pak jedu remote tds
int8_t return_tds_rtds_id_usage(uint8_t cnt)
{
  struct_DDS18s20 tds;
  int8_t ret = -1;
  uint8_t fin = 0;
  uint8_t active = 0;
  uint8_t findit = 0;

  if (findit == 0)
  {
    for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXDEVICES; idx++)
    {
      get_tds18s20(idx, &tds);
      if (tds.used == 1)
      {
        if (fin == cnt)
        {
          ret = idx + TDS_MEMORY_MAP_TDS;
          findit = 1;
          break;
        }
        fin++;
      }
    }
  }
  if (findit == 0)
  {
    for (uint8_t idx = 0; idx < MAX_RTDS; idx++)
    {
      remote_tds_get_active(idx, &active);
      if (active == 1)
      {
        if (fin == cnt)
        {
          ret = idx + TDS_MEMORY_MAP_RTDS;
          findit = 1;
          break;
        }
        fin++;
      }
    }
  }
  return ret;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************** Nastaveni NRF24L01 ****************/
/// ulozi cislo kanalu do eeprom
void nrf_save_channel(uint8_t channel)
{
  EEPROM.write(nrf_channel, channel);
}

uint8_t nrf_load_channel(void)
{
  return EEPROM.read(nrf_channel);
}

void nrf_save_power(rf24_pa_dbm_e power)
{
  EEPROM.write(nrf_power, power);
}

rf24_pa_dbm_e nrf_load_power(void)
{
  return EEPROM.read(nrf_power);
}

void nrf_save_write_name(uint8_t *nazev)
{
  for (uint8_t idx = 0; idx < 5; idx++)
  {
    EEPROM.write(nrf_write_name + idx, nazev[idx]);
  }
}

void nrf_load_write_name(uint8_t *nazev)
{
  for (uint8_t idx = 0; idx < 5; idx++)
  {
    nazev[idx] = EEPROM.read(nrf_write_name + idx);
  }
}

void nrf_save_read_name(uint8_t channel, uint8_t *nazev)
{
  if (channel == 0)
    for (uint8_t idx = 0; idx < 5; idx++)
      EEPROM.write(nrf_read1_name + idx, nazev[idx]);
  if (channel > 0)
    EEPROM.write(nrf_read2_name + channel - 1, nazev[0]);
}

void nrf_load_read_name(uint8_t channel, uint8_t *nazev)
{
  for (uint8_t idx = 0; idx < 5; idx++)
    nazev[idx] = EEPROM.read(nrf_read1_name + idx);

  if (channel > 0)
    nazev[0] = EEPROM.read(nrf_read2_name + channel - 1);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_temp_default(uint8_t cnt, uint8_t *ret_data, char *text)
{
  char c[5], d[5];
  int tt;
  char tmp[8];
  int8_t idx = 0;
  tmp[0] = 0;
  struct_DDS18s20 tds;

  idx = return_tds_rtds_id_usage(cnt);
  if (idx < 0)
  {
    strcpy(tmp, "t ERR ");
  }

  if (idx >= TDS_MEMORY_MAP_TDS && idx < TDS_MEMORY_MAP_RTDS)
  {
    get_tds18s20(idx, &tds);
    if (status_tds18s20[idx].online == True)
    {
      tt = status_tds18s20[idx].temp;
      if (tt < 0) tt = tt * -1;
      itoa(tt / 100, c, 10);
      if ((tt / 1000) >= 10)
        tecka = 0b00001000;
      else
        tecka = 0b00000100;
      if (tt < 10)
      {
        strcpy(d, "0");
        strcat(d, c);
        strcpy(c, d);
      }
      strcpy(tmp, "t");
      tmp[1] = 10 + idx;
      if (status_tds18s20[idx].temp < 0)
        tmp[1] = tmp[1] * -1;
      tmp[2] = 0;
      strcat(tmp, c);
      strcat(tmp, "C");
    }
    else
    {
      strcpy(tmp, "t");
      tmp[1] = 10 + idx;
      tmp[2] = 0;
      strcat(tmp, "ERR");
    }
  }

  if (idx >= TDS_MEMORY_MAP_RTDS && idx < 127)
  {
    idx = idx - TDS_MEMORY_MAP_RTDS;
    if (remote_tds_last_update[idx] < 180)
    {
      tt = remote_tds[idx];
      if (tt < 0) tt = tt * -1;
      itoa(tt, c, 10);
      if ((tt / 10) >= 10)
        tecka = 0b00001000;
      else
        tecka = 0b00000100;
      if (tt < 10)
      {
        strcpy(d, "0");
        strcat(d, c);
        strcpy(c, d);
      }
      strcpy(tmp, "R");
      tmp[1] = 10 + idx;
      if (remote_tds[idx] < 0)
        tmp[1] = tmp[1] * -1;
      tmp[2] = 0;
      strcat(tmp, c);
      strcat(tmp, "C");
    }
    else
    {
      strcpy(tmp, "R");
      tmp[1] = 10 + idx;
      tmp[2] = 0;
      strcat(tmp, "ERR");
    }
  }

  if (tmp[5] == 0)
    jednotky = pgm_read_word_near(&SixteenAlfaNumeric[' ']);
  else
    jednotky = pgm_read_word_near(&SixteenAlfaNumeric[tmp[5]]);
  ///////
  if (tmp[4] == 0)
    desitky = pgm_read_word_near(&SixteenAlfaNumeric[' ']);
  else
    desitky = pgm_read_word_near(&SixteenAlfaNumeric[tmp[4]]);
  //////
  stovky = pgm_read_word_near(&SixteenAlfaNumeric[tmp[3]]);
  tisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[2]]);
  /////
  if (tmp[1] >= 0)
    destisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[1]]);
  if (tmp[1] < 0)
  {
    destisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[1] * -1]);
    destisice = destisice | pgm_read_word_near(&SixteenAlfaNumeric['-']);
  }
  ///////
  statisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[0]]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void show_time(void)
{
  char c[4];
  char tmp[8];
  c[0] = 0;
  tmp[0] = 0;
  tecka = 0b00000000;
  if ((now.hour() < 24) && (now.minute() < 60) && (now.second() < 60))
  {
    itoa(now.hour(), c, 10);
    if (now.hour() < 10) strcat(tmp, "0");
    strcat(tmp, c);
    if ((now.second() % 2) == 0)
      strcat(tmp, "'");
    else
      strcat(tmp, ",");
    itoa(now.minute(), c, 10);
    if (now.minute() < 10) strcat(tmp, "0");
    strcat(tmp, c);
    strcat(tmp, " ");
  }
  else
  {
    strcpy(tmp, "ERR---");
  }
  jednotky = pgm_read_word_near(&SixteenAlfaNumeric[tmp[5]]);
  desitky = pgm_read_word_near(&SixteenAlfaNumeric[tmp[4]]);
  stovky = pgm_read_word_near(&SixteenAlfaNumeric[tmp[3]]);
  tisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[2]]);
  destisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[1]]);
  statisice = pgm_read_word_near(&SixteenAlfaNumeric[tmp[0]]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////
//// nacte nazev zarizeni
void device_get_name(char *name)
{
  char t;
  for (uint8_t i = 0; i < 9; i++)
  {
    t = EEPROM.read(device_nazev  + i);
    name[i] = t;
    name[i + 1] = 0;
    if (t == 0)
    {
      break;
    }
  }
}
//// ulozi nazev zarizeni
void device_set_name(char *name)
{
  char t;
  for (uint8_t i = 0; i < 9; i++)
  {
    t = name[i];
    EEPROM.write(device_nazev +  i, t);
    if (t == 0)
    {
      EEPROM.write(device_nazev +  i + 1, 0);
      break;
    }
  }
}
///////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void load_setup_network(void)
{
  for (uint8_t m = 0; m < 6; m++) device.mac[m] = EEPROM.read(device_mac + m);
  for (uint8_t m = 0; m < 4; m++) device.myIP[m] = EEPROM.read(device_ip + m);
  for (uint8_t m = 0; m < 4; m++) device.myMASK[m] = EEPROM.read(device_mask + m);
  for (uint8_t m = 0; m < 4; m++) device.myGW[m] = EEPROM.read(device_gw + m);
  for (uint8_t m = 0; m < 4; m++) device.myDNS[m] = EEPROM.read(device_dns + m);
  for (uint8_t m = 0; m < 9; m++) device.nazev[m] = EEPROM.read(device_nazev + m);
  for (uint8_t m = 0; m < 4; m++) device.mqtt_server[m] = EEPROM.read(device_mqtt_server + m);
  device.mqtt_port = (EEPROM.read(device_mqtt_port) << 8) + EEPROM.read(device_mqtt_port + 1);
  for (uint8_t m = 0; m < 20; m++) device.mqtt_user[m] = EEPROM.read(device_mqtt_user + m);
  for (uint8_t m = 0; m < 20; m++) device.mqtt_key[m] = EEPROM.read(device_mqtt_key + m);
  for (uint8_t m = 0; m < 4; m++) device.ntp_server[m] = EEPROM.read(device_ntp_server + m);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ulozi nastaveni site
void save_setup_network(void)
{
  for (uint8_t m = 0; m < 6; m++) EEPROM.write(device_mac + m, device.mac[m]);
  for (uint8_t m = 0; m < 4; m++) EEPROM.write(device_ip + m, device.myIP[m]);
  for (uint8_t m = 0; m < 4; m++) EEPROM.write(device_mask + m, device.myMASK[m]);
  for (uint8_t m = 0; m < 4; m++) EEPROM.write(device_gw + m, device.myGW[m]);
  for (uint8_t m = 0; m < 4; m++) EEPROM.write(device_dns + m, device.myDNS[m]);
  for (uint8_t m = 0; m < 9; m++) EEPROM.write(device_nazev + m, device.nazev[m]);
  for (uint8_t m = 0; m < 4; m++) EEPROM.write(device_mqtt_server + m, device.mqtt_server[m]);
  EEPROM.write(device_mqtt_port, device.mqtt_port >> 8);
  EEPROM.write(device_mqtt_port + 1, device.mqtt_port & 0xff);
  for (uint8_t m = 0; m < 20; m++) EEPROM.write(device_mqtt_user + m, device.mqtt_user[m]);
  for (uint8_t m = 0; m < 20; m++) EEPROM.write(device_mqtt_key + m, device.mqtt_key[m]);
  for (uint8_t m = 0; m < 4; m++) EEPROM.write(device_ntp_server + m, device.ntp_server[m]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ziska/nastavi nazev termostatu
void thermostat_ring_get_name(uint8_t idx, char *name)
{
  char t;
  for (uint8_t i = 0; i < 9; i++)
  {
    t = EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + i);
    name[i] = t;
    if (t == 0) break;
  }
}
void thermostat_ring_set_name(uint8_t idx, char *name)
{
  char t;
  for (uint8_t i = 0; i < 9; i++)
  {
    t = name[i];
    EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + i, t);
    if (t == 0) break;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// funkce ktera nastavuje/zjistuje jestli dany operacni termostat je pripraven
uint8_t thermostat_ring_get_output(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 19);
}
void thermostat_ring_set_output(uint8_t idx, uint8_t output)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 19, output);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
/// funkce ktera nastavuje/zjistuje jestli dany operacni termostat je pripraven
uint8_t thermostat_ring_get_active(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 17);
}
void thermostat_ring_set_active(uint8_t idx, uint8_t ready)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 17, ready);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// funkce ktera nastavuje/zjistuje jestli dany operacni termostat je pripraven
uint8_t thermostat_ring_get_status(uint8_t idx)
{
  return therm_stav[idx];
}
void thermostat_ring_set_status(uint8_t idx, uint8_t stav)
{
  therm_stav[idx] = stav;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
//// nastaveni programu k ringu termostatu
uint8_t thermostat_ring_get_program_id(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 15);
}
void thermostat_ring_set_program_id(uint8_t idx, uint8_t id)
{
  return EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 15, id);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
///mezni rozhodovaci teplota
int thermostat_ring_get_mezni(uint8_t idx)
{
  return (EEPROM.read(eeprom_thermostat_0 + (20 * idx) + 10) << 8) + EEPROM.read(eeprom_thermostat_0 + (20 * idx) + 11);
}
void thermostat_ring_set_mezni(uint8_t idx, int temp)
{
  EEPROM.write(eeprom_thermostat_0 + (20 * idx) + 10, temp >> 8);
  EEPROM.write(eeprom_thermostat_0 + (20 * idx) + 11, temp & 0xff);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
/// ziska/nastavi cislo tds k ringu termostatu
uint8_t thermostat_ring_get_asociate_tds(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 16);
}
void thermostat_ring_set_asociate_tds(uint8_t idx, uint8_t id)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 16, id);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//// nastaveni/ziska modu k ringu termostatu
uint8_t thermostat_ring_get_mode(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 18);
}
void thermostat_ring_set_mode(uint8_t idx, uint8_t id)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 18, id);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//****************************************************************************************************//
void send_mqtt_message_prefix_id_topic_payload(char *prefix, uint8_t id, char *topic, char *payload)
{
  char str_topic[64];
  char hostname[10];
  char tmp1[12];
  if (mqtt_client.connected())
  {
    device_get_name(hostname);
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, hostname);
    strcat(str_topic, "/");
    strcat(str_topic, prefix);
    strcat(str_topic, "/");
    itoa(id, tmp1, 10);
    strcat(str_topic, tmp1);
    strcat(str_topic, "/");
    strcat(str_topic, topic);
    mqtt_client.publish(str_topic, payload);
  }
}
///
void send_mqtt_message_prefix_id_idx_topic_payload(char *prefix, uint8_t id, uint8_t idx, char *topic, char *payload)
{
  char str_topic[64];
  char hostname[10];
  char tmp1[12];
  if (mqtt_client.connected())
  {
    device_get_name(hostname);
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, hostname);
    strcat(str_topic, "/");
    strcat(str_topic, prefix);
    strcat(str_topic, "/");
    itoa(id, tmp1, 10);
    strcat(str_topic, tmp1);
    strcat(str_topic, "/");
    itoa(idx, tmp1, 10);
    strcat(str_topic, tmp1);
    strcat(str_topic, "/");
    strcat(str_topic, topic);
    mqtt_client.publish(str_topic, payload);
  }
}
///
void send_mqtt_general_payload(char *topic, const char *payload)
{
  const char str_topic[64];
  char hostname[10];
  if (mqtt_client.connected())
  {
    device_get_name(hostname);
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, hostname);
    strcat(str_topic, "/");
    strcat(str_topic, topic);
    mqtt_client.publish(str_topic, payload);
  }
}
///
///
///
///
//// /thermctl_out/XXXXX/1wire/count
//// /thermctl_out/XXXXX/1wire/IDcko/rom
void send_mqtt_onewire(void)
{
  char str_topic[64];
  char hostname[10];
  char payload[64];
  char tmp1[4];
  if (mqtt_client.connected())
  {
    device_get_name(hostname);
    itoa(Global_HWwirenum, payload, 10);
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, hostname);
    strcat(str_topic, "/1wire/count");
    mqtt_client.publish(str_topic, payload);
    for (uint8_t i = 0; i < Global_HWwirenum; i++)
    {
      strcpy_P(str_topic, thermctl_header_out);
      strcat(str_topic, hostname);
      strcat(str_topic, "/1wire/");
      itoa(i, tmp1, 10);
      strcat(str_topic, tmp1);
      strcat(str_topic, "/rom");

      payload[0] = 0;
      createString(payload, ':', w_rom[i].rom, 8, 16);
      mqtt_client.publish(str_topic, payload);
    }
  }
}
///
///
///
///
//// /thermctl-out/XXXXX/tds/ID/temp
//// /thermctl-out/XXXXX/tds/ID/name
//// /thermctl-out/XXXXX/tds/ID/offset
//// /thermctl-out/XXXXX/tds/ID/online
//// /thermctl-out/XXXXX/tds/ID/rom
//// /thermctl-out/XXXXX/tds/ID/period
void send_mqtt_tds(void)
{
  struct_DDS18s20 tds;
  char payload[64];
  char tmp1[4];
  int tt;
  for (uint8_t id = 0; id < HW_ONEWIRE_MAXROMS; id++)
    if (get_tds18s20(id, &tds) == 1)
      if (tds.used == 1) if (status_tds18s20[id].online == True)
        {
          tt = status_tds18s20[id].temp / 10;
          itoa(tt, payload, 10);
          send_mqtt_message_prefix_id_topic_payload("tds", id, "temp", payload);
          strcpy(payload, tds.name);
          send_mqtt_message_prefix_id_topic_payload("tds", id, "name", payload);
          tt = tds.offset;
          itoa(tt, payload, 10);
          send_mqtt_message_prefix_id_topic_payload("tds", id, "offset", payload);
          tt = status_tds18s20[id].online;
          itoa(tt, payload, 10);
          send_mqtt_message_prefix_id_topic_payload("tds", id, "online", payload);
          payload[0] = 0;
          createString(payload, ':', tds.rom, 8, 16);
          send_mqtt_message_prefix_id_topic_payload("tds", id, "rom", payload);
          tt = tds.period;
          itoa(tt, payload, 10);
          send_mqtt_message_prefix_id_topic_payload("tds", id, "period", payload);
        }
}
///
///
///
//// /thermctl-out/XXXXX/ring/ID/name
//// /thermctl-out/XXXXX/ring/ID/ready
//// /thermctl-out/XXXXX/ring/ID/program
//// /thermctl-out/XXXXX/ring/ID/threshold
//// /thermctl-out/XXXXX/ring/ID/mode
//// /thermctl-out/XXXXX/ring/ID/status
//// /thermctl-out/XXXXX/ring/ID/tds
//// /thermctl-out/XXXXX/ring/ID/output
void send_mqtt_ring(void)
{
  //char str_topic[64];
  //char hostname[10];
  char payload[64];
  uint8_t tdsid;
  //char tmp1[12];
  //device_get_name(hostname);
  for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
    //// odeslu pouze pokud je ring pripraveny
    if (thermostat_ring_get_active(idx) != 0)
    {
      thermostat_ring_get_name(idx, payload);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "name", payload);
      itoa(thermostat_ring_get_active(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "active", payload);
      itoa(thermostat_ring_get_program_id(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "program", payload);
      itoa(thermostat_ring_get_mezni(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "threshold", payload);
      itoa(thermostat_ring_get_mode(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "mode", payload);
      convert_mode_text(thermostat_ring_get_mode(idx), payload);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "text_mode", payload);
      itoa(thermostat_ring_get_status(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "status", payload);
      tdsid = thermostat_ring_get_asociate_tds(idx);
      if (tdsid >= TDS_MEMORY_MAP_TDS && tdsid < TDS_MEMORY_MAP_RTDS)
      {
        itoa(tdsid, payload, 10);
        send_mqtt_message_prefix_id_topic_payload("ring", idx, "tds", payload);
      }
      if (tdsid >= TDS_MEMORY_MAP_RTDS && tdsid < 127)
      {
        itoa(tdsid - TDS_MEMORY_MAP_RTDS, payload, 10);
        send_mqtt_message_prefix_id_topic_payload("ring", idx, "rtds", payload);
      }
      itoa(thermostat_ring_get_output(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "output", payload);
    }
}
///
///
///
//// thermctl-out/XXXXX/prog/IDX/name  - 8znaku
//// thermctl-out/XXXXX/prog_interval/IDX/IDCko/week  - program plati v tech dnech
//// thermctl-out/XXXXX/prog_interval/IDX/IDcko/mode - pro jednotlive casove useky ruzny rezim
//// thermctl-out/XXXXX/prog_interval/IDX/IDcko/theshold - pro jednotlive casove useky ruzne teploty
//// thermctl-out/XXXXX/prog_interval/IDX/IDcko/active - pro jednotlivy usek povoleni zakazani
//// thermctl-out/XXXXX/prog_interval/IDX/IDcko/time - nastavi cas pro jednotlive intervaly
void send_mqtt_program(void)
{
  char payload[64];
  char tmp1[6];
  uint8_t act = 0;
  uint8_t start_hour, start_min, stop_hour, stop_min, active;
  for (uint8_t idx = 0; idx < AVAILABLE_PROGRAM; idx++)
  {
    act = thermostat_program_get_active(idx);
    if ( act != 0)
    {
      thermostat_program_get_name(idx, payload);
      send_mqtt_message_prefix_id_topic_payload("prog", idx, "name", payload);
      itoa(act, payload, 10);
      send_mqtt_message_prefix_id_topic_payload("prog", idx, "active", payload);
      for (uint8_t progid = 0 ; progid < MAX_PROGRAM_INTERVAL; progid++)
      {
        thermostat_program_get_time(idx, progid, &start_hour, &start_min, &stop_hour, &stop_min, &active);
        if (active == 1)
        {
          itoa(start_hour, tmp1, 10);
          strcpy(payload, tmp1);
          strcat(payload, ",");
          itoa(start_min, tmp1, 10);
          strcat(payload, tmp1);
          strcat(payload, ",");
          itoa(stop_hour, tmp1, 10);
          strcat(payload, tmp1);
          strcat(payload, ",");
          itoa(stop_min, tmp1, 10);
          strcat(payload, tmp1);
          send_mqtt_message_prefix_id_idx_topic_payload("prog_interval", idx, progid, "time", payload);
          itoa(active, tmp1, 10);
          strcpy(payload, tmp1);
          send_mqtt_message_prefix_id_idx_topic_payload("prog_interval", idx, progid, "active", payload);
          itoa(thermostat_program_get_threshold(idx, progid), tmp1, 10);
          strcpy(payload, tmp1);
          send_mqtt_message_prefix_id_idx_topic_payload("prog_interval", idx, progid, "threshold", payload);
          itoa(thermostat_program_get_week(idx, progid), payload, 10);
          send_mqtt_message_prefix_id_idx_topic_payload("prog_interval", idx, progid, "week", payload);
        }
      }
    }
  }
}
///
///
///
//// thermctl-out/XXXXX/pid/IDX/kp
//// thermctl-out/XXXXX/pid/IDX/ki
//// thermctl-out/XXXXX/pid/IDX/kd
void mqtt_send_pid_variable(void)
{
  char payload[32];
  for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
  {
    dtostrf(PID_p[idx], 7, 2, payload);
    send_mqtt_message_prefix_id_topic_payload("pid", idx, "kp", payload);
    dtostrf(PID_i[idx], 7, 2, payload);
    send_mqtt_message_prefix_id_topic_payload("pid", idx, "ki", payload);
    dtostrf(PID_d[idx], 7, 2, payload);
    send_mqtt_message_prefix_id_topic_payload("pid", idx, "kd", payload);
  }
}
///
///
///
///// thermctl-out/XXXXX/rtds/IDCko/name - nazev topicu
///// thermctl-out/XXXXX/rtds/IDCko/active - jeli aktivni
void send_mqtt_remote_tds_status(void)
{
  uint8_t active = 0;
  char payload[10];
  for (uint8_t idx = 0; idx < MAX_RTDS; idx++)
  {
    remote_tds_get_name(idx, &active, payload);
    /// odeslu pouze pokud je neco aktivni, jinak ne
    if (active == 1)
    {
      send_mqtt_message_prefix_id_topic_payload("rtds", idx, "name", payload);
      itoa(active, payload, 10);
      send_mqtt_message_prefix_id_topic_payload("rtds", idx, "active", payload);
      itoa(remote_tds[idx], payload, 10);
      send_mqtt_message_prefix_id_topic_payload("rtds", idx, "temp", payload);
      itoa(remote_tds_last_update[idx], payload, 10);
      send_mqtt_message_prefix_id_topic_payload("rtds", idx, "last_update", payload);
    }
  }
}
////
////
////
//// thermctl-out/XXXX/light/IDX/output
void send_light_controler(void)
{
  char payload[10];
  uint8_t output;
  for (uint8_t idx = 0; idx < MAX_SVETEL; idx++)
  {
    output = light_get_output(idx);
    if (output != LIGHT_FREE)
    {
      itoa(output, payload, 10);
      send_mqtt_message_prefix_id_topic_payload("light", idx, "output", payload);
    }
  }
}
////
/// odesle hodnotu pro svetelny kontroller
/// /lightctl-out/channel/IDX/value
void mqtt_send_light_value(uint8_t idx)
{
  char str_topic[64];
  char payload[8];
  char str1[8];
  uint8_t output = light_get_output(idx);
  strcpy_P(str_topic, lightctl_header_out);
  strcat(str_topic, "channel/");
  itoa(output, str1, 10);
  strcat(str_topic, str1);
  strcat(str_topic, "/value");
  itoa(light_value[idx], payload, 10);
  mqtt_client.publish(str_topic, payload);
}
////
////
////
////
//// thermctl-out/XXXXX/network/mac
//// thermctl-out/XXXXX/network/ip
//// thermctl-out/XXXXX/network/netmask
//// thermctl-out/XXXXX/network/gw
//// thermctl-out/XXXXX/network/dns
//// thermctl-out/XXXXX/network/ntp
//// thermctl-out/XXXXX/network/mqtt_host
//// thermctl-out/XXXXX/network/mqtt_port
//// thermctl-out/XXXXX/network/mqtt_user
//// thermctl-out/XXXXX/network/mqtt_key
//// thermctl-out/XXXXX/network/name
void send_network_config(void)
{
  char payload[20];
  payload[0] = 0;
  createString(payload, ':', device.mac, 6, 16);
  send_mqtt_general_payload("network/mac", payload);
  payload[0] = 0;
  createString(payload, '.', device.myIP, 4, 10);
  send_mqtt_general_payload("network/ip", payload);
  payload[0] = 0;
  createString(payload, '.', device.myMASK, 4, 10);
  send_mqtt_general_payload("network/netmask", payload);

  payload[0] = 0;
  createString(payload, '.', device.myGW, 4, 10);
  send_mqtt_general_payload("network/gw", payload);

  payload[0] = 0;
  createString(payload, '.', device.myDNS, 4, 10);
  send_mqtt_general_payload("network/dns", payload);

  payload[0] = 0;
  createString(payload, '.', device.ntp_server, 4, 10);
  send_mqtt_general_payload("network/ntp", payload);

  payload[0] = 0;
  createString(payload, '.', device.mqtt_server, 4, 10);
  send_mqtt_general_payload("network/mqtt_host", payload);

  itoa(device.mqtt_port, payload, 10);
  send_mqtt_general_payload("network/mqtt_port", payload);
  send_mqtt_general_payload("network/mqtt_user", device.mqtt_user);
  send_mqtt_general_payload("network/mqtt_key", device.mqtt_key);
  send_mqtt_general_payload("network/name", device.nazev);
}
////
////
////
////////////////////////////////////////////////////////////////
/// funkce prevadi ciselnou hodnotu na skutecne pojmenovani
uint8_t convert_text_mode(char *str2)
{
  uint8_t mode = 0;
  if (strcmp(str2, "off") == 0) mode = TERM_MODE_OFF;
  if (strcmp(str2, "heat") == 0) mode = TERM_MODE_MAX;
  if (strcmp(str2, "manual") == 0) mode = TERM_MODE_MAN_HEAT;
  if (strcmp(str2, "auto") == 0) mode = TERM_MODE_PROG;
  if (strcmp(str2, "cool") == 0) mode = TERM_MODE_CLIMATE;
  if (strcmp(str2, "fan_only") == 0) mode = TERM_MODE_FAN;
  return mode;
}

void convert_mode_text(uint8_t mode, char *str)
{
  if (mode == TERM_MODE_OFF)   strcpy(str, "off");
  if (mode == TERM_MODE_MAX)   strcpy(str, "heat");
  if (mode == TERM_MODE_MAN_HEAT)   strcpy(str, "manual");
  if (mode == TERM_MODE_PROG)   strcpy(str, "auto");
  if (mode == TERM_MODE_CLIMATE)   strcpy(str, "cool");
  if (mode == TERM_MODE_FAN)   strcpy(str, "fan_only");
}
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  char str1[64];
  char str2[128];
  char tmp1[16];
  boolean ret = 0;
  uint8_t cnt = 0;
  uint8_t id = 0;
  uint8_t id_interval = 0;
  struct_DDS18s20 tds;
  char *pch;
  uint8_t active;
  for (uint8_t j = 0; j < 128; j++) str2[j] = 0;
  ////
  ////
  //// /thermctl-in/global/time/set - nastaveni casu. payload json
  strcpy_P(str1, thermctl_header_in);
  strcat_P(str1, global_time_set);
  if (strcmp(str1, topic) == 0)
  {
    strncpy(str2, payload, length);
    deserializeJson(doc, str2);
    JsonObject root = doc.as<JsonObject>();
    if (root.containsKey("year") && root.containsKey("month") && root.containsKey("month") && root.containsKey("hour") && root.containsKey("minute") && root.containsKey("second"))
      rtc.adjust(DateTime(root["year"], root["month"], root["day"], root["hour"], root["minute"], root["second"]));
  }
  //// /thermctl-in/global/time/ntp - jednorazova aktualizace casu z ntp serveru
  strcpy_P(str1, thermctl_header_in);
  strcat_P(str1, global_time_ntp);
  if (strcmp(str1, topic) == 0)
  {
    rtc.adjust(DateTime(timeClient.getYear(), timeClient.getMonth() , timeClient.getDate(), timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds()));
  }

  //// /thermctl/global/ping - test spojeni
  ///
  //// nastaveni vizualizace
  //// /thermctl-in/XXXX/led/blink
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/led/blink");
  if (strcmp(str1, topic) == 0)
  {
    strncpy(str2, payload, length);
    led_blink = atoi(str2);
  }
  ///. nastavi intenzitu jasu dispalye
  //// /thermctl-in/XXXX/led/disp
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/led/disp");
  if (strcmp(str1, topic) == 0)
  {
    strncpy(str2, payload, length);
    cnt = atoi(str2);
    jas_disp = 255 - (15 * cnt);
    EEPROM.write(my_jas_disp, jas_disp);
    analogWrite(PWM_DISP, jas_disp);
  }
  //// nastavi intenzitu podsvetleni tlacitek
  //// /thermctl-in/XXXX/led/key
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/led/key");
  if (strcmp(str1, topic) == 0)
  {
    strncpy(str2, payload, length);
    cnt = atoi(str2);
    EEPROM.write(my_jas_key, cnt);
    analogWrite(PWM_KEY, cnt);
  }

  /// nastavi citlivost tlacitek
  //// /thermctl-in/XXXX/keyboard/sense
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/keyboard/sense");
  if (strcmp(str1, topic) == 0)
  {
    strncpy(str2, payload, length);
    keyboard_set_sense(atoi(str2));
    keyboard_apply_sense();
  }
  ///
  //// /thermctl-in/XXXX/tds/associate - asociace do tds si pridam mac 1wire - odpoved je pod jakem ID to mam ulozeno
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/tds/associate");
  if (strcmp(str1, topic) == 0)
  {
    strncpy(str2, payload, length);
    id = atoi(str2);
    if ( id < Global_HWwirenum)
    {
      for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXDEVICES; idx++)
      {
        get_tds18s20(idx, &tds);
        if (tds.used == 0 && w_rom[id].used == 1)
        {
          tds.used = 1;
          for (uint8_t i = 0; i < 8; i++)
            tds.rom[i] = w_rom[id].rom[i];
          tds.assigned_ds2482 = ds2482_address[w_rom[idx].assigned_ds2482].i2c_addr;
          set_tds18s20(idx, &tds);
          break;
        }
      }
    }
    else
    {
      log_error("tds/associate bad id");
    }
  }
  ///
  //// /thermctl-in/XXXX/tds/set/IDcko/name - nastavi cidlu nazev
  //// /thermctl-in/XXXX/tds/set/IDcko/offset
  //// /thermctl-in/XXXX/tds/set/IDcko/period
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/tds/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      if (cnt == 0) id = atoi(pch);
      if (id < HW_ONEWIRE_MAXROMS)
      {
        if ((cnt == 1) && (strcmp(pch, "name") == 0)) tds_set_name(id, str2);
        if ((cnt == 1) && (strcmp(pch, "offset") == 0)) tds_set_offset(id, atoi(str2));
        if ((cnt == 1) && (strcmp(pch, "period") == 0)) tds_set_period(id, atoi(str2));
      }
      else
      {
        log_error("tds/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }
  ////
  //// /thermctl-in/XXXX/tds/clear
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/tds/clear");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    id = atoi(str2);
    if (id < HW_ONEWIRE_MAXROMS)
      tds_set_clear(id);
    else
      log_error("tds/clear bad id");
  }
  ////
  ////
  //// thermctl-in/XXXXX/rtds/set/IDX/name - 8 znaku nastavi a udela prihlaseni
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rtds/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      if (cnt == 0) id = atoi(pch);
      if (id < MAX_RTDS)
      {
        if ((cnt == 1) && (strcmp(pch, "name") == 0))
        {
          remote_tds_get_active(id, &active);
          if (active == 0)
          {
            remote_tds_set_name(id, 1, str2);
            remote_tds_subscibe_topic(id);
          }
        }
      }
      else
      {
        log_error("rtds/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }
  ////
  //// /thermctl-in/XXXX/rtds/clear index vymaze a odhlasi
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rtds/clear");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    id = atoi(str2);
    if (id < MAX_RTDS)
    {
      remote_tds_unsubscibe_topic(id);
      remote_tds_set_name(id, 0, "");
    }
    else
    {
      log_error("rtds/clear bad id");
    }
  }
  ////
  //// rtds/NAME - hodnota, kde NAME je nazev cidla
  strcpy(str1, "/rtds/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    for (uint8_t idx = 0; idx < MAX_RTDS; idx++)
    {
      uint8_t active = 0;
      remote_tds_get_name(idx, &active, tmp1);
      if (active == 1 && strcmp(tmp1, str1) == 0)
      {
        remote_tds[idx] = atoi(str2);
        remote_tds_last_update[idx] = 0;
      }
    }
  }
  ///
  //// thermctl-in/XXXXX/prog/set/IDX/name  - 8znaku
  //// thermctl-in/XXXXX/prog/set/IDX/active  - 0-off, 1-heat, 2-cool,3.....
  //// thermctl-in/XXXXX/prog_interval/set/IDX/IDCko/week  - program plati v tech dnech
  //// thermctl-in/XXXXX/prog_interval/set/IDX/IDCko/output  - program pro tento vystup
  //// thermctl-in/XXXXX/prog_interval/set/IDX/IDcko/mode - pro jednotlive casove useky ruzny rezim
  //// thermctl-in/XXXXX/prog_interval/set/IDX/IDcko/theshold - pro jednotlive casove useky ruzne teploty
  //// thermctl-in/XXXXX/prog_interval/set/IDX/IDcko/active - pro jednotlivy usek povoleni zakazani
  //// thermctl-in/XXXXX/prog_interval/set/IDX/IDcko/time - nastavi cas pro jednotlive intervaly
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/prog/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      if (cnt == 0) id = atoi(pch);
      if (id < AVAILABLE_PROGRAM)
      {
        if ((cnt == 1) && (strcmp(pch, "name") == 0))  thermostat_program_set_name(id, str2);
        if ((cnt == 1) && (strcmp(pch, "active") == 0))  thermostat_program_set_active(id, atoi(str2));
      }
      else
      {
        log_error("prog/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }
  /////
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/prog_interval/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      if (cnt == 0) id = atoi(pch);
      if (cnt == 1) id_interval = atoi(pch);
      if (id < AVAILABLE_PROGRAM && id_interval < MAX_PROGRAM_INTERVAL)
      {
        if ((cnt == 2) && (strcmp(pch, "active") == 0))  thermostat_program_set_active(id, id_interval, atoi(str2));
        if ((cnt == 2) && (strcmp(pch, "threshold") == 0))  thermostat_program_set_threshold(id, id_interval , atoi(str2));
        if ((cnt == 2) && (strcmp(pch, "time") == 0)) thermostat_program_set_parse_interval(id, id_interval, str2);
      }
      else
      {
        log_error("prog_interval/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }
  ///
  //// thermctl-in/XXXXX/ring/default, nastavi vychozi ring na displaji
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/ring/default");
  if (strcmp(str1, topic) == 0)
  {
    strncpy(str2, payload, length);
    default_ring = atoi(str2);
    set_default_ring(default_ring);
  }
  ///
  //// thermctl-in/XXXXX/ring/set/IDcko/name
  //// thermctl-in/XXXXX/ring/set/IDcko/program
  //// thermctl-in/XXXXX/ring/set/IDcko/threshold
  //// thermctl-in/XXXXX/ring/set/IDcko/text_mode
  //// thermctl-in/XXXXX/ring/set/IDcko/mode
  //// thermctl-in/XXXXX/ring/set/IDcko/tds
  //// thermctl-in/XXXXX/ring/set/IDcko/rtds
  //// thermctl-in/XXXXX/ring/set/IDcko/ready
  //// thermctl-in/XXXXX/ring/set/IDcko/output
  //// thermctl-in/XXXXX/ring/set/IDcko/pid_kp
  //// thermctl-in/XXXXX/ring/set/IDcko/pid_ki
  //// thermctl-in/XXXXX/ring/set/IDcko/pid_kd
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/ring/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      if (cnt == 0) id = atoi(pch);
      if (id < MAX_THERMOSTAT)
      {
        if ((cnt == 1) && (strcmp(pch, "name") == 0)) thermostat_ring_set_name(id, str2);
        if ((cnt == 1) && (strcmp(pch, "program") == 0))
          if (atoi(str2) < AVAILABLE_PROGRAM)
            thermostat_ring_set_program_id(id, atoi(str2));
        if ((cnt == 1) && (strcmp(pch, "threshold-f") == 0)) thermostat_ring_set_mezni(id, atof(str2) * 10); /// thereshold hodnota presne ve floatu * 10
        if ((cnt == 1) && (strcmp(pch, "threshold") == 0)) thermostat_ring_set_mezni(id, atoi(str2));
        if ((cnt == 1) && (strcmp(pch, "text_mode") == 0)) thermostat_ring_set_mode(id, convert_text_mode(str2));
        if ((cnt == 1) && (strcmp(pch, "mode") == 0)) thermostat_ring_set_mode(id, atoi(str2));
        if ((cnt == 1) && (strcmp(pch, "tds") == 0)) thermostat_ring_set_asociate_tds(id, atoi(str2));
        if ((cnt == 1) && (strcmp(pch, "rtds") == 0)) thermostat_ring_set_asociate_tds(id, atoi(str2) + TDS_MEMORY_MAP_RTDS);
        if ((cnt == 1) && (strcmp(pch, "active") == 0)) thermostat_ring_set_active(id, atoi(str2));
        if ((cnt == 1) && (strcmp(pch, "output") == 0)) thermostat_ring_set_output(id, atoi(str2));
        if ((cnt == 1) && (strcmp(pch, "pid_kp") == 0)) pid_set_kp(id, atof(str2));
        if ((cnt == 1) && (strcmp(pch, "pid_ki") == 0)) pid_set_ki(id, atof(str2));
        if ((cnt == 1) && (strcmp(pch, "pid_kd") == 0)) pid_set_kd(id, atof(str2));
      }
      else
      {
        log_error("ring/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }

  /// nastaveni site
  //// thermctl-in/XXXXX/network/set/mac
  //// thermctl-in/XXXXX/network/set/ip
  //// thermctl-in/XXXXX/network/set/netmask
  //// thermctl-in/XXXXX/network/set/gw
  //// thermctl-in/XXXXX/network/set/dns
  //// thermctl-in/XXXXX/network/set/ntp
  //// thermctl-in/XXXXX/network/set/mqtt_host
  //// thermctl-in/XXXXX/network/set/mqtt_port
  //// thermctl-in/XXXXX/network/set/mqtt_user
  //// thermctl-in/XXXXX/network/set/mqtt_key
  //// thermctl-in/XXXXX/network/set/name
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/network/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      if (strcmp(pch, "mac") == 0)
      {
        parseBytes(str2, ':', device.mac, 6, 10);
        cnt = 1;
      }
      if (strcmp(pch, "ip") == 0)
      {
        parseBytes(str2, '.', device.myIP, 4, 10);
        cnt = 1;
      }
      if (strcmp(pch, "netmask") == 0)
      {
        parseBytes(str2, '.', device.myMASK, 4, 10);
        cnt = 1;
      }
      if (strcmp(pch, "gw") == 0)
      {
        parseBytes(str2, '.', device.myGW, 4, 10);
        cnt = 1;
      }
      if (strcmp(pch, "dns") == 0)
      {
        parseBytes(str2, '.', device.myDNS, 4, 10);
        cnt = 1;
      }
      if (strcmp(pch, "ntp") == 0)
      {
        parseBytes(str2, '.', device.ntp_server, 4, 10);
        cnt = 1;
      }
      if (strcmp(pch, "mqtt_host") == 0)
      {
        parseBytes(str2, '.', device.mqtt_server, 4, 10);
        cnt = 1;
      }
      if (strcmp(pch, "mqtt_port") == 0)
      {
        device.mqtt_port = atoi(str2);
        cnt = 1;
      }
      if (strcmp(pch, "mqtt_user") == 0)
      {
        strcpy(device.mqtt_user, str2);
        cnt = 1;
      }
      if (strcmp(pch, "mqtt_pass") == 0)
      {
        strcpy(device.mqtt_key, str2);
        cnt = 1;
      }
      if (strcmp(pch, "name") == 0)
      {
        strcpy(device.nazev, str2);
        mqtt_reconnect();
        cnt = 1;
      }
      pch = strtok (NULL, "/");
    }

    if (cnt == 1) save_setup_network();
  }

  //// priradi k menu pro rizeni intezity svetla vystup
  //// thermctl-in/XXXXX//light/set/IDX/output
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/light/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    strncpy(str2, payload, length);
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = 0;
    pch = strtok (str1, "/");
    while (pch != NULL)
    {
      if (cnt == 0) id = atoi(pch);
      if ((cnt == 1) && (strcmp(pch, "output") == 0)) light_set_output(id, atoi(str2));
      if ((cnt == 1) && (strcmp(pch, "value") == 0)) light_value[id] = atoi(str2);
      pch = strtok (NULL, "/");
      cnt++;
    }
  }


  //// thermctl-in/XXXXX/reload
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/reload");
  if (strcmp(str1, topic) == 0)
  {
    log_error("reload ..... ");
    resetFunc();
  }

  //// /thermctl-in/XXXXX/reset_default
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/reset_default");
  if (strcmp(str1, topic) == 0)
  {
    strncpy(str2, payload, length);
    EEPROM.write(set_default_values, atoi(str2));
    log_error("Next reboot ..... ");
    resetFunc();
  }
}
////////////////////////////////////////////////////////////////////////////////////////////
void send_mqtt_status(void)
{
  char str_topic[64];
  char hostname[10];
  char payload[64];
  if (mqtt_client.connected())
  {
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, device.nazev);
    strcat(str_topic, "/status/uptime");
    itoa(uptime, payload, 10);
    mqtt_client.publish(str_topic, payload);
    ///
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, device.nazev);
    strcat(str_topic, "/status/brigthness");
    itoa(aktual_light, payload, 10);
    mqtt_client.publish(str_topic, payload);
    ///
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, device.nazev);
    strcat(str_topic, "/status/light");
    itoa(jas_disp, payload, 10);
    mqtt_client.publish(str_topic, payload);
    ///
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, device.nazev);
    strcat(str_topic, "/status/load");
    itoa(load2, payload, 10);
    mqtt_client.publish(str_topic, payload);
    ///
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, device.nazev);
    strcat(str_topic, "/status/default_ring");
    itoa(default_ring, payload, 10);
    mqtt_client.publish(str_topic, payload);
  }
}
///////////////////////////
void mqtt_publis_output(uint8_t idx, uint8_t state)
{
  char str_topic[64];
  char payload[64];
  char str1[8];
  if (idx != 255)
  {
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, device.nazev);
    strcat(str_topic, "/power-output/");
    itoa(idx, str1, 10);
    strcat(str_topic, str1);
    strcat(str_topic, "/state");
    itoa(state, payload, 10);
    mqtt_client.publish(str_topic, payload);
  }
}
//////////////////////////////////////////////////////
///////////////////////////
void mqtt_publis_output_pwm(uint8_t idx, uint8_t mode, uint8_t pwm)
{
  char str_topic[64];
  char payload[64];
  char str1[8];
  if (idx != 255)
  {
    strcpy_P(str_topic, thermctl_header_out);
    strcat(str_topic, device.nazev);
    strcat(str_topic, "/power-output/");
    itoa(idx, str1, 10);
    strcat(str_topic, str1);
    if (mode == TERM_MODE_MAN_HEAT)
      strcat(str_topic, "/heat");
    if (mode == TERM_MODE_MAN_COOL)
      strcat(str_topic, "/cool");
    if (mode == TERM_MODE_FAN)
      strcat(str_topic, "/fan");
    if (mode == TERM_MODE_ERR)
      strcat(str_topic, "/err");
    strcat(str_topic, "/pwm");
    itoa(pwm, payload, 10);
    mqtt_client.publish(str_topic, payload);
  }
}


//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
boolean mqtt_reconnect(void)
{
  char nazev[10];
  char topic[26];
  boolean ret = true;
  ///  /thermctl/xxxxxxxx/#
  ///  /thermctl/global/#
  device_get_name(nazev);
  mqtt_client.disconnect();
  ret = mqtt_client.connect(nazev);
  if (ret == true)
  {
    strcpy_P(topic, thermctl_header_in);
    strcat(topic, nazev);
    strcat(topic, "/#");
    mqtt_client.subscribe(topic);
    strcpy_P(topic, thermctl_header_in);
    strcat(topic, "global/#");
    mqtt_client.subscribe(topic);
    //// /rtds/xxxxx
    for (uint8_t idx = 0; idx < 5; idx++)
      remote_tds_subscibe_topic(idx);
    //// svetelny controller
    strcpy_P(topic, lightctl_header_out);
    strcat(topic, "/#");
    mqtt_client.subscribe(topic);
  }
  return ret;
}
/*************************************************************************************************************/
/// funkce pro nastaveni odebirani topicu vzdalenych cidel
void remote_tds_subscibe_topic(uint8_t idx)
{
  char tmp1[64];
  char tmp2[64];
  uint8_t active = 0;
  remote_tds_get_name(idx, &active, tmp1);
  if (active == 1)
  {
    strcpy(tmp2, "/rtds/");
    strcat(tmp2, tmp1);
    mqtt_client.subscribe(tmp2);
  }
}
/// funkce pro zruseni odebirani topicu vzdalenych cidel
void remote_tds_unsubscibe_topic(uint8_t idx)
{
  char tmp1[64];
  char tmp2[64];
  uint8_t active = 0;
  remote_tds_get_name(idx, &active, tmp1);
  if (active == 1)
  {
    strcpy(tmp2, "/rtds/");
    strcat(tmp2, tmp1);
    mqtt_client.unsubscribe(tmp2);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*************************************************************************************************************//
/////vyhledani zarizeni na hw 1wire sbernici////////
uint8_t one_hw_search_device(uint8_t idx)
{
  uint8_t r;
  ds2482_address[idx].HWwirenum = 0;
  ds2482init(ds2482_address[idx].i2c_addr);
  ds2482reset(ds2482_address[idx].i2c_addr);
  ds2482owReset(ds2482_address[idx].i2c_addr);
  r = owMatchFirst(ds2482_address[idx].i2c_addr, tmp_rom);
  if (r == DS2482_ERR_NO_DEVICE) {
    /*chyba zadne zarizeni na sbernici*/
  }
  if (r) {
    /*jina chyba*/
  }
  ///
  if (r == DS2482_ERR_OK)
    while (1) {
      if (ds2482_address[idx].HWwirenum > HW_ONEWIRE_MAXDEVICES - 1) break;
      for (uint8_t a = 0; a < 8; a++)  w_rom[Global_HWwirenum].rom[a] = tmp_rom[a];
      w_rom[Global_HWwirenum].assigned_ds2482 = idx;
      w_rom[Global_HWwirenum].used = 1;
      r = owMatchNext(ds2482_address[idx].i2c_addr, tmp_rom);
      /// celkovy pocet detekovanych roms
      ds2482_address[idx].HWwirenum++;
      Global_HWwirenum++;
      if (r == DS2482_ERR_NO_DEVICE)
      { ///hledani dokonceno
        break;
      }
    }
  return r;
}
/// funkce mereni na sbernici
/*
  uint8_t mereni_hwwire(uint8_t maxidx = 2)
  {
  uint8_t status = 0;
  uint8_t t, e;
  struct_DDS18s20 tds;
  for (uint8_t idx = 0; idx < maxidx; idx++)
  {
    /// pro celou sbernici pustim zacatek mereni teploty
    if (ds2482_address[idx].hwwire_cekam == false)
    {
      status = owReset(ds2482_address[idx].i2c_addr);
      status = owSkipRom(ds2482_address[idx].i2c_addr);
      status = owWriteByte(ds2482_address[idx].i2c_addr, OW_CONVERT_T);
      ds2482_address[idx].hwwire_cekam = true;
    }
    t = 0;
    status = owReadByte(ds2482_address[idx].i2c_addr, &t);
    if (t != 0) ds2482_address[idx].hwwire_cekam = false;

    if (ds2482_address[idx].hwwire_cekam == false)
      for (uint8_t w = 0; w < HW_ONEWIRE_MAXROMS; w++)
      {
        get_tds18s20(w, &tds);
        if ((tds.used == 1) && (tds.assigned_ds2482 == ds2482_address[idx].i2c_addr))
        {
          status = 0;
          status = status + owReset(ds2482_address[idx].i2c_addr);
          status = status + owMatchRom(ds2482_address[idx].i2c_addr, tds.rom );
          status = status + owWriteByte(ds2482_address[idx].i2c_addr, OW_READ_SCRATCHPAD);
          status = status + owReadByte(ds2482_address[idx].i2c_addr, &e);     //0byte
          status_tds18s20[w].tempL = e;
          status = status + owReadByte(ds2482_address[idx].i2c_addr, &e);     //1byte
          status_tds18s20[w].tempH = e;
          status = status + owReadByte(ds2482_address[idx].i2c_addr, &e); //2byte
          status = status + owReadByte(ds2482_address[idx].i2c_addr, &e); //3byte
          status = status + owReadByte(ds2482_address[idx].i2c_addr, &e); //4byte
          status = status + owReadByte(ds2482_address[idx].i2c_addr, &e); //5byte
          status = status + owReadByte(ds2482_address[idx].i2c_addr, &e); //6byte
          status_tds18s20[w].CR = e; //count remain
          status = status + owReadByte(ds2482_address[idx].i2c_addr, &e); //7byte
          status_tds18s20[w].CP = e; // count per
          status = status + owReadByte(ds2482_address[idx].i2c_addr, &e); //8byte
          status_tds18s20[w].CRC = e; // crc soucet
          if (status == 0)
          {
            uint16_t temp = (uint16_t) status_tds18s20[w].tempH << 11 | (uint16_t) status_tds18s20[w].tempL << 3;
            status_tds18s20[w].temp = ((temp & 0xfff0) << 3) -  16 + (  (  (status_tds18s20[w].CP - status_tds18s20[t].CR) << 7 ) / status_tds18s20[w].CP ) + tds.offset;
            status_tds18s20[w].online = True;
            for (uint8_t av = 9; av > 0; av--) status_tds18s20[w].average_temp[av] = status_tds18s20[w].average_temp[av - 1];
            status_tds18s20[w].average_temp[0] = status_tds18s20[w].temp;
          }
          else
          {
            status_tds18s20[w].online = False;
          }
        }
      }
  }
  return status;
  }
*/

uint8_t mereni_hwwire(void)
{
  uint8_t status = 0;
  uint8_t t, e;
  struct_DDS18s20 tds;

  for (uint8_t w = 0; w < HW_ONEWIRE_MAXROMS; w++)
  {
    get_tds18s20(w, &tds);
    if (tds.used == 1)
    {
      if ((status_tds18s20[w].wait == false) && ((uptime - status_tds18s20[w].period_now) > tds.period))
      {
        owReset(tds.assigned_ds2482);
        owMatchRom(tds.assigned_ds2482, tds.rom );
        owWriteByte(tds.assigned_ds2482, OW_CONVERT_T);
        status_tds18s20[w].period_now = uptime;
        status_tds18s20[w].wait = true;
      }
      if (status_tds18s20[w].wait == true)
      {
        owReset(tds.assigned_ds2482);
        owMatchRom(tds.assigned_ds2482, tds.rom );
        owReadByte(tds.assigned_ds2482, &t);
        if (t != 0)
        {
          status_tds18s20[w].wait = false;
          status = owReset(tds.assigned_ds2482);
          status = status + owMatchRom(tds.assigned_ds2482, tds.rom );
          status = status + owWriteByte(tds.assigned_ds2482, OW_READ_SCRATCHPAD);
          status = status + owReadByte(tds.assigned_ds2482, &e);     //0byte
          status_tds18s20[w].tempL = e;
          status = status + owReadByte(tds.assigned_ds2482, &e);     //1byte
          status_tds18s20[w].tempH = e;
          status = status + owReadByte(tds.assigned_ds2482, &e); //2byte
          status = status + owReadByte(tds.assigned_ds2482, &e); //3byte
          status = status + owReadByte(tds.assigned_ds2482, &e); //4byte
          status = status + owReadByte(tds.assigned_ds2482, &e); //5byte
          status = status + owReadByte(tds.assigned_ds2482, &e); //6byte
          status_tds18s20[w].CR = e; //count remain
          status = status + owReadByte(tds.assigned_ds2482, &e); //7byte
          status_tds18s20[w].CP = e; // count per
          status = status + owReadByte(tds.assigned_ds2482, &e); //8byte
          status_tds18s20[w].CRC = e; // crc soucet
          if (status == 0)
          {
            uint16_t temp = (uint16_t) status_tds18s20[w].tempH << 11 | (uint16_t) status_tds18s20[w].tempL << 3;
            status_tds18s20[w].temp = ((temp & 0xfff0) << 3) -  16 + (  (  (status_tds18s20[w].CP - status_tds18s20[t].CR) << 7 ) / status_tds18s20[w].CP ) + tds.offset;
            status_tds18s20[w].online = True;
            for (uint8_t av = 9; av > 0; av--) status_tds18s20[w].average_temp[av] = status_tds18s20[w].average_temp[av - 1];
            status_tds18s20[w].average_temp[0] = status_tds18s20[w].temp;
          }
          else
          {
            status_tds18s20[w].online = False;
          }
        }

      }


    }
  }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*************************************************************************************************************************************************/
///   NASTAVENI PID REGULATORU ///
void pid_update_variable(void)
{
  pid0.SetTunings(PID_p[0], PID_i[0], PID_d[0]);
  pid1.SetTunings(PID_p[1], PID_i[1], PID_d[1]);
  pid2.SetTunings(PID_p[2], PID_i[2], PID_d[2]);
}

void pid_save_variable(void)
{
  EEPROMwriteFloat(pid_konst_p1, PID_p[0]);
  EEPROMwriteFloat(pid_konst_p2, PID_p[1]);
  EEPROMwriteFloat(pid_konst_p3, PID_p[2]);
  EEPROMwriteFloat(pid_konst_i1, PID_i[0]);
  EEPROMwriteFloat(pid_konst_i2, PID_i[1]);
  EEPROMwriteFloat(pid_konst_i3, PID_i[2]);
  EEPROMwriteFloat(pid_konst_d1, PID_d[0]);
  EEPROMwriteFloat(pid_konst_d2, PID_d[1]);
  EEPROMwriteFloat(pid_konst_d3, PID_d[2]);
}

void pid_load_variable(void)
{
  PID_p[0] = EEPROMreadFloat(pid_konst_p1);
  PID_p[1] = EEPROMreadFloat(pid_konst_p2);
  PID_p[2] = EEPROMreadFloat(pid_konst_p3);
  PID_i[0] = EEPROMreadFloat(pid_konst_i1);
  PID_i[1] = EEPROMreadFloat(pid_konst_i2);
  PID_i[2] = EEPROMreadFloat(pid_konst_i3);
  PID_d[0] = EEPROMreadFloat(pid_konst_d1);
  PID_d[1] = EEPROMreadFloat(pid_konst_d2);
  PID_d[2] = EEPROMreadFloat(pid_konst_d3);
}

void pid_set_kp(uint8_t id, float x)
{
  PID_p[id] = x;
  pid_update_variable();
  pid_save_variable();
}
void pid_set_ki(uint8_t id, float x)
{
  PID_i[id] = x;
  pid_update_variable();
  pid_save_variable();
}
void pid_set_kd(uint8_t id, float x)
{
  PID_d[id] = x;
  pid_update_variable();
  pid_save_variable();
}

/**********************************************************************************************************************************/
//// NASTAVENI PROGRAMU /////////////////////////////////
//// ziska pojmenovani programu
void thermostat_program_get_name(uint8_t ring_id, char *name)
{
  uint8_t t = 0;
  for (uint8_t i = 0; i < 9; i++)
  {
    t = EEPROM.read(thermostat_program_0_name + (ring_id * 90) + i);
    name[i] = t;
    if (t == 0) break;
  }
}
//// nastavi pojmenovani programu
void thermostat_program_set_name(uint8_t ring_id, char *name)
{
  for (uint8_t i = 0; i < 9; i++)
  {
    EEPROM.write(thermostat_program_0_name + (ring_id * 90) + i, name[i]);
    if (name[i] == 0) break;
  }
}
//// globalni povoleni programu a nastaveni rezimu
uint8_t thermostat_program_get_active(uint8_t ring_id)
{
  return EEPROM.read(thermostat_program_0_active + (ring_id * 90) );
}
void thermostat_program_set_active(uint8_t ring_id, uint8_t active)
{
  EEPROM.write(thermostat_program_0_active + (ring_id * 90), active);
}
/***************************************************************************************************************/
//// ziska jakych dnech program plati
uint8_t thermostat_program_get_week(uint8_t ring_id, uint8_t progid)
{
  return EEPROM.read(thermostat_interval_program_0_week_day + (ring_id * 90) + (progid * 6) );
}
//// nastavi jaky v jakych dnech program plati
void thermostat_program_set_week(uint8_t ring_id, uint8_t progid, uint8_t week)
{
  EEPROM.write(thermostat_interval_program_0_week_day + (ring_id * 90) + (progid * 6), week);
}
/***************************************************************************************************************/
//// ziska cas termostatu
void thermostat_program_get_time(uint8_t ring_id, uint8_t progid, uint8_t *start_hour, uint8_t *start_min, uint8_t *stop_hour, uint8_t *stop_min, uint8_t *active)
{
  uint8_t start = EEPROM.read(thermostat_interval_program_0_start_1 + (ring_id * 90) + (progid * 6));
  uint8_t stop = EEPROM.read(thermostat_interval_program_0_stop_1 + (ring_id * 90)  + (progid * 6));
  *start_hour = (start >> 3) & 0b00011111;
  *start_min = (start >> 1 & 0b00000011) * 15;
  *stop_hour = (stop >> 3) & 0b00011111;
  *stop_min = (stop & 0b00000011) * 15;
  *active = start & 0b00000001;
}
//// nastavi cas termostatu
void thermostat_program_set_time(uint8_t ring_id, uint8_t progid, uint8_t start_hour, uint8_t start_min, uint8_t stop_hour, uint8_t stop_min, uint8_t active)
{
  uint8_t start, stop;
  start = ((start_hour << 3) & 0b11111000) + (((start_min / 15) << 1) & 0b00000110 ) + (active & 0b00000001);
  stop = ((stop_hour << 3) & 0b11111000) + ((stop_min / 15) & 0b00000011);
  EEPROM.write(thermostat_interval_program_0_start_1 + (ring_id * 90) + (progid * 6), start);
  EEPROM.write(thermostat_interval_program_0_stop_1 + (ring_id * 90) + (progid * 6), stop);
}
/***************************************************************************************************************/

/***************************************************************************************************************/
//// ziska rozhodovaci uroven termostatu
uint16_t thermostat_program_get_threshold(uint8_t program, uint8_t progid)
{
  uint16_t ret = 0;
  ret = (EEPROM.read(thermostat_interval_program_0_threshold_high + (program * 90) + (progid * 6)) << 8)  + EEPROM.read(thermostat_interval_program_0_threshold_low + (program * 90) + (progid * 6));
  return ret;
}
//// nastavi rozhodovaci uroven termostatu
void thermostat_program_set_threshold(uint8_t program, uint8_t progid, uint16_t threshold)
{
  EEPROM.write(thermostat_interval_program_0_threshold_high + (program * 90) + (progid * 6), ((threshold >> 8) & 0xff));
  EEPROM.write(thermostat_interval_program_0_threshold_low + (program * 90) + (progid * 6), (threshold & 0xff));
}
/***************************************************************************************************************/
//// rozparsuje casovy format start_hour,start_min,stop_hour,stop_min,active a ulozi jej
void thermostat_program_set_parse_interval(uint8_t program, uint8_t progid, char *str)
{
  uint8_t cnt;
  uint8_t start_hour = 0, start_min = 0, stop_hour = 0, stop_min = 0, active = 0, week = 0;
  char *pch;
  cnt = 0;
  pch = strtok (str, ",");
  while (pch != NULL)
  {
    if (cnt == 0) start_hour = atoi(pch);
    if (cnt == 1) start_min = atoi(pch);
    if (cnt == 2) stop_hour = atoi(pch);
    if (cnt == 3) stop_min = atoi(pch);
    if (cnt == 4) week = atoi(pch);
    if (cnt == 5) active = atoi(pch);

    pch = strtok (NULL, ",");
    cnt++;
  }
  if (cnt == 6)
  {
    thermostat_program_set_time(program, progid, start_hour, start_min, stop_hour, stop_min, active);
    thermostat_program_set_week(program, progid, week);
  }
}
/***************************************************************************************************************/
void thermostat_program_set_active(uint8_t program, uint8_t progid, uint8_t set_active)
{
  uint8_t start_hour, start_min, stop_hour, stop_min, active;
  thermostat_program_get_time(program, progid, &start_hour, &start_min, &stop_hour, &stop_min, &active);
  if (active != set_active)
    thermostat_program_set_time(program, progid, start_hour, start_min, stop_hour, stop_min, set_active);
}
/***************************************************************************************************************/
///// vraci 1 pokud je termostat aktivni
/// now.dayOfTheWeek() 0..nedele; 6..sobota
uint8_t thermostat_running(uint8_t program, int16_t *threshold)
{
  uint8_t ret = 0;
  uint8_t start_hour, start_min, stop_hour, stop_min, active, week;
  int16_t t_start, t_stop, t_now;
  for (uint8_t interval_id = 0; interval_id < MAX_PROGRAM_INTERVAL; interval_id++)
  {
    thermostat_program_get_time(program, interval_id, &start_hour, &start_min, &stop_hour, &stop_min, &active);
    if (active == 1)
    {
      week = thermostat_program_get_week(program, interval_id);
      t_start = start_hour * 60 + start_min;
      t_stop = stop_hour * 60 + stop_min;
      t_now = now.hour() * 60 + now.minute();
      if (t_now >= t_start && t_now < t_stop && (week & (1 << now.dayOfTheWeek()) != 0))
      {
        *threshold = thermostat_program_get_threshold(program, interval_id);
        ret = 1;
        break;
      }
    }
  }
  return ret;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////
void thermostat(void)
{
  uint8_t tdsid = 0;
  uint8_t tmode = 0;
  uint8_t tout = 0;
  int16_t thresh = 0;
  uint8_t pwm = 200; // start hodnota
  uint8_t te = 0;
  uint8_t prg = 0;
  uint8_t act;
  uint8_t active = 0;
  struct_DDS18s20 tds;


  for (uint8_t tix = 0; tix < MAX_THERMOSTAT; tix++)
  {
    tdsid = thermostat_ring_get_asociate_tds(tix);
    tmode = thermostat_ring_get_mode(tix);
    tout = thermostat_ring_get_output(tix);
    thresh = thermostat_ring_get_mezni(tix);
    if (tmode == TERM_MODE_PROG)
    {
      prg = thermostat_ring_get_program_id(tix);
      te = thermostat_running(prg, &thresh);
      if (te == 0)
      {
        tmode = TERM_MODE_OFF;
        /// blikam s ledkou protoze program neni aktivni
        sbi(led_blink, LED_PROG_I);
        thermostat_ring_set_status(tix, TERM_STAV_STOP);
      }
      else
      {
        /// blikaci ledku vypnu, program aktivni
        cbi(led_blink, LED_PROG_I);
        /// zde si ziskam rezim vypnuto, topi, chladi, ventilator, podle toho si prenastavim ring mode, potrebuji nastavit pid regulator
        thermostat_ring_set_status(tix, TERM_STAV_BEZI);
        act = thermostat_program_get_active(prg);
        if (act == 1) tmode = TERM_MODE_MAN_HEAT;
        if (act == 2) tmode = TERM_MODE_MAN_COOL;
        if (act == 3) tmode = TERM_MODE_FAN;
      }
    }

    if (tmode == TERM_MODE_MAN_HEAT)
    {
      if (tix == 0) pid0.SetControllerDirection(DIRECT);
      if (tix == 1) pid1.SetControllerDirection(DIRECT);
      if (tix == 2) pid2.SetControllerDirection(DIRECT);
    }

    if (tmode == TERM_MODE_MAN_COOL)
    {
      if (tix == 0) pid0.SetControllerDirection(REVERSE);
      if (tix == 1) pid1.SetControllerDirection(REVERSE);
      if (tix == 3) pid2.SetControllerDirection(REVERSE);
    }

    if (tdsid >= TDS_MEMORY_MAP_TDS && tdsid < TDS_MEMORY_MAP_RTDS)
    {
      if (get_tds18s20(tdsid, &tds) == 1)
        if (tds.used == 1 && status_tds18s20[tdsid].online == True)
        {
          PID_Input[tix] = status_tds18s20[tdsid].temp / 100;
          PID_Setpoint[tix] = thresh;
          pwm = PID_Output[tix];
        }
        else
        {
          tmode = TERM_MODE_ERR;
          pwm = 0;
        }
    }

    if (tdsid >= TDS_MEMORY_MAP_RTDS && tdsid < 127)
    {
      act = tdsid - TDS_MEMORY_MAP_RTDS;
      remote_tds_get_active(act , &active);
      if (active == 1 && remote_tds_last_update[act] < 180)
      {
        PID_Input[tix] = remote_tds[act] / 10;
        PID_Setpoint[tix] = thresh;
        pwm = PID_Output[tix];
      }
      else
      {
        tmode = TERM_MODE_ERR;
        pwm = 0;
      }
    }



    if (tmode == TERM_MODE_OFF)
    {
      mqtt_publis_output(tout, POWER_OUTPUT_OFF);
    }
    if (tmode == TERM_MODE_MAX)
    {
      mqtt_publis_output(tout, POWER_OUTPUT_HEAT_MAX);
    }
    if (tmode == TERM_MODE_CLIMATE)
    {
      mqtt_publis_output(tout, POWER_OUTPUT_COOL_MAX);
    }
    if (tmode == TERM_MODE_MAN_HEAT || tmode == TERM_MODE_MAN_COOL || tmode == TERM_MODE_FAN || tmode == TERM_MODE_ERR)
    {
      mqtt_publis_output_pwm(tout, tmode, pwm);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////



/////
/// servisni logovatko pres mqtt ///
int printf_via_mqtt( char c, FILE * t)
{
  char topic[8];
  mqtt_log[mqtt_log_cnt] = c;
  mqtt_log[mqtt_log_cnt + 1] = 0;
  mqtt_log_cnt++;
  if (mqtt_log_cnt > 127 || c == '\n' || c == 0)
  {
    strcpy(topic, "log");
    send_mqtt_general_payload(topic, mqtt_log);
    mqtt_log_cnt = 0;
  }
}

void log_error(char *log)
{
  /// TODO vyresit logovani chyby

}



void set_default_ring(uint8_t ring)
{
  EEPROM.write(my_default_ring, ring);
}

uint8_t get_default_ring(void)
{
  return EEPROM.read(my_default_ring);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  char tmp1[20];
  char tmp2[20];
  uint8_t itmp;
  boolean bol;


  wdt_disable();

  fdevopen( &printf_via_mqtt, 0);

  struct_DDS18s20 tds;
  // put your setup code here, to run once:
  // komunikace přes sériovou linku rychlostí 19200
  //Serial.begin(38400);

  menu_history_init(&rm);
  menu_init(&rm);
  menu_root_setname(&rm, title_root_termostat);

  menu_item_set_properties(&item_show_temp, title_item_menu_temp, TYPE_FUNCTION_DYNAMIC, show_temp_default, 0, 0);
  menu_item_set_properties(&item_show_time, title_item_menu_time, TYPE_FUNCTION, show_time);
  menu_root_additem(&rm, &item_show_time);
  menu_root_additem(&rm, &item_show_temp);

  menu_init(&term_max);
  menu_init(&term_off);
  menu_init(&term_prog);
  menu_init(&term_climate);
  menu_init(&term_man);
  menu_init(&setup_menu);
  menu_init(&setup_menu_jas);
  menu_init(&setup_select_prog);
  menu_init(&setup_select_ring);
  menu_init(&setup_select_osvetleni);

  menu_root_setname(&term_man, title_term_man);
  menu_root_setname(&term_prog, title_term_prog);
  menu_root_setname(&term_max, title_term_max);
  menu_root_setname(&term_climate, title_term_climate);
  menu_root_setname(&term_off, title_term_off);
  menu_root_setname(&setup_menu, title_setup_menu);
  menu_root_setname(&setup_menu_jas, title_item_setup_jas);
  menu_root_setname(&setup_select_prog, title_item_select_prog);
  menu_root_setname(&setup_select_ring, title_item_select_ring);
  menu_root_setname(&setup_select_osvetleni, title_item_set_select_osvetleni);

  menu_item_set_properties(&item_zpet, title_menu_back, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_set_jas, title_item_setup_jas, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_select_prog, title_item_select_prog, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_select_ring, title_item_select_ring, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_select_reset, title_item_select_reset , TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_select_osvetleni, title_item_set_select_osvetleni, TYPE_FUNCTION_DYNAMIC, set_select_osvetleni, 0, 0);

  menu_root_additem(&setup_menu, &item_set_jas);
  menu_root_additem(&setup_menu, &item_select_prog);
  menu_root_additem(&setup_menu, &item_select_ring);
  menu_root_additem(&setup_menu, &item_select_reset);
  menu_root_additem(&setup_menu, &item_select_osvetleni);
  menu_root_additem(&setup_menu, &item_zpet);


  menu_item_set_properties(&item_set_man_temp, title_term_man, TYPE_DYNAMIC, set_man_term_temp, 0, 32);

  menu_item_set_properties(&item_set_jas_dyn, title_item_setup_jas_dynamic, TYPE_DYNAMIC, nullptr, 1, 16);
  menu_item_set_properties(&item_set_jas_auto, title_item_setup_jas_auto, TYPE_STATIC, nullptr);

  menu_item_set_properties(&item_set_select_prog, title_item_set_select_prog, TYPE_FUNCTION_DYNAMIC, set_select_active_prog, 0, 0);
  menu_item_set_properties(&item_set_select_ring, title_item_set_select_ring, TYPE_DYNAMIC, set_select_ring, 0, 2);

  menu_root_additem(&setup_menu_jas, &item_set_jas_dyn);
  menu_root_additem(&setup_menu_jas, &item_set_jas_auto);

  menu_root_additem(&term_man, &item_set_man_temp);

  menu_root_additem(&setup_select_prog, &item_set_select_prog);
  menu_root_additem(&setup_select_ring, &item_set_select_ring);

  menu_item_set_properties(&item_set_select_osvetleni, title_item_set_select_osvetleni, TYPE_DYNAMIC, nullptr, 1, 9);
  menu_root_additem(&setup_select_osvetleni, &item_set_select_osvetleni);




  //pinMode(NRF_IRQ, INPUT);
  //digitalWrite(NRF_IRQ, HIGH);
  //pinMode(NRF_CE, OUTPUT);
  //pinMode(NRF_CS, OUTPUT);

  pinMode(ETH_RST, OUTPUT);
  digitalWrite(ETH_RST, LOW);
  for (uint16_t i = 0; i < 8024; i++);
  digitalWrite(ETH_RST, HIGH);


  statisice = pgm_read_word_near(&SixteenAlfaNumeric['-']);
  destisice = pgm_read_word_near(&SixteenAlfaNumeric['-']);
  tisice = pgm_read_word_near(&SixteenAlfaNumeric['-']);
  stovky = pgm_read_word_near(&SixteenAlfaNumeric['-']);
  desitky = pgm_read_word_near(&SixteenAlfaNumeric['-']);
  jednotky = pgm_read_word_near(&SixteenAlfaNumeric['-']);
  display_pos = 0;




  ds2482_address[0].i2c_addr = 0b0011000;
  ds2482_address[0].HWwirenum = 0;

  for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXROMS; idx++ )
  {
    status_tds18s20[idx].wait = false;
  }


  noInterrupts();           // disable all interrupts
  //pinMode(ETH_INT, INPUT_PULLUP);


  pinMode(NRF_CE, OUTPUT);
  pinMode(NRF_CS, OUTPUT);

  pinMode(LED, OUTPUT);

  pinMode(tr1, OUTPUT);
  pinMode(tr2, OUTPUT);
  pinMode(tr3, OUTPUT);
  pinMode(tr4, OUTPUT);
  pinMode(tr5, OUTPUT);
  pinMode(tr6, OUTPUT);

  pinMode(rs485, OUTPUT);
  pinMode(PWM_DISP, OUTPUT);
  pinMode(PWM_KEY, OUTPUT);
  pinMode(DP, OUTPUT);

  pinMode(SER1, OUTPUT);
  pinMode(SER2, OUTPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(PCLK, OUTPUT);

  digitalWrite(tr1, 1);
  digitalWrite(tr2, 1);
  digitalWrite(tr3, 1);
  digitalWrite(tr4, 1);
  digitalWrite(tr5, 1);
  digitalWrite(tr6, 1);
  digitalWrite(DP, 1);

  //digitalWrite(rs485, LOW);

  digitalWrite(LED, 1);

  TCCR3A = 0;
  TCCR3B  = (1 << CS31);
  timer3_counter = 61100;/// cca 450Hz
  TCNT3 = timer3_counter;
  TIMSK3 |= (1 << TOIE3);

  TCCR2A = 0;
  TCCR2B  = (1 << CS22) | (1 << CS21) | (1 << CS20); /// delicka 128
  TIMSK2 |= (1 << TOIE2);

  interrupts();             // enable all interrupts
  SPI.begin();
  rtc.begin();

  printf_begin();




  for (uint8_t inic = 0; inic < 15; inic++)
  {
    if (inic == 0)
    {
      if (EEPROM.read(set_default_values) == 255)
      {

        strcpy(tmp1, "I0-FR-");
        show_direct(tmp1);

        EEPROM.write(set_default_values, 0);
        EEPROM.write(my_jas_disp, 240);
        EEPROM.write(my_jas_key, 240);
        EEPROM.write(my_rs_id, 31);
        keyboard_set_sense(255);
        for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
        {
          PID_p[idx] = 1;
          PID_i[idx] = 0.5;
          PID_d[idx] = 1;
          pid_save_variable();
          thermostat_ring_set_asociate_tds(idx, 255);
          thermostat_ring_set_mezni(idx, 220);
          thermostat_ring_set_program_id(idx, 0);
          thermostat_ring_set_status(idx, 0);
          thermostat_ring_set_active(idx, 0);
          thermostat_ring_set_output(idx, 255);
          thermostat_ring_set_mode(idx, 0);
          thermostat_ring_set_name(idx, "FREE");
        }
        for (uint8_t idx = 0; idx < MAX_RTDS; idx++)
        {
          remote_tds_set_name(idx, 0 , "");
        }
        for (uint8_t idx = 0; idx < AVAILABLE_PROGRAM; idx++)
        {
          thermostat_program_set_name(idx, "PROG");
          thermostat_program_set_active(idx, 0);
          for (uint8_t progid = 0; progid < MAX_PROGRAM_INTERVAL; progid++)
          {
            thermostat_program_set_time(idx, progid, 0, 0, 0, 0, 0);
            thermostat_program_set_threshold(idx, progid, 220);
            thermostat_program_set_week(idx, progid, 0);
          }
        }
        for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXDEVICES; idx++)
        {
          get_tds18s20(idx, &tds);
          strcpy(tds.name, "FREE");
          tds.used = 0;
          tds.offset = 0;
          tds.assigned_ds2482 = 0;
          tds.period = 10;
          set_tds18s20(idx, &tds);
        }

        for (uint8_t idx = 0; idx < MAX_SVETEL; idx++) light_set_output(idx, LIGHT_FREE);
        rtc.adjust(DateTime(2020, 12, 14, 17, 14, 0));
        device.mac[0] = 2; device.mac[1] = 1; device.mac[2] = 2; device.mac[3] = 3; device.mac[4] = 4; device.mac[5] = 8;
        device.myIP[0] = 192; device.myIP[1] = 168; device.myIP[2] = 2; device.myIP[3] = 112;
        device.myMASK[0] = 255; device.myMASK[1] = 255; device.myMASK[2] = 255; device.myMASK[3] = 0;
        device.myGW[0] = 192; device.myGW[1] = 168; device.myGW[2] = 2; device.myGW[3] = 1;
        device.myDNS[0] = 192; device.myDNS[1] = 168; device.myDNS[2] = 2; device.myDNS[3] = 1;
        device.mqtt_server[0] = 192; device.mqtt_server[1] = 168; device.mqtt_server[2] = 2; device.mqtt_server[3] = 1;
        device.ntp_server[0] = 192; device.ntp_server[1] = 168; device.ntp_server[2] = 2; device.ntp_server[3] = 1;
        device.mqtt_port = 1883;
        strcpy(device.mqtt_user, "saric");
        strcpy(device.mqtt_key, "no");
        save_setup_network();
        device_set_name("TERM E2");
        char hostname[10];
        device_get_name(hostname);
        nrf_save_channel(76);
        set_default_ring(0);
        delay(1000);
      }
    }

    if (inic == 1)
    {
      strcpy(tmp1, "I1-LN-");
      show_direct(tmp1);
      load_setup_network();
    }

    if (inic == 2)
    {
      strcpy(tmp1, "I2-OV-");
      show_direct(tmp1);
      jas_disp = EEPROM.read(my_jas_disp);
      jas_key = EEPROM.read(my_jas_key);
      rsid = EEPROM.read(my_rs_id);
      analogWrite(PWM_DISP, jas_disp);
      analogWrite(PWM_KEY, jas_key);
      pid_load_variable();
      default_ring = get_default_ring();
      for (uint8_t idx = 0; idx < MAX_SVETEL; idx++)
      {
        light_value[idx] = LIGHT_DEFAULT;
        light_state[idx] = LIGHT_DEFAULT;
      }
    }


    if (inic == 3)
    {
      itoa(ds2482_address[0].i2c_addr, tmp1, 10);
      if (ds2482reset(ds2482_address[0].i2c_addr) == DS2482_ERR_OK)
      {
        strcpy(tmp1, "I3-1W-");
        show_direct(tmp1);
      }
      else
      {
        strcpy(tmp1, "I3-ERR");
        show_direct(tmp1);
      }
      Global_HWwirenum = 0;
      one_hw_search_device(0);
    }

    if (inic == 4)
    {
      strcpy(tmp1, "I4-MPC");
      show_direct(tmp1);
      init_mpc();
      write_to_mcp(255);
      for (uint8_t i = 0; i < 8; i++)
      {
        write_to_mcp(~(1 << i));
        tecka = (1 << i);
        delay(50);
      }
      for (uint8_t i = 0; i < 8; i++)
      {
        write_to_mcp(~(1 << (7 - i)));
        tecka = (1 << (7 - i));
        delay(50);
      }
      tecka = 0;
      write_to_mcp(255);
    }

    if (inic == 5)
    {
      strcpy(tmp1, "I5-KEY");
      show_direct(tmp1);
      keyboard_apply_sense();
    }

    if (inic == 6)
    {
      if (!rtc.isrunning())
      {
        strcpy(tmp1, "I6RTEE");
        show_direct(tmp1);
      }
      else
      {
        strcpy(tmp1, "I6RTOK");
        show_direct(tmp1);
      }
    }

    if (inic == 7)
    {
      strcpy(tmp1, "I7-PID");
      show_direct(tmp1);
      /// nastaveni vychozich hodnot pro regulator
      for (uint8_t idx = 0; idx < 3; idx++)
      {
        PID_Input[idx] = 225;
        PID_Output[idx] = 0;
        PID_Setpoint[idx] = 225;
      }
      pid0.SetMode(AUTOMATIC);
      pid1.SetMode(AUTOMATIC);
      pid2.SetMode(AUTOMATIC);
      pid0.SetOutputLimits(0, 255);
      pid1.SetOutputLimits(0, 255);
      pid2.SetOutputLimits(0, 255);
      pid0.SetSampleTime(30000);
      pid1.SetSampleTime(30000);
      pid2.SetSampleTime(30000);
      pid_update_variable();
    }

    if (inic == 9)
    {
      if ( Enc28J60Network::linkStatus() == 1)
      {
        strcpy(tmp1, "I9-UP-");
        link_status = 1;
      }
      else
      {
        strcpy(tmp1, "I9DOWN");
        link_status = 0;
      }
      show_direct(tmp1);
    }

    if (inic == 8)
    {
      strcpy(tmp1, "I09ETH");
      show_direct(tmp1);
      Ethernet.begin(device.mac, device.myIP, device.myDNS, device.myGW, device.myMASK);

    }

    if (inic == 10)
    {
      strcpy(tmp1, "I-MQQT");
      show_direct(tmp1);
      mqtt_client.setServer(device.mqtt_server, device.mqtt_port);
      mqtt_client.setCallback(mqtt_callback);
      if (link_status == 1) mqtt_reconnect();
    }



    if (inic == 11)
    {
      strcpy(tmp1, "I11NTP");
      show_direct(tmp1);
      timeClient.begin();
      timeClient.setTimeOffset(3600);
      tmp2[0] = 0;
      createString(tmp2, '.', device.ntp_server, 4, 10);
      timeClient.setPoolServerName(tmp2);
      timeClient.setUpdateInterval(60000);
      if (link_status == 1) timeClient.update();

    }

    if (inic == 12)
    {
      strcpy(tmp1, "I-RTDS");
      show_direct(tmp1);
    }


    if (inic == 13)
    {
      strcpy(tmp1, "I-NRF-");
      show_direct(tmp1);
      radio.begin();
      radio.enableAckPayload();                     // Allow optional ack payloads
      radio.enableDynamicPayloads();                // Ack payloads are dynamic payload
      radio.openWritingPipe(nrf_addresses[1]);
      radio.openReadingPipe(1, nrf_addresses[0]);
      radio.startListening();
      radio.setPALevel(nrf_load_power());
      radio.setChannel(nrf_load_channel());
      uint8_t counter;
      radio.writeAckPayload(1, &counter, 1);
      radio.printDetails();
    }


    if (inic == 14)
    {
      strcpy(tmp1, "I14HOT");
      show_direct(tmp1);
      send_mqtt_general_payload("log", "START");
    }

    delay(300);
  }

  wdt_enable(WDTO_4S);

}
///konec setup




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t init_mpc(void)
{
  Wire.beginTransmission(0x20);
  Wire.write(0x0A); // ICON
  Wire.write(0xff);
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x00); // IODIRA register
  Wire.write(0x00); // set entire PORT A to output
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x10); // IODIRB register
  Wire.write(0xff); // set entire PORT B to input
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x16); // GPPUB register
  Wire.write(0xff); // pullup
  Wire.endTransmission();
}
uint8_t read_from_mcp(void)
{
  uint8_t input;
  // read the inputs of PORT B
  Wire.beginTransmission(0x20);
  Wire.write(0x19);
  Wire.endTransmission();
  Wire.requestFrom(0x20, 1);
  input = Wire.read();
  return input;
}
void write_to_mcp(uint8_t data)
{
  // write that input to PORT A
  Wire.beginTransmission(0x20);
  Wire.write(0x09); // address PORT A
  Wire.write(data);    // PORT A
  Wire.endTransmission();
}

void find_rs_device(void)
{


}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///potreba udelat
/*

  0. hw problem zvlnene napeti; objednat jine kondenzatory + civku - dobry vsechno funguje, potreba lepsi design desky, kratke spoje
     - kondenzator k ds18s20

  1. kalibraci tlacitek - vyreseno nastavenim pres mqtt - potreba otestovat

  2. podsviceni tlacitek - vyreseni nastavenim preq mqtt - potreba otestovat

  3. reinit ntp - hotovo, pokud nefunguje ntp server ulozi spatne casy

  4. remote tds
     - vyresit problem s prepisem 2x zapisuji do stejneho idx, musim udelat kontrolu zda se pouziva, pokud ano tak ignoruji - hotovo
     - last update - vyreseno

  5. fix init11 mqtt connect - otestovat, nyni mqtt_init vraci stav connect a ne stav subscribe - hotovo

  6. mereni loadu procesoru - vymyslet lepe
            - pocet prijatych mqqt zprav
            - pocet zpracovanych mqtt zprav
            - pocet eventu od klavesnice

  7. opravit uptime - otestovat - hotovo

  8. mqtt nastaveni jasu displeje - otestovat - ok

  9. mqtt nastaveni site, ip, mask, gw, mqtt - hotovo
      - ziskat stav nastaveni - hotovo

  10. preprogramovat fuse nemaz eeprom - ok; vraceno zmet

  11. napojeni rs485 zarizeni

  12. ochrana proti preteceni indexu - ok

  13. preposilat nrf zpravy

  14. opravit logovatko

  15. kdyz mode prog - blika = neaktivni a prepnuti jinam stale blika - potreba otestovat - opraveno

  16. nrf24
      - nastavit kanal
      - nastavit nazev write kanalu
      - nastavit nazev read kanalu 1..5
      - vysilaci vykon
    - zatim jen pripraveno

  17. bootloader po ethernetu

  18. otestovat remote_tds jako vstup pro pid regulator
      - remote_tds jako vstup pro ring termostat - potreba otestovat - potreba vyresit zpetnou vazbu
      - otestovat neexistujici rtds - ok
      - otestovat last_update  - ok

  19. potreba otestovat remote_tds_last_update + tds18s20 active; temp_show_default remote_tds funguje

  20. lepe se umet zotavit ze spatnych stavu

  21. blikatko otestovat - hotovo

  22. tecku displayje - hotovo

  23. ring main. tj, ktery ring nastavuji pres display - hotovo
      - mqtt - ok

  24. vyresit presnost ds18s20, prumer z nekolika hodnot
      - pres mqtt umet nastavit periodu mereni - podpora je pripravena - potreba otestovat
      - uplne preprogramovat mereni na hwwire, neposilat zacatek mereni na vsechny zarizeni sbernice ale adresovat primo

  25. vyresit mikro ventilator

  26. otestovat predelane metody tds/clear, rtds/clear - hotovo

  27. rozdil odesilani mac u rom tds - hotovo

  28. pridat do menu reset - hotovo

  29. vyresit problem s merenim osvetleni

  30. umet nastavit sit pres rs485

  31 watchdog

  32. ovladani svetel - hotovo
     - podpora pro zpetnou vazbu, napr praskla zarovka, light_state

  33. ring_thermostat kdyz neni zadny aktivni vracet -1

  34. nedovolit nastavit pres mqqt program, kdyz neni aktivni
*/



void loop()
{
  char str1[32];
  char str2[20];
  char str3[20];
  char curr_menu[10];
  char curr_item[10];
  uint8_t itmp;
  char cmd[MAX_TEMP_BUFFER];
  char args[MAX_TEMP_BUFFER];
  struct_DDS18s20 tds;
  uint8_t pipeno;
  uint8_t nrf_payload;


  wdt_reset();

  load_2++;

  if ( Enc28J60Network::linkStatus() == 1)
    link_status = 1;
  else
    link_status = 0;


  if (link_status == 1)
  {
    if (!mqtt_client.connected())
      mqtt_reconnect();
    else
      mqtt_status = 1;
  }
  else
  {
    mqtt_client.disconnect();
    mqtt_status = 0;
  }

  mqtt_client.loop();

  pid0.Compute();
  pid1.Compute();
  pid2.Compute();



  //radio.startListening();
  //if (radio.testRPD() == 1) printf("RPD signal ok \n\r");

  if ( radio.available(&pipeno))
  {
    radio.read( &nrf_payload, 1 );
    nrf_payload += 1;
    radio.writeAckPayload(pipeno, &nrf_payload, 1 );
  }


  find_rs_device();
  /*
    read_at(uart_recv);

    if (start_at == 255)
    {
    start_at = 0;

    new_parse_at(uart_recv, str1, str2);
    id = atoi(str1);
    cmd[0] = 0;
    args[0] = 0;
    new_parse_at(str2, cmd, args);


    /////////////////////////////////////
    ///prikazy pouze pro tuto jednotku
    if (id == rsid)
    {
    }
    }
  */



  led = 0;
  //led_old = 0;
  key = 0;


  if ((milis - milis_key) > 50)
  {
    milis_key = milis;
    key_now = ~Bit_Reverse(read_from_mcp());
    for (uint8_t idx = 0; idx < 8; idx++)
    {
      uint8_t t = (1 << idx);
      if ((key_now & t) != 0)
      {
        ///key press
        // pokud drzim tlacitko tak nic
        if ((key_release & t) != 0)
        {
          bitSet(key, idx);
          bitClear(key_release, idx);
        }
        key_press_cnt[idx]++;
        if (key_press_cnt[idx] > 20)
        {
          bitSet(key_press, idx);
          /// ochrana proti preteceni
          key_press_cnt[idx] = 21;
        }
      }
      else
      {
        ///key release
        bitSet(key_release, idx);
        bitClear(key, idx);
        bitClear(key_press, idx);
        key_press_cnt[idx] = 0;
      }
    }
  }







  if (mqtt_status == 0)
  {
    led = led + LED_ERR;
  }




  term_mode = thermostat_ring_get_mode(default_ring);
  if (term_mode == TERM_MODE_OFF)
  {
    led = led + LED_OFF;
    cbi(led_blink, LED_PROG_I);
  }
  if (term_mode == TERM_MODE_MAX)
  {
    led = led + LED_MAX;
    cbi(led_blink, LED_PROG_I);
  }
  if (term_mode == TERM_MODE_PROG)
  {
    led = led + LED_PROG;
  }
  if (term_mode == TERM_MODE_MAN_HEAT)
  {
    led = led + LED_MAX + LED_UP + LED_DOWN;
    cbi(led_blink, LED_PROG_I);
  }
  if (term_mode == TERM_MODE_CLIMATE)
  {
    led = led + LED_CLIMA;
    cbi(led_blink, LED_PROG_I);
  }
  if (term_mode == TERM_MODE_MAN_COOL)
  {
    led = led + LED_CLIMA + LED_UP + LED_DOWN;
    cbi(led_blink, LED_PROG_I);
  }

  /// blikatko
  if ((uptime % 2) == 0)
    led_blink_old = led_blink;

  for (uint8_t ledidx = 0; ledidx < 8; ledidx++)
  {
    uint8_t lc = (1 << ledidx);
    if ((led_blink_old & lc) != 0)
    {
      if ((led & lc) == 0)
        sbi(led, ledidx);
      else
        cbi(led, ledidx);
    }
  }
  led_blink_old = 0;

  /// zapisuji do led registru pouze pri zmene
  if (led != led_old)
  {
    led_old = led;
    write_to_mcp(~led);
  }


  get_current_menu(curr_menu);
  get_current_item(curr_item);
  //// defualt screen uplne hlavni menu
  if (strcmp_P(curr_menu, title_root_termostat) == 0)
  {
    /// prepinej zobrazeni cas/teplota/stav termostatu
    if (key == TL_OK)
    {
      menu_next();
      key = 0;
    }
    if (key == TL_OFF)
    {
      thermostat_ring_set_mode(default_ring, TERM_MODE_OFF);
      menu_set_root_menu(&term_off);
      delay_show_menu = 0;
      key = 0;
    }
    if (key == TL_MAX)
    {
      thermostat_ring_set_mode(default_ring, TERM_MODE_MAX);
      menu_set_root_menu(&term_max);
      delay_show_menu = 0;
      key = 0;
    }
    if (key == TL_PROG)
    {
      thermostat_ring_set_mode(default_ring, TERM_MODE_PROG);
      menu_set_root_menu(&term_prog);
      delay_show_menu = 0;
      key = 0;
    }
    if (key == TL_CLIMA)
    {
      thermostat_ring_set_mode(default_ring, TERM_MODE_CLIMATE);
      menu_set_root_menu(&term_climate);
      delay_show_menu = 0;
      key = 0;
    }
    if ((key == TL_UP) || (key == TL_DOWN))
    {
      thermostat_ring_set_mode(default_ring, TERM_MODE_MAN_HEAT);
      term_man.args3 = (thermostat_ring_get_mezni(default_ring) - 160) / 5;
      menu_set_root_menu(&term_man);
      delay_show_menu = 0;
      key = 0;
    }

    if (key_press == TL_OK)
    {
      menu_set_root_menu(&setup_menu);
      key_press = 0;
      key = 0;
    }

  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  //////////////////// hlavni setup menu
  if (strcmp_P(curr_menu, title_setup_menu) == 0)
  {
    if (key == TL_UP) menu_next();
    if (key == TL_DOWN) menu_prev();
    ////
    if (key == TL_OK)
    {
      /// podmenu jas
      if (strcmp_P(curr_item, title_item_setup_jas) == 0)
      {
        menu_set_root_menu(&setup_menu_jas);
        setup_menu_jas.args3 = (255 - jas_disp) / 15;
        key = 0;
      }
      /// menu mqtt status
      //if (strcmp_P(curr_item, title_item_mqtt_status) == 0)
      //{
      //}
      if (strcmp_P(curr_item, title_item_select_prog) == 0)
      {
        setup_select_prog.args3 = rev_get_show_prog(thermostat_ring_get_program_id(default_ring));
        menu_set_root_menu(&setup_select_prog);
        key = 0;
      }
      if (strcmp_P(curr_item, title_item_select_ring) == 0)
      {
        setup_select_ring.args3 = default_ring;
        menu_set_root_menu(&setup_select_ring);
        key = 0;
      }
      /// menu nastaveni intenzity svetla
      if (strcmp_P(curr_item, title_item_set_select_osvetleni) == 0)
      {
        if (setup_menu.args2 != LIGHT_FREE)
        {
          setup_select_osvetleni.args1 = setup_menu.args2;
          setup_select_osvetleni.args3 = light_value[setup_menu.args2];
          menu_set_root_menu(&setup_select_osvetleni);
        }
        key = 0;
      }
      /// menu zpet
      if (strcmp_P(curr_item, title_menu_back) == 0)
      {
        menu_back_root_menu();
        key_press = 0;
        key = 0;
      }
    }
    if (key_press == TL_OK)
    {
      if (strcmp_P(curr_item, title_item_select_reset) == 0)
      {
        resetFunc();
      }
      key = 0;
      key_press = 0;
    }
  }
  /// konec hlavniho setup menu
  /////////////////////////////////
  /// menu vyber programu
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  if (strcmp_P(curr_menu, title_item_select_prog) == 0)
  {
    if (key == TL_UP)
    {
      menu_next();
    }
    if (key == TL_DOWN)
    {
      menu_prev();
    }
    if (key == TL_OK)
    {
      if (setup_select_prog.args2 != PROG_FREE) thermostat_ring_set_program_id(default_ring, setup_select_prog.args2);
      menu_back_root_menu();
    }
  }
  /// menu nastaveni intenzity svetla
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  if (strcmp_P(curr_menu, title_item_set_select_osvetleni) == 0)
  {
    itmp = 0;
    if (key == TL_UP)
    {
      menu_next();
      itmp = 1;
    }
    if (key == TL_DOWN)
    {
      menu_prev();
      itmp = 1;
    }
    if (key == TL_OK)
    {
      itmp = 1;
      menu_back_root_menu();
    }
    if (itmp == 1)
    {
      light_value[setup_select_osvetleni.args1] = setup_select_osvetleni.args3;
      mqtt_send_light_value(setup_select_osvetleni.args1);
    }
  }
  /// menu vyber programu
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  if (strcmp_P(curr_menu, title_item_select_ring) == 0)
  {
    if (key == TL_UP)
    {
      menu_next();
    }
    if (key == TL_DOWN)
    {
      menu_prev();
    }
    if (key == TL_OK)
    {
      default_ring = setup_select_ring.args3;
      set_default_ring(setup_select_ring.args3);
      menu_back_root_menu();
    }
  }
  /////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////

  get_current_menu(curr_menu);
  get_current_item(curr_item);
  /// nastaveni jasu
  if (strcmp_P(curr_menu, title_item_setup_jas) == 0)
  {
    if (key == TL_UP)
    {
      menu_next();
    }
    if (key == TL_DOWN)
    {
      menu_prev();
    }
    jas_disp = 255 - (15 * setup_menu_jas.args3);
    analogWrite(PWM_DISP, jas_disp);
    if (key == TL_OK)
    {
      EEPROM.write(my_jas_disp, jas_disp);
      menu_back_root_menu();
      key = 0;
    }
  }


  ///////////////////////////////////////////////////////////////////
  //// rucni nastaveni teploty
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  if (strcmp_P(curr_menu, title_term_man) == 0)
  {
    if (key == TL_UP)
    {
      menu_next();
      delay_show_menu = 0;
    }
    if (key == TL_DOWN)
    {
      menu_prev();
      delay_show_menu = 0;
    }
    if (delay_show_menu > 10 || key == TL_OK)
    {
      delay_show_menu = 0;
      menu_back_root_menu();
      thermostat_ring_set_mezni(default_ring, 160 + (term_man.args3 * 5));
    }
    key = 0;
  }
  //////////////////////////////
  //// reset
  //get_current_menu(curr_menu);
  //get_current_item(curr_item);

  //}
  /////////////////////////
  /// vraceni se zpet po 10 sec
  if (strcmp_P(curr_menu, title_term_off) == 0)
  {
    if (key == TL_OFF) menu_back_root_menu();
    if (delay_show_menu > 5) menu_back_root_menu();
  }
  if (strcmp_P(curr_menu, title_term_max) == 0)
  {
    if (key == TL_MAX) menu_back_root_menu();
    if (delay_show_menu > 5) menu_back_root_menu();
  }
  if (strcmp_P(curr_menu, title_term_prog) == 0)
  {
    if (key == TL_PROG) menu_back_root_menu();
    if (delay_show_menu > 10) menu_back_root_menu();
  }
  if (strcmp_P(curr_menu, title_term_climate) == 0)
  {
    if (key == TL_CLIMA) menu_back_root_menu();
    if (delay_show_menu > 10) menu_back_root_menu();
  }
  if (strcmp_P(curr_menu, title_error) == 0)
  {
    if (delay_show_menu > 2) menu_back_root_menu();
  }

  ///globalni back
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  if (strcmp_P(curr_item, title_menu_back) == 0)
  {
    if (key == TL_OK)
    {
      menu_back_root_menu();
      key = 0;
    }
  }

  ////////////////////
  /// kazdych 10sec
  if ((milis - milis_10s) > 4540)
  {
    milis_10s = milis;
    send_mqtt_onewire();
    send_mqtt_status();
    send_mqtt_ring();
    send_mqtt_tds();
    send_mqtt_program();
    thermostat();
    //mqtt_send_pid_variable();
    //send_mqtt_remote_tds_status();
    //send_network_config();
    send_light_controler();
  }


  if ((milis - milis_05s) > 225)
  {
    milis_05s = milis;
    now = rtc.now();
    menu_display();
    digitalWrite(LED, 1);
  }


  ////////////////////
  /// kazdou 1sec
  if ((milis - milis_1s) > 454)
  {
    delay_show_menu++;
    milis_1s = milis;
    uptime++;
    mereni_hwwire();

    /// pocitadlo posledni aktualizace remote tds
    for (uint8_t idx = 0; idx < MAX_RTDS; idx++)
      remote_tds_last_update[idx]++;

    /// nastaveni dynamickeho menu pro programy
    use_prog = count_use_prog();
    if (use_prog > 0) use_prog = use_prog - 1;
    menu_item_update_limits(&item_set_select_prog, 0, use_prog);

    /// nastaveni dynamickeho menu teplomeru
    use_tds = count_use_tds();
    use_tds = use_tds + count_use_rtds();
    if (use_tds > 0) use_tds = use_tds - 1;
    menu_item_update_limits(&item_show_temp, 0, use_tds);

    use_light = count_use_light();
    if (use_light > 0) use_light = use_light - 1;
    menu_item_update_limits(&item_select_osvetleni, 0, use_light);

    /// automaticke nastaveni jasu displaye
    aktual_light = analogRead(LIGHT);
    itmp = (aktual_light / 4 / 15 * 15);
    if (jas_disp == 0) // Automatika
    {
      if (itmp > 240) itmp = 240;
      analogWrite(PWM_DISP, itmp);
    }
  }





}



/////////////////////////////////////////////////////////////////////////////////////////////////////

void parseBytes(const char* str, char sep, byte* bytes, int maxBytes, int base) {
  for (int i = 0; i < maxBytes; i++) {
    bytes[i] = strtoul(str, NULL, base);  // Convert byte
    str = strchr(str, sep);               // Find next separator
    if (str == NULL || *str == '\0') {
      break;                            // No more separators, exit
    }
    str++;                                // Point to next character after separator
  }
}



void createString(const char* str, char sep, byte* bytes, uint8_t maxBytes, uint8_t base)
{
  char tmp1[8];
  for (uint8_t a = 0; a < maxBytes; a++ )
  {
    itoa(bytes[a], tmp1, base);
    strcat(str, tmp1);
    if (a < maxBytes - 1)
    {
      tmp1[0] = sep;
      tmp1[1] = 0;
      strcat(str, tmp1);
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/// funkce ktere prevedou data floatu na uint8_t a zpet
void getFloat(uint8_t *ar, float *x)
{
  union {
    byte b[4];
    float f;
  } data;
  for (int i = 0; i < 4; i++) data.b[i] = ar[i];
  *x = data.f;
}

void getInt(uint8_t *ar, float x)
{
  union {
    byte b[4];
    float f;
  } data;
  data.f = x;
  for (int i = 0; i < 4; i++) ar[i] = data.b[i];
}

///////////////////////////////////////////////////////
//// cteni zapis eeprom float datovy typ
float EEPROMreadFloat(unsigned int addr)
{
  union {
    byte b[4];
    float f;
  } data;
  for (int i = 0; i < 4; i++) data.b[i] = EEPROM.read(addr + i);
  return data.f;
}

void EEPROMwriteFloat(unsigned int addr, float x)
{
  union {
    byte b[4];
    float f;
  } data;
  data.f = x;
  for (int i = 0; i < 4; i++) EEPROM.write(addr + i, data.b[i]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_OVF_vect)
{
  uint8_t backup = SREG;
  timer2_counter++;
  if (timer2_counter < 30)
    timer2_counter = 0;
  load2 = load_2;
  load_2 = 0;
  SREG = backup;
}


ISR(TIMER3_OVF_vect)        // interrupt service routine
{
  uint8_t backup = SREG;
  TCNT3 = timer3_counter;   // preload timer
  milis++;
  digitalWrite(tr1, 1);
  digitalWrite(tr2, 1);
  digitalWrite(tr3, 1);
  digitalWrite(tr4, 1);
  digitalWrite(tr5, 1);
  digitalWrite(tr6, 1);
  digitalWrite(DP, 1);
  if (display_pos == 0)
  {
    shiftout(~statisice);
    if ((tecka & 0b00000001) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr1, 0);
  }
  if (display_pos == 1)
  {
    shiftout(~destisice);
    if ((tecka & 0b00000010) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr2, 0);
  }
  if (display_pos == 2)
  {
    shiftout(~tisice);
    if ((tecka & 0b00000100) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr3, 0);
  }
  if (display_pos == 3)
  {
    shiftout(~stovky);
    if ((tecka & 0b00001000) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr4, 0);
  }
  if (display_pos == 4)
  {
    shiftout(~desitky);
    if ((tecka & 0b00010000) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr5, 0);
  }
  if (display_pos == 5)
  {
    shiftout(~jednotky);
    if ((tecka & 0b00100000) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr6, 0);
  }
  display_pos++;
  if (display_pos == 6) display_pos = 0;
  SREG = backup;
}


///////////////////////////////////////////////////////////////////
void shiftout(uint16_t data)
{
  digitalWrite(PCLK, 0);
  uint8_t i;
  for (i = 0; i < 8; i++)  {
    digitalWrite(SCLK, LOW);
    digitalWrite(SER2, !!((data & 0xff) & (1 << i)) );
    digitalWrite(SER1, !!((data >> 8) & (1 << i)) );
    digitalWrite(SCLK, HIGH);
  }
  digitalWrite(PCLK, 1);
}
//////////////////////////////////////////////////////////////////
uint8_t Bit_Reverse( uint8_t x )
{
  x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
  x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
  x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
  return x;
}


































/*

     if (ladeni > 0)
  {
    nRF.printDetails();
    ladeni = 0;
  }

  if (scan_rf_net_enable == 1)
  {
    scan_rf_network(scan_rf_net_channel);
    scan_rf_net_channel++;
    if (scan_rf_net_channel > 127)
    {
      stop_scan_rf_network();
      scan_rf_network_public();
    }
  }

  //  if (nrf_interupt_enable == 1)
  //  {
  //    nrf_interupt_enable  = 0;
    nrf_irq_runtime();
  //  }

       if (scan_rf_net_enable == 0)
    {
      nRF.stopListening();
      strcpy(nrf_send_data, "master ping");
      nRF.write(nrf_send_data, strlen(nrf_send_data));
      //nrf_interupt_enable = 1;
      digitalWrite(LED, 0);
      nRF.startListening();
    }

     if (scan_rf_net_enable == 0)
    if (nRF.available(&pipeno))
    {
      nrf_payload_len = nRF.getDynamicPayloadSize();
      if (nrf_payload_len > 0)
      {
        nRF.read(&nrf_receive_data, nrf_payload_len);
        nrf_receive_data[nrf_payload_len] = 0;
        nRF.writeAckPayload(pipeno, nrf_receive_data, nrf_payload_len);
        mqtt_client.publish("/data_prijem", nrf_receive_data);
      }
    }


     //// /thermctl-in/XXXX/rf/scan - 0|1 zapnuti/vypnuti scanovani rf site
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rf/scan");
  if (strcmp(str1, topic) == 0)
  {
    if (atoi(payload) == 1)
    {

      start_scan_rf_network();
    }
    else
    {
      stop_scan_rf_network();
      scan_rf_network_public();
    }
  }
  ///// //thermctl-in/XXXX/rf/stat - statisticke informace
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rf/stat");
  {
    ladeni = ladeni + 1;
  }


  void start_scan_rf_network(void)
  {
  scan_rf_net_enable = 1;
  scan_rf_net_channel = 0;
  for (uint8_t chan = 0; chan < 128; chan++) used_channels[chan] = 0;
  //nRF.setAutoAck(false);
  nRF.startListening();
  nRF.stopListening();
  for (uint8_t pipe = 1; pipe < 6; pipe++)
    nRF.closeReadingPipe(pipe);

  }

  void scan_rf_network(uint8_t channel)
  {
  uint8_t rep = 0;
  if (channel < 128)
  {
    nRF.setChannel(channel);
    while (rep < 128)
    {

      nRF.startListening();
      delayMicroseconds(128);
      nRF.stopListening();
      if (nRF.testCarrier())
        used_channels[channel]++;
      rep++;
    }
  }
  }

  void stop_scan_rf_network(void)
  {
  scan_rf_net_enable = 0;
  scan_rf_net_channel = 0;
  nRF.setChannel(device.nrf_channel);
  }


  void scan_rf_network_public(void)
  {
  char topic[64];
  char payload[128];
  char tmp1[8];
  char tmp2[8];
  strcpy_P(topic, thermctl_header_out);
  strcat(topic, device.nazev);
  strcat(topic, "/rf/used");
  for (uint8_t radek = 0; radek < 8; radek++)
  {
    itoa(radek, tmp1, 16);
    strcpy(payload, "0x");
    strcat(payload, tmp1);
    strcat(payload, " :");
    for (uint8_t sloupec = 0; sloupec < 16; sloupec++)
    {
      itoa(used_channels[(radek * 15) + sloupec], tmp1, 10);
      strcat(payload, tmp1);
      strcat(payload, " ");
    }
    mqtt_client.publish(topic, payload);
  }
  }
*/

/*

  ////////////////////////////////////
  void send_ident(void)
  {
  char str1[14];
  char str2[18];
  strcpy_P(str1, at_device_ident);
  strcpy_P(str2, at_room_control_OK);
  send_at(rsid, str1, str2);
  }

  void send_device_name(void)
  {
  char str1[14];
  char str2[10];
  strcpy_P(str1, at_device_name);
  device_get_name(str2);
  send_at(rsid, str1, str2);
  }
  /////////////////////////////////////////////////////////////
  void send_wire_count(void)
  {
  char str1[10];
  char str2[4];
  itoa(Global_HWwirenum, str2, 10);
  strcpy_P(str1, at_1w_count);
  send_at(rsid, str1, str2);
  }

  /////////////////////////////////////////////////////////////
  void send_count_tds(void)
  {
  char str1[12];
  char str2[4];
  itoa(count_use_tds(), str2, 10);
  strcpy(str1, "tds/count");
  send_at(rsid, str1, str2);
  }
  /////////////////////////////////////////////////////////////
  uint8_t send_temp(uint8_t id)
  {
  uint8_t ret = 0;
  struct_DDS18s20 tds;
  char str1[10];
  char str2[16];

  if (get_tds18s20(id, &tds) == 1)
    if (tds.used == 1)
    {
      if (status_tds18s20[id].online == True)
      {
        int tt = status_tds18s20[id].temp / 10;
        //dtostrf(tt / 10, 4, 2, str1);
        itoa(tt, str1, 10);
        itoa(id, str2, 10);
        strcat(str2, ",");
        strcat(str2, str1);
        strcpy(str1, "tds/temp");
        send_at(rsid, str1, str2);
        ret = 1;
      }
      else
      {
        itoa(id, str2, 10);
        strcat(str2, ",ERR");
        strcpy(str1, "tds/temp");
        send_at(rsid, str1, str2);
        ret = 0;
      }
    }
  return ret;
  }

  uint8_t send_wire_mac(uint8_t id)
  {
  char str1[32];
  char str2[32];
  char str3[4];
  uint8_t ret = 0;
  if (id < Global_HWwirenum)
  {
    itoa(id, str1, 10);
    str2[0] = 0;
    for (uint8_t tmp3 = 0; tmp3 < 8; tmp3++)
    {
      uint8_t tmp2 = w_rom[id].rom[tmp3];
      if (tmp2 < 10) strcat(str2, "0");
      itoa(tmp2, str3, 16);
      strcat(str2, str3);
      if (tmp3 < 7) strcat(str2, ":");
    }
    strcat(str1, ",");
    strcat(str1, str2);
    strcpy(str2, "1w/mac");
    send_at(rsid, str2, str1);
    ret = 1;
  }
  return ret;
  }

  uint8_t send_available_program(void)
  {
  char str3[10];
  itoa(max_program, str3, 10);
  send_at(rsid, "term/avail_prog", str3);
  }

  uint8_t send_thermostat_program(uint8_t id)
  {
  uint8_t ret = 0;
  char str3[18];
  char str2[14];
  if (id >= 0 && id < MAX_THERMOSTAT)
  {
    itoa(thermostat_get_program_id(id), str3, 10);
    itoa(id, str2, 10);
    strcat(str2, ",");
    strcat(str2, str3);
    strcpy_P(str3, at_term_ring_prog);
    send_at(rsid, str3, str2);
    ret = 1;
  }

  return ret;
  }

  uint8_t send_thermostat_output(uint8_t id)
  {
  uint8_t ret = 0;
  char str3[18];
  char str2[14];
  if (id >= 0 && id < MAX_THERMOSTAT)
  {
    itoa(thermostat_get_output(id), str3, 10);
    itoa(id, str2, 10);
    strcat(str2, ",");
    strcat(str2, str3);
    strcpy_P(str3, at_term_ring_output);
    send_at(rsid, str3, str2);
    ret = 1;
  }
  }

  uint8_t send_ring_name(uint8_t id)
  {
  char str3[16];
  char str2[14];
  uint8_t ret = 0;
  if (id >= 0 && id < MAX_THERMOSTAT)
  {
    thermostat_get_name(id, str3);
    itoa(id, str2, 10);
    strcat(str2, ",");
    strcat(str2, str3);
    strcpy(str3, "term/ring/name");
    send_at(rsid, str3, str2);
    ret = 1;
  }
  return ret;
  }


  uint8_t send_know_name(uint8_t id)
  {
  uint8_t ret = 0;
  char tmp1[10];
  char tmp2[12];
  struct_DDS18s20 tds;
  get_tds18s20(id, &tds);
  if (tds.used == 1)
  {
    itoa(id, tmp1, 10);
    strcpy(tmp2, tmp1);
    strcat(tmp2, ",");
    strcat(tmp2,  tds.name);
    strcpy(tmp1, "tds/name");
    send_at(rsid, tmp1, tmp2);
    ret = 1;
  }
  return ret;
  }


  void send_intensive_light(void)
  {
  char str2[8];
  char str1[10];
  itoa(aktual_light, str2, 10);
  strcpy_P(str1, at_term_light);
  send_at(rsid, str1, str2);
  }

  void send_jas_light(void)
  {
  char str2[8];
  char str1[18];
  itoa(((255 - jas) / 15), str2, 10);
  strcpy_P(str1, at_term_brightness);
  send_at(rsid, str1, str2);
  }

  uint8_t send_set_thermostat_temp(uint8_t id)
  {
  char str2[22];
  char str3[12];
  uint8_t ret = 0;
  if (id >= 0 && id < MAX_THERMOSTAT)
  {
    itoa(thermostat_get_mezni(id), str2, 10);
    itoa(id, str3, 10);
    strcat(str3, ",");
    strcat(str3, str2);
    strcpy_P(str2, at_term_ring_threshold);
    send_at(rsid, str2, str3);
    ret = 1;
  }
  return ret;
  }

  void send_thermostat_state(uint8_t id)
  {
  char str3[16];
  char str2[10];
  itoa(thermostat_get_stav(id), str3, 10);
  itoa(id, str2, 10);
  strcat(str2, ",");
  strcat(str2, str3);
  strcpy(str3, "term/ring/state");
  send_at(rsid, str3, str2);
  }

  void send_thermostat_ready(uint8_t id)
  {
  char str3[16];
  char str2[10];
  itoa(thermostat_get_ready(id), str3, 10);
  itoa(id, str2, 10);
  strcat(str2, ",");
  strcat(str2, str3);
  strcpy(str3, "term/ring/ready");
  send_at(rsid, str3, str2);
  }

  void send_thermostat_mode(void)
  {
  char str3[4];
  char str2[10];
  thermostat_get_mode();
  itoa(term_mode, str3, 10);
  strcpy(str2, "term/mode");
  send_at(rsid, str2, str3);
  }

  void send_uptime(void)
  {
  char str3[16];
  char str2[16];
  itoa(uptime, str3, 10);
  strcpy(str2, "device/uptime");
  send_at(rsid, str2, str3);
  }

  uint8_t send_set_offset(uint8_t id)
  {
  uint8_t ret = 0;
  char str2[12];
  char str3[16];
  struct_DDS18s20 tds;
  get_tds18s20(id, &tds);
  if (tds.used == 1)
  {
    ret = 1;
    itoa(tds.offset, str3, 10);
    itoa(id, str2, 10);
    strcat(str2, ",");
    strcat(str2, str3);

    strcpy(str3, "tds/offset");
    send_at(rsid, str3, str2);
  }
  return ret;
  }

  //// odesle prizareni thermostat - tds cidlo
  uint8_t send_thermostat_asociate_tds(uint8_t id)
  {
  char str2[18];
  char str3[14];
  uint8_t ret = 0;
  itoa(thermostat_get_asociate_tds(id), str2, 10);
  itoa(id, str3, 10);
  strcat(str3, ",");
  strcat(str3, str2);
  strcpy(str2, "term/ring/tds");
  send_at(rsid, str2, str3);
  ret = 1;
  return ret;
  }
*/



/*



      if (nRF.begin())
      {
        nRF.setChannel(device.nrf_channel);
        nRF.enableAckPayload();
        nRF.enableDynamicPayloads();
        //nRF.setRetries(50, 50);
        for (uint8_t pipe = 1; pipe < 6; pipe++)
          nRF.closeReadingPipe(pipe);

        nRF.openWritingPipe(addresses[1]);        // Both radios listen on the same pipes by default, but opposite addresses
        nRF.openReadingPipe(1, addresses[0]);     // Open a reading pipe on address 0, pipe 1


        nRF.startListening();                       // Start listening
        nRF.setPALevel(RF24_PA_HIGH);
        nRF.writeAckPayload(1, &devnull, sizeof(devnull));        // Pre-load an ack-paylod into the FIFO buffer for pipe 1
        //radio.printDetails();
      }
      else
      {
        strcpy(tmp1, "NRFERR");
        show_direct(tmp1);
        delay(1000);
      }
   /
*/
