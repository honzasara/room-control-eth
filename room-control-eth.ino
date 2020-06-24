#include "UserSettings.h"
#include <PID_v1.h>
#include "saric_utils.h"

#include <NTPClient.h>

#include <ArduinoJson.h>

#include <SPI.h>
#include <PubSubClient.h>
#include <UIPEthernet.h>

#include <avr/wdt.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"

#include "printf.h"


#include <avr/pgmspace.h>
#include "font_alfa.h"

#include <avr/wdt.h>
#include <avr/boot.h>

#include <RTClib.h>

#include <EEPROM.h>
#include "saric_thermostat.h"
#include "saric_tds_function.h"
#include "saric_mqtt_network.h"
#include "saric_nrf.h"

#define RS_MODE_SERIAL 0
#define RS_MODE_AT 1
#define RS_MODE_MODBUS 2

#define RS_NONE 255
#define MAX_RS_DEVICE 32
#define RS_SEND_PAUSE 50

#define WD_FREE 0
#define WD_MQTT_LOOP 1
#define WD_MAIN_LOOP 2
#define WD_1SEC 3
#define WD_05SEC 4
#define WD_2MSEC 5
#define WD_TIMER2 6
#define WD_TIMER3 7

#define STATUS_BIT_HEAT_OR_COOL 0

#define SELFTEST_ERR_RTC 0
#define SELFTEST_ERR_NTP 1
#define SELFTEST_NRF_SCAN 4
#define SELFTEST_MQTT_LINK 5
#define SELFTEST_ETH_LINK 6
#define SELFTEST_RESTART_NEEDED 7

#define MAX_SVETEL  8
#define LIGHT_DEFAULT 255
#define LIGHT_FREE  255

#define TYPE_FUNCTION 0
#define TYPE_STATIC 1
#define TYPE_DYNAMIC 2
#define TYPE_FUNCTION_DYNAMIC 3



#define MAX_RTDS 5

#define TDS_MEMORY_MAP_TDS 0
#define TDS_MEMORY_MAP_RTDS 16







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




typedef struct StructRSDevice
{
  uint8_t last_seen;
  uint8_t type;
};

#define bootloader_tag 0
#define time_offset 1
#define set_default_values 90
#define my_sense 91
#define my_default_ring 92

#define my_serial_mode 94
#define my_serial_speed 95
#define my_jas_disp 96
#define freex 97
#define my_jas_key 98
#define my_rs_id 99








#define remote_tds_name0  1450
#define remote_tds_name1  1460
#define remote_tds_name2  1470
#define remote_tds_name3  1480
#define remote_tds_name4  1490



#define light_output_0 1516
#define light_output_7 1523





///define next 1534

//////////////////////////////////////////////////////////////
RTC_DS1307 rtc;
DateTime now;

EthernetClient ethClient;
EthernetUDP udpClient;
PubSubClient mqtt_client(ethClient);


RF24 radio(NRF_CE, NRF_CS);
RF24Network network(radio);
RF24Mesh mesh(radio, network);






uint8_t key = 0;
uint8_t key_now = 0;
uint8_t key_press = 0;
uint8_t key_release = 0;
uint8_t key_press_cnt[8];
uint8_t led, led_old, led_blink_1, led_blink_2, led_blink_old;
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
uint16_t tisice = 0;
uint16_t stovky = 0;
uint16_t desitky = 0;
uint16_t jednotky = 0;
uint8_t tecka = 0;
long int milis = 0;
long int milis_05s = 0;
long int milis_1s = 0;
long int milis_10s = 0;
long int milis_1ms = 0;
long int milis_key = 0;
uint16_t light_curr = 0;
uint8_t delay_show_menu = 0;
uint8_t start_at = 0;
uint8_t re_at = 0;
uint8_t r_id = 0;
char uart_recv[UART_RECV_MAX_SIZE];

uint8_t default_ring = 0;

unsigned long load = 0;
unsigned long load_max = 0;
unsigned long load_min = 0xffffffff;

StructRSDevice rs_device[MAX_RS_DEVICE];
uint8_t rs_find = 0;
uint8_t rs_death_time = 0;
uint8_t serial_mode = 0;
uint8_t serial_max_used_tx = 0;
uint8_t rs_start_find_device = 0;

uint16_t light_min = 0;
uint16_t light_max = 0;
uint8_t auto_jas = 0;



uint8_t keyboard_event = 0;

uint8_t selftest_data = 0;



uint8_t watchdog_state = 0;

uint8_t use_tds = 0;
uint8_t use_rtds = 0;
uint8_t use_prog = 0;
uint8_t use_light = 0;
uint8_t use_light_curr = 0;
uint8_t use_ring = 0;
int remote_tds[5];
int remote_tds_last_update[5];
uint8_t light_value[MAX_SVETEL];
uint8_t light_state[MAX_SVETEL];

uint8_t last_output_update[MAX_THERMOSTAT];



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
const char global_time_offset[] PROGMEM = "global/time/ntp_offset";
const char thermctl_header_in[] PROGMEM  = "/thermctl-in/";
const char thermctl_header_out[] PROGMEM  = "/thermctl-out/";

const char termbig_header_in[] PROGMEM  = "/termbig-in/";
const char termbig_header_out[] PROGMEM  = "/termbig-out/";

const char lightctl_header_in[] PROGMEM  = "/lightctl-in/";
const char lightctl_header_out[] PROGMEM  = "/lightctl-out/";

const char thermctl_subscribe[] PROGMEM = "/ctl/thermctl/subscribe";
const char termbig_subscribe[] PROGMEM = "/ctl/termbig/subscribe";

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

const char title_item_set_manual_mode[] PROGMEM = "nR MOD";
const char title_item_set_manual_mode_heat[] PROGMEM = "r TOP ";
const char title_item_set_manual_mode_cool[] PROGMEM = "r CHL ";

const char title_item_selftest[] PROGMEM = "-STAV-";
const char title_item_selftest_text[] PROGMEM = "SF-$$$";

const char title_item_pwm_power[] PROGMEM = "VYKON-";
const char title_item_pwm_power_text[] PROGMEM = "V--$$$";

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
RootMenu setup_select_mode;
RootMenu menu_show_stav;
RootMenu menu_pwm_power_show;

ItemMenu item_set_jas,  item_select_prog, item_select_ring, item_select_mode, item_select_reset, item_select_osvetleni, item_stav, item_pwm_power, item_zpet;
ItemMenu item_set_jas_dyn, item_set_jas_auto;
ItemMenu item_set_man_temp;
ItemMenu item_set_select_prog;
ItemMenu item_set_select_ring;
ItemMenu item_set_select_osvetleni;
ItemMenu item_set_select_mode;
ItemMenu item_stav_show;
ItemMenu item_pwm_power_show;

ItemMenu item_set_select_mode_heat, item_set_select_mode_cool;













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
  delay_show_menu = 0;
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
/////////////////////////////////////
void set_select_ring(uint8_t cnt, uint8_t *ret_data, char *text)
{
  uint8_t fix = 0;
  char tmp1[8];
  char c[4];
  uint8_t findit = 0;
  uint8_t ring = 0;
  for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
    if (thermostat_ring_get_active(idx) != RING_FREE)
    {
      if (fix == cnt)
      {
        findit = 1;
        ring = idx;
        break;
      }
      fix++;
    }
  if (findit == 1)
  {
    *ret_data = ring;
    strcpy(tmp1, "RING");
    if (ring < 10) strcat(tmp1, "-");
    itoa(ring, c, 10);
    strcat(tmp1, c);
  }
  if (findit != 1)
  {
    *ret_data = RING_FREE;
    strcpy(tmp1, "RING-E");
  }
  show_direct(tmp1);
}
///////////////////////////////////////
//////////////////////////////////////
void selftest_show(uint8_t cnt, uint8_t *ret_data, char *text)
{
  itoa(selftest_data, text, 10);
}

void termostat_pwm_power_show(uint8_t cnt, uint8_t *ret_data, char *text)
{
  itoa( thermostat_ring_get_power(default_ring), text, 10);
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
/////////////////////////////
/// reverzni funkce, index menu ring vraci na skutecny index ringu
uint8_t rev_get_show_ring(uint8_t cnt)
{
  uint8_t index = 0;
  for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
    if (thermostat_ring_get_active(idx) != RING_FREE)
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
uint8_t count_use_active_rings(void)
{
  uint8_t cnt = 0;
  for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
  {
    if (thermostat_ring_get_active(idx) != RING_FREE) cnt++;
  }
  return cnt;
}
////
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
uint8_t send_at(uint8_t id, char *cmd, char *args)
{
  uint8_t ret = 0;
  char tmp1[MAX_TEMP_BUFFER];
  char tmp2[8];
  if (rs_death_time == 0)
  {
    tmp1[0] = 0;
    tmp2[0] = 0;
    digitalWrite(rs485, HIGH);
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
    ret = 1;
  }
  else
    ret = 0;
  return ret;
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t time_get_offset(void)
{
  return EEPROM.read(time_offset);
}

void time_set_offset(uint8_t offset)
{
  EEPROM.write(time_offset, offset);
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void send_know_device(void)
{
  char str_topic[64];
  char payload[64];
  for (uint8_t idx = 0; idx < MAX_KNOW_MQTT; idx++)
  {
    if (know_mqtt[idx].type != TYPE_FREE)
    {
      itoa(know_mqtt[idx].type, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "know_mqtt_device", idx, "type", payload);
      itoa(know_mqtt[idx].last_update, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "know_mqtt_device", idx, "last_update", payload);
      strcpy(payload, know_mqtt[idx].device);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "know_mqtt_device", idx, "device", payload);
    }
  }
}



void send_device_status(void)
{
  char str_topic[64];
  char payload[64];
  if (mqtt_client.connected())
  {
    strcpy(str_topic, "status/uptime");
    itoa(uptime, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    strcpy(str_topic, "status/brigthness");
    itoa(light_curr, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    strcpy(str_topic, "status/light");
    itoa(jas_disp, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    strcpy(str_topic, "status/auto_brigthness");
    itoa(auto_jas, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    strcpy(str_topic, "status/load_min");
    itoa(load_min, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    strcpy(str_topic, "status/load_max");
    itoa(load_max, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    load_max = 0;
    load_min = 0xffffffff;
    ///
    strcpy(str_topic, "status/default_ring");
    itoa(default_ring, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    ///
    strcpy(str_topic, "status/keyborad/event");
    itoa(keyboard_event, payload, 10);
    keyboard_event = 0;
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);

    strcpy(str_topic, "status/selftest");
    itoa(selftest_data, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);

    strcpy(str_topic, "status/rtds/count");
    itoa(use_rtds, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);

    strcpy(str_topic, "status/light/count");
    itoa(use_light_curr, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);

    strcpy(str_topic, "rs485/mode");
    itoa(serial_mode, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);

    strcpy(str_topic, "rs485/speed");
    itoa(serial_get_speed(), payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);

    strcpy(str_topic, "rs485/queue/tx");
    itoa(serial_max_used_tx, payload, 10);
    send_mqtt_general_payload(&mqtt_client, str_topic, payload);
    serial_max_used_tx = 0;

    itoa(time_get_offset(), payload, 10);
    send_mqtt_general_payload(&mqtt_client, "time/ntp_offset", payload);

  }
}
///
//// /thermctl_out/XXXXX/1wire/count
//// /thermctl_out/XXXXX/1wire/IDcko/rom
void send_mqtt_onewire(void)
{
  char str_topic[64];
  char hostname[10];
  char payload[64];
  char tmp1[4];
  itoa(Global_HWwirenum, payload, 10);
  send_mqtt_general_payload(&mqtt_client, "1wire/count", payload);
  for (uint8_t i = 0; i < Global_HWwirenum; i++)
  {
    //sariccreateString(payload, ':', w_rom[i].rom, 8, 16);
    send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "1wire", i, "rom", payload);
    ///
    //saricitoa(w_rom[i].assigned_ds2482, payload, 10);
    send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "1wire", i, "assigned", payload);
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
  long avg = 0;
  for (uint8_t id = 0; id < HW_ONEWIRE_MAXROMS; id++)
    if (get_tds18s20(id, &tds) == 1)
      if (tds.used == 1) if (status_tds18s20[id].online == True)
        {
          tt = status_tds18s20[id].temp / 10;
          itoa(tt, payload, 10);
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "temp", payload);
          avg = 0;
          for (uint8_t c = 0; c < MAX_AVG_TEMP; c++) avg = avg + status_tds18s20[id].average_temp[c];
          avg = avg / MAX_AVG_TEMP;
          avg = avg / 10;
          itoa(avg, payload, 10);
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "temp_avg", payload);

          strcpy(payload, tds.name);
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "name", payload);
          tt = tds.offset;
          itoa(tt, payload, 10);
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "offset", payload);
          tt = status_tds18s20[id].online;
          itoa(tt, payload, 10);
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "online", payload);
          payload[0] = 0;
          createString(payload, ':', tds.rom, 8, 16);
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "rom", payload);
          tt = tds.period;
          itoa(tt, payload, 10);
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "period", payload);

          tt = (uptime & 0xff) - status_tds18s20[id].period_now;
          itoa(tt, payload, 10);
          send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "tds", id, "start_at", payload);
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
    if (thermostat_ring_get_active(idx) != RING_FREE)
    {
      thermostat_ring_get_name(idx, payload);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "name", payload);
      itoa(thermostat_ring_get_active(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "active", payload);
      itoa(thermostat_ring_get_program_id(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "program", payload);
      itoa(thermostat_ring_get_mezni(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "threshold", payload);
      itoa(thermostat_ring_get_mode(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "mode", payload);
      convert_mode_text(thermostat_ring_get_mode(idx), payload);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "text_mode", payload);
      itoa(thermostat_ring_get_state(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "status", payload);
      tdsid = thermostat_ring_get_asociate_tds(idx);
      if (tdsid >= TDS_MEMORY_MAP_TDS && tdsid < TDS_MEMORY_MAP_RTDS)
      {
        itoa(tdsid, payload, 10);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "tds", payload);
      }
      if (tdsid >= TDS_MEMORY_MAP_RTDS && tdsid < 127)
      {
        itoa(tdsid - TDS_MEMORY_MAP_RTDS, payload, 10);
        send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "rtds", payload);
      }
      itoa(thermostat_ring_get_output(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "output", payload);

      itoa(thermostat_ring_get_status_data(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "status_bites", payload);

      itoa(last_output_update[idx], payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "ring", idx, "output_update", payload);
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
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "prog", idx, "name", payload);
      itoa(act, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "prog", idx, "active", payload);
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
          send_mqtt_message_prefix_id_idx_topic_payload(&mqtt_client, "prog_interval", idx, progid, "time", payload);
          itoa(active, tmp1, 10);
          strcpy(payload, tmp1);
          send_mqtt_message_prefix_id_idx_topic_payload(&mqtt_client, "prog_interval", idx, progid, "active", payload);
          itoa(thermostat_program_get_threshold(idx, progid), tmp1, 10);
          strcpy(payload, tmp1);
          send_mqtt_message_prefix_id_idx_topic_payload(&mqtt_client, "prog_interval", idx, progid, "threshold", payload);
          itoa(thermostat_program_get_week(idx, progid), payload, 10);
          send_mqtt_message_prefix_id_idx_topic_payload(&mqtt_client, "prog_interval", idx, progid, "week", payload);
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
void mqtt_send_pid_variable(uint8_t idx)
{
  char payload[32];
  dtostrf(thermostat_get_pid_p(idx), 7, 2, payload);
  send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "pid", idx, "kp", payload);
  dtostrf(thermostat_get_pid_i(idx), 7, 2, payload);
  send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "pid", idx, "ki", payload);
  dtostrf(thermostat_get_pid_d(idx), 7, 2, payload);
  send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "pid", idx, "kd", payload);
  itoa(thermostat_get_pid_time(idx), payload, 10);
  send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "pid", idx, "time", payload);

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
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "rtds", idx, "name", payload);
      itoa(active, payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "rtds", idx, "active", payload);
      itoa(remote_tds[idx], payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "rtds", idx, "temp", payload);
      itoa(remote_tds_last_update[idx], payload, 10);
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "rtds", idx, "last_update", payload);
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
      send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "light", idx, "output", payload);
    }
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
    strcpy_P(str_topic, termbig_header_in);
    strcat(str_topic, "power-output/");
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
    strcpy_P(str_topic, termbig_header_in);
    strcat(str_topic, "power-output/");
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

////

void send_mesh_status(void)
{
  char str_topic[64];
  char payload[64];

  strcpy(str_topic, "rf/mesh/id");
  itoa(mesh.getNodeID(), payload, 10);
  send_mqtt_general_payload(&mqtt_client, str_topic, payload);

  strcpy(str_topic, "rf/mesh/neighbour/count");
  itoa(mesh.addrListTop, payload, 10);
  send_mqtt_general_payload(&mqtt_client, str_topic, payload);

  for (uint8_t idx = 0; idx < mesh.addrListTop; idx++)
  {
    itoa(mesh.addrList[idx].nodeID, payload, 10);
    send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "rf/mesh/neighbour", idx, "id", payload);
    itoa(mesh.addrList[idx].address, payload, 10);
    send_mqtt_message_prefix_id_topic_payload(&mqtt_client, "rf/mesh/neighbour", idx, "address", payload);
  }


}
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
  if (strcmp(str2, "cool") == 0) mode = TERM_MODE_CLIMATE_MAX;
  if (strcmp(str2, "fan_only") == 0) mode = TERM_MODE_FAN;
  return mode;
}

void convert_mode_text(uint8_t mode, char *str)
{
  if (mode == TERM_MODE_OFF)   strcpy(str, "off");
  if (mode == TERM_MODE_MAX)   strcpy(str, "heat");
  if (mode == TERM_MODE_MAN_HEAT)   strcpy(str, "manual");
  if (mode == TERM_MODE_PROG)   strcpy(str, "auto");
  if (mode == TERM_MODE_CLIMATE_MAX)   strcpy(str, "cool");
  if (mode == TERM_MODE_FAN)   strcpy(str, "fan_only");
}
///
///


///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  char str1[64];
  const char tmp1[16];
  char tmp2[16];
  static char my_payload[128];
  boolean ret = 0;
  uint8_t cnt = 0;
  uint8_t id = 0;
  uint8_t id_interval = 0;
  struct_DDS18s20 tds;
  char *pch;
  uint8_t active;
  NTPClient timeClient(udpClient);
  for (uint8_t j = 0; j < 128; j++) my_payload[j] = 0;
  ////

  mqtt_receive_message++;
  strncpy(my_payload, (char*) payload, length);

  strcpy_P(str1, termbig_subscribe);
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    know_mqtt_create_or_update(my_payload, TYPE_TERMBIG);
  }

  strcpy_P(str1, thermctl_subscribe);
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    know_mqtt_create_or_update(my_payload, TYPE_THERMCTL);
  }

  ////
  //// /thermctl-in/global/time/set - nastaveni casu. payload json
  strcpy_P(str1, thermctl_header_in);
  strcat_P(str1, global_time_set);
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, (char*) payload, length);
    deserializeJson(doc, my_payload);
    JsonObject root = doc.as<JsonObject>();
    if (root.containsKey("year") && root.containsKey("month") && root.containsKey("month") && root.containsKey("hour") && root.containsKey("minute") && root.containsKey("second"))
      rtc.adjust(DateTime(root["year"], root["month"], root["day"], root["hour"], root["minute"], root["second"]));
  }
  //// /thermctl-in/global/time/ntp - jednorazova aktualizace casu z ntp serveru
  strcpy_P(str1, thermctl_header_in);
  strcat_P(str1, global_time_ntp);
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    if (ntp_update(&timeClient, &rtc, time_get_offset()) == 1)
      cbi(selftest_data, SELFTEST_ERR_NTP);
    else
      sbi(selftest_data, SELFTEST_ERR_NTP);
  }
  //// /termbig-in/global/time/offset - nastaveni offsetu casu
  strcpy_P(str1, thermctl_header_in);
  strcat_P(str1, global_time_offset);
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    time_set_offset(atoi(my_payload));
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
    mqtt_process_message++;
    uint16_t t_led_blink = atoi(my_payload);
    led_blink_1 = t_led_blink;
    led_blink_2 = t_led_blink >> 8;
  }
  ///. nastavi intenzitu jasu dispalye
  //// /thermctl-in/XXXX/led/disp
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/led/disp");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    cnt = atoi(my_payload);
    cnt = cnt & 0b00001111;
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
    mqtt_process_message++;
    cnt = atoi(my_payload);
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
    mqtt_process_message++;
    keyboard_set_sense(atoi(my_payload));
    keyboard_apply_sense();
  }
  ///
  //// /thermctl-in/XXXX/rf/scan - 0|1 zapnuti/vypnuti scanovani rf site
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rf/scan");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    if (atoi(my_payload) == 1)
    {
      start_scan_rf_network(&radio);
      selftest_set_0(SELFTEST_NRF_SCAN);
    }
    else
    {
      stop_scan_rf_network(&radio);
      scan_rf_network_public(&mqtt_client);
      selftest_clear_0(SELFTEST_NRF_SCAN);
    }
  }
  ///// //thermctl-in/XXXX/rf/stat - statisticke informace
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rf/stat");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    scan_rf_net_enable = 2;
  }

  //// /thermctl-in/XXXX/rf/set/channel
  //// nastaveni vysilacihi/prijmaciho kanalu
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rf/set/channel");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
  }



  //// /thermctl-in/XXXX/tds/associate - asociace do tds si pridam mac 1wire - odpoved je pod jakem ID to mam ulozeno
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/tds/associate");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    id = atoi(my_payload);
    if (tds_associate(id) == 0)
      log_error(&mqtt_client, "tds/associate bad id");
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
    mqtt_process_message++;
    //strncpy(str2, payload, length);
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
        if ((cnt == 1) && (strcmp(pch, "name") == 0)) tds_set_name(id, my_payload);
        if ((cnt == 1) && (strcmp(pch, "offset") == 0)) tds_set_offset(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "period") == 0)) tds_set_period(id, atoi(my_payload));
      }
      else
      {
        log_error(&mqtt_client, "tds/set bad id");
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
    mqtt_process_message++;
    //strncpy(str2, payload, length);
    id = atoi(my_payload);
    if (id < HW_ONEWIRE_MAXROMS)
      tds_set_clear(id);
    else
      log_error(&mqtt_client, "tds/clear bad id");
  }
  ////
  ////
  //// thermctl-in/XXXXX/rtds/set/IDX/name - 8 znaku nastavi a udela prihlaseni
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rtds/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
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
            remote_tds_set_name(id, 1, my_payload);
            remote_tds_subscibe_topic(id);
          }
        }
      }
      else
      {
        log_error(&mqtt_client, "rtds/set bad id");
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
    mqtt_process_message++;
    //strncpy(str2, payload, length);
    id = atoi(my_payload);
    if (id < MAX_RTDS)
    {
      remote_tds_unsubscibe_topic(id);
      remote_tds_set_name(id, 0, "");
    }
    else
    {
      log_error(&mqtt_client, "rtds/clear bad id");
    }
  }
  //// ziska nastaveni remote_tds
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rtds/get");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    send_mqtt_remote_tds_status();
  }
  ////
  //// rtds/NAME - hodnota, kde NAME je nazev cidla
  strcpy(str1, "/rtds/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
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
        remote_tds[idx] = atoi(my_payload);
        remote_tds_last_update[idx] = 0;
      }
    }
  }
  ///
  //// thermctl-in/XXXXX/prog/set/IDX/name  - 8znaku
  //// thermctl-in/XXXXX/prog/set/IDX/active  - 0-off, 1-heat, 2-cool,3.....
  //// thermctl-in/XXXXX/prog_interval/set/IDX/IDcko/theshold - pro jednotlive casove useky ruzne teploty
  //// thermctl-in/XXXXX/prog_interval/set/IDX/IDcko/active - pro jednotlivy usek povoleni zakazani
  //// thermctl-in/XXXXX/prog_interval/set/IDX/IDcko/time - nastavi cas pro jednotlive intervaly
  //// thermctl-in/XXXXX/prog/clear - IDX
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/prog/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
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
        if ((cnt == 1) && (strcmp(pch, "name") == 0))  thermostat_program_set_name(id, my_payload);
        if ((cnt == 1) && (strcmp(pch, "active") == 0))  thermostat_program_set_active(id, atoi(my_payload));
      }
      else
      {
        log_error(&mqtt_client, "prog/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }
  ////
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/prog/clear");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
    id = atoi(my_payload);
    if (id < AVAILABLE_PROGRAM)
    {
      strcpy(tmp2, "PROG");
      thermostat_program_set_name(id, tmp2);
      thermostat_program_set_active(id, 0);
      for (uint8_t progid = 0; progid < MAX_PROGRAM_INTERVAL; progid++)
      {
        thermostat_program_set_time(id, progid, 0, 0, 0, 0, 0);
        thermostat_program_set_threshold(id, progid, 220);
        thermostat_program_set_week(id, progid, 0);
      }
      for (uint8_t tix = 0; tix < MAX_THERMOSTAT; tix++)
      {
        if (thermostat_ring_get_program_id(tix) == id)
        {
          thermostat_ring_set_program_id(tix, PROG_FREE);
        }
      }
    }
    else
    {
      log_error(&mqtt_client, "prog/clear bad id");
    }
  }
  /////
  /////
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/prog_interval/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
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
        if ((cnt == 2) && (strcmp(pch, "active") == 0))  thermostat_program_set_interval_active(id, id_interval, atoi(my_payload));
        if ((cnt == 2) && (strcmp(pch, "threshold") == 0))  thermostat_program_set_threshold(id, id_interval , atoi(my_payload));
        if ((cnt == 2) && (strcmp(pch, "time") == 0)) thermostat_program_set_parse_interval(id, id_interval, my_payload);
      }
      else
      {
        log_error(&mqtt_client, "prog_interval/set bad id");
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
    mqtt_process_message++;
    //strncpy(str2, payload, length);
    default_ring = atoi(my_payload);
    set_default_ring(default_ring);
  }
  ///
  //// thermctl-in/XXXXX/ring/get/IDcko/pid
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/ring/get/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    //strncpy(str2, payload, length);
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
      if (cnt == 0) id = atoi(my_payload);
      if (id < MAX_THERMOSTAT)
      {
        if ((cnt == 1) && (strcmp(pch, "pid") == 0))  mqtt_send_pid_variable(id);
      }
      else
      {
        log_error(&mqtt_client, "ring/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }
  ///
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
  //// thermctl-in/XXXXX/ring/set/IDcko/pid_time
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/ring/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    //strncpy(str2, payload, length);
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
        if ((cnt == 1) && (strcmp(pch, "name") == 0)) thermostat_ring_set_name(id, my_payload);
        if ((cnt == 1) && (strcmp(pch, "program") == 0))
          if (atoi(my_payload) < AVAILABLE_PROGRAM)
            thermostat_ring_set_program_id(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "threshold-f") == 0)) thermostat_ring_set_mezni(id, atof(my_payload) * 10); /// thereshold hodnota presne ve floatu * 10
        if ((cnt == 1) && (strcmp(pch, "threshold") == 0)) thermostat_ring_set_mezni(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "text_mode") == 0))
        {
          active = convert_text_mode(my_payload);
          thermostat_ring_set_mode(id, active);
          if (active == TERM_MODE_MAN_HEAT)
            thermostat_ring_update_bites(id, STATUS_BIT_HEAT_OR_COOL, 0);
          if (active == TERM_MODE_MAN_COOL)
            thermostat_ring_update_bites(id, STATUS_BIT_HEAT_OR_COOL, 1);
        }
        if ((cnt == 1) && (strcmp(pch, "mode") == 0))
        {
          active = atoi(my_payload);
          thermostat_ring_set_mode(id, active);
          if (active == TERM_MODE_MAN_HEAT)
            thermostat_ring_update_bites(id, STATUS_BIT_HEAT_OR_COOL, 0);
          if (active == TERM_MODE_MAN_COOL)
            thermostat_ring_update_bites(id, STATUS_BIT_HEAT_OR_COOL, 1);
        }
        if ((cnt == 1) && (strcmp(pch, "tds") == 0)) thermostat_ring_set_asociate_tds(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "rtds") == 0)) thermostat_ring_set_asociate_tds(id, atoi(my_payload) + TDS_MEMORY_MAP_RTDS);
        if ((cnt == 1) && (strcmp(pch, "active") == 0)) thermostat_ring_set_active(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "output") == 0)) thermostat_ring_set_output(id, atoi(my_payload));
        if ((cnt == 1) && (strcmp(pch, "pid_kp") == 0)) thermostat_ring_pid_set_kp(id, atof(my_payload));
        if ((cnt == 1) && (strcmp(pch, "pid_ki") == 0)) thermostat_ring_pid_set_ki(id, atof(my_payload));
        if ((cnt == 1) && (strcmp(pch, "pid_kd") == 0)) thermostat_ring_pid_set_kd(id, atof(my_payload));
        if ((cnt == 1) && (strcmp(pch, "pid_time") == 0)) thermostat_ring_pid_set_time(id, atoi(my_payload));
      }
      else
      {
        log_error(&mqtt_client, "ring/set bad id");
      }
      pch = strtok (NULL, "/");
      cnt++;
    }

  }
  /////
  //// nastaveni ringu do vychoziho stavu
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/ring/clear");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
    id = atoi(my_payload);
    if (id < MAX_THERMOSTAT)
    {
      thermostat_ring_clear(id);
    }
    else
    {
      log_error(&mqtt_client, "ring/clear bad id");
    }
  }
  ////
  //// ziskani nastaveni site
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/network/get/config");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    send_network_config(&mqtt_client);
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
    mqtt_process_message++;
    cnt = 0;
    for (uint8_t f = strlen(str1); f < strlen(topic); f++)
    {
      str1[cnt] = topic[f];
      str1[cnt + 1] = 0;
      cnt++;
    }
    cnt = setting_network(str1, my_payload);
    if (cnt == 1)
    {
      save_setup_network();
      sbi(selftest_data, SELFTEST_RESTART_NEEDED);
    }
    if (cnt == 2)
    {
      sbi(selftest_data, SELFTEST_RESTART_NEEDED);
    }
  }

  //// priradi k menu pro rizeni intezity svetla vystup
  //// thermctl-in/XXXXX//light/set/IDX/output
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/light/set/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
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
      if ((cnt == 1) && (strcmp(pch, "output") == 0)) light_set_output(id, atoi(my_payload));
      if ((cnt == 1) && (strcmp(pch, "value") == 0)) light_value[id] = atoi(my_payload);
      pch = strtok (NULL, "/");
      cnt++;
    }
  }
  ///
  /// zpetna vazba od vystupu
  strcpy_P(str1, termbig_header_out);
  strcat(str1, "output/");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
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
      if (cnt == 1)
      {
        for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
          if (thermostat_ring_get_output(idx) == id)
          {
            if (strcmp(pch, "pwm")) last_output_update[id] = 0;
            if (strcmp(pch, "state")) last_output_update[id] = 0;
            break;
          }
      }
      pch = strtok (NULL, "/");
      cnt++;
    }
  }
  ///
  //// /thermctl-in/rs485/set/mode
  /// serial-half-duplex generic
  /// at
  /// modbus
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rs485/set/mode");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
    serial_set_mode(atoi(my_payload));
  }
  //// /thermctl-in/rs485/set/speed
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rs485/set/speed");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
    serial_set_speed(atoi(my_payload));
    Serial.end();
    Serial.begin(serial_get_speed() * 9600);
  }

  //// /thermctl-in/rs485/send

  //// /thermctl-in/rs485/find_at_devices
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/rs485/find_at_devices");
  if (strncmp(str1, topic, strlen(str1)) == 0)
  {
    mqtt_process_message++;
    rs_start_find_device = 1;
    rs_find = 0;
    for (uint8_t idx = 0; idx < MAX_RS_DEVICE; idx++)
    {
      rs_device[idx].last_seen = RS_NONE;
      rs_device[idx].type = RS_NONE;
    }
  }


  //// thermctl-in/XXXXX/reload
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/reload");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    log_error(&mqtt_client, "reload ..... ");
    resetFunc();
  }

  //// thermctl-in/XXXXX/reload
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/bootloader");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    log_error(&mqtt_client, "bootloader ..... ");
    EEPROM.write(bootloader_tag, 255);
    wdt_enable(WDTO_1S);
    while (1);
  }

  //// /thermctl-in/XXXXX/reset_default
  strcpy_P(str1, thermctl_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/default");
  if (strcmp(str1, topic) == 0)
  {
    mqtt_process_message++;
    //strncpy(str2, payload, length);
    EEPROM.write(set_default_values, atoi(my_payload));
  }
}
////////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
byte mqtt_reconnect(void)
{
  char nazev[10];
  char topic[26];
  byte ret = MQTT_DISCONNECTED;
  ///  /thermctl/xxxxxxxx/#
  ///  /thermctl/global/#
  if (!mqtt_client.connected())
    if (mqtt_client.connect(nazev))
    {
      device_get_name(nazev);
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

      strcpy_P(topic, thermctl_subscribe);
      mqtt_client.subscribe(topic);

      strcpy_P(topic, termbig_subscribe);
      mqtt_client.subscribe(topic);
      /// zpetna vazba od vystupu
      strcpy_P(topic, termbig_header_out);
      mqtt_client.subscribe(topic);
    }
  ret = mqtt_client.state();
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


/////////////////////////////////////////////////////////////////////////////////////////////////////


void selftest(void)
{
  if (!rtc.isrunning())
    sbi(selftest_data, SELFTEST_ERR_RTC);
  else
    cbi(selftest_data, SELFTEST_ERR_RTC);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**********************************************************************************************************************************/


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
      te = thermostat_running(prg, &thresh, now.hour(), now.minute(), now.dayOfTheWeek());
      switch (te)
      {
        case PROG_FREE:
          {
            tmode = TERM_MODE_OFF;
            /// blikam s ledkou protoze program neni aktivni, nebo program neni vybran
            blink_led_set(LED_PROG_I, 3);
            blink_led_set(LED_ERR_I, 1);
            thermostat_ring_set_state(tix, TERM_STAV_STOP);
            thermostat_ring_set_power(tix, 0);
          }
        case PROG_NO_INTERVAL:
          {
            tmode = TERM_MODE_OFF;
            /// blikam s ledkou protoze program neni aktivni, nebo program neni vybran
            blink_led_set(LED_PROG_I, 1);
            thermostat_ring_set_state(tix, TERM_STAV_STOP);
            thermostat_ring_set_power(tix, 0);
            break;
          }
        case PROG_ACTIVE:
          {
            /// blikaci ledku vypnu, program aktivni
            blink_led_off(LED_PROG_I);
            blink_led_off(LED_ERR_I);
            /// zde si ziskam rezim vypnuto, topi, chladi, ventilator, podle toho si prenastavim ring mode, potrebuji nastavit pid regulator
            thermostat_ring_set_state(tix, TERM_STAV_BEZI);
            act = thermostat_program_get_active(prg);
            if (act == 1) tmode = TERM_MODE_MAN_HEAT;
            if (act == 2) tmode = TERM_MODE_MAN_COOL;
            if (act == 3) tmode = TERM_MODE_FAN;
            break;
          }
        default: break;
      }
    }

    if (tmode == TERM_MODE_MAN_HEAT)
    {
      //pidy[tix].SetControllerDirection(DIRECT);
      thermostat_pid_setdirection_direct(tix);
    }

    if (tmode == TERM_MODE_MAN_COOL)
    {
      //pidy[tix].SetControllerDirection(REVERSE);
      thermostat_pid_setdirection_reverse(tix);
    }

    if (tdsid >= TDS_MEMORY_MAP_TDS && tdsid < TDS_MEMORY_MAP_RTDS)
    {
      if (get_tds18s20(tdsid, &tds) == 1)
        if (tds.used == 1 && status_tds18s20[tdsid].online == True)
        {
          thermostat_pid_input(tix, status_tds18s20[tdsid].temp / 100);
          thermostat_pid_setpoint(tix, thresh);
          pwm = thermostat_pid_output(tix);
          thermostat_ring_set_power(tix, pwm);
        }
        else
        {
          tmode = TERM_MODE_ERR;
          pwm = 0;
          thermostat_ring_set_power(tix, pwm);
        }
    }

    if (tdsid >= TDS_MEMORY_MAP_RTDS && tdsid < 127)
    {
      act = tdsid - TDS_MEMORY_MAP_RTDS;
      remote_tds_get_active(act , &active);
      if (active == 1 && remote_tds_last_update[act] < 180)
      {
        thermostat_pid_input(tix, remote_tds[act] / 10);
        thermostat_pid_setpoint(tix, thresh);
        pwm = thermostat_pid_output(tix);
        thermostat_ring_set_power(tix, pwm);
      }
      else
      {
        tmode = TERM_MODE_ERR;
        pwm = 0;
        thermostat_ring_set_power(tix, pwm);
      }
    }

    if (tmode == TERM_MODE_MAN_HEAT)
    {
      if (pwm < 25)
        blink_led_set(LED_UP_I, 1);
      else
        blink_led_off(LED_UP_I);
    }

    if (tmode == TERM_MODE_MAN_COOL)
    {
      if (pwm < 25)
        blink_led_set(LED_DOWN_I, 1);
      else
        blink_led_off(LED_DOWN_I);
    }

    if (tmode == TERM_MODE_OFF)
    {
      mqtt_publis_output(tout, POWER_OUTPUT_OFF);
      thermostat_ring_set_power(tix, 0);
    }
    if (tmode == TERM_MODE_MAX)
    {
      mqtt_publis_output(tout, POWER_OUTPUT_HEAT_MAX);
      thermostat_ring_set_power(tix, 255);
    }
    if (tmode == TERM_MODE_CLIMATE_MAX)
    {
      mqtt_publis_output(tout, POWER_OUTPUT_COOL_MAX);
      thermostat_ring_set_power(tix, 255);
    }
    if (tmode == TERM_MODE_MAN_HEAT || tmode == TERM_MODE_MAN_COOL || tmode == TERM_MODE_FAN || tmode == TERM_MODE_ERR)
    {
      mqtt_publis_output_pwm(tout, tmode, pwm);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/////

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_default_ring(uint8_t ring)
{
  EEPROM.write(my_default_ring, ring);
}

uint8_t get_default_ring(void)
{
  return EEPROM.read(my_default_ring);
}


/////////////////////////////
///#define my_serial_mode 94
//#define my_serial_speed 95

void serial_set_mode(uint8_t mode)
{
  EEPROM.write(my_serial_mode, mode);
  serial_mode = mode;
}

uint8_t serial_get_mode(void)
{
  return EEPROM.read(my_serial_mode);
}

void serial_set_speed(uint8_t speedy)
{
  EEPROM.write(my_serial_speed, speedy);
}

uint8_t serial_get_speed(void)
{
  return EEPROM.read(my_serial_speed);
}

void nrf_mesh_reinit(void)
{
  mesh.setNodeID(0);
  mesh.begin(nrf_load_channel());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  char tmp1[20];
  char tmp2[20];
  char str2[16];
  uint8_t itmp;
  boolean bol;
  NTPClient timeClient(udpClient);

  wdt_enable(WDTO_8S);
  wdt_reset();

  mqtt_set_public_mqtt_client(&mqtt_client);
  fdevopen( &printf_via_mqtt, 0);
  printf_begin();


  struct_DDS18s20 tds;

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
  menu_init(&setup_select_mode);
  menu_init(&menu_show_stav);
  menu_init(&menu_pwm_power_show);

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
  menu_root_setname(&setup_select_mode, title_item_set_manual_mode);
  menu_root_setname(&menu_show_stav, title_item_selftest);
  menu_root_setname(&menu_pwm_power_show, title_item_pwm_power);

  menu_item_set_properties(&item_zpet, title_menu_back, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_set_jas, title_item_setup_jas, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_select_prog, title_item_select_prog, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_select_ring, title_item_select_ring, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_select_reset, title_item_select_reset , TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_select_osvetleni, title_item_set_select_osvetleni, TYPE_FUNCTION_DYNAMIC, set_select_osvetleni, 0, 0);

  menu_item_set_properties(&item_select_mode, title_item_set_manual_mode, TYPE_STATIC, nullptr, 0, 0);
  menu_item_set_properties(&item_stav, title_item_selftest, TYPE_STATIC, nullptr, 0, 0);
  menu_item_set_properties(&item_pwm_power, title_item_pwm_power, TYPE_STATIC, nullptr, 0, 0);

  menu_root_additem(&setup_menu, &item_set_jas);
  menu_root_additem(&setup_menu, &item_select_prog);
  menu_root_additem(&setup_menu, &item_select_ring);
  menu_root_additem(&setup_menu, &item_select_mode);
  menu_root_additem(&setup_menu, &item_pwm_power);
  menu_root_additem(&setup_menu, &item_select_osvetleni);
  menu_root_additem(&setup_menu, &item_select_reset);
  menu_root_additem(&setup_menu, &item_stav);
  menu_root_additem(&setup_menu, &item_zpet);


  menu_item_set_properties(&item_set_man_temp, title_term_man, TYPE_DYNAMIC, set_man_term_temp, 0, 32);

  menu_item_set_properties(&item_set_jas_dyn, title_item_setup_jas_dynamic, TYPE_DYNAMIC, nullptr, 1, 16);
  menu_item_set_properties(&item_set_jas_auto, title_item_setup_jas_auto, TYPE_STATIC, nullptr);

  menu_item_set_properties(&item_set_select_prog, title_item_set_select_prog, TYPE_FUNCTION_DYNAMIC, set_select_active_prog, 0, 0);
  menu_item_set_properties(&item_set_select_ring, title_item_set_select_ring, TYPE_FUNCTION_DYNAMIC, set_select_ring, 0, 0);

  menu_root_additem(&setup_menu_jas, &item_set_jas_dyn);
  menu_root_additem(&setup_menu_jas, &item_set_jas_auto);

  menu_root_additem(&term_man, &item_set_man_temp);

  menu_root_additem(&setup_select_prog, &item_set_select_prog);
  menu_root_additem(&setup_select_ring, &item_set_select_ring);

  menu_item_set_properties(&item_set_select_osvetleni, title_item_set_select_osvetleni, TYPE_DYNAMIC, nullptr, 1, 9);
  menu_root_additem(&setup_select_osvetleni, &item_set_select_osvetleni);


  menu_item_set_properties(&item_set_select_mode_heat, title_item_set_manual_mode_heat, TYPE_STATIC, 0, 0);
  menu_item_set_properties(&item_set_select_mode_cool, title_item_set_manual_mode_cool, TYPE_STATIC, nullptr, 0, 0);

  menu_root_additem(&setup_select_mode, &item_set_select_mode_heat);
  menu_root_additem(&setup_select_mode, &item_set_select_mode_cool);

  menu_item_set_properties(&item_stav_show, title_item_selftest_text, TYPE_DYNAMIC, selftest_show, 0, 0);
  menu_root_additem(&menu_show_stav, &item_stav_show);

  menu_item_set_properties(&item_pwm_power_show, title_item_pwm_power_text, TYPE_DYNAMIC, termostat_pwm_power_show, 0, 0);
  menu_root_additem(&menu_pwm_power_show, &item_pwm_power_show);



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
  TCCR2B  = (1 << CS22); //delicka 64
  timer2_counter = 5;
  TCNT2 = timer2_counter;
  TIMSK2 |= (1 << TOIE2);


  interrupts();             // enable all interrupts
  SPI.begin();
  rtc.begin();

  thermostat_init_pid();



  for (uint8_t inic = 0; inic < 15; inic++)
  {
    wdt_reset();
    if (inic == 0)
    {
      if (EEPROM.read(set_default_values) == 255)
      {

        strcpy(tmp1, "I0-FR-");
        show_direct(tmp1);

        randomSeed(analogRead(A0));
        EEPROM.write(set_default_values, 0);
        EEPROM.write(my_jas_disp, 240);
        EEPROM.write(my_jas_key, 240);
        EEPROM.write(my_rs_id, 31);
        keyboard_set_sense(255);
        ///
        for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
          thermostat_ring_clear(idx);
        ///
        for (uint8_t idx = 0; idx < MAX_RTDS; idx++)
        {
          strcpy(tmp2, "");
          remote_tds_set_name(idx, 0 , tmp2);
        }
        ///
        for (uint8_t idx = 0; idx < AVAILABLE_PROGRAM; idx++)
        {
          strcpy(str2, "PROG");
          thermostat_program_set_name(idx, str2);
          thermostat_program_set_active(idx, 0);
          for (uint8_t interval_id = 0; interval_id < MAX_PROGRAM_INTERVAL; interval_id++)
          {
            thermostat_program_set_time(idx, interval_id, 0, 0, 0, 0, 0);
            thermostat_program_set_threshold(idx, interval_id, 220);
            thermostat_program_set_week(idx, interval_id, 0);
          }
        }
        ///
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
        ///
        for (uint8_t idx = 0; idx < MAX_SVETEL; idx++) light_set_output(idx, LIGHT_FREE);
        ///
        rtc.adjust(DateTime(2020, 12, 14, 17, 14, 0));
        ///
        device.mac[0] = 2; device.mac[1] = 1; device.mac[2] = 2; device.mac[3] = random(100, 150); device.mac[4] = random(3, 200); device.mac[5] = random(13, 204);
        //device.mac[0] = 2; device.mac[1] = 1; device.mac[2] = 2; device.mac[3] = 17; device.mac[4] = 22; device.mac[5] = 204;
        device.myIP[0] = 192; device.myIP[1] = 168; device.myIP[2] = 2; device.myIP[3] = 110;
        device.myMASK[0] = 255; device.myMASK[1] = 255; device.myMASK[2] = 255; device.myMASK[3] = 0;

        device.myGW[0] = 192; device.myGW[1] = 168; device.myGW[2] = 2; device.myGW[3] = 1;
        device.myDNS[0] = 192; device.myDNS[1] = 168; device.myDNS[2] = 2; device.myDNS[3] = 1;
        device.mqtt_server[0] = 192; device.mqtt_server[1] = 168; device.mqtt_server[2] = 2; device.mqtt_server[3] = 1;
        device.ntp_server[0] = 192; device.ntp_server[1] = 168; device.ntp_server[2] = 2; device.ntp_server[3] = 1;
        device.mqtt_port = 1883;
        strcpy(device.mqtt_user, "saric");
        strcpy(device.mqtt_key, "no");
        save_setup_network();
        strcpy(str2, "TERM E1");
        device_set_name(str2);
        char hostname[10];
        device_get_name(hostname);
        ///
        nrf_save_channel(76);
        set_default_ring(0);
        time_set_offset(1);
        ///
        serial_set_mode(RS_MODE_SERIAL);
        /// speed je nasobek 9600
        serial_set_speed(1);
        //delay(1000);

      }
    }
    ////
    if (inic == 1)
    {
      strcpy(tmp1, "I1-LN-");
      show_direct(tmp1);
      load_setup_network();
    }
    ////
    if (inic == 2)
    {
      strcpy(tmp1, "I2-OV-");
      show_direct(tmp1);
      jas_disp = EEPROM.read(my_jas_disp);
      jas_key = EEPROM.read(my_jas_key);
      rsid = EEPROM.read(my_rs_id);
      analogWrite(PWM_DISP, jas_disp);
      analogWrite(PWM_KEY, jas_key);
      auto_jas = 0;
      light_min = 0xffff;
      light_max = 0;
      ///
      default_ring = get_default_ring();
      ///
      for (uint8_t idx = 0; idx < MAX_SVETEL; idx++)
      {
        light_value[idx] = LIGHT_DEFAULT;
        light_state[idx] = LIGHT_DEFAULT;
      }
      for (uint8_t idx = 0; idx < MAX_KNOW_MQTT; idx++)
      {
        strcpy(know_mqtt[idx].device, "");
        know_mqtt[idx].type = 0;
        know_mqtt[idx].last_update = 0;
      }
      count_know_mqtt = 0;

      //// kvuli lepsimu nabehu pocitani nastavim vychozi hodnotu na 2000 = 20 stupnu
      for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXROMS; idx++)
        for (uint8_t cnt = 0; cnt < MAX_AVG_TEMP; cnt++)
          status_tds18s20[idx].average_temp[cnt] = 20000;

      ///
      for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
        last_output_update[idx] = 0;

    }
    ////
    if (inic == 3)
    {
      ds2482_address[0].i2c_addr = 0b0011000;
      ds2482_address[0].HWwirenum = 0;
      ///
      for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXROMS; idx++ )
      {
        status_tds18s20[idx].wait = false;
        status_tds18s20[idx].period_now = 0;
      }
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
    ////
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
    ////
    if (inic == 5)
    {
      strcpy(tmp1, "I5-KEY");
      show_direct(tmp1);
      keyboard_apply_sense();
    }
    ////
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
    ////
    if (inic == 7)
    {
      strcpy(tmp1, "I7TERM");
      show_direct(tmp1);
      thermostat_init_pid();
      /// nastaveni vychozich hodnot pro regulator
      for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
      {
        thermostat_ring_set_state(idx, 0);
        thermostat_ring_set_power(idx, 0);
        thermostat_ring_get_pid_eeprom(idx);
        thermostat_ring_update_pid_parameters(idx);
      }
    }
    ////
    if (inic == 8)
    {
      strcpy(tmp1, "I-NRF-");
      show_direct(tmp1);
      nrf_mesh_reinit();
    }
    ////
    if (inic == 9)
    {
      strcpy(tmp1, "I09ETH");
      show_direct(tmp1);
      Ethernet.begin(device.mac, device.myIP, device.myDNS, device.myGW, device.myMASK);
    }
    ////
    if (inic == 10)
    {
      if ( Enc28J60Network::linkStatus() == 1)
      {
        strcpy(tmp1, "I9-UP-");
        selftest_clear_0(SELFTEST_ETH_LINK);
      }
      else
      {
        strcpy(tmp1, "I9DOWN");
        selftest_set_0(SELFTEST_ETH_LINK);
      }
      show_direct(tmp1);
    }
    ////
    if (inic == 11)
    {
      strcpy(tmp1, "I-MQQT");
      show_direct(tmp1);
      mqtt_client.setServer(device.mqtt_server, device.mqtt_port);
      mqtt_client.setCallback(mqtt_callback);
      send_mqtt_set_header(thermctl_header_out);
      mqtt_reconnect();
    }
    ////
    if (inic == 12)
    {
      if (selftest_get_0(SELFTEST_ETH_LINK) == 0)
      {
        if (ntp_check(&timeClient) == 0)
        {
          selftest_set_0(SELFTEST_ERR_NTP);
          strcpy(tmp1, "11NTPE");
          show_direct(tmp1);
        }
        else
        {
          selftest_clear_0(SELFTEST_ERR_NTP);
          strcpy(tmp1, "11NTP-");
          show_direct(tmp1);
        }
      }
      delay(250);
    }
    ////
    if (inic == 13)
    {
      strcpy(tmp1, "RS 485");
      show_direct(tmp1);
      serial_mode = serial_get_mode();
      Serial.begin(serial_get_speed() * 9600);
      rs_start_find_device = 0;
      rs_find = 0;
      for (uint8_t idx = 0; idx < MAX_RS_DEVICE; idx++)
      {
        rs_device[idx].last_seen = RS_NONE;
        rs_device[idx].type = RS_NONE;
      }
    }
    ////
    if (inic == 14)
    {
      strcpy(tmp1, "I14HOT");
      show_direct(tmp1);
    }

    delay(250);
  }


}
///konec setup

//////////////////////////////////////////////////////////////////////
void selftest_set_0(uint8_t what)
{
  sbi(selftest_data, what) ;
}

void selftest_clear_0(uint8_t what)
{
  cbi(selftest_data, what) ;
}

uint8_t selftest_get_0(uint8_t what)
{
  return selftest_data & (1 << what);
}
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
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
void find_rs_device(void)
{
  char str1[32];
  char str2[32];
  strcpy(str1, "");
  strcpy(str2, "device/ident");
  if (rs_find < 32)
  {
    if (send_at(rs_find, str2, str1) == 1)
    {
      rs_device[rs_find].last_seen = 0;
      rs_find++;
    }
  }
  else
  {
    rs_find = 0;
    rs_start_find_device = 0;
  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void blink_led_set(uint8_t led_index, uint8_t freq)
{
  uint16_t reg = (led_blink_2 << 8) + led_blink_1;
  freq = freq & 0b00000011;
  reg = reg | (freq << (led_index * 2));
  led_blink_2 = reg >> 8;
  led_blink_1 = reg;
}

void blink_led_off(uint8_t led_index)
{
  uint16_t negr = ~(0b00000011 << (led_index * 2));
  uint16_t reg = (led_blink_2 << 8) + led_blink_1;
  reg = reg & negr;
  led_blink_2 = reg >> 8;
  led_blink_1 = reg;
}

uint8_t blink_led_get_freq(uint8_t led_index)
{
  uint16_t reg = (led_blink_2 << 8) + led_blink_1;
  uint16_t negr = (0b00000011 << (led_index * 2));
  uint16_t out = reg & negr;
  out = out >> (led_index * 2);
  return out;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///potreba udelat
/*
  -1.

  0. hw problem zvlnene napeti; objednat jine kondenzatory + civku - dobry vsechno funguje, potreba lepsi design desky, kratke spoje
     - kondenzator k ds18s20

  1. klavesnice
     - kalibraci tlacitek - vyreseno nastavenim pres mqtt - hotovo
     - podsviceni tlacitek - vyreseni nastavenim preq mqtt - hotovo
     - nocni rezim led podsvetleni tlacitek - na soucasne revizi nejde, mozne reseni je vyuzit svetlo vodivy plast


  2. display
     - mqtt nastaveni jasu displeje - hotovo
     - tecku displayje - hotovo
     - menu zjistit pwm vykon, udelat nove menu - hotovo
     - umet blikat s kazdou ledkou jinak rychle 4 rychlosti 0,1,2,5hz - hotovo
        - registr led_blink, rozdelim na 4 useky, budu potrebovat 2 byty
     - blikat pri rucne nastavene teplote - hotovo
        - UP ledka pri rezimu pro topeni
        - DOWN ledka pro rezimu pro chlazeni
     - blikat ledka progam
        - neni dosazen pozadovany stav pouze svitit
        - dosazene nastaveni blikat ledkou
        - neni definovan program blikat rychle
     - blikat s ledkou modu, kdy nam output modul neakceptuje stav, nebo prestal odpovidat

  3. reinit ntp
     - pokud nefunguje ntp server ulozi spatne casy - fixnuto - hotovo
     - ziskat zpetnou vazbu - hotovo
     - ulozit do selfcheck promene - hotovo
     - zjistit duvod proc se mi prepsalo nastaveni adresy, mozna fixnuto
     - pokud nebezi rtc, zkusit udelat ntp update, zkusit periodicky, brat hodnotu ze selfdiag
     - funkce presunuty do pomocne knihovny - zakladni test - hotovo


  4. remote tds
     - vyresit problem s prepisem 2x zapisuji do stejneho idx, musim udelat kontrolu zda se pouziva, pokud ano tak ignoruji - hotovo
     - last update - vyreseno

  5. fix mqtt connect - otestovat, nyni mqtt_init vraci stav connect a ne stav subscribe - hotovo
     - kompletne predelano funkce vraci stav mqtt clienta

  6. metriky zarizeni
     - mereni loadu procesoru - hotovo
     - pocet odeslanych mqqt zprav - potreba otestovat - hotovo
     - pocet prijatych mqqt zprav - potreba otestovat - hotovo
     - pocet zpracovanych mqtt zprav - potreba otestovat - hotovo
     - pocet eventu od klavesnice - potreba otestovat - hotovo
     - pocitadlo spatne odeslanych mqtt - hotovo
     - opravit uptime - otestovat - hotovo

  7. mqtt nastaveni site, ip, mask, gw, mqtt - hotovo
      - ziskat stav nastaveni - hotovo
      - pres mqqt si to nechat poslat - hotovo
      - totalni unsubscribe
      - random generovani mac adresy posledni 3 byty, uplne pri prvnim initu
      - udelat zarizeni pro externi nastaveni

  8. preprogramovat fuse nemaz eeprom - ok; vraceno zmet

  9. napojeni rs485 zarizeni
     - pres mqtt typ linky, modbus, obecny rs485
        - rychlost cislo * 9600
        - mode
     - umet resit sinclair
     - umet odeslat zpravu

  11. opravit logovatko
      - mam opraveno, potreba overit spravnou funkcnost logu - hotovo
      - potreba otestovat vsechny chybove stavy

  12. kdyz mode prog - blika = neaktivni a prepnuti jinam stale blika - potreba otestovat - opraveno

  13. nrf24 mesh
      - nastavit kanal
      - scan site - hotovo
      - otestovat

  14. bootloader po ethernetu
        zakladni flash pameti- hotovo
        moznost naprogramovat eeprom
        CRC soucet dat, na zaklade toho se prepnout
        umet poslat vlastni dump zpet
        vymyslet build version

  15. otestovat remote_tds jako vstup pro pid regulator
      - remote_tds jako vstup pro ring termostat - potreba otestovat - potreba vyresit zpetnou vazbu
      - otestovat neexistujici rtds - ok
      - otestovat last_update  - ok

  16. potreba otestovat remote_tds_last_update + tds18s20 active; temp_show_default remote_tds funguje

  20. lepe se umet zotavit ze spatnych stavu
      - nedostupna sit - hotovo
      - linka down - hotovo
      - nedostupna default gw
      - nedostupny mqtt  - hotovo
         --- musi se udelat reload mqtt_clienta
      - ^^^ blikat cervenou ledkou

  21. ring main. tj, ktery ring nastavuji pres display - hotovo
      - mqtt - ok

  22. vyresit presnost ds18s20, prumer z nekolika hodnot
      - pres mqtt umet nastavit periodu mereni - podpora je pripravena - potreba otestovat - hotovo
      - uplne preprogramovat mereni na hwwire, neposilat zacatek mereni na vsechny zarizeni sbernice ale adresovat primo - hotovo

  23. vyresit mikro ventilator

  24.

  25. rozdil odesilani mac u rom tds - hotovo

  26. pridat do menu reset - hotovo

  27. vyresit problem s merenim osvetleni
    - pro ruzne fotorezistory jinou prevodni funkci - hotovo
    - otestovat novy zpusob automaticke regulace jasu - hotovo

  28. diagnosticke rozhrani rs485
    - umet nastavit sit
    - umet nastavit bootloader priznak

  29 watchdog - potreba poradne otestovat
     - pridana podpora pro jednotlive casti programu
       - timer2, timer3, 05sec, 1sec, mqtt_loop, main_loop, 2 bity jeste volne

  30. ovladani svetel - hotovo
     - podpora pro zpetnou vazbu, napr. praskla zarovka, light_state

  31. ping termbig
      - novy topic, start nove zarizeni se annoncuje nove dostupne zarizeni
        -- kolik si jich drzet v pameti?

  32. zpetna vazba od termbig
      - stav vystupu, jestli je nastaveno jak pozaduji
      - stav vystupu, zda je dostupny

  33. metriku pro tds offset - hotovo

  34. poslat do mqtt topicu /ctl/thermctl/subscribe XXXXX
      - payload nazev zarizeni
      - opravit funkci pro odeslani -- hotovo

  35. umet nastavit pres mqtt bitove rozliseni mereni
     - resitelne je to pouze pro ds1822

  40. zpetna vazba na neco spatne
      - bude potreba predelat na globalni selfcheck - hotovo
      - budu status registr, kde jednotlive bity budou chyby - hotovo
      - nebezici rtc - potreba otestovat - hotovo
      - nepodarilo se spojit s ntp serverem - hotovo
      - pocet co se nepodarilo obecne mqtt - potreba otestovat
      - selfdiag zobrazit na displaji - hotovo
      - mqtt_status - hotovo
      - link_status - hotovo

  41. pres mqtt odesilat i prumernout teplotu z mereni - hotovo
      - potreba inciliazovat pole pri prvnim spustenim - hotovo

  42. funkce vymaz cele nastaveni, nastaveni defaultu
      - ringu - otestovano - hotovo
      - tds - otestovano - hotovo
      - programy - hotovo

  43. kdyz je nejspise nefunkcni mqtt nelze se dostat do menu
      - jen nekdy
      - problem je ten, ze nestaci millis counter, mqtt.loop je moc dlouho

  44. timeout pro navrat z menu - promyslet
      - pouze ze setup_menu - hotovo

  45. restart needed priznak - hotovo

  46. pid regulator
      - moznost nastavit cas regulace pres mqtt - hotovo
      - promyslet jestli jako vstupni promenou pro pid regulator vzit aktualni teplotu a nebo prumernou - vzit aktualni hodnotu - hotovo

  48. vymyslet default_ring, kdyz neni zadny aktivni

*/
/*
  /thermctl-in/{{DEVICE}}/ring/set/0/active
  /thermctl-in/{{DEVICE}}/ring/set/0/tds
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

  uint8_t id;
  unsigned long load_now;



  load_now = millis();

  if ( Enc28J60Network::linkStatus() == 1)
    selftest_clear_0(SELFTEST_ETH_LINK);
  else
    selftest_set_0(SELFTEST_ETH_LINK);


  if (selftest_get_0(SELFTEST_ETH_LINK) == 0)
  {
    if (mqtt_reconnect() == 0)
      selftest_clear_0(SELFTEST_MQTT_LINK);
    else
      selftest_set_0(SELFTEST_MQTT_LINK);
  }
  else
  {
    mqtt_client.disconnect();
    selftest_set_0(SELFTEST_MQTT_LINK);
  }

  mqtt_client.loop();
  sbi(watchdog_state, WD_MQTT_LOOP);





  for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
    thermostat_pid_compute(idx);


  if (scan_rf_net_enable == 0)
  {
    mesh.update();
    mesh.DHCP();
  }

  if (scan_rf_net_enable == 1)
  {
    scan_rf_network(&radio, scan_rf_net_channel);
    /// kazdy kanal proskenuji 128x
    nrf_scan_check_cnt ++;
    if (nrf_scan_check_cnt > 127)
    {
      scan_rf_net_channel++;
      nrf_scan_check_cnt = 0;
    }
    if (scan_rf_net_channel > 127)
    {
      selftest_clear_0(SELFTEST_NRF_SCAN);
      stop_scan_rf_network(&radio);
      scan_rf_network_public(&mqtt_client);
      nrf_mesh_reinit();
    }
  }

  if (scan_rf_net_enable == 2)
  {
    scan_rf_net_enable = 0;
    radio.printDetails();
  }


  if (serial_mode == RS_MODE_AT)
  {
    itmp = Serial.availableForWrite();

    if ((SERIAL_TX_BUFFER_SIZE - 1 - itmp) > serial_max_used_tx)
      serial_max_used_tx = (SERIAL_TX_BUFFER_SIZE - 1 - itmp);

    if (itmp == (SERIAL_TX_BUFFER_SIZE - 1))
    {
      digitalWrite(rs485, LOW);
      rs_death_time = RS_SEND_PAUSE;
    }

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

  }

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
        keyboard_event++;
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




  if (selftest_get_0(SELFTEST_MQTT_LINK) != 0)
  {
    led = led + LED_ERR;
    //blink_led_set(LED_ERR_I, 3);
  }



  term_mode = thermostat_ring_get_mode(default_ring);
  if (term_mode == TERM_MODE_OFF)
  {
    led = led + LED_OFF;
    blink_led_off(LED_PROG_I);
    blink_led_off(LED_ERR_I);
    blink_led_off(LED_UP_I);
    blink_led_off(LED_DOWN_I);
  }
  if (term_mode == TERM_MODE_MAX)
  {
    led = led + LED_MAX;
    blink_led_off(LED_PROG_I);
    blink_led_off(LED_ERR_I);
    blink_led_off(LED_UP_I);
    blink_led_off(LED_DOWN_I);
  }
  if (term_mode == TERM_MODE_PROG)
  {
    led = led + LED_PROG;
    blink_led_off(LED_UP_I);
    blink_led_off(LED_DOWN_I);
  }
  if (term_mode == TERM_MODE_MAN_HEAT)
  {
    led = led + LED_MAX + LED_UP + LED_DOWN;
    blink_led_off(LED_PROG_I);
    blink_led_off(LED_ERR_I);
  }
  if (term_mode == TERM_MODE_CLIMATE_MAX)
  {
    led = led + LED_CLIMA;
    blink_led_off(LED_PROG_I);
    blink_led_off(LED_ERR_I);
    blink_led_off(LED_UP_I);
    blink_led_off(LED_DOWN_I);
  }
  if (term_mode == TERM_MODE_MAN_COOL)
  {
    led = led + LED_CLIMA + LED_UP + LED_DOWN;
    blink_led_off(LED_PROG_I);
    blink_led_off(LED_ERR_I);
  }

  /// blikatko
  if (((milis / 500) % 2) == 0)
  {
    for (uint8_t idx = 0; idx < 8; idx++)
      if (blink_led_get_freq(idx) == 1)
        sbi(led_blink_old, idx);
  }

  if (((milis / 250) % 2) == 0)
  {
    for (uint8_t idx = 0; idx < 8; idx++)
      if (blink_led_get_freq(idx) == 2)
        sbi(led_blink_old, idx);
  }

  if (((milis / 100) % 2) == 0)
  {
    for (uint8_t idx = 0; idx < 8; idx++)
      if (blink_led_get_freq(idx) == 3)
        sbi(led_blink_old, idx);
  }


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
      delay_show_menu = 0;
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
      thermostat_ring_set_mode(default_ring, TERM_MODE_CLIMATE_MAX);
      menu_set_root_menu(&term_climate);
      delay_show_menu = 0;
      key = 0;
    }
    if ((key == TL_UP) || (key == TL_DOWN))
    {
      if (thermostat_ring_get_status_bites(default_ring, STATUS_BIT_HEAT_OR_COOL) == 0)
        thermostat_ring_set_mode(default_ring, TERM_MODE_MAN_HEAT);
      else
        thermostat_ring_set_mode(default_ring, TERM_MODE_MAN_COOL);
      term_man.args3 = (thermostat_ring_get_mezni(default_ring) - 160) / 5;
      menu_set_root_menu(&term_man);
      delay_show_menu = 0;
      key = 0;
    }

    if (key_press == TL_OK)
    {
      menu_set_root_menu(&setup_menu);
      delay_show_menu = 0;
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
    ////
    if (key == TL_OK)
    {
      delay_show_menu = 0;
      /// podmenu jas
      if (strcmp_P(curr_item, title_item_setup_jas) == 0)
      {
        menu_set_root_menu(&setup_menu_jas);
        if (jas_disp == 0)
        {
          setup_menu_jas.current_item = 1;
          setup_menu_jas.args3 = 0;
        }
        else
        {
          setup_menu_jas.current_item = 0;
          setup_menu_jas.args3 = (255 - jas_disp) / 15;
        }
        key = 0;
      }
      /// menu mqtt status
      if (strcmp_P(curr_item, title_item_set_manual_mode) == 0)
      {
        itmp = thermostat_ring_get_status_bites(default_ring, STATUS_BIT_HEAT_OR_COOL);
        if (itmp == 0)
          setup_select_mode.current_item = 0;
        else
          setup_select_mode.current_item = 1;
        menu_set_root_menu(&setup_select_mode);
        key = 0;
      }
      ////
      if (strcmp_P(curr_item, title_item_select_prog) == 0)
      {
        setup_select_prog.args3 = rev_get_show_prog(thermostat_ring_get_program_id(default_ring));
        menu_set_root_menu(&setup_select_prog);
        key = 0;
      }
      //
      if (strcmp_P(curr_item, title_item_select_ring) == 0)
      {
        setup_select_ring.args3 = rev_get_show_ring(default_ring);
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
      /// menu show stav
      if (strcmp_P(curr_item, title_item_selftest) == 0)
      {
        menu_set_root_menu(&menu_show_stav);
        key = 0;
      }
      /// menu show pwm vykon
      if (strcmp_P(curr_item, title_item_pwm_power) == 0)
      {
        menu_set_root_menu(&menu_pwm_power_show);
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
  /// menu vyber vychoziho ringu
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
      if (setup_select_ring.args2 != RING_FREE)
      {
        set_default_ring(setup_select_ring.args2);
        default_ring = setup_select_ring.args2;
      }
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
    if (setup_menu_jas.current_item == 0)
    {
      jas_disp = 255 - (15 * setup_menu_jas.args3);
      analogWrite(PWM_DISP, jas_disp);
    }
    if (setup_menu_jas.current_item == 1)
    {
      jas_disp = 0;
    }
    if (key == TL_OK)
    {
      EEPROM.write(my_jas_disp, jas_disp);
      menu_back_root_menu();
      key = 0;
    }
  }
  /////////////////
  /// menu vyberu manualniho modu/// nastaveni modu rucniho ovladani topi/chladi
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  if (strcmp_P(curr_menu, title_item_set_manual_mode) == 0)
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
      if (setup_select_mode.current_item == 0)
        thermostat_ring_update_bites(default_ring, STATUS_BIT_HEAT_OR_COOL, 0);
      if (setup_select_mode.current_item == 1)
        thermostat_ring_update_bites(default_ring, STATUS_BIT_HEAT_OR_COOL, 1);
      menu_back_root_menu();
      key = 0;
    }
  }
  //////////////////////////
  /// zobrazeni pwm vykonu
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  if (strcmp_P(curr_menu, title_item_pwm_power) == 0)
    if (key == TL_OK)
    {
      menu_back_root_menu();
      key = 0;
    }
  /////////////////
  /// zobrazeni selftestu v decimalnim vyjadreni
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  if (strcmp_P(curr_menu, title_item_selftest) == 0)
    if (key == TL_OK)
    {
      menu_back_root_menu();
      key = 0;
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

  if (strcmp_P(curr_menu, title_setup_menu) == 0)
  {
    if (delay_show_menu > 10) menu_back_root_menu();
  }
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
    if (delay_show_menu > 5) menu_back_root_menu();
  }
  if (strcmp_P(curr_menu, title_term_climate) == 0)
  {
    if (key == TL_CLIMA) menu_back_root_menu();
    if (delay_show_menu > 5) menu_back_root_menu();
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
  if ((milis - milis_10s) > 10000)
  {
    milis_10s = milis;
    //device_get_name(str1);
    send_mqtt_onewire();
    send_mqtt_status(&mqtt_client);
    send_device_status();
    send_mqtt_ring();
    send_mqtt_tds();
    send_mqtt_program();
    thermostat();
    //mqtt_send_pid_variable();
    //send_mqtt_remote_tds_status();
    //send_network_config();
    send_light_controler();
    send_know_device();
    send_mesh_status();
  }

  ///////////////////////
  /// kazdych 50ms
  if ((milis - milis_05s) > 50)
  {
    milis_05s = milis;
    now = rtc.now();
    menu_display();
    digitalWrite(LED, 1);

    if ((serial_mode == RS_MODE_AT) && (rs_start_find_device == 1))
    {
      find_rs_device();
    }
    sbi(watchdog_state, WD_05SEC);
  }

  ////////////////////
  /// kazdou 1sec
  if ((milis - milis_1s) > 1000)
  {
    delay_show_menu++;
    milis_1s = milis;
    uptime++;
    mereni_hwwire(uptime);
    ///
    /// odesle ja ziji
    strcpy_P(str2, thermctl_subscribe);
    device_get_name(str1);
    send_mqtt_payload(&mqtt_client, str2, str1);
    ///
    /// aktualizuje seznam znamych mqtt zarizeni
    update_know_mqtt_device();
    ///
    for (uint8_t idx = 0; idx < MAX_RS_DEVICE; idx++)
    {
      if (rs_device[idx].type != RS_NONE)
      {
        if (rs_device[idx].last_seen < 60)
          rs_device[idx].last_seen++;
      }
    }
    /// incrementuje cas nedostupnosti output jednoty
    for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
      if (last_output_update[idx] < 254)
        last_output_update[idx]++;
    ///
    /// kontroluje zakladni funkce
    selftest();
    ///
    /// pocitadlo posledni aktualizace remote tds
    for (uint8_t idx = 0; idx < MAX_RTDS; idx++)
      /// pozor na preteceni bufferu
      if (remote_tds_last_update[idx] < 254)
        remote_tds_last_update[idx]++;
    ///
    /// nastaveni dynamickeho menu pro programy
    use_prog = count_use_prog();
    if (use_prog > 0) use_prog = use_prog - 1;
    menu_item_update_limits(&item_set_select_prog, 0, use_prog);
    ///
    /// nastaveni dynamickeho menu teplomeru
    use_tds = count_use_tds();
    use_rtds = count_use_rtds();
    use_tds = use_tds + use_rtds;
    if (use_tds > 0) use_tds = use_tds - 1;
    menu_item_update_limits(&item_show_temp, 0, use_tds);
    ///
    /// ziska pocet pouzitych svetelnych regulatoru
    use_light = count_use_light();
    use_light_curr = use_light;
    if (use_light > 0) use_light = use_light - 1;
    menu_item_update_limits(&item_select_osvetleni, 0, use_light);
    ///
    /// nastaveni dynamickeho menu vyberu ringu termostatu
    use_ring = count_use_active_rings();
    if (use_ring > 0) use_ring = use_ring - 1;
    menu_item_update_limits(&item_set_select_ring, 0, use_ring);
    ///
    /// automaticke nastaveni jasu displaye
    light_curr = analogRead(LIGHT);
    if (light_curr < light_min) light_min = light_curr;
    if (light_curr > light_max) light_max = light_curr;
    ///
    if (jas_disp == 0) // Automatika
    {
      auto_jas = (float) (light_curr - light_min) / (light_max - light_min) * 255;
      itmp = auto_jas;
      if (itmp > 240) itmp = 240;
      analogWrite(PWM_DISP, itmp);
    }
    /// nastavim watchdog, ze jsem tady byl
    sbi(watchdog_state, WD_1SEC);
  }


  if ((milis - milis_1ms) > 0)
  {
    milis_1ms = milis;
    if (rs_death_time > 1)
      rs_death_time--;
    sbi(watchdog_state, WD_2MSEC);

  }

  sbi(watchdog_state, WD_MAIN_LOOP);
  sbi(watchdog_state, WD_FREE);

  if (watchdog_state == 255)
  {
    watchdog_state = 0;
    wdt_reset();
  }

  load = millis() - load_now;
  if (load < load_min) load_min = load;
  if (load > load_max) load_max = load;

}
//////////////////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_OVF_vect)
{
  uint8_t backup = SREG;
  TCNT2 = timer2_counter;
  milis++;
  sbi(watchdog_state, WD_TIMER2);
  SREG = backup;
}


ISR(TIMER3_OVF_vect)        // interrupt service routine
{
  uint8_t backup = SREG;
  TCNT3 = timer3_counter;   // preload timer
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
  sbi(watchdog_state, WD_TIMER3);
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

    itoa(setup_menu_jas.current_item, str1, 10);
    log_error(&mqtt_client, str1);
*/
