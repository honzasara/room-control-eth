#include <NTPClient.h>

#include <ArduinoJson.h>


#include <PubSubClient.h>
#include <UIPEthernet.h>


#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

#include <avr/pgmspace.h>
#include "font_alfa.h"

#include <avr/wdt.h>
#include <avr/boot.h>

#include <RTClib.h>
#include <saric_ds2482.h>
#include <ow.h>
#include <EEPROM.h>

#define TYPE_FUNCTION 0
#define TYPE_STATIC 1
#define TYPE_DYNAMIC 2

#define HW_ONEWIRE_MAXROMS 5
#define HW_ONEWIRE_MAXDEVICES 5
#define MAX_THERMOSTAT 3
#define MAX_TEMP 320
#define MIN_TEMP 160
#define AVAILABLE_PROGRAM 9

#define TERM_READY 1
#define TERM_STOP 0

#define TERM_STAV_STOP 0
#define TERM_STAV_TOPI 1
#define TERM_OK 2
#define TERM_CLIMA 2

#define TERM_MODE_OFF 0
#define TERM_MODE_MAX 1
#define TERM_MODE_PROG 2
#define TERM_MODE_MAN_HEAT 3
#define TERM_MODE_CLIMATE 4
#define TERM_MODE_MAN_COOL 5

#define POWER_OUTPUT_OFF 0
#define POWER_OUTPUT_MAX 1

#define set_default_values 90
#define my_sense 91
/*
  #define dostupny_program0 92
  #define dostupny_program1 93
  #define dostupny_program2 94
  #define dostupny_program3 95
*/
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
//#define SENSE 13   /// TIMER1

#define SER1  24
#define SER2  27
#define SCLK  25
#define PCLK  26

#define LIGHT A7

#define NRF_CE 29
#define NRF_CS 28
#define NRF_IRQ 30 ///INT6



#define ETH_RST 2
#define ETH_INT 22

#define LED 0

#define TL_OFF   0b10000000
#define TL_MAX   0b01000000
#define TL_PROG  0b00100000
#define TL_CLIMA  0b00001000
#define TL_OK    0b00000100
#define TL_DOWN  0b00000010
#define TL_UP    0b00000001


#define LED_OFF  0b10000000
#define LED_MAX 0b01000000
#define LED_PROG  0b00100000
#define LED_CLIMA 0b00010000
#define LED_OK   0b00001000
#define LED_DOWN 0b00000100
#define LED_UP   0b00000010
#define LED_ERR  0b00000001

#define MAX_TEMP_BUFFER 32
#define UART_RECV_MAX_SIZE 64

#define MAX_ITEMS_ROOT_MENU 12
#define MAX_HISTORY 5



StaticJsonDocument<512> doc;

struct struct_ds2482
{
  uint8_t i2c_addr;
  uint8_t HWwirenum;
  uint8_t hwwire_cekam;
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
};

typedef struct struct_status_DDS18s20
{
  uint8_t tempL;
  uint8_t tempH;
  uint8_t CR;
  uint8_t CP;
  uint8_t CRC;
  int temp;
  uint8_t online;
};

typedef void (*function)(char *tmp, uint8_t args);


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
  uint8_t nrf_channel;
};

#define nextfree 260
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



//////////////////////////////////////////////////////////////
RTC_DS1307 rtc;
DateTime now;

EthernetClient ethClient;
EthernetUDP udpClient;
PubSubClient mqtt_client(ethClient);
NTPClient timeClient(udpClient);

RF24 nRF(NRF_CE, NRF_CS);


struct_my_device device;

uint8_t therm_stav[MAX_THERMOSTAT];
struct_1w_rom w_rom[HW_ONEWIRE_MAXROMS];
struct_ds2482 ds2482_address[1];
struct_status_DDS18s20 status_tds18s20[HW_ONEWIRE_MAXROMS];
uint8_t last_sync = 0;
uint8_t Global_HWwirenum = 0;
uint8_t tmp_rom[8];
uint8_t key = 0;
uint8_t key_now = 0;
uint8_t key_press = 0;
uint8_t key_release = 0;
uint8_t key_press_cnt[8];
uint8_t led, led_old;
volatile uint8_t rsid;
uint8_t ppwm = 0;
uint32_t uptime = 0;
//uint32_t max_program = 0;
uint8_t term_mode = 0;
uint16_t timer3_counter;
uint8_t display_pos;
uint8_t jas_disp, jas_key;
uint8_t statisice = ' ';
uint8_t destisice = ' ';
uint8_t  tisice = ' ';
uint8_t  stovky = ' ';
uint8_t  desitky = ' ';
uint8_t  jednotky = ' ';
uint8_t tecka = 0;
uint8_t echo = 0;
long int milis = 0;
long int milis_05s = 0;
long int milis_1s = 0;
long int milis_10s = 0;
long int milis_key = 0;
uint16_t aktual_light = 0;
uint8_t delay_show_menu = 0;
uint8_t start_at = 0;
uint8_t re_at = 0;
uint8_t r_id = 0;
char uart_recv[UART_RECV_MAX_SIZE];

uint8_t ladeni = 0;
uint8_t scan_rf_net_enable = 0;
uint8_t scan_rf_net_channel = 0;

byte adresaPrijimac[] = {0, 0, 0, 0, 1};
byte adresaVysilac[] = {1, 0, 0, 0, 1};

uint8_t used_channels[128];
static char nrf_receive_data[32];
static char nrf_send_data[32];

uint8_t link_status;

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
const char mqtt_header_in[] PROGMEM  = "/thermctl-in/";
const char mqtt_header_out[] PROGMEM  = "/thermctl-out/";

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

const char title_item_mqtt_status[] PROGMEM = "-MQQT-";
const char title_item_setup_jas[] PROGMEM = "-nJAS-";
const char title_item_select_prog[] PROGMEM = "-nPROG";
const char title_item_setup_jas_dynamic[] PROGMEM = "JAS $$";
const char title_item_setup_jas_auto[] PROGMEM = "JAS AT";

const char title_item_set_select_prog[] PROGMEM = "PROG $";

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
RootMenu setup_mqtt_status;
RootMenu setup_select_prog;

ItemMenu item_set_jas, item_mqtt_status, item_select_prog , item_zpet;
ItemMenu item_set_jas_dyn, item_set_jas_auto;
ItemMenu item_set_man_temp;
ItemMenu item_set_select_prog;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_direct(char *tmp)
{
  jednotky = tmp[5];
  desitky = tmp[4];
  stovky = tmp[3];
  tisice = tmp[2];
  destisice = tmp[1];
  statisice = tmp[0];
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
void menu_display(void)
{
  static char tmp[8];
  static char tmp2[5];
  RootMenu _rm;
  uint8_t t, r, i, p;
  _rm = *RootHistory.history[RootHistory.menu_max];

  if (_rm.items == 0)
  {
    strcpy_P(tmp, _rm.name);
    show_direct(tmp);
    goto endfce;
  }

  if (_rm.Item[_rm.current_item]->type == TYPE_FUNCTION)
  {
    _rm.Item[_rm.current_item]->on_show(0, 0);
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
      _rm.Item[_rm.current_item]->on_show(_rm.args3, tmp2);
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
  }

  if (type == TYPE_DYNAMIC)
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
  }

  if (type == TYPE_DYNAMIC)
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


void set_man_term_temp(uint8_t cnt, char *text)
{
  int tepl;
  tepl = 160 + (cnt * 5);
  itoa(tepl, text, 10);
}

void set_select_active_prog(uint8_t cnt, char *text)
{

  itoa(cnt, text, 10);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void kalibrace_touchpad(void)
{
  uint8_t itmp = EEPROM.read(my_sense);
  //analogWrite(SENSE, itmp);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t count_use_tds(void)
{
  uint8_t cnt = 0;
  for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXROMS; idx++)
    if ( EEPROM.read(wire_know_rom_0 + (idx * 20)) == 1) cnt++;
  return cnt;
}
////////////////////////////////////////////////////////////////////////////////////////////
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
    ret = 1;
  }
  return ret;
}
////////////////////////////////////////////////////////////
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
}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

void show_temp_default(void)
{
  char c[4];
  int tt;
  char tmp[8];
  uint8_t idx = 0;
  tmp[0] = 0;
  struct_DDS18s20 tds;

  get_tds18s20(idx, &tds);
  if ((tds.used == 1) && (status_tds18s20[idx].online == True))
  {
    tt = status_tds18s20[idx].temp;
    if (tt < 0) tt = tt * -1;
    itoa(tt / 100, c, 10);
    if ((tt / 1000) > 10)
      tecka = 0b00001000;
    else
      tecka = 0b00000100;
    if (tt == 0)
    {
      c[0] = '0';
      c[1] = '0';
    }
    if (status_tds18s20[idx].temp >= 0)
      strcpy(tmp, "t ");
    if (status_tds18s20[idx].temp < 0)
      strcpy(tmp, "t-");
    strcat(tmp, c);
    strcat(tmp, "C");
  }
  else
  {
    strcpy(tmp, "t ERR ");
  }

  if (tmp[5] == 0) jednotky = ' ';
  else jednotky = tmp[5];
  if (tmp[4] == 0)  desitky = ' ';
  else desitky = tmp[4];
  stovky = tmp[3];
  tisice = tmp[2];
  destisice = tmp[1];
  statisice = tmp[0];
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
  jednotky = tmp[5];
  desitky = tmp[4];
  stovky = tmp[3];
  tisice = tmp[2];
  destisice = tmp[1];
  statisice = tmp[0];
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
int tds_get_offset(uint8_t idx, int offset)
{
  struct_DDS18s20 tds;
  get_tds18s20(idx, &tds);
  return tds.offset;
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
  device.mqtt_port = EEPROM.read(device_mqtt_port) << 8 + EEPROM.read(device_mqtt_port + 1);
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
void thermostat_get_name(uint8_t idx, char *name)
{
  char t;
  for (uint8_t i = 0; i < 9; i++)
  {
    t = EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + i);
    name[i] = t;
    if (t == 0) break;
  }
}
void thermostat_set_name(uint8_t idx, char *name)
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
//////////////
uint8_t thermostat_get_output(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 19);
}
//////////////////////
void thermostat_set_output(uint8_t idx, uint8_t output)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 19, output);
}
//////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// funkce ktera nastavuje/zjistuje jestli dany operacni termostat je pripraven
//////////////
uint8_t thermostat_get_ready(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 17);
}
//////////////////////
void thermostat_set_ready(uint8_t idx, uint8_t ready)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 17, ready);
}
//////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// funkce ktera nastavuje/zjistuje jestli dany operacni termostat je pripraven
//////////////

uint8_t thermostat_get_status(uint8_t idx)
{
  return therm_stav[idx];
}
//////////////////////
void thermostat_set_status(uint8_t idx, uint8_t stav)
{
  therm_stav[idx] = stav;
}


////////////////////////////////////////////////////////////////////////////////////////
///////// globalni termostat mode
/*
  uint8_t thermostat_get_mode(void)
  {
  term_mode = EEPROM.read(eeprom_term_mode);
  return term_mode;
  }

  void thermostat_set_mode(uint8_t in)
  {
  term_mode = in;
  EEPROM.write(eeprom_term_mode, term_mode);
  }
  /*
  /*
  //////// seznam vsech dostupnych programu
  void thermostat_get_availability_program(void)
  {
  max_program = EEPROM.read(dostupny_program0) | EEPROM.read(dostupny_program1) << 8 | EEPROM.read(dostupny_program2) << 16 | EEPROM.read(dostupny_program3) << 24;
  }
  void thermostat_set_availability_program(void)
  {
  EEPROM.write(dostupny_program0, max_program & 0xff);
  EEPROM.write(dostupny_program1, (max_program >> 8) & 0xff);
  EEPROM.write(dostupny_program2, (max_program >> 16) & 0xff);
  EEPROM.write(dostupny_program3, (max_program >> 24) & 0xff);
  }
*/

//// nastaveni programu k ringu termostatu
uint8_t thermostat_get_program_id(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 15);
}
void thermostat_set_program_id(uint8_t idx, uint8_t id)
{
  return EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 15, id);
}

///mezni rozhodovaci teplota
int thermostat_get_mezni(uint8_t idx)
{
  return (EEPROM.read(eeprom_thermostat_0 + (20 * idx) + 10) << 8) + EEPROM.read(eeprom_thermostat_0 + (20 * idx) + 11);
}
void thermostat_set_mezni(uint8_t idx, int temp)
{
  EEPROM.write(eeprom_thermostat_0 + (20 * idx) + 10, temp >> 8);
  EEPROM.write(eeprom_thermostat_0 + (20 * idx) + 11, temp & 0xff);
}

///////////////
uint8_t thermostat_get_asociate_tds(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 16);
}
void thermostat_set_asociate_tds(uint8_t idx, uint8_t id)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 16, id);
}

//// nastaveni modu k ringu termostatu
uint8_t thermostat_get_ring_mode(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 18);
}
void thermostat_set_ring_mode(uint8_t idx, uint8_t id)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 18, id);
}

///////////////////////////////////////////////////////////////
void send_mqtt_message_prefix_id_topic_payload(char *prefix, uint8_t id, char *topic, char *payload)
{

  char str_topic[64];
  char hostname[10];
  char tmp1[12];
  if (mqtt_client.connected())
  {
    device_get_name(hostname);
    strcpy_P(str_topic, mqtt_header_out);
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

void send_mqtt_general_payload(char *topic, char *payload)
{
  char str_topic[64];
  char hostname[10];
  char tmp1[12];
  if (mqtt_client.connected())
  {
    device_get_name(hostname);
    strcpy_P(str_topic, mqtt_header_out);
    strcat(str_topic, hostname);
    strcat(str_topic, "/");
    strcat(str_topic, topic);
    mqtt_client.publish(str_topic, payload);
  }
}
/////////////////////////////////////////////
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
    strcpy_P(str_topic, mqtt_header_out);
    strcat(str_topic, hostname);
    strcat(str_topic, "/1wire/count");
    mqtt_client.publish(str_topic, payload);
    for (uint8_t i = 0; i < Global_HWwirenum; i++)
    {
      strcpy_P(str_topic, mqtt_header_out);
      strcat(str_topic, hostname);
      strcat(str_topic, "/1wire/");
      itoa(i, tmp1, 10);
      strcat(str_topic, tmp1);
      strcat(str_topic, "/rom");

      payload[0] = 0;
      for (uint8_t a = 0; a < 8; a++ )
      {
        itoa(w_rom[i].rom[a], tmp1, 16);
        strcat(payload, tmp1);
        if (a < 7)
        {
          tmp1[0] = ':';
          tmp1[1] = 0;
          strcat(payload, tmp1);
        }
      }
      mqtt_client.publish(str_topic, payload);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
//// /thermctl-out/XXXXX/tds/ID/temp
//// /thermctl-out/XXXXX/tds/ID/name
//// /thermctl-out/XXXXX/tds/offset
//// /thermctl-out/XXXXX/tds/online
//// /thermctl-out/XXXXX/tds/rom
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
          for (uint8_t a = 0; a < 8; a++ )
          {
            itoa(tds.rom[a], tmp1, 16);
            strcat(payload, tmp1);
            if (a < 7)
            {
              tmp1[0] = ':';
              tmp1[1] = 0;
              strcat(payload, tmp1);
            }
          }
          send_mqtt_message_prefix_id_topic_payload("tds", id, "rom", payload);
        }
}
//////////////////////////////////////////////////


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
  //char tmp1[12];
  //device_get_name(hostname);
  for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
    if (thermostat_get_ring_mode(idx) != 255)
    {
      thermostat_get_name(idx, payload);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "name", payload);

      itoa(thermostat_get_ready(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "ready", payload);

      itoa(thermostat_get_program_id(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "program", payload);

      itoa(thermostat_get_mezni(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "threshold", payload);

      itoa(thermostat_get_ring_mode(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "mode", payload);

      convert_mode_text(thermostat_get_ring_mode(idx), payload);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "text_mode", payload);

      itoa(thermostat_get_status(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "status", payload);

      itoa(thermostat_get_asociate_tds(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "tds", payload);

      itoa(thermostat_get_output(idx), payload, 10);
      send_mqtt_message_prefix_id_topic_payload("ring", idx, "output", payload);
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t convert_text_mode(char *str2)
{
  uint8_t mode = 0;
  if (strcmp(str2, "off") == 0) mode = TERM_MODE_OFF;
  if (strcmp(str2, "heat") == 0) mode = TERM_MODE_MAX;
  if (strcmp(str2, "manual") == 0) mode = TERM_MODE_MAN_HEAT;
  if (strcmp(str2, "auto") == 0) mode = TERM_MODE_PROG;
  if (strcmp(str2, "cool") == 0) mode = TERM_MODE_CLIMATE;
  //if (strcmp(str2, "fan_only") == 0) mode = TERM_MODE_CLIMATE;

  return mode;
}

void convert_mode_text(uint8_t mode, char *str)
{
  if (mode == TERM_MODE_OFF)   strcpy(str, "off");
  if (mode == TERM_MODE_MAX)   strcpy(str, "heat");
  if (mode == TERM_MODE_MAN_HEAT)   strcpy(str, "manual");
  if (mode == TERM_MODE_PROG)   strcpy(str, "auto");
  if (mode == TERM_MODE_CLIMATE)   strcpy(str, "cool");
  //if (mode == TERM_MODE_FAN)   strcpy(str, "fan_only");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  char str1[64];
  char str2[128];
  char tmp1[4];
  uint8_t cnt = 0;
  uint8_t id = 0;
  struct_DDS18s20 tds;
  char *pch;
  for (uint8_t j = 0; j < 128; j++) str2[j] = 0;

  //// /thermctl-in/global/time/set - nastaveni casu. payload json
  strcpy_P(str1, mqtt_header_in);
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
  strcpy_P(str1, mqtt_header_in);
  strcat_P(str1, global_time_ntp);
  if (strcmp(str1, topic) == 0)
  {
    timeClient.begin();
    timeClient.update();
    timeClient.end();
    rtc.adjust(DateTime(timeClient.getYear(), timeClient.getMonth() , timeClient.getDate(), timeClient.getHours(), timeClient.getMinutes(), timeClient.getSeconds()));
  }

  //// /thermctl/global/ping - test spojeni

  //// /thermctl-in/XXXX/rf/scan - 0|1 zapnuti/vypnuti scanovani rf site
  strcpy_P(str1, mqtt_header_in);
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
  ///////////////
  //// /thermctl-in/XXXX/tds/associate - asociace do tds si pridam mac 1wire - odpoved je pod jakem ID to mam ulozeno
  strcpy_P(str1, mqtt_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/tds/associate");
  if (strcmp(str1, topic) == 0)
  {
    strncpy(str2, payload, length);
    if (atoi(str2) < Global_HWwirenum)
      for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXDEVICES; idx++)
      {
        get_tds18s20(idx, &tds);
        if (tds.used == 0 && w_rom[atoi(str2)].used == 1)
        {
          tds.used = 1;
          for (uint8_t i = 0; i < 8; i++)
            tds.rom[i] = w_rom[atoi(str2)].rom[i];
          tds.assigned_ds2482 = ds2482_address[w_rom[idx].assigned_ds2482].i2c_addr;
          set_tds18s20(idx, &tds);
          break;
        }
      }
  }

  //// /thermctl-in/XXXX/tds/set/IDcko/name - nastavi cidlu nazev
  //// /thermctl-in/XXXX/tds/set/IDcko/offset
  //// /thermctl-in/XXXX/tds/set/IDcko/clear
  strcpy_P(str1, mqtt_header_in);
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
      if ((cnt == 1) && (strcmp(pch, "name") == 0)) tds_set_name(id, str2);
      if ((cnt == 1) && (strcmp(pch, "offset") == 0)) tds_set_offset(id, atoi(str2));
      if ((cnt == 1) && (strcmp(pch, "clear") == 0)) tds_set_clear(id);
      pch = strtok (NULL, "/");
      cnt++;
    }
  }



  //// thermctl-in/XXXXX/ring/set/IDcko/name
  //// thermctl-in/XXXXX/ring/set/IDcko/program
  //// thermctl-in/XXXXX/ring/set/IDcko/threshold
  //// thermctl-in/XXXXX/ring/set/IDcko/text_mode
  //// thermctl-in/XXXXX/ring/set/IDcko/mode
  //// thermctl-in/XXXXX/ring/set/IDcko/tds
  //// thermctl-in/XXXXX/ring/set/IDcko/ready
  //// thermctl-in/XXXXX/ring/set/IDcko/output
  strcpy_P(str1, mqtt_header_in);
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
      if ((cnt == 1) && (strcmp(pch, "name") == 0)) thermostat_set_name(id, str2);
      if ((cnt == 1) && (strcmp(pch, "program") == 0)) thermostat_set_program_id(id, atoi(str2));
      if ((cnt == 1) && (strcmp(pch, "threshold-f") == 0)) thermostat_set_mezni(id, atof(str2) * 10); /// thereshold hodnota presne ve floatu * 10
      if ((cnt == 1) && (strcmp(pch, "threshold") == 0)) thermostat_set_mezni(id, atoi(str2));
      if ((cnt == 1) && (strcmp(pch, "text_mode") == 0)) thermostat_set_ring_mode(id, convert_text_mode(str2));
      if ((cnt == 1) && (strcmp(pch, "mode") == 0)) thermostat_set_ring_mode(id, atoi(str2));
      if ((cnt == 1) && (strcmp(pch, "tds") == 0)) thermostat_set_asociate_tds(id, atoi(str2));
      if ((cnt == 1) && (strcmp(pch, "ready") == 0)) thermostat_set_ready(id, atoi(str2));
      if ((cnt == 1) && (strcmp(pch, "output") == 0)) thermostat_set_output(id, atoi(str2));
      pch = strtok (NULL, "/");
      cnt++;
    }
  }


  //// /thermctl-in/XXXXX/reset_default
  strcpy_P(str1, mqtt_header_in);
  strcat(str1, device.nazev);
  strcat(str1, "/reset_default");
  if (strcmp(str1, topic) == 0)
  {
    EEPROM.write(set_default_values, 255);
  }

}

////////////////////////////////////////////////////////////////////////////////////////////
void send_mqtt_status(void)
{
  if (mqtt_client.connected())
  {
    char str_topic[64];
    char hostname[10];
    char payload[64];
    strcpy_P(str_topic, mqtt_header_out);
    strcat(str_topic, device.nazev);
    strcat(str_topic, "/status/uptime");
    itoa(uptime, payload, 10);
    mqtt_client.publish(str_topic, payload);


    strcpy_P(str_topic, mqtt_header_out);
    strcat(str_topic, device.nazev);
    strcat(str_topic, "/status/brigthness");
    itoa(aktual_light, payload, 10);
    mqtt_client.publish(str_topic, payload);
  }
}
///////////////////////////
void mqtt_publis_output(uint8_t idx, uint8_t state)
{
  char str_topic[64];
  char payload[64];
  char str1[8];
  strcpy_P(str_topic, mqtt_header_out);
  strcat(str_topic, "power-output/");
  itoa(idx, str1, 10);
  strcat(str_topic, str1);
  strcat(str_topic, "/state");
  itoa(state, payload, 10);
  mqtt_client.publish(str_topic, payload);

}
//////////////////////////////////////////////////////
boolean mqtt_reconnect(void)
{
  char nazev[10];
  char topic[26];
  boolean ret;
  ///  /thermctl/xxxxxxxx/#
  ///  /thermctl/global/#
  device_get_name(nazev);
  ret = mqtt_client.connect(nazev);
  strcpy_P(topic, mqtt_header_in);
  strcat(topic, nazev);
  strcat(topic, "/#");
  ret = ret && mqtt_client.subscribe(topic);
  strcpy_P(topic, mqtt_header_in);
  strcat(topic, "global/#");
  ret = ret && mqtt_client.subscribe(topic);
  return ret;
}
////////////////////////////////////////////////////////


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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void thermostat(void)
{
  uint8_t tdsid = 0;
  uint8_t tmode = 0;
  uint8_t tout = 0;
  struct_DDS18s20 tds;
  for (uint8_t tix = 0; tix < MAX_THERMOSTAT; tix++)
  {
    tdsid = thermostat_get_asociate_tds(tix);
    tmode = thermostat_get_ring_mode(tix);
    tout = thermostat_get_output(tix);
    if (tmode == TERM_MODE_OFF)
    {
      mqtt_publis_output(tout, POWER_OUTPUT_OFF);
    }
    if (tmode == TERM_MODE_MAX)
    {
      mqtt_publis_output(tout, POWER_OUTPUT_MAX);
    }

    if (tmode == TERM_MODE_MAN_HEAT)
    {
      if (get_tds18s20(tdsid, &tds) == 1)
        if (tds.used == 1) if (status_tds18s20[tdsid].online == True)
            if (status_tds18s20[tdsid].temp );
    }

  }
}


char mqtt_log[128];
uint8_t mqtt_log_cnt = 0;


int printf_via_mqtt( char c, FILE *t)
{
  mqtt_log[mqtt_log_cnt] = c;
  mqtt_log[mqtt_log_cnt + 1] = 0;
  mqtt_log_cnt++;

  if (mqtt_log_cnt > 127 || c == '\n' || c == 0)
  {
    mqtt_client.publish("/log", mqtt_log);
    mqtt_log_cnt = 0;
  }

}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void) {
  char tmp1[20];
  char tmp2[20];
  uint8_t itmp;
  boolean bol;
  uint8_t devnull[2]
  ;
  device.nrf_channel = 100;

  fdevopen( &printf_via_mqtt, 0);

  struct_DDS18s20 tds;
  // put your setup code here, to run once:
  // komunikace přes sériovou linku rychlostí 19200
  //Serial.begin(38400);

  menu_history_init(&rm);
  menu_init(&rm);
  menu_root_setname(&rm, title_root_termostat);

  menu_item_set_properties(&item_show_temp, title_item_menu_temp, TYPE_FUNCTION, show_temp_default);
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

  menu_root_setname(&term_man, title_term_man);
  menu_root_setname(&term_prog, title_term_prog);
  menu_root_setname(&term_max, title_term_max);
  menu_root_setname(&term_climate, title_term_climate);
  menu_root_setname(&term_off, title_term_off);
  menu_root_setname(&setup_menu, title_setup_menu);
  menu_root_setname(&setup_menu_jas, title_item_setup_jas);
  menu_root_setname(&setup_select_prog, title_item_select_prog);

  menu_item_set_properties(&item_zpet, title_menu_back, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_set_jas, title_item_setup_jas, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_mqtt_status, title_item_mqtt_status, TYPE_STATIC, nullptr);
  menu_item_set_properties(&item_select_prog, title_item_select_prog, TYPE_STATIC, nullptr);


  menu_root_additem(&setup_menu, &item_set_jas);
  menu_root_additem(&setup_menu, &item_mqtt_status);
  menu_root_additem(&setup_menu, &item_select_prog);
  menu_root_additem(&setup_menu, &item_zpet);


  menu_item_set_properties(&item_set_man_temp, title_term_man, TYPE_DYNAMIC, set_man_term_temp, 0, 32);

  menu_item_set_properties(&item_set_jas_dyn, title_item_setup_jas_dynamic, TYPE_DYNAMIC, nullptr, 1, 16);
  menu_item_set_properties(&item_set_jas_auto, title_item_setup_jas_auto, TYPE_STATIC, nullptr);

  menu_item_set_properties(&item_set_select_prog, title_item_set_select_prog, TYPE_DYNAMIC, set_select_active_prog, 1, 8);

  menu_root_additem(&setup_menu_jas, &item_set_jas_dyn);
  menu_root_additem(&setup_menu_jas, &item_set_jas_auto);

  menu_root_additem(&term_man, &item_set_man_temp);

  menu_root_additem(&setup_select_prog, &item_set_select_prog);






  pinMode(NRF_IRQ, INPUT);
  pinMode(NRF_CE, OUTPUT);
  pinMode(NRF_CS, OUTPUT);

  pinMode(ETH_RST, OUTPUT);
  digitalWrite(ETH_RST, LOW);
  for (uint16_t i = 0; i < 4024; i++);
  digitalWrite(ETH_RST, HIGH);


  statisice = '-';
  destisice = '-';
  tisice = '-';
  stovky = '-';
  desitky = '-';
  jednotky = '-';
  display_pos = 0;




  ds2482_address[0].i2c_addr = 0b0011000;
  ds2482_address[0].HWwirenum = 0;
  ds2482_address[0].hwwire_cekam = false;

  noInterrupts();           // disable all interrupts
  //pinMode(ETH_INT, INPUT_PULLUP);


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



  //TCCR0B = TCCR0B & 0b11111000 | 0x02;
  //TCCR1B = TCCR1B & 0b11111000 | 0x02;

  interrupts();             // enable all interrupts

  rtc.begin();





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
        //max_program = 0;
        //thermostat_set_availability_program();
        //thermostat_set_mode(0);
        EEPROM.write(my_sense, 127);
        for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
        {
          thermostat_set_asociate_tds(idx, 255);
          thermostat_set_mezni(idx, 220);
          thermostat_set_program_id(idx, 0);
          thermostat_set_status(idx, 0);
          thermostat_set_ready(idx, 0);
          thermostat_set_output(idx, 255);
          thermostat_set_asociate_tds(idx, 255);
          thermostat_set_ring_mode(idx, 0);
          thermostat_set_name(idx, "FREE");
        }
        for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXDEVICES; idx++)
        {
          get_tds18s20(idx, &tds);
          strcpy(tds.name, "FREE");
          tds.used = 0;
          tds.offset = 0;
          tds.assigned_ds2482 = 0;
          set_tds18s20(idx, &tds);
        }
        rtc.adjust(DateTime(2018, 12, 14, 17, 14, 0));
        device.mac[0] = 2; device.mac[1] = 1; device.mac[2] = 2; device.mac[3] = 3; device.mac[4] = 4; device.mac[5] = 5;
        device.myIP[0] = 192; device.myIP[1] = 168; device.myIP[2] = 2; device.myIP[3] = 111;
        device.myMASK[0] = 255; device.myMASK[1] = 255; device.myMASK[2] = 255; device.myMASK[3] = 0;
        device.myGW[0] = 192; device.myGW[1] = 168; device.myGW[2] = 2; device.myGW[3] = 1;
        device.myDNS[0] = 192; device.myDNS[1] = 168; device.myDNS[2] = 2; device.myDNS[3] = 1;
        device.mqtt_server[0] = 192; device.mqtt_server[1] = 168; device.mqtt_server[2] = 2; device.mqtt_server[3] = 1;
        device.ntp_server[0] = 192; device.ntp_server[1] = 168; device.ntp_server[2] = 2; device.ntp_server[3] = 1;
        device.mqtt_port = 1883;
        strcpy(device.mqtt_user, "saric");
        strcpy(device.mqtt_key, "no");
        save_setup_network();
        device_set_name("TERM E1");
        char hostname[10];
        device_get_name(hostname);
        delay(2000);
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
      //thermostat_get_availability_program();
      //thermostat_get_mode();
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
      kalibrace_touchpad();
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
      strcpy(tmp1, "I7-NRF");
      show_direct(tmp1);

      if (nRF.begin())
      {
        nRF.setPALevel(RF24_PA_LOW);
        nRF.setChannel(device.nrf_channel);
        nRF.enableAckPayload();
        nRF.enableDynamicPayloads();
        //nRF.setRetries(50, 50);
        for (uint8_t pipe = 0; pipe < 6; pipe++)
          nRF.closeReadingPipe(pipe);
        nRF.openWritingPipe(adresaVysilac);
        nRF.openReadingPipe(1, adresaPrijimac);

        nRF.powerDown();
        nRF.stopListening();
        nRF.startListening();
        nRF.powerUp();
        nRF.writeAckPayload(1, &devnull, sizeof(devnull));
      }
      else
      {
        strcpy(tmp1, "NRFERR");
        show_direct(tmp1);
        delay(1000);
      }
    }

    if (inic == 8)
    {
      if ( Enc28J60Network::linkStatus() == 1)
      {
        strcpy(tmp1, "I8-UP-");
        link_status = 1;
      }
      else
      {
        strcpy(tmp1, "I8DOWN");
        link_status = 0;
      }
      show_direct(tmp1);
    }

    if (inic == 9)
    {
      strcpy(tmp1, "I9MQQT");
      show_direct(tmp1);
      mqtt_client.setServer(device.mqtt_server, 1883);
      mqtt_client.setCallback(mqtt_callback);
    }

    if (inic == 10)
    {
      strcpy(tmp1, "I10ETH");
      show_direct(tmp1);
      Ethernet.begin(device.mac, device.myIP, device.myDNS, device.myGW, device.myMASK);
    }

    if (inic == 11)
    {
      strcpy(tmp1, "I11CON");
      show_direct(tmp1);
      delay(1000);
      if (link_status == 1 && !mqtt_client.connected())
        bol = mqtt_reconnect();
      if (bol == false)
      {
        strcpy(tmp1, "I11ERR");
        show_direct(tmp1);
      }
    }

    if (inic == 12)
    {
      strcpy(tmp1, "I12NTP");
      show_direct(tmp1);
      timeClient.begin();
      timeClient.setTimeOffset(3600);
      tmp2[0] = 0;
      for (uint8_t i = 0; i < 4; i++)
      {
        itoa(device.ntp_server[i], tmp1, 10);
        strcat(tmp2, tmp1);
        if (i < 3)
          strcat(tmp2, ".");
      }
      timeClient.setPoolServerName(tmp2);
      timeClient.update();
      timeClient.end();
    }


    if (inic == 13)
    {
      strcpy(tmp1, "I13NPP");
      show_direct(tmp1);
      ladeni = ladeni + 1;
      delay(1000);
    }

    if (inic == 14)
    {
      strcpy(tmp1, "I14HOT");
      show_direct(tmp1);
    }

    delay(500);
  }



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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void split(uint32_t number)
{
  jednotky = (number / 1) % 10;
  desitky = (number / 10) % 10;
  stovky = (number / 100) % 10;
  tisice = (number / 1000) % 10;
  destisice = (number / 10000) % 10;
  statisice = (number / 100000) % 10;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t Bit_Reverse( uint8_t x )
{
  x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
  x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
  x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
  return x;
}







////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///potreba udelat
/*
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
  uint8_t nrf_payload_len;


  if ( Enc28J60Network::linkStatus() == 1)
    link_status = 1;
  else
    link_status = 0;


  if (link_status == 1)
  {
    if (!mqtt_client.connected()) {
      mqtt_reconnect();
    }
    mqtt_client.loop();
  }
  else
    mqtt_client.disconnect();


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
  led_old = 0;
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





  if (last_sync > 100)
  {
    led = led + LED_ERR;
  }

  term_mode = thermostat_get_ring_mode(0);
  if (term_mode == TERM_MODE_OFF) led = led + LED_OFF;
  if (term_mode == TERM_MODE_MAX) led = led + LED_MAX;
  if (term_mode == TERM_MODE_PROG) led = led + LED_PROG;
  if (term_mode == TERM_MODE_MAN_HEAT) led = led + LED_UP + LED_DOWN;
  if (term_mode == TERM_MODE_CLIMATE) led = led + LED_CLIMA;

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
      thermostat_set_ring_mode(0, TERM_MODE_OFF);
      menu_set_root_menu(&term_off);
      delay_show_menu = 0;
      key = 0;
    }
    if (key == TL_MAX)
    {
      thermostat_set_ring_mode(0, TERM_MODE_MAX);
      menu_set_root_menu(&term_max);
      delay_show_menu = 0;
      key = 0;
    }
    if (key == TL_PROG)
    {
      thermostat_set_ring_mode(0, TERM_MODE_PROG);
      menu_set_root_menu(&term_prog);
      delay_show_menu = 0;
      key = 0;
    }
    if (key == TL_CLIMA)
    {
      thermostat_set_ring_mode(0, TERM_MODE_CLIMATE);
      menu_set_root_menu(&term_climate);
      delay_show_menu = 0;
      key = 0;
    }
    if ((key == TL_UP) || (key == TL_DOWN))
    {
      thermostat_set_ring_mode(0, TERM_MODE_MAN_HEAT);
      term_man.args3 = (thermostat_get_mezni(0) - 160) / 5;
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
      if (strcmp_P(curr_item, title_item_mqtt_status) == 0)
      {
        //// vyber programu, vychozi pro term id 0
      }
      if (strcmp_P(curr_item, title_item_select_prog) == 0)
      {
        setup_select_prog.args3 = thermostat_get_program_id(0);
        if (setup_select_prog.args3 == 0) setup_select_prog.args3 = 1;
        menu_set_root_menu(&setup_select_prog);
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
      thermostat_set_program_id(0, setup_select_prog.args3);
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
      thermostat_set_mezni(0, 160 + (term_man.args3 * 5));
    }
    key = 0;
  }

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
    mereni_hwwire(1);
    send_mqtt_tds();
    thermostat();
  }


  if ((milis - milis_05s) > 225)
  {
    milis_05s = milis;
    now = rtc.now();
    menu_display();
    digitalWrite(LED, 1);
    if (last_sync < 200)
      last_sync++;


  }


  ////////////////////
  /// kazdou 1sec
  if ((milis - milis_1s) > 454)
  {
    delay_show_menu++;
    milis_1s = milis;
    uptime++;


    aktual_light = analogRead(LIGHT);
    itmp = (aktual_light / 4 / 15 * 15);
    if (jas_disp == 0) // Automatika
    {
      if (itmp > 240) itmp = 240;
      analogWrite(PWM_DISP, itmp);
    }

    if (scan_rf_net_enable == 0)
    {
      nRF.stopListening();
      strcpy(nrf_send_data, "master ping");
      nRF.write(nrf_send_data, strlen(nrf_send_data));
      nRF.startListening();
    }

  }



  if (scan_rf_net_enable == 0)
    if (nRF.available(&pipeno))
    {
      digitalWrite(LED, 0);
      nrf_payload_len = nRF.getDynamicPayloadSize();
      if (nrf_payload_len > 0)
      {
        nRF.read(&nrf_receive_data, nrf_payload_len);
        nrf_receive_data[nrf_payload_len] = 0;
        nRF.writeAckPayload(pipeno, nrf_receive_data, nrf_payload_len);
        mqtt_client.publish("/data_prijem", nrf_receive_data);
      }
    }

}






///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER3_OVF_vect)        // interrupt service routine
{
  uint8_t backup = SREG;
  uint16_t lov;

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
    lov = pgm_read_word_near(&SixteenAlfaNumeric[statisice]);
    shiftout(~lov);
    if ((tecka & 0b00000001) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr1, 0);
  }
  if (display_pos == 1)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[destisice]);
    shiftout(~lov);
    if ((tecka & 0b00000010) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr2, 0);
  }
  if (display_pos == 2)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[tisice]);
    shiftout(~lov);
    if ((tecka & 0b00000100) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr3, 0);
  }
  if (display_pos == 3)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[stovky]);
    shiftout(~lov);
    if ((tecka & 0b00001000) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr4, 0);
  }
  if (display_pos == 4)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[desitky]);
    shiftout(~lov);
    if ((tecka & 0b00010000) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr5, 0);
  }
  if (display_pos == 5)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[jednotky]);
    shiftout(~lov);
    if ((tecka & 0b00100000) != 0)  digitalWrite(DP, 0);
    digitalWrite(tr6, 0);
  }
  display_pos++;
  if (display_pos == 6) display_pos = 0;


  SREG = backup;
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
  strcpy_P(topic, mqtt_header_out);
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
