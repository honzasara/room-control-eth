#include "SSD1306.h"
#include <PubSubClient.h>
#include <UIPEthernet.h>


//#include <nRF24L01.h>
//#include <RF24.h>
//#include <RF24_config.h>

#include <avr/pgmspace.h>
#include "font_alfa.h"
#include "Font5x8.h"
#include <avr/wdt.h>

#include <RTClib.h>
#include <saric_ds2482.h>
#include <ow.h>
#include <EEPROM.h>



#define HW_ONEWIRE_MAXROMS 5
#define HW_ONEWIRE_MAXDEVICES 5
#define MAX_THERMOSTAT 3
#define MAX_TEMP 320
#define MIN_TEMP 160
#define AVAILABLE_PROGRAM 20

#define TERM_READY 1
#define TERM_STOP 0

#define TERM_STAV_STOP 0
#define TERM_STAV_TOPI 1
#define TERM_OK 2

#define TERM_MODE_OFF 0
#define TERM_MODE_MAX 1
#define TERM_MODE_PROG 2
#define TERM_MODE_MAN 3
#define TERM_MODE_CLIMATE 4

#define set_default_values 90
#define my_sense 91
#define dostupny_program0 92
#define dostupny_program1 93
#define dostupny_program2 94
#define dostupny_program3 95
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
   uint8_t stav 18 freee
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
#define TL_STAV  0b00001000
#define TL_OK    0b00000100
#define TL_DOWN  0b00000010
#define TL_UP    0b00000001


#define LED_OFF  0b10000000
#define LED_MAX 0b01000000
#define LED_PROG  0b00100000
#define LED_STAV 0b00010000
#define LED_OK   0b00001000
#define LED_DOWN 0b00000100
#define LED_UP   0b00000010
#define LED_ERR  0b00000001

#define MAX_TEMP_BUFFER 32
#define UART_RECV_MAX_SIZE 64

#define MAX_ITEMS_ROOT_MENU 6
#define MAX_HISTORY 6





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

typedef void (*function)();

typedef struct ItemMenu
{
  char *name;
  function on_show;
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
#define nextttt 338



//////////////////////////////////////////////////////////////
RTC_DS1307 rtc;
DateTime now;

EthernetClient ethClient;
PubSubClient mqtt_client(ethClient);





struct_my_device device;

uint8_t therm_stav[MAX_THERMOSTAT];
struct_1w_rom w_rom[HW_ONEWIRE_MAXROMS];
struct_ds2482 ds2482_address[1];
struct_status_DDS18s20 status_tds18s20[HW_ONEWIRE_MAXROMS];
uint8_t last_sync = 0;
uint8_t Global_HWwirenum = 0;
uint8_t tmp_rom[8];
uint8_t key, delay_key, press_key;
uint8_t led, led_old;
volatile uint8_t rsid;
uint8_t ppwm = 0;
uint32_t uptime = 0;
uint32_t max_program = 0;
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

const char title_root_climate[] PROGMEM = "Clima";
const char title_root_termostat[] PROGMEM = "Trmst";
const char title_root_fastsette[] PROGMEM = "FstSt";
const char title_root_termprog[] PROGMEM = "TrmProg";
const char title_root_termmax[] PROGMEM = "TrmMax";
const char title_root_termoff[] PROGMEM = "TrmOff";

const char title_root_setup_menu[] PROGMEM = "Setup";
const char title_error[] PROGMEM = "error";

const char title_item_menu_temp[] PROGMEM = "Teplota";
const char title_item_menu_time[] PROGMEM = "Cas";
const char title_item_menu_termstav[] PROGMEM = "TermStav";

const   char title_item_menu_termset[] PROGMEM = "Termset";

const char title_item_menu_setup_bus[] PROGMEM =  "RS485";
const char title_item_menu_setup_jas[] PROGMEM =  "n JAS ";
const char title_item_menu_back[] PROGMEM = " ZPET ";
//const char title_item_setup_rs_id[] PROGMEM  = "nRS $$";
const char title_item_setup_jas[] PROGMEM  = "JAS $$";

const char mqtt_header[] PROGMEM  = "/thermctl/";
char external_text[8];








RootMenu rm;
RootMenu term_fast_set;

RootMenu term_max;
RootMenu term_climate;
RootMenu term_off;
RootMenu term_prog;
RootMenu setup_menu;
RootMenu setup_menu_rs_id;
RootMenu setup_menu_rs_jas;


RootMenu rm_error;

ItemMenu menu_show_time,   menu_show_termstav;
ItemMenu menu_show_temp;

ItemMenu menu_show_term_set_global;
ItemMenu menu_show_term_max;
ItemMenu menu_show_term_climate;
ItemMenu menu_show_term_off;
ItemMenu menu_show_term_prog;

ItemMenu menu_setup_bus, menu_setup_jas, menu_back;

ItemMenu menu_setup_rs_id, menu_setup_set_jas;

ItemMenu item_error;

Struct_RootHistory RootHistory;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// primitivni funkce
void kalibrace_touchpad(void);
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_item_set_properties(ItemMenu *item, char *name, function on_show )
{
  item->name = name;
  item->on_show = on_show;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_display(void)
{
  static char tmp[8];
  RootMenu _rm;
  uint8_t t, te;
  _rm = *RootHistory.history[RootHistory.menu_max];

  if (_rm.Item[_rm.current_item]->on_show != nullptr)
  {
    _rm.Item[_rm.current_item]->on_show();
  }

  if (_rm.Item[_rm.current_item]->on_show == nullptr)
  {
    t = 0;
    strcpy_P(tmp, _rm.Item[_rm.current_item]->name);

    for (uint8_t i = 0; i < strlen(tmp); i++)
    {
      if (tmp[i] == '$')
      {
        tmp[i] = external_text[t];
        t++;
      }
    }
    /// praso hack s teckou; nefunguje idealne
    if (t == 0) tecka = 0;
    show_default(tmp);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void get_current_menu(char *text)
{
  strcpy_P(text, RootHistory.history[RootHistory.menu_max]->name);
}

void get_current_item(char *tmp)
{
  RootMenu _rm;
  _rm = *RootHistory.history[RootHistory.menu_max];
  strcpy_P(tmp, _rm.Item[_rm.current_item]->name);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool menu_next(void)
{
  bool ok;
  RootMenu *_rm;
  _rm = RootHistory.history[RootHistory.menu_max];

  if (_rm->current_item < _rm->items - 1)
  {
    _rm->current_item++;
    ok = true;
  }
  else ok = false;

  RootHistory.history[RootHistory.menu_max] = _rm;
  return ok;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool menu_prev(void)
{
  bool ok;
  RootMenu *_rm;
  _rm = RootHistory.history[RootHistory.menu_max];

  if (_rm->current_item > 0)
  {
    _rm->current_item--;
    ok = true;
  }
  else ok = false;

  RootHistory.history[RootHistory.menu_max] = _rm;
  return ok;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_rotate(void)
{
  RootMenu *_rm;
  _rm = RootHistory.history[RootHistory.menu_max];

  if (_rm->current_item < _rm->items - 1)
  {
    _rm->current_item++;
  }
  else
    _rm->current_item = 0;

  RootHistory.history[RootHistory.menu_max] = _rm;
}
//////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tds_get_name(uint8_t idx, char *name)
{

}

void tds_set_name(uint8_t idx, char *name)
{

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
    if (t == 0) break;
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
    if (t == 0) break;
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
void thermostat_set_output(uint8_t idx, uint8_t ready)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 19, ready);
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

uint8_t thermostat_get_stav(uint8_t idx)
{
  return therm_stav[idx];
}
//////////////////////
void thermostat_set_stav(uint8_t idx, uint8_t stav)
{
  therm_stav[idx] = stav;
}


////////////////////////////////////////////////////////////////////////////////////////
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

uint8_t thermostat_get_program_id(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 15);
}

void thermostat_set_program_id(uint8_t idx, uint8_t id)
{
  return EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 15, id);
}

int thermostat_get_mezni(uint8_t idx)
{
  return (EEPROM.read(eeprom_thermostat_0 + (20 * idx) + 10) << 8) + EEPROM.read(eeprom_thermostat_0 + (20 * idx) + 11);
}

void thermostat_set_mezni(uint8_t idx, int temp)
{
  EEPROM.write(eeprom_thermostat_0 + (20 * idx) + 10, temp >> 8);
  EEPROM.write(eeprom_thermostat_0 + (20 * idx) + 11, temp & 0xff);
}

uint8_t thermostat_get_asociate_tds(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (20 * idx)) + 16);
}

void thermostat_set_asociate_tds(uint8_t idx, uint8_t id)
{
  EEPROM.write((eeprom_thermostat_0 + (20 * idx)) + 16, id);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
}

void mqtt_reconnect(void)
{
  char nazev[10];
  char topic[20]; ///  /thermctl/xxxxxxxx/#
                ///  /thermctl/all/#
  device_get_name(*nazev);
  mqtt_client.connect(nazev);
  strcpy_P(topic, mqtt_header);
  strcat(topic, nazev);
  strcat(topic, "/#");
  mqtt_client.subscribe(topic);
  strcpy_P(topic, mqtt_header);
  strcat(topic, "all/#");
  mqtt_client.subscribe(topic);
}
////////////////////////////////////////////////////////

void glcd_init_header(void)
{
  GLCD_SetFont(Font5x8, 5, 8, GLCD_Overwrite);
  GLCD_Clear();
  GLCD_GotoXY(0, 0);
  GLCD_PrintString("Pokojovy termostat");
  GLCD_GotoXY(0, 16);
  GLCD_PrintString("inicializace");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void) {
  char tmp1[20];
  char tmp2[20];
  uint8_t itmp;

  struct_DDS18s20 tds;
  // put your setup code here, to run once:
  // komunikace přes sériovou linku rychlostí 19200
  //Serial.begin(38400);




  //RF24 nRF(NRF_CE, NRF_CS);
  //nRF.begin();
  //nRF.setPALevel(RF24_PA_LOW);
  //nRF.startListening();


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


  menu_history_init(&rm);

  menu_init(&rm);
  menu_init(&term_fast_set);

  menu_init(&term_max);
  menu_init(&term_off);
  menu_init(&term_prog);
  menu_init(&setup_menu);

  menu_init(&setup_menu_rs_id);
  menu_init(&setup_menu_rs_jas);

  menu_init(&rm_error);

  menu_root_setname(&rm, title_root_termostat);
  menu_root_setname(&term_fast_set, title_root_fastsette);
  menu_root_setname(&term_prog, title_root_termprog);
  menu_root_setname(&term_max, title_root_termmax);
  menu_root_setname(&term_climate, title_root_climate);
  menu_root_setname(&term_off, title_root_termoff);
  menu_root_setname(&setup_menu, title_root_setup_menu);
  menu_root_setname(&setup_menu_rs_id, title_item_menu_setup_bus);
  menu_root_setname(&setup_menu_rs_jas, title_item_menu_setup_jas);

  menu_root_setname(&rm_error, title_error);

  menu_item_set_properties(&menu_show_temp, title_item_menu_temp, show_temp_default);
  menu_item_set_properties(&menu_show_time, title_item_menu_time, show_time);
  menu_item_set_properties(&menu_show_term_set_global, title_item_menu_termset, show_term_set_global);
  menu_item_set_properties(&menu_show_termstav, title_item_menu_termstav, show_termstav);

  menu_item_set_properties(&menu_show_term_max, title_root_termmax, show_term_max);
  menu_item_set_properties(&menu_show_term_climate, title_root_climate, show_term_climate);
  menu_item_set_properties(&menu_show_term_off, title_root_termoff, show_term_off);
  menu_item_set_properties(&menu_show_term_prog, title_root_termprog, show_term_prog);

  menu_item_set_properties(&menu_setup_jas, title_item_menu_setup_jas , nullptr);
  menu_item_set_properties(&menu_setup_bus, title_item_menu_setup_bus, nullptr);
  menu_item_set_properties(&menu_back, title_item_menu_back, back);


  //menu_item_set_properties(&menu_setup_rs_id, title_item_setup_rs_id, nullptr);
  menu_item_set_properties(&menu_setup_set_jas, title_item_setup_jas, nullptr);



  menu_item_set_properties(&item_error, title_error, f_error);


  menu_root_additem(&rm, &menu_show_time);
  menu_root_additem(&rm, &menu_show_temp);
  menu_root_additem(&rm, &menu_show_termstav);
  menu_root_additem(&term_fast_set, &menu_show_term_set_global);


  menu_root_additem(&term_prog, &menu_show_term_prog);
  menu_root_additem(&term_max, &menu_show_term_max);
  menu_root_additem(&term_off, &menu_show_term_off);
  menu_root_additem(&term_climate, &menu_show_term_climate);

  menu_root_additem(&setup_menu, &menu_setup_jas);
  menu_root_additem(&setup_menu, &menu_setup_bus);
  menu_root_additem(&setup_menu, &menu_back);


  menu_root_additem(&setup_menu_rs_id, &menu_setup_rs_id);
  menu_root_additem(&setup_menu_rs_jas, &menu_setup_set_jas);

  menu_root_additem(&rm_error, &item_error);


  ds2482_address[0].i2c_addr = 0b0011000;
  ds2482_address[0].HWwirenum = 0;
  ds2482_address[0].hwwire_cekam = false;

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

  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B  = (1 << CS31);
  timer3_counter = 61100;/// cca 450Hz
  TCNT3 = timer3_counter;
  TIMSK3 |= (1 << TOIE3);



  //TCCR0B = TCCR0B & 0b11111000 | 0x02;
  //TCCR1B = TCCR1B & 0b11111000 | 0x02;

  interrupts();             // enable all interrupts

  rtc.begin();


  GLCD_Setup();
  glcd_init_header();
  GLCD_flip();
  GLCD_Render();
  delay(100);


  for (uint8_t inic = 0; inic < 11; inic++)
  {
    if (inic == 0)
    {
      if (EEPROM.read(set_default_values) == 255)
      {
        glcd_init_header();
        GLCD_GotoXY(0, 30);
        GLCD_PrintString("... reset hodnot");
        GLCD_flip();
        GLCD_Render();
        EEPROM.write(set_default_values, 0);
        EEPROM.write(my_jas_disp, 240);
        EEPROM.write(my_jas_key, 240);
        EEPROM.write(my_rs_id, 31);
        max_program = 0;
        thermostat_set_availability_program();
        thermostat_set_mode(0);
        EEPROM.write(my_sense, 127);
        for (uint8_t idx = 0; idx < MAX_THERMOSTAT; idx++)
        {
          thermostat_set_asociate_tds(idx, 255);
          thermostat_set_mezni(idx, 220);
          thermostat_set_program_id(idx, 0);
          thermostat_set_stav(idx, 0);
          thermostat_set_ready(idx, 0);
          thermostat_set_output(idx, 255);
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
        device_set_name("TERM E1");
        rtc.adjust(DateTime(2018, 12, 14, 17, 14, 0));
        device.mac[0] = 2; device.mac[1] = 1; device.mac[2] = 2; device.mac[3] = 3; device.mac[4] = 4; device.mac[5] = 5;
        device.myIP[0] = 192; device.myIP[1] = 168; device.myIP[2] = 2; device.myIP[3] = 111;
        device.myMASK[0] = 255; device.myMASK[1] = 255; device.myMASK[2] = 255; device.myMASK[3] = 0;
        device.myGW[0] = 192; device.myGW[1] = 168; device.myGW[2] = 2; device.myGW[3] = 1;
        device.myDNS[0] = 192; device.myDNS[1] = 168; device.myDNS[2] = 2; device.myDNS[3] = 1;
        device.mqtt_server[0] = 192; device.mqtt_server[1] = 168; device.mqtt_server[2] = 2; device.mqtt_server[3] = 1;
        device.mqtt_port = 1883;
        strcpy(device.mqtt_user, "saric");
        strcpy(device.mqtt_key, "no");
        save_setup_network();
      }
    }

    if (inic == 1)
    {
      glcd_init_header();
      GLCD_GotoXY(0, 30);
      GLCD_PrintString("... sitove volby");
      GLCD_flip();
      GLCD_Render();
      load_setup_network();
    }

    if (inic == 2)
    {
      glcd_init_header();
      GLCD_GotoXY(0, 30);
      GLCD_PrintString("... ostatni volby");
      GLCD_flip();
      GLCD_Render();
      jas_disp = EEPROM.read(my_jas_disp);
      jas_key = EEPROM.read(my_jas_key);
      rsid = EEPROM.read(my_rs_id);
      analogWrite(PWM_DISP, jas_disp);
      analogWrite(PWM_KEY, jas_key);
      thermostat_get_availability_program();
      thermostat_get_mode();
    }


    if (inic == 3)
    {
      glcd_init_header();
      GLCD_GotoXY(0, 30);
      GLCD_PrintString("... init 1w rozhrani");

      itoa(ds2482_address[0].i2c_addr, tmp1, 10);
      if (ds2482reset(ds2482_address[0].i2c_addr) == DS2482_ERR_OK)
      {
        strcpy(tmp2, "ds2482:");
        strcat(tmp2, tmp1);
        strcat(tmp2, "=OK");
        GLCD_GotoXY(0, 40);
        GLCD_PrintString(tmp2);
      }
      else
      {
        strcpy(tmp2, "ds2482:");
        strcat(tmp2, tmp1);
        strcat(tmp2, "=ERR");
        GLCD_GotoXY(0, 40);
        GLCD_PrintString(tmp2);
      }
      Global_HWwirenum = 0;
      one_hw_search_device(0);
      strcpy(tmp2, "found 1wire:");
      itoa(Global_HWwirenum, tmp1, 10);
      strcat(tmp2, tmp1);
      if (Global_HWwirenum == 0) strcat(tmp2, "!!!");
      GLCD_GotoXY(0, 52);
      GLCD_PrintString(tmp2);
      GLCD_flip();
      GLCD_Render();
    }

    if (inic == 4)
    {
      glcd_init_header();
      GLCD_GotoXY(0, 30);
      GLCD_PrintString("... init klavesnice");
      GLCD_flip();
      GLCD_Render();
      init_mpc();
      write_to_mcp(255);
      for (uint8_t k = 0; k < 2; k++)
      {
        for (uint8_t i = 0; i < 8; i++)
        {
          write_to_mcp(~(1 << i));
          tecka = (1 << i);
          delay(100);
        }
        for (uint8_t i = 0; i < 8; i++)
        {
          write_to_mcp(~(1 << (7 - i)));
          tecka = (1 << (7 - i));
          delay(100);
        }
      }
      tecka = 0;
      write_to_mcp(255);
    }

    if (inic == 5)
    {
      glcd_init_header();
      GLCD_GotoXY(0, 30);
      GLCD_PrintString("... kalibrace klavesnice");
      GLCD_flip();
      GLCD_Render();
      kalibrace_touchpad();
    }

    if (inic == 6)
    {
      if (!rtc.isrunning())
      {
        glcd_init_header();
        GLCD_GotoXY(0, 30);
        GLCD_PrintString("... init RTC");
        GLCD_GotoXY(0, 42);
        GLCD_PrintString("... chyba!!!");
        GLCD_flip();
        GLCD_Render();
      }
      else
      {
        glcd_init_header();
        GLCD_GotoXY(0, 30);
        GLCD_PrintString("... init RTC");
        GLCD_GotoXY(0, 42);
        GLCD_PrintString("... OK ...");
        GLCD_flip();
        GLCD_Render();
      }
    }

    if (inic == 7)
    {
      glcd_init_header();
      GLCD_GotoXY(0, 30);
      GLCD_PrintString("... init mqtt");
      GLCD_flip();
      GLCD_Render();
      mqtt_client.setServer(device.mqtt_server, 1883);
      mqtt_client.setCallback(mqtt_callback);
    }

    if (inic == 8)
    {
      glcd_init_header();
      GLCD_GotoXY(0, 30);
      GLCD_PrintString("... init pripojeni");
      GLCD_flip();
      GLCD_Render();
      Ethernet.begin(device.mac, device.myIP, device.myDNS, device.myGW, device.myMASK);
      
    }

    if (inic == 9)
    {
      glcd_init_header();
      GLCD_GotoXY(0, 30);
      GLCD_PrintString("... mqtt connect");
      GLCD_flip();
      GLCD_Render();
      mqtt_reconnect();
    }

    if (inic == 10)
    {
      glcd_init_header();
      GLCD_GotoXY(0, 30);
      GLCD_PrintString("... hotovo ...");
      GLCD_flip();
      GLCD_Render();
    }

    delay(1000);
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


void kalibrace_touchpad(void)
{
  uint8_t itmp = EEPROM.read(my_sense);
  //analogWrite(SENSE, itmp);
}

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
void show_default(char *tmp)
{
  jednotky = tmp[5];
  desitky = tmp[4];
  stovky = tmp[3];
  tisice = tmp[2];
  destisice = tmp[1];
  statisice = tmp[0];
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_term_global(uint8_t key, uint8_t term_id)
{
  int tt = thermostat_get_mezni(term_id);
  if (key == TL_UP)
  {
    delay_show_menu = 0;
    if (tt < MAX_TEMP)
      tt = tt + 5;
    thermostat_set_mezni(term_id, tt);

  }
  if (key == TL_DOWN)
  {
    delay_show_menu = 0;
    if (tt > MIN_TEMP)
      tt = tt - 5;
    thermostat_set_mezni(term_id, tt);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_term_set_global(void)
{
  show_term_set_global_id(0);
}

void show_term_set_global_id(uint8_t id)
{
  /// todo nejak vyresit predani pres parametr ///
  uint8_t term_id = term_fast_set.args1;
  char tmp[10];
  char tmmp[10];
  int tt;
  char c[5];
  tmp[0] = 0;
  tmmp[0] = 0;
  uint8_t te = 0;
  if (thermostat_get_ready(term_id) == 1)
  {
    thermostat_get_name(term_id, tmmp);
    if (strlen(tmmp) == 0)
    {
      strcpy(tmp, "I");
      itoa(term_id,  c, 10);
      strcat(tmp, c);
    }
    else
      for (uint8_t p = 0; p < 2; p++)
      {
        tmp[p] = tmmp[p];
        tmp[p + 1] = 0;
      }
    strcat(tmp, " ");
    te = strlen(tmp);
    tt = thermostat_get_mezni(term_id);
    itoa(tt,  c, 10);
    //// magic kvuli cislu 0.5
    if (tt < 10 ) {
      c[1] = c[0];
      c[0] = '0';
      c[2] = 0;
    }
    /// konec magicu
    if ((tt / 10) >= 10)
      te = te + 1;
    //if ((tt / 10) < 10)
    //  te = te + 2;
    tecka = (1 << te);
    strcat(tmp, c);
  }
  else
  {
    strcpy(tmp, "ID");
    itoa(term_id,  c, 10);
    strcat(tmp, c);
    strcat(tmp, "ERR");
    tecka = 0;
  }
  for (uint8_t j = strlen(tmp); j < 6; j++) tmp[j] = ' ';
  jednotky = tmp[5];
  desitky = tmp[4];
  stovky = tmp[3];
  tisice = tmp[2];
  destisice = tmp[1];
  statisice = tmp[0];
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_term_max(void)
{
  tecka = 0b00000000;
  jednotky = ' ';
  desitky = ' ';
  stovky = 'X';
  tisice = 'A';
  destisice = 'M';
  statisice = ' ';
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_term_climate(void)
{
  tecka = 0b00000000;
  jednotky = ' ';
  desitky = ' ';
  stovky = 'N';
  tisice = 'I';
  destisice = 'M';
  statisice = ' ';
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_term_off(void)
{
  tecka = 0b00000000;
  jednotky = ' ';
  desitky = ' ';
  stovky = 'F';
  tisice = 'F';
  destisice = 'O';
  statisice = ' ';
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_term_prog(void)
{
  char c[3];
  tecka = 0b00000000;
  uint8_t pi = thermostat_get_program_id(0);

  itoa(pi,  c, 10);
  if (pi < 10)
  {
    jednotky = c[0];
    desitky = ' ';
  }
  else
  {
    desitky = c[0];
    jednotky = c[1];
  }

  if ((max_program & (1 << pi)) != 0 )
  {
    stovky = 'A';
  }
  else
  {
    stovky = 'D';
  }

  tisice = ' ';
  destisice = 'R';
  statisice = 'P';
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_term_prog(uint8_t key)
{
  uint8_t poc;
  poc = thermostat_get_program_id(0);
  if (key == TL_UP)
  {
    if (poc < AVAILABLE_PROGRAM) poc++;
    delay_show_menu = 0;
  }
  ////
  if (key == TL_DOWN)
  {
    if (poc > 0) poc--;
    delay_show_menu = 0;
  }
  ///
  thermostat_set_program_id(0, poc);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void back(void)
{
  char tmp[8];
  get_current_item(tmp);
  show_default(tmp);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_termstav(void)
{
  char tmp[8];
  char tml[5];
  tmp[0] = 0;
  tecka = 0b00000000;
  /// TODO neco mi stale prepisuje pametovou banku, overit co by mohlo
  strcpy(tmp, "-    -");

  if (thermostat_get_stav(0) == TERM_STAV_STOP)
  {
    strcpy(tmp, "T OFF ");
  }
  if (thermostat_get_stav(0) == TERM_STAV_TOPI)
  {
    strcpy(tmp, "TON");
    struct_DDS18s20 tds;
    get_tds18s20(0, &tds);
    if  ((tds.used == 1) && (status_tds18s20[0].online == True) && (thermostat_get_ready(0) == 1))
    {
      int tt = (status_tds18s20[0].temp / 100) - int(thermostat_get_mezni(0));
      if (tt >= 0)
      {
        itoa(tt, tml, 10);
        if (tt > 0 && tt < 10)
          strcat(tmp, " 0");
        else
          strcat(tmp, " ");
      }
      else
      {
        itoa(tt * (-1), tml, 10);
        if (tt < 0 && tt > -10)
          strcat(tmp, "-0");
        else
          strcat(tmp, "-");
      }

      if ( ((tt < 100) && (tt >= 0)) || ( (tt > -100) && (tt < 0) ) )
        tecka = 0b00010000;
      else
        tecka = 0b00000000;

      strcat(tmp, tml);
    }

    else
      strcpy(tmp, "T ERR");
  }
  if (thermostat_get_stav(0) == TERM_OK)
  {
    strcpy(tmp, "T *OK*");
  }
  jednotky = tmp[5];
  desitky = tmp[4];
  stovky = tmp[3];
  tisice = tmp[2];
  destisice = tmp[1];
  statisice = tmp[0];
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t Bit_Reverse( uint8_t x )
{
  x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
  x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
  x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
  return x;
}




void external_text_2(uint8_t input)
{
  char c[4];
  external_text[0] = 0;
  if (input < 10) strcpy(external_text, " ");
  itoa(input, c, 10);
  strcat(external_text, c);
  tecka = 0;
}

void external_text_string(char *input)
{
  external_text[0] = 0;
  if (strlen(input) < 2) strcpy(external_text, " ");
  strcat(external_text, input);
  tecka = 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void f_error(void)
{
  jednotky = external_text[5];
  desitky = external_text[4];
  stovky = external_text[3];
  tisice = external_text[2];
  destisice = external_text[1];
  statisice = external_text[0];
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
  uint8_t id;
  uint8_t itmp;
  char cmd[MAX_TEMP_BUFFER];
  char args[MAX_TEMP_BUFFER];
  struct_DDS18s20 tds;




  //if ( Enc28J60Network::linkStatus() == 1)
  //  statisice = 'A';
  //else
  //  statisice = 'E';

  
    if (!mqtt_client.connected()) {
      mqtt_reconnect();
    }
    mqtt_client.loop();
  

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


  get_current_menu(curr_menu);
  get_current_item(curr_item);


  led = 0;
  led_old = 0;
  key = 0;
  delay_key = ~Bit_Reverse(read_from_mcp());
  if (delay_key != 0)
  {
    if ((milis - milis_key) > 100)
    {
      milis_key = milis;
      press_key = delay_key;
    }
  }
  if (delay_key == 0 && press_key != 0 )
  {
    key = press_key;
    press_key = 0;
  }


  /*
    if ((key & TL_OFF) != 0)
    {
    led = led + LED_OFF;
    }

    if ((key & TL_PROG) != 0)
    {
    led = led + LED_PROG;
    }

    if ((key & TL_MAX) != 0)
    {
    led = led + LED_MAX;
    }

    if ((key & TL_STAV) != 0)
    {
    led = led + LED_STAV;
    }

    if ((key & TL_OK) != 0)
    {
    led = led + LED_OK;
    }

    if ((key & TL_DOWN) != 0)
    {
    led = led + LED_DOWN;
    }

    if ((key & TL_UP) != 0)
    {
    led = led + LED_UP;
    }
  */


  if (last_sync > 100)
  {
    led = led + LED_ERR;
  }

  //Serial.println(rsid);

  thermostat_get_mode();
  if (term_mode == TERM_MODE_OFF) led = led + LED_OFF;
  if (term_mode == TERM_MODE_MAX) led = led + LED_MAX;
  if (term_mode == TERM_MODE_PROG) led = led + LED_PROG;
  if (term_mode == TERM_MODE_MAN) led = led + LED_UP + LED_DOWN;
  if (term_mode == TERM_MODE_CLIMATE) led = led + LED_MAX + LED_PROG;

  /// zapisuji do led registru pouze pri zmene
  if (led != led_old)
  {
    led_old = led;
    write_to_mcp(~led);
  }

  ///globalni back
  if (strcmp_P(curr_item, title_item_menu_back) == 0)
  {
    if (key == TL_OK)
    {
      menu_back_root_menu();
      key = 0;
    }
  }

  //// defualt screen uplne hlavni menu
  if (strcmp_P(curr_menu, title_root_termostat) == 0)
  {
    //// hotkeys pridej/uber okamzitou teplotu
    if ((key == TL_UP) || (key == TL_DOWN))
    {
      thermostat_set_mode(TERM_MODE_MAN);
      delay_show_menu = 0;
      menu_set_root_menu(&term_fast_set);
      key = 0;
    }
    /// prepinej zobrazeni cas/teplota/stav termostatu
    if (key == TL_STAV)
    {
      menu_rotate();
      key = 0;
    }
    //// prepnuti do menu setup
    if (key == TL_OK)
    {
      menu_set_root_menu(&setup_menu);
      key = 0;
    }
    //// hotkeys menu vupnuti termostatu
    if (key == TL_OFF)
    {
      thermostat_set_mode(TERM_MODE_OFF);
      menu_set_root_menu(&term_off);
      delay_show_menu = 0;
      key = 0;
    }
    //// hotkeys top na maximalni teplotu
    if (key == TL_MAX)
    {
      thermostat_set_mode(TERM_MODE_MAX);
      delay_show_menu = 0;
      menu_set_root_menu(&term_max);
      key = 0;
    }
    //// hotkeys prepni program
    if (key == TL_PROG)
    {
      thermostat_set_mode(TERM_MODE_PROG);
      delay_show_menu = 0;
      menu_set_root_menu(&term_prog);
      key = 0;
    }

    if (key == (TL_MAX + TL_PROG))
    {
      thermostat_set_mode(TERM_MODE_CLIMATE);
      delay_show_menu = 0;
      menu_set_root_menu(&term_climate);
      key = 0;
    }
  }
  //// menu termostat off
  if (strcmp_P(curr_menu, title_root_termoff) == 0)
  {
    if (key == TL_OFF)
    {
      menu_back_root_menu();
      key = 0;
    }
  }
  //// menu termostat max
  if (strcmp_P(curr_menu, title_root_termmax) == 0)
  {
    if (key == TL_MAX)
    {
      menu_back_root_menu();
      key = 0;
    }
  }
  //// menu termostat climate
  if (strcmp_P(curr_menu, title_root_climate) == 0)
  {
    if (key == (TL_MAX + TL_PROG))
    {
      menu_back_root_menu();
      key = 0;
    }
  }
  //// menu termostat program
  if (strcmp_P(curr_menu, title_root_termprog) == 0)
  {
    set_term_prog(key);
    if (key == TL_PROG)
    {
      menu_back_root_menu();
      key = 0;
    }
  }
  //// menu pridej/uber okamzitou teplotu
  if (strcmp_P(curr_menu, title_root_fastsette) == 0)
  {
    set_term_global(key, 0);
    if (key == TL_OK)
    {
      menu_back_root_menu();
      key = 0;
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  //////////////////// hlavni setup menu
  if (strcmp_P(curr_menu, title_root_setup_menu) == 0)
  {
    if (key == TL_UP) menu_next();
    if (key == TL_DOWN) menu_prev();
    ////
    if (key == TL_OK)
    {
      /// podmenu rs485
      if (strcmp_P(curr_item, title_item_menu_setup_bus) == 0)
      {
        menu_set_root_menu(&setup_menu_rs_id);
        setup_menu_rs_id.args1 = rsid;
        key = 0;
      }
      /// podmenu jas
      if (strcmp_P(curr_item, title_item_menu_setup_jas) == 0)
      {
        menu_set_root_menu(&setup_menu_rs_jas);
        setup_menu_rs_jas.args1 = (255 - jas_disp) / 15;
        key = 0;
      }
    }
  }
  /// konec hlavniho setup menu
  /////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  /// nastaveni jasu
  if (strcmp_P(curr_menu, title_item_menu_setup_jas) == 0)
  {
    if (strcmp_P(curr_item, title_item_setup_jas) == 0)
    {
      uint8_t tmp8_1 = setup_menu_rs_jas.args1;
      if (key == TL_MAX)
      {
        if (tmp8_1 < 17)tmp8_1++;
      }
      if (key == TL_OFF)
      {
        if (tmp8_1 > 1)tmp8_1--;
      }
      setup_menu_rs_jas.args1 = tmp8_1;
      jas_disp = 255 - (15 * tmp8_1);
      if (tmp8_1 == 17)
      {
        /// auto jas
        external_text_string("A");
      }
      else
      {
        /// manualni jas
        external_text_2(tmp8_1);
        analogWrite(PWM_DISP, jas_disp);
      }
      if (key == TL_OK)
      {
        EEPROM.write(my_jas_disp, jas_disp);
        menu_back_root_menu();
      }
    }
  }

  /////////////////////////
  /*
    get_current_menu(curr_menu);
    get_current_item(curr_item);
    /// nasteveni rs id
    if (strcmp_P(curr_menu, title_item_menu_setup_bus) == 0)
    {
    if (strcmp_P(curr_item, title_item_setup_rs_id) == 0)
    {
      char tmp8_1 = setup_menu_rs_id.args1;
      if (key == TL_MAX)
      {
        tmp8_1++;
        if (tmp8_1 > 31) tmp8_1 = 0;
      }
      if (key == TL_OFF)
      {
        tmp8_1--;
        if (tmp8_1 < 0) tmp8_1 = 31;
      }
      external_text_2(tmp8_1);
      setup_menu_rs_id.args1 = uint8_t(tmp8_1);

      ///
      if (key == TL_OK)
      {
        menu_back_root_menu();
        rsid = uint8_t(tmp8_1);
        EEPROM.write(my_rs_id, rsid);
        key = 0;
      }
    }
    }
    //// konec nastaveni rs id
    ///////////////////////////////////////////////
  */



  ////////////////////
  /// kazdych 10sec
  if ((milis - milis_10s) > 3500)
  {
    milis_10s = milis;
  }


  if ((milis - milis_05s) > 175)
  {
    milis_05s = milis;
    now = rtc.now();
    menu_display();
    if (last_sync < 200)
      last_sync++;
  }


  ////////////////////
  /// kazdou 1sec
  if ((milis - milis_1s) > 350)
  {
    milis_1s = milis;
    uptime++;
    mereni_hwwire(1);

    aktual_light = analogRead(LIGHT);
    itmp = (aktual_light / 4 / 15 * 15);
    if (jas_disp == 0) // Automatika
    {
      if (itmp > 240) itmp = 240;
      analogWrite(PWM_DISP, itmp);
    }
    ;
    /// vraceni se zpet po 10 sec
    if (strcmp_P(curr_menu, title_root_fastsette) == 0)
    {
      delay_show_menu++;
      if (delay_show_menu > 10) menu_back_root_menu();
    }
    if (strcmp_P(curr_menu, title_root_termoff) == 0)
    {
      delay_show_menu++;
      if (delay_show_menu > 5) menu_back_root_menu();
    }
    if (strcmp_P(curr_menu, title_root_termmax) == 0)
    {
      delay_show_menu++;
      if (delay_show_menu > 5) menu_back_root_menu();
    }
    if (strcmp_P(curr_menu, title_root_termprog) == 0)
    {
      delay_show_menu++;
      if (delay_show_menu > 10) menu_back_root_menu();
    }
    if (strcmp_P(curr_menu, title_root_climate) == 0)
    {
      delay_show_menu++;
      if (delay_show_menu > 10) menu_back_root_menu();
    }
    if (strcmp_P(curr_menu, title_error) == 0)
    {
      delay_show_menu++;
      if (delay_show_menu > 2) menu_back_root_menu();
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

  digitalWrite(LED, 0);

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
  digitalWrite(LED, 1);
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  void external_text_1_float3(uint8_t input1, float input2)
  {
  char c[4];
  external_text[0] = 0;
  itoa(input1, c, 10);
  strcat(external_text, c);
  itoa(input2 * 10, c, 10);
  strcat(external_text, c);
  tecka = 0b00010000;
  }
*/
/*
  void external_text_1_2(uint8_t input1, uint8_t input2, uint8_t mode)
  {
  char c[4];
  external_text[0] = 0;
  itoa(input1, c, 10);
  strcat(external_text, c);
  if ((mode == 10) && (input2 < 10)) strcat(external_text, "0");
  if ((mode == 16) && (input2 < 16)) strcat(external_text, "0");
  itoa(input2, c, mode);
  strcat(external_text, c);
  tecka = 0;
  }
*/
