#include <avr/pgmspace.h>
#include "font_alfa.h"

#include <RTClib.h>
#include <saric_ds2482.h>
#include <ow.h>
#include <EEPROM.h>

#define HW_ONEWIRE_MAXROMS 5
#define HW_ONEWIRE_MAXDEVICES 5
#define MAX_THERMOSTAT 3
#define MAX_TEMP 32.0
#define MIN_TEMP 16.0


uint8_t max_program = 0;
uint8_t term_mode = 0;

int timer1_counter;
uint8_t display_pos;

uint8_t statisice = '-';
uint8_t destisice = '-';
uint8_t  tisice = '-';
uint8_t  stovky = '-';
uint8_t  desitky = '-';
uint8_t  jednotky = '-';

uint8_t tecka = 0xff;

long int milis = 0;
long int milis_05s = 0;
long int milis_1s = 0;
long int milis_10s = 0;
long int milis_key = 0;


#define gnd0  2
#define gnd1  7
#define gnd2  8
#define gnd3  9
#define gnd4  10
#define gnd5  12

#define rs485 3
#define PWM   11

#define DP    14

#define SER1  15
#define SCLK  16
#define PCLK  17

#define LIGHT A7



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



#define TERM_STAV_STOP 0
#define TERM_STAV_TOPI 1
#define TERM_STAV_HIST 2

#define TERM_MODE_OFF 0
#define TERM_MODE_MAX 1
#define TERM_MODE_PROG 2
#define TERM_MODE_MAN 3

#define eeprom_term_mode 97
#define eeprom_max_program 98
#define my_rs_id 99


#define eeprom_thermostat_0 100
/*
   char name 0..9
   float mezni 10..14
   uint8_t program 15
   uint8_t asociate_tds 16
*/
#define eeprom_thermostat_1 120

#define eeprom_thermostat_2 130


#define wire_know_rom_0 140
#define wire_know_rom_4 220





typedef struct struct_thermostat
{
  uint8_t stav;
};



struct_1w_rom w_rom[HW_ONEWIRE_MAXROMS];
struct_ds2482 ds2482_address[1];

//struct_DDS18s20 tds18s20[HW_ONEWIRE_MAXROMS];
struct_status_DDS18s20 status_tds18s20[HW_ONEWIRE_MAXROMS];

struct_thermostat thermostat[MAX_THERMOSTAT];



uint8_t Global_HWwirenum = 0;
uint8_t tmp_rom[8];

uint8_t key, delay_key;
uint8_t led, led_old;

uint8_t rsid;

uint8_t ppwm = 0;

#define TL_OFF  0b10000000
#define TL_MAX 0b01000000
#define TL_PROG  0b00100000
#define TL_STAV 0b00001000
#define TL_OK   0b00000100
#define TL_DOWN 0b00000010
#define TL_UP   0b00000001


#define LED_OFF  0b10000000
#define LED_MAX 0b01000000
#define LED_PROG  0b00100000
#define LED_STAV 0b00010000
#define LED_OK   0b00001000
#define LED_DOWN 0b00000100
#define LED_UP   0b00000010
#define LED_ERR  0b00000001

const char daysOfTheWeek[7][12] PROGMEM = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


const char title_root_termostat[] PROGMEM = "Termostat";
const char title_root_fastsette[] PROGMEM = "FastSetT";
const char title_root_termprog[] PROGMEM = "TermProg";
const char title_root_termmax[] PROGMEM = "TermMax";
const char title_root_termoff[] PROGMEM = "TermOff";


const char title_root_setup_menu[] PROGMEM = "Setup";
const char title_error[] PROGMEM = "error";


const char title_item_menu_temp[] PROGMEM = "Teplota";
const char title_item_menu_time[] PROGMEM = "Cas";
const char title_item_menu_termstav[] PROGMEM = "TermStav";

const   char title_item_menu_termset[] PROGMEM = "Termset";



const char title_item_menu_setup_bus[] PROGMEM =  "nRS485";
const char title_item_menu_setup_tds[] PROGMEM = "nTDS$$";
const char title_item_menu_back[] PROGMEM = "ZPET  ";



const char title_item_setup_rs_id[] PROGMEM  = "nRS $$";



uint8_t delay_show_menu = 0;

uint8_t start_at = 0;
uint8_t re_at = 0;

#define MAX_TEMP_BUFFER 32
#define UART_RECV_MAX_SIZE 64
uint8_t r_id = 0;
char uart_recv[UART_RECV_MAX_SIZE];



typedef void (*function)();

#define MAX_ITEMS_ROOT_MENU 8
#define MAX_HISTORY 6

char external_text[8];


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


RTC_DS1307 rtc;
DateTime now;

RootMenu rm;
RootMenu term_fast_set;

RootMenu term_max;
RootMenu term_off;
RootMenu term_prog;
RootMenu setup_menu;
RootMenu setup_menu_date;
RootMenu setup_menu_time;
RootMenu setup_menu_rs_id;



RootMenu rm_error;



ItemMenu menu_show_temp, menu_show_time, menu_show_termstav;
ItemMenu menu_show_term_set_global;

ItemMenu menu_show_term_max;
ItemMenu menu_show_term_off;
ItemMenu menu_show_term_prog;


ItemMenu menu_setup_bus,  menu_back;


ItemMenu menu_setup_rs_id;



ItemMenu item_error;

Struct_RootHistory RootHistory;

///////////////////////////////////////////////
/*

   typedef struct struct_DDS18s20
  {
  uint8_t volno;
  uint8_t rom[8];
  uint8_t assigned_ds2482;
  int offset;
  char nazev[8];
  };
*/

////////////////////////////////////////////////////////////////////////
void get_tds18s20(uint8_t idx, struct_DDS18s20 *tds)
{
  tds->used = EEPROM.read(wire_know_rom_0 + (idx * 20));
  for (uint8_t m = 0; m < 8; m++) tds->rom[m] = EEPROM.read(wire_know_rom_0 + (idx * 20) + 1 + m);
  tds->assigned_ds2482 = EEPROM.read(wire_know_rom_0 + (idx * 20) + 9);
  tds->offset = (EEPROM.read(wire_know_rom_0 + (idx * 20) + 10) << 8) + EEPROM.read(wire_know_rom_0 + (idx * 20) + 11);
  for (uint8_t m = 0; m < 8; m++) tds->name[m] = EEPROM.read(wire_know_rom_0 + (idx * 20) + 12 + m);
}

void set_tds18s20(uint8_t idx, struct_DDS18s20 *tds)
{
  EEPROM.write(wire_know_rom_0 + (idx * 20), tds->used);
  for (uint8_t m = 0; m < 8; m++) EEPROM.write(wire_know_rom_0 + (idx * 20) + 1 + m, tds->rom[m]);
  EEPROM.write(wire_know_rom_0 + (idx * 20) + 9, tds->assigned_ds2482 );
  EEPROM.write(wire_know_rom_0 + (idx * 20) + 10, (tds->offset >> 8) & 0xff);
  EEPROM.write(wire_know_rom_0 + (idx * 20) + 11, (tds->offset) & 0xff);
  for (uint8_t m = 0; m < 8; m++) EEPROM.write(wire_know_rom_0 + (idx * 20) + 12 + m, tds->name[m]);
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void new_parse_at(char *input, char *out1, char *out2)
{

  uint8_t count = 0;
  uint8_t q = 0;

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
  strcpy(tmp1, "at+");
  itoa(id, tmp2, 10);
  strcat(tmp1, tmp2);
  strcat(tmp1, ",");
  strcat(tmp1, cmd);
  strcat(tmp1, ",");
  strcat(tmp1, args);
  strcat(tmp1, ";");
  Serial.println(tmp1);

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
  //Serial.print("menu_max: "); Serial.println(RootHistory.menu_max);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void menu_root_setname(RootMenu *rm, char *name)
{
  //strcpy(rm->name, name);
  rm->name = name;
}
///////////////////////////////////////
bool menu_root_additem(RootMenu *rm, ItemMenu *item)
{
  if (rm->items < MAX_ITEMS_ROOT_MENU)
  {
    rm->Item[rm->items] = item;
    rm->items++;
    return true;
  }
  else
    return false;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

void thermostat_get_name(uint8_t idx, char *name)
{
  char t;
  for (uint8_t i = 0; i < 9; i++)
  {
    t = EEPROM.read((eeprom_thermostat_0 + (10 * idx)) + i);
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
    EEPROM.write((eeprom_thermostat_0 + (10 * idx)) + i, t);
    if (t == 0) break;
  }
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

void thermostat_get_max_program(void)
{
  max_program = EEPROM.read(eeprom_max_program);
}

void thermostat_set_max_program(void)
{
  EEPROM.write(eeprom_max_program, max_program);
}

uint8_t thermostat_get_program_id(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (10 * idx)) + 15);
}

void thermostat_set_program_id(uint8_t idx, uint8_t id)
{
  return EEPROM.write((eeprom_thermostat_0 + (10 * idx)) + 15, id);
}

float thermostat_get_mezni(uint8_t idx)
{
  return EEPROMreadFloat((eeprom_thermostat_0 + (10 * idx)) + 10);
}

float thermostat_set_mezni(uint8_t idx, float temp)
{
  EEPROMwriteFloat((eeprom_thermostat_0 + (10 * idx)) + 10, temp);
}

uint8_t thermostat_get_asociate_tds(uint8_t idx)
{
  return EEPROM.read((eeprom_thermostat_0 + (10 * idx)) + 16);
}

void thermostat_set_asociate_tds(uint8_t idx, uint8_t id)
{
  EEPROM.write((eeprom_thermostat_0 + (10 * idx)) + 16, id);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void) {
  char tmp1[20];
  char tmp2[20];

  // put your setup code here, to run once:
  // komunikace přes sériovou linku rychlostí 9600 baud
  Serial.begin(9600);
  // zapnutí komunikace knihovny s teplotním čidlem






  menu_history_init(&rm);

  menu_init(&rm);
  menu_init(&term_fast_set);

  menu_init(&term_max);
  menu_init(&term_off);
  menu_init(&term_prog);
  menu_init(&setup_menu);

  menu_init(&setup_menu_date);
  menu_init(&setup_menu_time);
  menu_init(&setup_menu_rs_id);


  menu_init(&rm_error);

  menu_root_setname(&rm, title_root_termostat);
  menu_root_setname(&term_fast_set, title_root_fastsette);
  menu_root_setname(&term_prog, title_root_termprog);
  menu_root_setname(&term_max, title_root_termmax);
  menu_root_setname(&term_off, title_root_termoff);
  menu_root_setname(&setup_menu, title_root_setup_menu);
  menu_root_setname(&setup_menu_rs_id, title_item_menu_setup_bus);


  menu_root_setname(&rm_error, title_error);

  menu_item_set_properties(&menu_show_temp, title_item_menu_temp, show_temp_default);
  menu_item_set_properties(&menu_show_time, title_item_menu_time, show_time);
  menu_item_set_properties(&menu_show_term_set_global, title_item_menu_termset, show_term_set_global);
  menu_item_set_properties(&menu_show_termstav, title_item_menu_termstav, show_termstav);

  menu_item_set_properties(&menu_show_term_max, title_root_termmax, show_term_max);
  menu_item_set_properties(&menu_show_term_off, title_root_termoff, show_term_off);
  menu_item_set_properties(&menu_show_term_prog, title_root_termprog, show_term_prog);


  menu_item_set_properties(&menu_setup_bus, title_item_menu_setup_bus, nullptr);
  menu_item_set_properties(&menu_back, title_item_menu_back, back);



  menu_item_set_properties(&menu_setup_rs_id, title_item_setup_rs_id, nullptr);



  menu_item_set_properties(&item_error, title_error, f_error);


  menu_root_additem(&rm, &menu_show_time);
  menu_root_additem(&rm, &menu_show_temp);
  menu_root_additem(&rm, &menu_show_termstav);

  menu_root_additem(&term_fast_set, &menu_show_term_set_global);

  menu_root_additem(&term_prog, &menu_show_term_prog);
  menu_root_additem(&term_max, &menu_show_term_max);
  menu_root_additem(&term_off, &menu_show_term_off);


  menu_root_additem(&setup_menu, &menu_setup_bus);
  menu_root_additem(&setup_menu, &menu_back);


  menu_root_additem(&setup_menu_rs_id, &menu_setup_rs_id);


  menu_root_additem(&rm_error, &item_error);




  //thermostat_set_name(0, "GL");

  ds2482_address[0].i2c_addr = 0b0011000;
  ds2482_address[0].HWwirenum = 0;
  ds2482_address[0].hwwire_cekam = false;

  pinMode(gnd0, OUTPUT);
  pinMode(gnd1, OUTPUT);
  pinMode(gnd2, OUTPUT);
  pinMode(gnd3, OUTPUT);
  pinMode(gnd4, OUTPUT);
  pinMode(gnd5, OUTPUT);

  pinMode(rs485, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(DP, OUTPUT);

  pinMode(SER1, OUTPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(PCLK, OUTPUT);
  //analogReference(INTERNAL);


  display_pos = 0;

  digitalWrite(rs485, HIGH);

  rsid = EEPROM.read(my_rs_id);
  thermostat_get_max_program();
  thermostat_get_mode();

  rtc.begin();

  itoa(ds2482_address[0].i2c_addr, tmp1, 10);
  if (ds2482reset(ds2482_address[0].i2c_addr) == DS2482_ERR_OK)
  {
    strcpy(tmp2, "ds2482: ");
    strcat(tmp2, tmp1);
    strcat(tmp2, " = OK");
    Serial.print("init: ds2482 OK: "); Serial.println(ds2482_address[0].i2c_addr);
  }
  else
  {
    strcpy(tmp2, "ds2482: ");
    strcat(tmp2, tmp1);
    strcat(tmp2, " = ERR");
    Serial.print("init: ds2482 error: "); Serial.println(ds2482_address[0].i2c_addr);
  }

  Global_HWwirenum = 0;
  one_hw_search_device(0);
  strcpy(tmp2, "found 1wire: ");
  itoa(Global_HWwirenum, tmp1, 10);
  strcat(tmp2, tmp1);
  if (Global_HWwirenum == 0) strcat(tmp2, " !!! ");
  Serial.println(tmp2);

  init_mpc();
  write_to_mcp(0b111111111);

  if (!rtc.isrunning())
  {
    Serial.println("RTC not running");
    rtc.adjust(DateTime(2018, 12, 14, 15, 14, 0));
  }
  else
  {
    Serial.println("RTC OK");
    now = rtc.now();
    Serial.print(now.hour()); Serial.print(":"); Serial.print(now.minute()); Serial.print(":") ; Serial.println(now.second());
  }


  // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  timer1_counter = 65357; //350Hz
  //timer1_counter = 65457; //350Hz

  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

  digitalWrite(gnd0, 1);
  digitalWrite(gnd1, 1);
  digitalWrite(gnd2, 1);
  digitalWrite(gnd3, 1);
  digitalWrite(gnd4, 1);
  digitalWrite(gnd5, 1);
  digitalWrite(DP, 1);

  struct_DDS18s20 tds;

  //setPwmFrequency(PWM, 8);
  TCCR2B = TCCR2B & 0b11111000 | 0x02;
  //analogWrite(PWM, 50);



  /*
    for (uint8_t idx = 0; idx < ds2482_address[0].HWwirenum; idx++ )
    {
    for (uint8_t m = 0; m < 8; m++)
    {
      tds.rom[m] = w_rom[idx].rom[m];
      Serial.print(w_rom[idx].rom[m]);
    }
    tds.volno = 1;
    tds.offset = 3000;
    tds.assigned_ds2482 = ds2482_address[w_rom[idx].assigned_ds2482].i2c_addr;
    //set_tds18s20(idx, &tds);
    }
    Serial.println(" ");


    get_tds18s20(0, &tds);
    for (uint8_t m = 0; m < 8; m++)
    {
    //tds.rom[m];
    Serial.print(tds.rom[m]);
    }
  */
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
      { //hledani dokonceno
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
  shiftOut(SER1, SCLK, LSBFIRST, data & 0xff);
  shiftOut(SER1, SCLK, LSBFIRST, data >> 8);
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
void set_term_global(uint8_t key)
{
  float tt = thermostat_get_mezni(0);
  if ((key & TL_UP) != 0)
  {
    delay_show_menu = 0;
    if (tt < MAX_TEMP)
      tt = tt + 0.5;
    thermostat_set_mezni(0, tt);

  }
  if ((key & TL_DOWN) != 0)
  {
    delay_show_menu = 0;
    if (tt > MIN_TEMP)
      tt = tt - 0.5;
    thermostat_set_mezni(0, tt);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_term_set_global(void)
{
  char tmp[10];
  char tmmp[10];
  float tt;
  char c[5];
  tmp[0] = 0;
  tmmp[0] = 0;
  uint8_t te = 0;
  thermostat_get_name(0, tmmp);
  for (uint8_t p = 0; p < 2; p++)
  {
    tmp[p] = tmmp[p];
    tmp[p + 1] = 0;
  }
  strcat(tmp, " ");
  tt = thermostat_get_mezni(0) * 10;
  te = strlen(tmp);
  itoa(tt,  c, 10);
  if ((tt / 10) > 10)
    te = te + 1;
  else
    te = te + 2;
  tecka = (1 << te);
  if (tt == 0) tecka = 0;
  strcat(tmp, c);
  //Serial.println(tmp);
  //Serial.println(tecka);
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

  stovky = 'G';
  tisice = 'O';
  destisice = 'R';
  statisice = 'P';
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_term_prog(uint8_t key)
{
  uint8_t poc;
  poc = thermostat_get_program_id(0);
  //if ( pi == 255) poc = 5;
  //else poc = pi;
  /////
  if ((key & TL_UP) != 0)
  {
    if (poc < max_program) poc++;
    delay_show_menu = 0;
  }
  ////
  if ((key & TL_DOWN) != 0)
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  #define TERM_STAV_STOP 0
  #define TERM_STAV_TOPI 1
  #define TERM_STAV_HIST 2
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void show_termstav(void)
{
  char tmp[8];
  tmp[0] = 0;
  tecka = 0b00000000;
  if (thermostat[0].stav == TERM_STAV_STOP)
  {
    strcpy(tmp, "T VYP");
  }
  if (thermostat[0].stav == TERM_STAV_TOPI)
  {
    strcpy(tmp, "T ZAP");
  }
  if (thermostat[0].stav == TERM_STAV_HIST)
  {
    strcpy(tmp, "T ***");
  }
  jednotky = ' ';
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void external_text_1_float3(uint8_t input1, float input2)
{
  char c[4];
  external_text[0] = 0;
  itoa(input1, c, 10);
  strcat(external_text, c);
  itoa(input2 * 10, c, 10);
  //dtostrf(input2, 3, 1, c);
  strcat(external_text, c);
  tecka = 0b00010000;
}

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


void external_text_2(uint8_t input)
{
  char c[4];
  external_text[0] = 0;
  if (input < 10) strcpy(external_text, " ");
  itoa(input, c, 10);
  strcat(external_text, c);
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
void loop()
{
  char str1[32];
  char str2[20];
  char str3[20];
  char curr_menu[10];
  char curr_item[10];
  uint8_t id;
  char cmd[MAX_TEMP_BUFFER];
  char args[MAX_TEMP_BUFFER];
  struct_DDS18s20 tds;


  read_at(uart_recv);


  if (start_at == 255)
  {
    start_at = 0;

    new_parse_at(uart_recv, str1, str2);
    id = atoi(str1);

    new_parse_at(str2, cmd, args);

    /// AT+255,css,0; nastavi sekundy cas set second
    strcpy(str1, "css");
    if ((id == rsid || id == 255) && strcmp(cmd, str1) == 0)
    {
      if (atoi(args) < 60)
      {
        rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), atoi(args)));
        send_at(rsid, cmd, "OK");
      }
      else
        send_at(rsid, cmd, "ERR");
    }

    /// AT+255,csm,54; nastavi minuty cas set minute
    strcpy(str1, "csm");
    if ((id == rsid || id == 255) && strcmp(cmd, str1) == 0)
    {
      if (atoi(args) < 60)
      {
        rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), atoi(args), now.second()));
        send_at(rsid, cmd, "OK");
      }
      else
        send_at(rsid, cmd, "ERR");
    }
    /// AT+255,csh,12; nastavi hodiny    cas set hour
    strcpy(str1, "csh");
    if ((id == rsid || id == 255) && strcmp(cmd, str1) == 0)
    {
      if (atoi(args) < 24)
      {
        rtc.adjust(DateTime(now.year(), now.month(), now.day(), atoi(args), now.minute(), now.second()));
        send_at(rsid, cmd, "OK");
      }
      else
        send_at(rsid, cmd, "ERR");
    }

    if (id == rsid)
    {
      ///at+12,ping; //// ping
      strcpy(str1, "ping");
      if (strcmp(cmd, str1) == 0)
      {
        strcpy(str2, "OK");
        send_at(rsid, cmd, str2);
      }
      //////////////////////////////////////////////////////////////////////////////////
      ///at+12,gt,ID; ziska teplotu,, odpoved at+12,gt,ID,212;
      strcpy(str1, "gt");
      if (strcmp(cmd, str1) == 0)
      {
        uint8_t tmp2 = atoi(args);
        get_tds18s20(tmp2, &tds);
        if ((tds.used == 1) && (status_tds18s20[tmp2].online == True))
        {
          int tt = status_tds18s20[tmp2].temp / 100;
          itoa(tt, str1, 10);
          strcpy(str2, args);
          strcat(str2, ",");
          strcat(str2, str1);
          send_at(rsid, cmd, str2);
        }
        else
        { ///// nenalezeno a nebo offline
          strcpy(str2, args);
          strcat(str2, ",");
          strcat(str2, "ERR");
          send_at(rsid, cmd, str2);
        }
      }
      ///////////////////////////////////////////////////////////////////////////////////
      ///AT+2,wc; vrati pocet 1w cidel ;wire count
      /// odpoved AT+2,wc,2;
      strcpy(str1, "wc");
      if (strcmp(cmd, str1) == 0)
      {
        itoa(Global_HWwirenum, str1, 10);
        send_at(rsid, cmd, str1);
      }
      /////////
      /// AT+2,wm; vrati rom adresy
      strcpy(str1, "wm");
      if (strcmp(cmd, str1) == 0)
        for (uint8_t tmp1 = 0; tmp1 < Global_HWwirenum; tmp1++)
        {
          itoa(tmp1, str1, 10);
          str2[0] = 0;
          for (uint8_t tmp3 = 0; tmp3 < 8; tmp3++)
          {
            uint8_t tmp2 = w_rom[tmp1].rom[tmp3];
            if (tmp2 < 10) strcat(str2, "0");
            itoa(tmp2, str3, 16);
            strcat(str2, str3);
            if (tmp3 < 7) strcat(str2, ":");
          }
          strcat(str1, ",");
          strcat(str1, str2);
          send_at(rsid, cmd, str1);
        }
      ///////////////////////////////////
      ///at+12,stn,id,GL; ///nastavi nazev pro termostat, max 2 znaky zobrazuji jinak 8
      strcpy(str1, "stn");
      if (strcmp(cmd, str1) == 0)
      {
        new_parse_at(args, str3, str2);
        if ( strlen(str2) < 10)
        {
          thermostat_set_name(atoi(str3), str2);
        }
      }
      //////////////////////////////////////
      ///at+12,gtn,id; /// ziska nazev termostatu
      strcpy(str1, "gtn");
      if (strcmp(cmd, str1) == 0)
      {
        thermostat_get_name(atoi(args), str3);
        send_at(rsid, cmd, str3);
      }
      /////////////////////////////////////////
      ///at+12,gtp,id; /// ziska nastaveny program id
      strcpy(str1, "gtp");
      if (strcmp(cmd, str1) == 0)
      {
        itoa(thermostat_get_program_id(atoi(args)), str3, 10);
        send_at(rsid, cmd, str3);
      }
      ////
      ///at+12,stp,idterm,idprog /// nastavi pro program id pro termostat
      strcpy(str1, "stp");
      if (strcmp(cmd, str1) == 0)
      {
        new_parse_at(args, str3, str2);
        thermostat_set_program_id(atoi(str3), atoi(str2));
      }



      ///at+12,skm,ID; nastavi mac adresu 1wire do znamych tds
      strcpy(str1, "skm");
      if (strcmp(cmd, str1) == 0)
      {
        if (atoi(args) < Global_HWwirenum)
        {
          for (uint8_t idx = 0; idx < HW_ONEWIRE_MAXDEVICES; idx++)
          {
            get_tds18s20(idx, &tds);
            if (tds.used == 0)
            {
              tds.used = 1;
              for (uint8_t i = 0; i < 8; i++)
                tds.rom[i] = w_rom[atoi(args)].rom[i];
              set_tds18s20(idx, &tds);
              break;
            }
          }
        }
      }
      /////////////////
      ///at+12,skn,ID,XXXX; ///nastavi nazev cidlu
      strcpy(str1, "skn");
      if (strcmp(cmd, str1) == 0)
      {
        new_parse_at(args, str3, str2);
        get_tds18s20(atoi(str3), &tds);
        strcpy(tds.name, str2);
        set_tds18s20(atoi(str3), &tds);
      }
      ///////////////////////////
      ///at+12,gkn,ID; ///zjisti nazev cidla
      strcpy(str1, "gkn");
      if (strcmp(cmd, str1) == 0)
      {
        get_tds18s20(atoi(args), &tds);
        send_at(rsid, cmd, tds.name);
      }
      //////////////////////////
      ///at+12,ckm,ID; vymaze rom adresu ze znamych tds
      strcpy(str1, "ckm");
      if (strcmp(cmd, str1) == 0)
      {
        if (atoi(args) < HW_ONEWIRE_MAXDEVICES)
        {
          get_tds18s20(atoi(args), &tds);
          tds.used = 0;
          set_tds18s20(atoi(args), &tds);
        }
      }
      ////////////////////////////////////
      ///at+12,stdsoffset,ID,xxxx; nastavi offset cidlu
      strcpy(str1, "stdsoffset");
      if (strcmp(cmd, str1) == 0)
      {
        new_parse_at(args, str3, str2);
        get_tds18s20(atoi(str3), &tds);
        tds.offset = atoi(str2);
        set_tds18s20(atoi(str3), &tds);
      }
      /////////////////////////////////////
      ///at+12,gtdsoffset,ID; ziska nastaveny offset
      strcpy(str1, "gtdsoffset");
      if (strcmp(cmd, str1) == 0)
      {
        get_tds18s20(atoi(str3), &tds);
        itoa(tds.offset, str3, 10);
        send_at(rsid, cmd, str3);
      }
      /////////////////////////////////////
      ///at+12,spc,COUNT; nastavi pocet dostupnych programu
      strcpy(str1, "spc");
      if (strcmp(cmd, str1) == 0)
      {
        max_program = atoi(args);
        thermostat_set_max_program();
      }
      //////////////////////////////////////
      strcpy(str1, "gpc"); ///ziska pocet nastavenych programu
      if (strcmp(cmd, str1) == 0)
      {
        itoa(max_program, str3, 10);
        send_at(rsid, cmd, str3);
      }
      ////////////////////////////////////////
      strcpy(str1, "gtm");  /// at+12,gtm; ziska nastaveny term_mode
      if (strcmp(cmd, str1) == 0)
      {
        thermostat_get_mode();
        itoa(term_mode, str3, 10);
        send_at(rsid, cmd, str3);
      }
      ///////////////////////////////////////////
      strcpy(str1, "stm"); /// at+12,stm,ID; nastavi nastaveny term_mode
      if (strcmp(cmd, str1) == 0)
      {
        thermostat_set_mode(atoi(args));
      }
      //////////////////////////////////////////////////////
      ///at+12,spwm,VAL
      strcpy(str1, "spwm"); /// at+12,spwm,VAL /// nastavi hodnotu jasu dispalye
      if (strcmp(cmd, str1) == 0)
      {
        analogWrite(PWM, atoi(args));
      }

    }


  }

  get_current_menu(curr_menu);
  get_current_item(curr_item);


  led = 0;
  led_old = 0;
  key = 0;
  delay_key = ~Bit_Reverse(read_from_mcp());
  if (delay_key != 0)
  {
    if ((milis - milis_key) > 200)
    {
      milis_key = milis;
      key = delay_key;
    }
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

  thermostat_get_mode();
  if (term_mode == TERM_MODE_OFF) led = led + LED_OFF;
  if (term_mode == TERM_MODE_MAX) led = led + LED_MAX;
  if (term_mode == TERM_MODE_PROG) led = led + LED_PROG;
  if (term_mode == TERM_MODE_MAN) led = led + LED_UP + LED_DOWN;

  /// zapisuji do led registru pouze pri zmene
  if (led != led_old)
  {
    led_old = led;
    write_to_mcp(~led);
  }

  ///globalni back
  if (strcmp_P(curr_item, title_item_menu_back) == 0)
  {
    if ((key & TL_OK) != 0)
    {
      menu_back_root_menu();
      key = 0;
    }
  }

  //// defualt screen uplne hlavni menu
  if (strcmp_P(curr_menu, title_root_termostat) == 0)
  {
    //// hotkeys pridej/uber okamzitou teplotu
    if (((key & TL_UP) != 0) || ((key & TL_DOWN) != 0))
    {
      thermostat_set_mode(TERM_MODE_MAN);
      delay_show_menu = 0;
      menu_set_root_menu(&term_fast_set);
      key = 0;
    }
    /// prepinej zobrazeni cas/teplota/stav termostatu
    if ((key & TL_STAV) != 0)
    {
      menu_rotate();
      key = 0;
    }
    //// prepnuti do menu setup
    if ((key & TL_OK) != 0)
    {
      menu_set_root_menu(&setup_menu);
      key = 0;
    }
    //// hotkeys menu vupnuti termostatu
    if ((key & TL_OFF) != 0)
    {
      thermostat_set_mode(TERM_MODE_OFF);
      menu_set_root_menu(&term_off);
      delay_show_menu = 0;
      key = 0;
    }
    //// hotkeys top na maximalni teplotu
    if ((key & TL_MAX) != 0)
    {
      thermostat_set_mode(TERM_MODE_MAX);
      delay_show_menu = 0;
      menu_set_root_menu(&term_max);
      key = 0;
    }
    //// hotkeys prepni program
    if ((key & TL_PROG) != 0)
    {
      thermostat_set_mode(TERM_MODE_PROG);
      delay_show_menu = 0;
      menu_set_root_menu(&term_prog);
      key = 0;
    }
  }
  //// menu termostat off
  if (strcmp_P(curr_menu, title_root_termoff) == 0)
  {
    if ((key & TL_OFF) != 0)
    {
      menu_back_root_menu();
      key = 0;
    }
  }
  //// menu termostat max
  if (strcmp_P(curr_menu, title_root_termmax) == 0)
  {
    if ((key & TL_MAX) != 0)
    {
      menu_back_root_menu();
      key = 0;
    }
  }
  //// menu termostat program
  if (strcmp_P(curr_menu, title_root_termprog) == 0)
  {
    set_term_prog(key);
    if ((key & TL_OK) != 0)
    {
      menu_back_root_menu();
      key = 0;
    }
  }
  //// menu pridej/uber okamzitou teplotu
  if (strcmp_P(curr_menu, title_root_fastsette) == 0)
  {
    set_term_global(key);
    if ((key & TL_OK) != 0)
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
    if ((key & TL_UP) != 0) menu_next();
    if ((key & TL_DOWN) != 0) menu_prev();
    ////
    if ((key & TL_OK) != 0)
    {
      if (strcmp_P(curr_item, title_item_menu_setup_bus) == 0)
      {
        menu_set_root_menu(&setup_menu_rs_id);
        setup_menu_rs_id.args1 = rsid;
        key = 0;
      }
    }
  }
  /// konec hlavniho setup menu
  /////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////
  get_current_menu(curr_menu);
  get_current_item(curr_item);
  /// nasteveni rs id
  if (strcmp_P(curr_menu, title_item_menu_setup_bus) == 0)
  {
    if (strcmp_P(curr_item, title_item_setup_rs_id) == 0)
    {
      uint8_t tmp8_1 = setup_menu_rs_id.args1;
      if ((key & TL_MAX) != 0)
      {
        tmp8_1++;
        if (tmp8_1 > 99) tmp8_1 = 0;
      }
      if ((key & TL_OFF) != 0)
      {
        tmp8_1--;
        if (tmp8_1 < 0) tmp8_1 = 99;
      }
      external_text_2(tmp8_1);
      setup_menu_rs_id.args1 = tmp8_1;

      ///
      if ((key & TL_OK) != 0)
      {
        menu_back_root_menu();
        rsid = tmp8_1;
        EEPROM.write(my_rs_id, rsid);
        key = 0;
      }
    }
  }
  //// konec nastaveni rs id
  ///////////////////////////////////////////////




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
  }


  ////////////////////
  /// kazdou 1sec
  if ((milis - milis_1s) > 350)
  {
    milis_1s = milis;
    mereni_hwwire(1);

    ////Serial.println(analogRead(LIGHT));
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
ISR(TIMER1_OVF_vect)        // interrupt service routine
{
  uint8_t backup = SREG;
  uint8_t show;
  uint16_t lov;

  TCNT1 = timer1_counter;   // preload timer

  milis++;



  digitalWrite(gnd0, 1);
  digitalWrite(gnd1, 1);
  digitalWrite(gnd2, 1);
  digitalWrite(gnd3, 1);
  digitalWrite(gnd4, 1);
  digitalWrite(gnd5, 1);
  digitalWrite(DP, 1);

  if (display_pos == 0)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[statisice]);
    shiftout(0xffff - lov);
    if ((tecka & 0b00000001) != 0)  digitalWrite(DP, 0);
    digitalWrite(gnd0, 0);
  }
  if (display_pos == 1)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[destisice]);
    shiftout(0xffff - lov);
    if ((tecka & 0b00000010) != 0)  digitalWrite(DP, 0);
    digitalWrite(gnd1, 0);
  }
  if (display_pos == 2)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[tisice]);
    shiftout(0xffff - lov);
    if ((tecka & 0b00000100) != 0)  digitalWrite(DP, 0);
    digitalWrite(gnd2, 0);
  }
  if (display_pos == 3)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[stovky]);
    shiftout(0xffff - lov);
    if ((tecka & 0b00001000) != 0)  digitalWrite(DP, 0);
    digitalWrite(gnd3, 0);
  }
  if (display_pos == 4)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[desitky]);
    shiftout(0xffff - lov);
    if ((tecka & 0b00010000) != 0)  digitalWrite(DP, 0);
    digitalWrite(gnd4, 0);
  }
  if (display_pos == 5)
  {
    lov = pgm_read_word_near(&SixteenAlfaNumeric[jednotky]);
    shiftout(0xffff - lov);
    if ((tecka & 0b00100000) != 0)  digitalWrite(DP, 0);
    digitalWrite(gnd5, 0);
  }
  display_pos++;
  if (display_pos == 6) display_pos = 0;
  SREG = backup;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  uint8_t find_in_interval(int start, int stop, uint8_t day)
  {
  uint8_t ret = 0;

  if (day & (1 << now.dayOfTheWeek()) != 0)
  {
    if ( ((now.hour() * 60 + now.minute()) >= start) && ((now.hour() * 60 + now.minute()) <= stop))
      ret = 1;
    else
      ret = 2;
  }
  else
    ret = 3;
  return ret;
  }
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
/*
  uint8_t timeprogram_new(void)
  {
  /// programtime status registry
  /// bit 0 ... free
  uint8_t state;
  uint8_t fre = 0;
  uint8_t idx = 0;
  /// hledam volnou pamet pro novy program
  for (idx = 0; idx < MAX_PROGRAM_TIME + 1; idx++)
  {
    state = thermostat_program_time_get_status(idx);
    /// mam volnou pamet pro novy program
    if ((state & 1) == 0)
    {
      state = state | 0b00000001;
      thermostat_program_time_set_status(idx, state);
      fre++;
      break;
    }
  }
  if (fre == 0)
  {
    strcpy(external_text, "nTFULL");
    idx = 255;
  }
  return idx;
  }
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/*
  uint8_t program_new(void)
  {
  uint8_t idx = 0;
  uint8_t state;
  uint8_t fre;
  for (idx = 0; idx < MAX_PROGRAM + 1; idx++)
  {
    state = thermostat_program_get_status(idx);
    if ((state & 1) == 0)
    {
      state = state | 0b00000001;
      thermostat_program_set_status(idx, state);
      fre++;
      break;
    }
  }
  if (fre == 0)
  {
    strcpy(external_text, "nTFULL");
    idx = 255;
  }

  return idx;
  }

  uint8_t save_program(uint8_t prog, uint8_t val)
  {
  uint8_t ret = 255;
  for (uint8_t idx = 0; idx < MAX_PROGRAM_TIME + 1; idx++)
    if (thermostat_program_get_timerange(prog, idx) == 255)
    {
      thermostat_program_set_timerange(prog, idx, val);
      ret = idx;
      break;
    }
  return ret;
  }

  void delete_program(uint8_t prog, uint8_t idx)
  {
  for (uint8_t i = 0; i < MAX_PROGRAM_TIME + 1; i++)
    if (thermostat_program_get_timerange(prog, i) == idx)
    {
      thermostat_program_set_timerange(prog, i, 255);
      break;
    }
  }
*/

/*
  uint8_t thermostat_program_time_get_status(uint8_t idx)
  {
  uint8_t state;
  state = EEPROM.read(program_time_status_0 + idx);
  return state;
  }

  void thermostat_program_time_set_status(uint8_t idx, uint8_t state)
  {
  EEPROM.write(program_time_status_0 + idx, state);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  uint8_t thermostat_program_get_starttime_hod(uint8_t idx)
  {
  uint8_t state;
  state = EEPROM.read(program_time_0 + (10 * idx) + 0);
  if (state > 23) state = 0;
  return state;
  }
  void thermostat_program_set_starttime_hod(uint8_t idx, uint8_t state)
  {
  EEPROM.write(program_time_0 + (10 * idx) + 0, state);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  uint8_t thermostat_program_get_starttime_min(uint8_t idx)
  {
  uint8_t state;
  state = EEPROM.read(program_time_0 + (10 * idx) + 1);
  if (state > 59) state = 0;
  return state;
  }
  void thermostat_program_set_starttime_min(uint8_t idx, uint8_t state)
  {
  EEPROM.write(program_time_0 + (10 * idx) + 1, state);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  uint8_t thermostat_program_get_stoptime_hod(uint8_t idx)
  {
  uint8_t state;
  state = EEPROM.read(program_time_0 + (10 * idx) + 2);
  if (state > 23) state = 0;
  return state;
  }
  void thermostat_program_set_stoptime_hod(uint8_t idx, uint8_t state)
  {
  EEPROM.write(program_time_0 + (10 * idx) + 2, state);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  uint8_t thermostat_program_get_stoptime_min(uint8_t idx)
  {
  uint8_t state;
  state = EEPROM.read(program_time_0 + (10 * idx) + 3);
  if (state > 59) state = 0;
  return state;
  }
  void thermostat_program_set_stoptime_min(uint8_t idx, uint8_t state)
  {
  EEPROM.write(program_time_0 + (10 * idx) + 3, state);
  }


  uint8_t thermostat_program_get_day(uint8_t idx)
  {
  uint8_t state;
  state = EEPROM.read(program_time_0 + (10 * idx) + 4);
  if (state > 0b01111111) state = 0;
  return state;
  }
  void thermostat_program_set_day(uint8_t idx, uint8_t state)
  {
  EEPROM.write(program_time_0 + (10 * idx) + 4, state);
  }

  float thermostat_program_get_mezni(uint8_t idx)
  {
  return EEPROMreadFloat(program_time_0 + (10 * idx) + 5);
  }

  void thermostat_program_set_mezni(uint8_t idx, float temp)
  {
  EEPROMwriteFloat(program_time_0 + (10 * idx) + 5, temp);
  }
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// program status registr
/*
   7 -
   6 -
   5 -
   4 -
   3 -
   2 -
   1 - pouze tento program je aktivni pro globalni termostat
   0 - program aktivni pro pouzivani
*/
/*
  uint8_t thermostat_program_get_status(uint8_t idx)
  {
  uint8_t state;
  state = EEPROM.read(program_status_0 + idx);
  return state;
  }

  void thermostat_program_set_status(uint8_t idx, uint8_t state)
  {
  EEPROM.write(program_status_0 + idx, state);
  }


  /// ziska pouze jeden aktivni program. Vice programu to je chyba
  uint8_t thermostat_get_active_program_global(void)
  {
  uint8_t ret = 255;
  for (uint8_t idx = 0; idx < MAX_PROGRAM + 1; idx++)
  {
    if ((thermostat_program_get_status(idx) & 0b00000010) != 0)
    {
      ret = idx;
      break;
    }
  }
  return ret;
  }
  /// nastavuje aktivni program. Muze byt aktivni pouze jeden
  void thermostat_set_active_program_global(uint8_t acidx)
  {
  uint8_t state;
  /// prvne vsechno nastavim jako neaktivni
  thermostat_deactive_program_global();
  //// nastavim tento program je aktivni
  state = thermostat_program_get_status(acidx);
  state = state | 0b00000010;
  thermostat_program_set_status(acidx, state);
  }
  /// neni aktivni zadny program.
  void thermostat_deactive_program_global(void)
  {
  uint8_t state;
  for (uint8_t idx = 0; idx < MAX_PROGRAM + 1; idx++)
  {
    state = thermostat_program_get_status(idx);
    state = state & 0b11111101;
    thermostat_program_set_status(idx, state);
  }
  }

  ///////////////////////////////////////////////////////////
  //// ziskava pro jednotlive programy casove plany.
  //// pozice se resi dynamicky
  //// 255 znamena volna pozice. Index casoveho programu neznamena misto
  uint8_t thermostat_program_get_timerange(uint8_t prog, uint8_t idx )
  {
  uint8_t state;
  state = EEPROM.read(program_0 + (10 * prog) + idx);
  return state;
  }

  void thermostat_program_set_timerange(uint8_t prog, uint8_t idx, uint8_t state)
  {
  EEPROM.write(program_0 + (10 * prog) + idx, state);
  }
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
