#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

// Piny E-Paper
#define PIN_CS   1
#define PIN_DC   6
#define PIN_RST  7
#define PIN_BUSY 8

#define PIN_SCK         2   // GP2 -> SCK
#define PIN_MOSI        3   // GP3 -> MOSI
#define PIN_MISO        4   // GP4 -> MISO

// Piny WS2812
#define PIN_WS2812 16
#define NUM_LEDS   1
static Adafruit_NeoPixel neo(NUM_LEDS, PIN_WS2812, NEO_GRB + NEO_KHZ800);
#define SPI_SPEED_HZ 2000000UL // dopasowane do kodu referencyjnego
#define DIAG_MODE 1
#define BUSY_BYPASS 0  // 1 = ignoruj pin BUSY (użyj opóźnień stałych)
#define USE_REF_INIT 1  // 1 = użyj sekwencji z działającego kodu (112x208 UC8151D)
// Tryb porównawczy dwóch sterowników: 1 = wykonaj dwie sekwencje (UC8151/IL0373 oraz SSD1680/IL3820)
#define MULTI_DRIVER_DEBUG 0
#define TRI_COLOR_TEST 1
#define ORIENTATION_SWEEP 1
static bool g_busyActiveLow = true; // autodetekcja
static bool g_busyPolarityKnown = false;

static inline void diagDelayWait(uint32_t ms){
#if BUSY_BYPASS
  delay(ms);
#else
  uint32_t t0=millis();
  while(millis()-t0<ms) delay(1);
#endif
}
// Globalny timeout BUSY (ms)
#define EPD_BUSY_TIMEOUT_MS 5000
static void ledSetDimGreen(){ neo.setPixelColor(0, neo.Color(0, 40, 0)); neo.show(); }
static void ledSetBlue(){ neo.setPixelColor(0, neo.Color(0,0,40)); neo.show(); }
static void ledSetRed(){ neo.setPixelColor(0, neo.Color(40,0,0)); neo.show(); }
static void ledSetWhite(){ neo.setPixelColor(0, neo.Color(40,40,40)); neo.show(); }
static void ledOff(){ neo.setPixelColor(0, 0); neo.show(); }
static bool g_ledOk = true;
static void ledTestCycle(){
  if(!g_ledOk) return; // skip if fallback
  static uint8_t phase=0;
  switch(phase++ & 7){
    case 0: ledSetRed(); break;
    case 1: ledSetDimGreen(); break;
    case 2: ledSetBlue(); break;
    case 3: neo.setPixelColor(0, neo.Color(40,40,0)); neo.show(); break; // yellow
    case 4: ledSetWhite(); break;
    case 5: neo.setPixelColor(0, neo.Color(0,40,40)); neo.show(); break; // cyan
    case 6: neo.setPixelColor(0, neo.Color(40,0,40)); neo.show(); break; // magenta
    case 7: neo.setPixelColor(0, neo.Color(10,10,10)); neo.show(); break; // dim gray
  }
}

// ====== Pomocnicze funkcje dla E-Paper ======
void epd_cmd(uint8_t c) {
  digitalWrite(PIN_DC, LOW);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(c);
  digitalWrite(PIN_CS, HIGH);
}

// Odczyt statusu sterownika (0x71). W wielu panelach wymaga tylko komendy bez dalszych danych
uint8_t epd_status(){
  epd_cmd(0x71);
  // Brak dedykowanego MISO? Jeśli połączony MISO to odczytaj:
  return SPI.transfer(0x00);
}

void epd_data(uint8_t d) {
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(d);
  digitalWrite(PIN_CS, HIGH);
}

void epd_wait() {
  if(BUSY_BYPASS) return;
  if(!g_busyPolarityKnown) return;
  bool active = g_busyActiveLow ? LOW : HIGH;
  uint32_t t0 = millis();
  while(digitalRead(PIN_BUSY)==active){
    if(millis()-t0>EPD_BUSY_TIMEOUT_MS){ Serial.println("[EPD] BUSY timeout 5s"); break; }
    delay(5);
  }
  delay(2);
}

void epd_reset() {
  for(int i=0;i<3;i++){ digitalWrite(PIN_RST, LOW); delay(25); digitalWrite(PIN_RST, HIGH); delay(25);} 
}

static void epd_power_off_deep(){
  epd_cmd(0x50); epd_data(0xF7);
  epd_cmd(0x02); epd_wait();
  delay(150);
  epd_cmd(0x07); epd_data(0xA5);
}

static void detectBusyBefore(){
  int pre = digitalRead(PIN_BUSY);
  g_busyActiveLow = (pre==HIGH); // jeśli HIGH = gotów -> aktywny LOW
  g_busyPolarityKnown=true;
  Serial.print("[EPD] BUSY pre="); Serial.print(pre); Serial.print(" assume active "); Serial.println(g_busyActiveLow?"LOW":"HIGH");
}

static void refineBusyAfterPowerOn(){
  if(BUSY_BYPASS) return;
  int first = digitalRead(PIN_BUSY); int prev=first; bool changed=false;
  uint32_t t0=millis();
  while(millis()-t0<150){
    int v=digitalRead(PIN_BUSY); if(v!=prev){ changed=true; prev=v; break; } delay(2);
  }
  if(changed){ g_busyActiveLow = (prev==LOW); Serial.print("[EPD] BUSY transition -> active "); Serial.println(g_busyActiveLow?"LOW":"HIGH"); }
  else { Serial.println("[EPD] BUSY no transition"); }
}

struct EpdVariant{uint16_t w;uint16_t h;uint8_t power[5];uint8_t panel[2];uint8_t vcom;const char* name;};
static const EpdVariant variants[]={
  {296,160,{0x03,0x00,0x2B,0x2B,0x09},{0xBF,0x0D},0x77,"A_296x160"},
  {296,160,{0x07,0x00,0x0F,0x0F,0x0D},{0x1F,0x0D},0x57,"B_296x160"},
  {112,208,{0x03,0x00,0x2B,0x2B,0x13},{0x1F,0x0D},0x57,"C_112x208"},
  {112,208,{0x03,0x00,0x2B,0x2B,0x13},{0x1F,0x0D},0xF7,"D_112x208_vF7"},
};
static uint16_t curW=296,curH=160;
static bool g_transpose=false; // zamiana x<->y przy rysowaniu
static bool g_invertBits=false; // inwersja bitów (0<->1) w buforze logicznym
static bool g_autoStatus=false; // czy okresowo wypisywać STAT
static bool g_inited=false;     // czy panel zainicjalizowany
static bool g_altPush=false;    // tryb alternatywnego wypełniania old buffer (0x10)
static bool g_swapWriteOrder=false; // zamiana kolejności zapisu 0x10/0x13
static bool g_dataDebug=false;      // szczegółowe logowanie danych
static bool g_fullStream=false;     // wypisuj każdy bajt (wolne!)

// Prosty FNV-1a hash dla strumienia (8-bit)
static uint32_t fnv1a_hash(const uint8_t* d,size_t n){
  uint32_t h=2166136261u; for(size_t i=0;i<n;i++){ h^=d[i]; h*=16777619u; } return h; }

// Zliczanie bajtów: różne wartości w strumieniu
struct ByteStats{ uint16_t count[256]; };
static void byteStatsReset(ByteStats &bs){ memset(&bs,0,sizeof(bs)); }
static void byteStatsFeed(ByteStats &bs,const uint8_t* d,size_t n){ for(size_t i=0;i<n;i++) bs.count[d[i]]++; }
static void byteStatsPrint(const char* tag,const ByteStats &bs){
  Serial.print("[DBG] "); Serial.print(tag); Serial.print(" nonzero bytes: ");
  int shown=0; for(int i=0;i<256;i++){ if(bs.count[i]){ Serial.print(i); Serial.print('='); Serial.print(bs.count[i]); Serial.print(' '); if(++shown>=24){ Serial.print("..."); break; } } }
  Serial.println();
}
// Framebuffer and drawing utilities (moved up for scope)
static uint8_t frame[296 * 160 / 8];
static void setPixel(int x,int y,bool black){
  // Przekształcenia orientacji
  int tx=x, ty=y;
  if(g_transpose){ tx=y; ty=x; }
  if(tx<0||ty<0||tx>=296||ty>=160) return;
  int index=(tx + ty*296)/8; int bit=7-(tx & 7);
  bool drawBlack = black ^ g_invertBits; // ewentualna inwersja
  if(drawBlack) frame[index]&=~(1<<bit); else frame[index]|=(1<<bit);
}
static void epd_init_variant(const EpdVariant& v){
#if USE_REF_INIT
  // Referencyjna sekwencja (112x208) z kodu w lib/pio_ws2812_E-ink/ws2812.c
  epd_reset();
  detectBusyBefore();
  epd_cmd(0x01); epd_data(0x03); epd_data(0x00); epd_data(0x2B); epd_data(0x2B); epd_data(0x13); // POWER SETTING
  epd_cmd(0x00); epd_data(0x1F); epd_data(0x0D); // PANEL SETTING
  epd_cmd(0x61); epd_data(112); epd_data(208>>8); epd_data(208 & 0xFF); // RESOLUTION
  epd_cmd(0x04); diagDelayWait(50); epd_wait(); // POWER ON
  refineBusyAfterPowerOn();
  epd_cmd(0x50); epd_data(0x57); // VCOM & data interval
  curW=112; curH=208;
  Serial.println("[EPD] REF INIT 112x208");
  return;
#endif
  Serial.print("[EPD] VAR "); Serial.println(v.name);
  epd_reset();
  epd_cmd(0x12); delay(10); epd_wait();
  epd_cmd(0x01); for(int i=0;i<5;i++) epd_data(v.power[i]);
  epd_cmd(0x06); epd_data(0x17); epd_data(0x17); epd_data(0x17);
  epd_cmd(0x04); epd_wait();
  epd_cmd(0x00); epd_data(v.panel[0]); epd_data(v.panel[1]);
  epd_cmd(0x50); epd_data(v.vcom);
  epd_cmd(0x61); epd_data(v.w>>8); epd_data(v.w & 0xFF); epd_data(v.h>>8); epd_data(v.h & 0xFF);
  curW=v.w; curH=v.h;
}
static void epd_pattern_half(){
  // Góra czarna (bit=0), dół biała (bit=1) – jak make_test_bands() w referencji
  memset(frame,0xFF,sizeof(frame));
  uint16_t effectiveH = (curH>160)?160:curH; // nasz bufor ma 160 linii
  uint16_t effectiveW = (curW>296)?296:curW;
  for(uint16_t y=0; y<effectiveH/2; ++y){
    for(uint16_t x=0; x<effectiveW; ++x) setPixel(x,y,true);
  }
}
static void epd_push(){
  Serial.print("[EPD] push begin BUSY="); Serial.print(digitalRead(PIN_BUSY));
  Serial.print(" order="); Serial.print(g_swapWriteOrder?"13->10":"10->13");
  Serial.print(" old="); Serial.print(g_altPush?"FF":"00");
  Serial.print(" dbg="); Serial.println(g_dataDebug?"ON":"OFF");

  ByteStats bsOld, bsNew; byteStatsReset(bsOld); byteStatsReset(bsNew);
  uint32_t hOld=0, hNew=0;
  const uint8_t oldFill = g_altPush?0xFF:0x00;

  auto dumpSample=[&](const char* tag,const uint8_t* buf,size_t n){
    Serial.print("[DBG] "); Serial.print(tag); Serial.print(" sample(32): ");
    size_t lim = n<32?n:32; for(size_t i=0;i<lim;i++){ if(i) Serial.print(' '); uint8_t b=buf[i]; if(b<16) Serial.print('0'); Serial.print(b,HEX);} Serial.println();
  };

  if(g_swapWriteOrder){
    // NEW first
    epd_cmd(0x13);
    for(size_t i=0;i<sizeof(frame);++i){
      uint8_t b = frame[i];
      epd_data(b);
      if(g_dataDebug){ if(g_fullStream){ Serial.print("N"); if(b<16) Serial.print('0'); Serial.print(b,HEX); Serial.print(' ');} bsNew.count[b]++; }
    }
    if(g_dataDebug){ hNew=fnv1a_hash(frame,sizeof(frame)); dumpSample("NEW",frame,sizeof(frame)); }
    // OLD after
    epd_cmd(0x10);
    for(size_t i=0;i<sizeof(frame);++i){
      epd_data(oldFill);
      if(g_dataDebug){ if(g_fullStream){ Serial.print("O"); if(oldFill<16) Serial.print('0'); Serial.print(oldFill,HEX); Serial.print(' ');} bsOld.count[oldFill]++; }
    }
    if(g_dataDebug){ uint8_t tmp[32]; memset(tmp,oldFill,32); dumpSample("OLD",tmp,32); }
  }else{
    // OLD first
    epd_cmd(0x10);
    for(size_t i=0;i<sizeof(frame);++i){
      epd_data(oldFill);
      if(g_dataDebug){ if(g_fullStream){ Serial.print("O"); if(oldFill<16) Serial.print('0'); Serial.print(oldFill,HEX); Serial.print(' ');} bsOld.count[oldFill]++; }
    }
    if(g_dataDebug){ uint8_t tmp[32]; memset(tmp,oldFill,32); dumpSample("OLD",tmp,32); }
    // NEW second
    epd_cmd(0x13);
    for(size_t i=0;i<sizeof(frame);++i){
      uint8_t b = frame[i];
      epd_data(b);
      if(g_dataDebug){ if(g_fullStream){ Serial.print("N"); if(b<16) Serial.print('0'); Serial.print(b,HEX); Serial.print(' ');} bsNew.count[b]++; }
    }
    if(g_dataDebug){ hNew=fnv1a_hash(frame,sizeof(frame)); dumpSample("NEW",frame,sizeof(frame)); }
  }

  if(g_dataDebug){
    byteStatsPrint("OLD",bsOld); byteStatsPrint("NEW",bsNew);
    Serial.print("[DBG] HASH_NEW=0x"); Serial.println(hNew,HEX);
  }
  epd_cmd(0x12);
  diagDelayWait(900); epd_wait(); Serial.println("[EPD] Refresh done");
  if(g_dataDebug && g_fullStream) Serial.println(); // newline after stream
  Serial.print("[EPD] push end BUSY="); Serial.println(digitalRead(PIN_BUSY));
}

// Bardzo agresywne czyszczenie: różne polaryzacje i kolejności zapisu
static void ultraClear(){
  if(!g_inited){ epd_init_variant(variants[2]); g_inited=true; }
  bool origAlt=g_altPush; bool origOrder=g_swapWriteOrder;
  Serial.println("[CLR] Ultra clear start");
  for(int phase=0; phase<6; ++phase){
    g_altPush = (phase & 1);          // naprzemiennie 0x00/0xFF
    g_swapWriteOrder = (phase & 2);   // co dwa kroki zmiana kolejności
    memset(frame, (phase & 4)?0x00:0xFF, sizeof(frame)); // w ostatnich dwóch fazach (phase>=4) wymuś pełne czarne
    Serial.print("[CLR] phase="); Serial.print(phase);
    Serial.print(" alt="); Serial.print(g_altPush);
    Serial.print(" swap="); Serial.print(g_swapWriteOrder);
    Serial.print(" frameFill="); Serial.println((phase & 4)?"BLACK":"WHITE");
    epd_push();
    delay(300);
  }
  // Przywróć domyślne
  g_altPush=origAlt; g_swapWriteOrder=origOrder;
  memset(frame,0xFF,sizeof(frame)); epd_push();
  Serial.println("[CLR] Ultra clear end (final WHITE push)");
}

// Test połówek z dwiema kolejnymi polaryzacjami old buffer
static void polarityBands(){
  if(!g_inited){ epd_init_variant(variants[2]); g_inited=true; }
  // Faza 1: altPush=0 (old=0x00)
  g_altPush=false; memset(frame,0xFF,sizeof(frame));
  for(int y=0;y<80;y++) for(int x=0;x<curW && x<296;x++) setPixel(x,y,true); // górna połowa czarna
  epd_push(); delay(500);
  // Faza 2: altPush=1 (old=0xFF)
  g_altPush=true; memset(frame,0xFF,sizeof(frame));
  for(int y=80;y<160 && y<curH;y++) for(int x=0;x<curW && x<296;x++) setPixel(x,y,true); // dolna połowa czarna
  epd_push(); delay(500);
  // Reset
  g_altPush=false; memset(frame,0xFF,sizeof(frame)); epd_push();
  Serial.println("[CLR] polarity bands done");
}
static void epd_sweep(){
  // Funkcja nieużywana w trybie MULTI_DRIVER_DEBUG – pozostawiona dla zgodności
#if !MULTI_DRIVER_DEBUG
  #if USE_REF_INIT
    epd_init_variant(variants[2]);
    epd_pattern_half(); epd_push();
    Serial.print("[EPD] STATUS after ref=0x"); Serial.println(epd_status(),HEX);
    Serial.println("[EPD] Power cycle test");
    epd_power_off_deep(); delay(400);
    epd_init_variant(variants[2]); epd_pattern_half(); epd_push();
    Serial.print("[EPD] STATUS after reinit=0x"); Serial.println(epd_status(),HEX);
    return;
  #else
    for(size_t i=0;i<sizeof(variants)/sizeof(variants[0]);++i){
      epd_init_variant(variants[i]); epd_pattern_half(); epd_push();
      Serial.print("[EPD] STATUS after variant=0x"); Serial.println(epd_status(),HEX);
      delay(1500);
    }
  #endif
#endif
}

// Tri-color dodatkowe testy
#if TRI_COLOR_TEST
static void triColorStripes(){
  // Sekwencja pionowych pasków do oceny orientacji i ghostingu
  memset(frame,0xFF,sizeof(frame));
  for(uint16_t x=0; x<curW && x<296; x+=24){
    for(uint16_t y=0; y<curH && y<160; ++y){
      for(uint16_t k=0; k<12 && (x+k)<curW && (x+k)<296; ++k){ setPixel(x+k,y,true); }
    }
  }
  epd_push();
  delay(400);
  // Pełne czyszczenie
  memset(frame,0xFF,sizeof(frame)); epd_push();
}
#endif

#if ORIENTATION_SWEEP
// Parametry prób orientacji
struct OrientAttempt { uint16_t w,h; bool transpose; bool invert; const char* name; };
static const OrientAttempt orientList[] = {
  {112,208,false,false,"W112xH208"},
  {208,112,false,false,"W208xH112"},
  {112,208,true,false,"TRP_112x208"},
  {208,112,true,false,"TRP_208x112"},
  {112,208,false,true,"INV_112x208"},
};

static void epd_init_uc8151(uint16_t w,uint16_t h){
  epd_reset(); detectBusyBefore();
  epd_cmd(0x01); epd_data(0x03); epd_data(0x00); epd_data(0x2B); epd_data(0x2B); epd_data(0x13);
  epd_cmd(0x00); epd_data(0x1F); epd_data(0x0D);
  epd_cmd(0x61); epd_data(w); epd_data(h>>8); epd_data(h & 0xFF);
  epd_cmd(0x04); diagDelayWait(50); epd_wait(); refineBusyAfterPowerOn();
  epd_cmd(0x50); epd_data(0x57);
  curW=w; curH=h;
}

static void orientationSweep(){
  Serial.println("[EPD] Orientation sweep start");
  for(size_t i=0;i<sizeof(orientList)/sizeof(orientList[0]);++i){
    const auto &o=orientList[i];
    g_transpose=o.transpose; g_invertBits=o.invert;
    epd_init_uc8151(o.w,o.h);
    memset(frame,0xFF,sizeof(frame));
    // Wzór: pionowe pasy 8px (wizualizacja szerokości) + numerowanie co 32px grubszą kolumną
    uint16_t effW = (curW>296?296:curW); uint16_t effH=(curH>160?160:curH);
    for(uint16_t x=0; x<effW; ++x){
      bool bar = (x/8)%2==0;
      if(bar){ for(uint16_t y=0;y<effH;y++) setPixel(x,y,true); }
      if((x%32)==0){ for(uint16_t y=0;y<effH;y++) setPixel(x,y,true); }
    }
    epd_push();
    Serial.print("[EPD] ORIENT "); Serial.print(i); Serial.print(":"); Serial.print(o.name); Serial.print(" status=0x"); Serial.println(epd_status(),HEX);
    delay(500);
    // czyszczenie między próbami
    memset(frame,0xFF,sizeof(frame)); epd_push(); delay(250);
  }
  Serial.println("[EPD] Orientation sweep end");
  // Przywróć flagi bazowe
  g_transpose=false; g_invertBits=false;
}
#endif

static const uint8_t font5x7[] = { 0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00 };
static void drawText(const char* txt, int x, int y) {
  while (*txt) {
    char c = *txt;
    if (c < 32 || c > 126) c = ' ';
    const uint8_t* ch = &font5x7[((c - 32) % 2) * 5];
    for (int i=0; i<5; i++) {
      uint8_t line = ch[i];
      for (int j=0; j<7; j++) if (line & (1<<j)) setPixel(x+i, y+j, true);
    }
    x += 6; txt++;
  }
}
static void patternDigits(){
  if(!g_inited){ epd_init_variant(variants[2]); g_inited=true; }
  memset(frame,0xFF,sizeof(frame));
  const char *txt="0123456789 ABCDEF";
  drawText(txt, 2, 2);
  epd_push();
  Serial.println("[CMD] digits shown");
}

static void patternLetters(){
  if(!g_inited){ epd_init_variant(variants[2]); g_inited=true; }
  memset(frame,0xFF,sizeof(frame));
  const char *txt="TEST ABC xyz";
  drawText(txt, 2, 2);
  epd_push();
  Serial.println("[CMD] letters shown");
}

static void fullBlackWhiteRed(){
  if(!g_inited){ epd_init_variant(variants[2]); g_inited=true; }
  // Czarny
  memset(frame,0xFF,sizeof(frame)); for(size_t i=0;i<sizeof(frame);++i) frame[i]=0x00; epd_push();
  // Biały
  memset(frame,0xFF,sizeof(frame)); epd_push();
  // Czerwony (jeśli 3-kolor: często wymaga osobnej komendy – fallback: ponowne czarne + białe może aktywować czerwone piksle w pasach)
  // Tutaj dodajemy próbę "stim" – szybkie przełączenia by wyrównać ładunki
  for(int r=0;r<2;r++){
    memset(frame,0xFF,sizeof(frame)); epd_push();
    for(size_t i=0;i<sizeof(frame);++i) frame[i]=0x00; epd_push();
  }
  Serial.println("[CMD] full B/W/(stim red) cycle done");
}

static void quickClear(){ memset(frame,0xFF,sizeof(frame)); epd_push(); Serial.println("[CMD] clear white"); }

// ================== Dodatkowe: test różnych formatów 0x61 i ramka ==================
struct ResTry { const char* tag; uint16_t w; uint16_t h; bool threeByte; };
static const ResTry resList[] = {
  {"A112x208_3B",112,208,true},
  {"B208x112_3B",208,112,true},
  {"C296x160_4B",296,160,false},
  {"D160x296_4B",160,296,false},
};

static void sendResolution(uint16_t w,uint16_t h,bool three){
  epd_cmd(0x61);
  if(three){ // spotykane w ref: width(1), height hi, height lo
    epd_data(w & 0xFF);
    epd_data(h>>8);
    epd_data(h & 0xFF);
  }else{ // typowe 4 bajty: w hi, w lo, h hi, h lo
    epd_data(w>>8); epd_data(w & 0xFF); epd_data(h>>8); epd_data(h & 0xFF);
  }
  curW=w; curH=h;
}

static void patternBorder(){
  if(!g_inited){ epd_init_variant(variants[2]); g_inited=true; }
  memset(frame,0xFF,sizeof(frame));
  uint16_t effW = curW>296?296:curW; uint16_t effH=curH>160?160:curH;
  for(uint16_t x=0;x<effW;x++){ setPixel(x,0,true); if(effH>1) setPixel(x,effH-1,true);} // top/bottom
  for(uint16_t y=0;y<effH;y++){ setPixel(0,y,true); if(effW>1) setPixel(effW-1,y,true);} // left/right
  // Krzyż środka
  for(uint16_t x=0;x<effW;x++) if(x%2==0) setPixel(x, effH/2, true);
  for(uint16_t y=0;y<effH;y++) if(y%2==0) setPixel(effW/2, y, true);
  drawText("B",2,2);
  epd_push();
  Serial.println("[CMD] border pattern");
}

static void testResolutionVariants(){
  Serial.println("[RES] start");
  for(size_t i=0;i<sizeof(resList)/sizeof(resList[0]);++i){
    const auto &r=resList[i];
    Serial.print("[RES] "); Serial.println(r.tag);
    epd_reset(); detectBusyBefore();
    // power/panel like ref
    epd_cmd(0x01); epd_data(0x03); epd_data(0x00); epd_data(0x2B); epd_data(0x2B); epd_data(0x13);
    epd_cmd(0x00); epd_data(0x1F); epd_data(0x0D);
    sendResolution(r.w,r.h,r.threeByte);
    epd_cmd(0x04); diagDelayWait(40); epd_wait(); refineBusyAfterPowerOn();
    epd_cmd(0x50); epd_data(0x57);
    memset(frame,0xFF,sizeof(frame));
    // Pasy pionowe co 16 px + litera identyfikacyjna
    uint16_t effW = curW>296?296:curW; uint16_t effH=curH>160?160:curH;
    for(uint16_t x=0;x<effW;x++) if((x/16)%2==0) for(uint16_t y=0;y<effH;y++) setPixel(x,y,true);
    char label = 'A'+(char)i;
    char txt[2]={label,'\0'}; drawText(txt,4,4);
    epd_push();
    delay(400);
    memset(frame,0xFF,sizeof(frame)); epd_push(); delay(200);
  }
  Serial.println("[RES] end");
}

// ================== Interfejs komend szeregowych ==================
static void serialCommands(){
  while(Serial.available()){
    char c = Serial.read();
    switch(c){
      case '0': patternDigits(); break;
      case '1': patternLetters(); break;
      case '2': triColorStripes(); break;
      case '3': orientationSweep(); break;
  case '4': fullBlackWhiteRed(); break;
  case '5': quickClear(); break;
      case '6': testResolutionVariants(); break;
      case '7': patternBorder(); break;
  case 'r': case 'R': epd_power_off_deep(); delay(300); epd_init_variant(variants[2]); g_inited=true; Serial.println("[CMD] reinit 112x208 ref"); break;
  case 's': case 'S': g_autoStatus=!g_autoStatus; Serial.print("[CMD] auto status -> "); Serial.println(g_autoStatus?"ON":"OFF"); break;
  case 'b': case 'B': if(!g_inited){ epd_init_variant(variants[2]); g_inited=true; } memset(frame,0xFF,sizeof(frame)); for(size_t i=0;i<sizeof(frame);++i) frame[i]=0x00; epd_push(); Serial.println("[CMD] force black"); break; 
  case 'w': case 'W': if(!g_inited){ epd_init_variant(variants[2]); g_inited=true; } memset(frame,0xFF,sizeof(frame)); epd_push(); Serial.println("[CMD] force white"); break; 
  case '9': g_altPush=!g_altPush; Serial.print("[CMD] altPush -> "); Serial.println(g_altPush?"FF old":"00 old"); break;
  case 'o': case 'O': g_swapWriteOrder=!g_swapWriteOrder; Serial.print("[CMD] swap write order -> "); Serial.println(g_swapWriteOrder?"13->10":"10->13"); break;
  case 'u': case 'U': ultraClear(); break;
  case 'y': case 'Y': polarityBands(); break;
  case 'd': case 'D': g_dataDebug=!g_dataDebug; Serial.print("[CMD] data debug -> "); Serial.println(g_dataDebug?"ON":"OFF"); break;
  case 'f': case 'F': g_fullStream=!g_fullStream; Serial.print("[CMD] full stream -> "); Serial.println(g_fullStream?"ON":"OFF"); break;
  case 't': case 'T': Serial.print("[STAT] BUSY="); Serial.print(digitalRead(PIN_BUSY)); Serial.print(" STATUS=0x"); Serial.println(epd_status(),HEX); break;
      case 'i': case 'I':
        g_invertBits = !g_invertBits; Serial.print("[CMD] invert toggled -> "); Serial.println(g_invertBits?"ON":"OFF"); patternDigits(); break;
      case 'x': case 'X':
        g_transpose = !g_transpose; Serial.print("[CMD] transpose toggled -> "); Serial.println(g_transpose?"ON":"OFF"); patternDigits(); break;
      case 'h': case 'H':
  Serial.println("[HELP] 0=cyfry 1=litery 2=paski 3=orient 4=clean 5=white 6=res 7=border r=reinit s=stat i=inv x=transpose h=help");
        break;
      default: break;
    }
  }
}

#if DIAG_MODE
// =================== DIAGNOSTYKA ROZSZERZONA ===================
struct ParamMut { uint8_t vcom; uint8_t panel0; uint8_t panel1; const char* tag; };
static const ParamMut mutList[] = {
  {0x11,0xBF,0x0D,"v11"}, {0x17,0xBF,0x0D,"v17"}, {0x57,0x1F,0x0D,"v57"}, {0x77,0x1F,0x0D,"v77"}, {0xF7,0x1F,0x0D,"vF7"}
};

static void dumpPins(){
  Serial.print("[PIN] CS="); Serial.print(digitalRead(PIN_CS));
  Serial.print(" DC="); Serial.print(digitalRead(PIN_DC));
  Serial.print(" RST="); Serial.print(digitalRead(PIN_RST));
  Serial.print(" BUSY="); Serial.println(digitalRead(PIN_BUSY));
}

static void wigglePins(){
  Serial.println("[DIAG] Wiggle SCK/MOSI/CS/DC start");
  // Temporarily control pins manually (disable SPI by ending transaction)
  SPI.endTransaction();
  pinMode(PIN_SCK, OUTPUT); pinMode(PIN_MOSI, OUTPUT); pinMode(PIN_CS, OUTPUT); pinMode(PIN_DC, OUTPUT);
  digitalWrite(PIN_CS, HIGH); digitalWrite(PIN_DC, LOW);
  // Pattern 0xAA repeated -> toggles MOSI each bit while SCK pulses
  for(int r=0;r<4;r++){
    digitalWrite(PIN_CS, LOW);
    for(int i=0;i<64;i++){
      bool bit = (i & 1);
      digitalWrite(PIN_MOSI, bit?HIGH:LOW);
      digitalWrite(PIN_SCK, HIGH); delayMicroseconds(4);
      digitalWrite(PIN_SCK, LOW);  delayMicroseconds(4);
    }
    digitalWrite(PIN_CS, HIGH); delay(5);
  }
  // DC toggle pulses
  for(int i=0;i<10;i++){ digitalWrite(PIN_DC, (i&1)?HIGH:LOW); delay(10);} 
  // Restore SPI
  SPI.beginTransaction(SPISettings(SPI_SPEED_HZ, MSBFIRST, SPI_MODE0));
  Serial.println("[DIAG] Wiggle done");
}

static void paramSweep(){
  Serial.println("[DIAG] Param sweep start");
  for(size_t m=0;m<sizeof(mutList)/sizeof(mutList[0]);++m){
    const auto &pm = mutList[m];
    Serial.print("[DIAG] Mut "); Serial.println(pm.tag);
    epd_reset(); epd_cmd(0x12); delay(10); epd_wait();
    // Use base power setting from first variant
    epd_cmd(0x01); for(int i=0;i<5;i++) epd_data(variants[0].power[i]);
    epd_cmd(0x06); epd_data(0x17); epd_data(0x17); epd_data(0x17);
    epd_cmd(0x04); epd_wait();
    epd_cmd(0x00); epd_data(pm.panel0); epd_data(pm.panel1);
    epd_cmd(0x50); epd_data(pm.vcom);
    // Try both resolutions each mutation
    uint16_t wList[2]={296,112}; uint16_t hList[2]={160,208};
    for(int ri=0;ri<2;ri++){
      uint16_t w=wList[ri]; uint16_t h=hList[ri];
      epd_cmd(0x61); epd_data(w>>8); epd_data(w & 0xFF); epd_data(h>>8); epd_data(h & 0xFF);
      memset(frame,0xFF,sizeof(frame));
      for(uint16_t y=0;y< (h<160?h:160)/2; y++) for(uint16_t x=0; x< (w<296?w:296); x++) setPixel(x,y,true);
      epd_cmd(0x10); for(size_t i=0;i<sizeof(frame);++i) epd_data(0xFF);
      epd_cmd(0x13); for(size_t i=0;i<sizeof(frame);++i) epd_data(frame[i]);
      epd_cmd(0x12); diagDelayWait(1200); epd_wait();
      Serial.print("[DIAG] Mut "); Serial.print(pm.tag); Serial.print(" res "); Serial.print(w); Serial.print("x"); Serial.print(h); Serial.print(" status=0x"); Serial.println(epd_status(),HEX);
      delay(800);
    }
  }
  Serial.println("[DIAG] Param sweep end");
}
#endif

#if DIAG_MODE
// Test różnych trybów SPI (MODE0 vs MODE3) z ręcznym przełączeniem
static void spiModeDiagnostic(){
  Serial.println("[DIAG] SPI mode diagnostic start");
  uint8_t modes[2]={SPI_MODE0,SPI_MODE3};
  for(int mi=0; mi<2; ++mi){
    uint8_t mode=modes[mi];
    SPI.endTransaction();
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, mode));
    Serial.print("[DIAG] MODE="); Serial.println(mode==SPI_MODE0?0:3);
    // Send simple command frame pattern: 0x00 as cmd then 16 bytes of incrementing data
    digitalWrite(PIN_DC, LOW); digitalWrite(PIN_CS, LOW); SPI.transfer(0x00); digitalWrite(PIN_CS,HIGH);
    digitalWrite(PIN_DC, HIGH); digitalWrite(PIN_CS, LOW); for(int i=0;i<16;i++) SPI.transfer(i); digitalWrite(PIN_CS,HIGH);
    // Short wait to view on analyzer
    delay(50);
  }
  // Restore nominal transaction
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(SPI_SPEED_HZ, MSBFIRST, SPI_MODE0));
  Serial.println("[DIAG] SPI mode diagnostic end");
}
#endif

// (original framebuffer code relocated above)

// epd_flush usunięte – używamy epd_push()

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_WS2812, OUTPUT);

  // Wymuszenie pinów SPI identycznie jak w działającym przykładzie (spi0: GP2/GP3/GP4)
  SPI.setSCK(PIN_SCK);    // GP2
  SPI.setTX(PIN_MOSI);    // GP3
  SPI.setRX(PIN_MISO);    // GP4 (jeśli panel nie używa – bez szkody)
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_SPEED_HZ, MSBFIRST, SPI_MODE0));
  neo.begin();
  neo.setBrightness(80); // wyraźna jasność testowa
  // Reset impuls: WS2812 potrzebuje low > 280us by start
  neo.show(); delay(1);
  // Sekwencja testowa kolorów startowych
  ledSetRed(); delay(150);
  ledSetDimGreen(); delay(150);
  ledSetBlue(); delay(150);
  ledSetWhite(); delay(150);
  if(!neo.canShow()) { // if library reports issues (rare) mark fallback
    g_ledOk=false; pinMode(PIN_WS2812, OUTPUT); digitalWrite(PIN_WS2812, LOW);
  }

  // Bufor = białe tło
  memset(frame, 0xFF, sizeof(frame));

  Serial.println("[EPD] UC8151 manual mode");
  ledSetDimGreen();
  Serial.println("[HELP] Komendy: 0 cyfry,1 litery,2 paski,3 orient,4 clean,5 white,6 res,7 border,r reinit,i invert,x transpose,h help");
  Serial.println("[INFO] Dodatkowe: b=czarny w=bialy 9=altPush o=swapOrder u=ultraClear y=polBands d=dbg f=full t=stat1");
}

void loop() {
  static uint32_t last=0;
  if(g_autoStatus && millis()-last>1000){
    last=millis();
    Serial.print("[STAT] BUSY="); Serial.print(digitalRead(PIN_BUSY));
    Serial.print(" STATUS=0x"); Serial.print(epd_status(), HEX);
    ledTestCycle();
    Serial.println();
  }
  serialCommands();
}
