#include <Arduino.h>
#include <SPI.h>
#include <GxEPD2_3C.h>

// Fixed wiring for the target board.
#define PIN_SCK   2  // GP2  -> SCK (SPI0)
#define PIN_MOSI  3  // GP3  -> MOSI (SPI0 TX)
#define PIN_MISO  4  // GP4  -> MISO (optional for e-paper)
#define PIN_CS    1  // GP1  -> CS#
#define PIN_DC    6  // GP6  -> D/C
#define PIN_RST   7  // GP7  -> RST#
#define PIN_BUSY  8  // GP8  -> BUSY

static constexpr uint32_t BUSY_TIMEOUT_MS = 9000;
static constexpr uint32_t BUSY_TIMEOUT_US = BUSY_TIMEOUT_MS * 1000UL;

// Helper: derive tri-colour 2.13" UC8151/IL0373 driver (GDEW0213Z16) and trim busy timeout.
class GxEPD2_213c_Timeout : public GxEPD2_213c
{
public:
  GxEPD2_213c_Timeout(int16_t cs, int16_t dc, int16_t rst, int16_t busy)
  : GxEPD2_213c(cs, dc, rst, busy)
  {
    _busy_timeout = BUSY_TIMEOUT_US;
  }
};

// Driver variant: 2.13" tri-colour (IL0373) using the fixed pinout.
GxEPD2_3C<GxEPD2_213c_Timeout, GxEPD2_213c::HEIGHT> display(GxEPD2_213c_Timeout(/*CS=*/PIN_CS, /*DC=*/PIN_DC, /*RST=*/PIN_RST, /*BUSY=*/PIN_BUSY));

static void fullClear()
{
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
  } while (display.nextPage());
}

static void drawDiagnostics()
{
  display.setRotation(1);
  const uint16_t w = display.width();
  const uint16_t h = display.height();
  display.setTextColor(GxEPD_BLACK);

  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.drawFastHLine(0, h / 2, w, GxEPD_BLACK);
    display.drawFastVLine(w / 2, 0, h, GxEPD_BLACK);
    display.drawRect(0, 0, w, h, GxEPD_BLACK);
    display.fillRect(w / 2 - 24, h / 2 - 24, 48, 48, GxEPD_RED);
    display.setCursor(10, 16);
    display.print(F("GxEPD2_213c"));
    display.setCursor(10, 36);
    display.print(F("w="));
    display.print(w);
    display.print(F(" h="));
    display.print(h);
    display.setCursor(10, h - 40);
    display.print(F("BUSY="));
    display.print(digitalRead(PIN_BUSY));
    display.setCursor(10, h - 20);
    display.print(F("timeout="));
    display.print(BUSY_TIMEOUT_MS);
    display.print(F("ms"));
  } while (display.nextPage());
}

void setup()
{
  Serial.begin(115200);
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  display.init(115200, true, 20, false);
  Serial.println(F("[EPD] init done (driver GxEPD2_213c tri-colour)"));
  display.setRotation(1);
  Serial.print(F("[EPD] width/height after rotation: "));
  Serial.print(display.width());
  Serial.print('x');
  Serial.println(display.height());
  fullClear();
  drawDiagnostics();
  Serial.println(F("[EPD] diagnostic frame queued"));
  Serial.println(F("[NOTE] Full refresh can take >10s on tri-colour panels"));
}

void loop()
{
  // Panel retains the last image; nothing to do
}
