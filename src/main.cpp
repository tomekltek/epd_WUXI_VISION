#include <Arduino.h>
#include <SPI.h>
#include <GxEPD2_3C.h>
#include <ctype.h>

// Fixed wiring for the target board.
#define PIN_SCK   2  // GP2  -> SCK (SPI0)
#define PIN_MOSI  3  // GP3  -> MOSI (SPI0 TX)
#define PIN_MISO  4  // GP4  -> MISO (optional for e-paper)
#define PIN_CS    1  // GP1  -> CS#
#define PIN_DC    6  // GP6  -> D/C
#define PIN_RST   7  // GP7  -> RST#
#define PIN_BUSY  8  // GP8  -> BUSY

static constexpr uint32_t BUSY_TIMEOUT_MS = 5000;
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

static int16_t g_offsetX = 0;
static int16_t g_offsetY = 0;

static void fullClear();
static void drawDiagnostics();
static void refreshDisplay();
static void handleSerial();
static void processCommand(const String &line);
static void showHelp();
static void printStatus();
static bool parseOffsetValues(const String &input, int16_t &outX, int16_t &outY);

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
  const int16_t ox = g_offsetX;
  const int16_t oy = g_offsetY;
  display.setTextColor(GxEPD_BLACK);

  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.drawFastHLine(ox, (h / 2) + oy, w, GxEPD_BLACK);
    display.drawFastVLine((w / 2) + ox, oy, h, GxEPD_BLACK);
    display.drawRect(ox, oy, w, h, GxEPD_BLACK);
    const int16_t boxSize = 48;
    const int16_t boxX = (w / 2) + ox - (boxSize / 2);
    const int16_t boxY = (h / 2) + oy - (boxSize / 2);
    display.fillRect(boxX, boxY, boxSize, boxSize, GxEPD_RED);
    display.setCursor(10 + ox, 16 + oy);
    display.print(F("GxEPD2_213c"));
    display.setCursor(10 + ox, 36 + oy);
    display.print(F("w="));
    display.print(w);
    display.print(F(" h="));
    display.print(h);
    display.setCursor(10 + ox, h - 40 + oy);
    display.print(F("BUSY="));
    display.print(digitalRead(PIN_BUSY));
    display.setCursor(10 + ox, h - 20 + oy);
    display.print(F("timeout="));
    display.print(BUSY_TIMEOUT_MS);
    display.print(F("ms"));
    display.setCursor(10 + ox, h - 4 + oy);
    display.print(F("offset="));
    display.print(g_offsetX);
    display.print(',');
    display.print(g_offsetY);
  } while (display.nextPage());
}

static void refreshDisplay()
{
  fullClear();
  drawDiagnostics();
  Serial.println(F("[EPD] display refreshed"));
}

static void showHelp()
{
  Serial.println(F("[HELP] Commands:"));
  Serial.println(F("  h            - print this help"));
  Serial.println(F("  s            - show current offsets/status"));
  Serial.println(F("  d            - redraw diagnostics frame"));
  Serial.println(F("  o <x> <y>    - set pixel offsets (can be negative)"));
  Serial.println(F("  r            - reset offsets to 0,0"));
}

static void printStatus()
{
  Serial.print(F("[STAT] offsetX="));
  Serial.print(g_offsetX);
  Serial.print(F(" offsetY="));
  Serial.println(g_offsetY);
}

static bool parseOffsetValues(const String &input, int16_t &outX, int16_t &outY)
{
  String trimmed = input;
  trimmed.trim();
  int spacePos = trimmed.indexOf(' ');
  if (spacePos < 0) return false;
  String first = trimmed.substring(0, spacePos);
  String second = trimmed.substring(spacePos + 1);
  first.trim();
  second.trim();
  if (first.length() == 0 || second.length() == 0) return false;
  outX = static_cast<int16_t>(first.toInt());
  outY = static_cast<int16_t>(second.toInt());
  return true;
}

static void processCommand(const String &line)
{
  if (line.length() == 0) return;
  const char cmd = tolower(line.charAt(0));
  switch (cmd)
  {
    case 'h':
      showHelp();
      break;
    case 's':
      printStatus();
      break;
    case 'd':
      Serial.println(F("[CMD] redraw"));
      refreshDisplay();
      break;
    case 'r':
      g_offsetX = 0;
      g_offsetY = 0;
      Serial.println(F("[CMD] offsets reset to 0,0"));
      refreshDisplay();
      break;
    case 'o':
    {
      int16_t newX = g_offsetX;
      int16_t newY = g_offsetY;
      if (!parseOffsetValues(line.substring(1), newX, newY))
      {
        Serial.println(F("[ERR] usage: o <x> <y>"));
        break;
      }
      g_offsetX = newX;
      g_offsetY = newY;
      Serial.print(F("[CMD] offsets updated to "));
      Serial.print(g_offsetX);
      Serial.print(F(","));
      Serial.println(g_offsetY);
      refreshDisplay();
      break;
    }
    default:
      Serial.println(F("[ERR] unknown command; use 'h' for help"));
      break;
  }
}

static void handleSerial()
{
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  processCommand(line);
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
  refreshDisplay();
  showHelp();
  printStatus();
  Serial.println(F("[NOTE] Full refresh can take >10s on tri-colour panels"));
}

void loop()
{
  handleSerial();
}
