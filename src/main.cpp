#include <Arduino.h>
#include <SPI.h>
#include <GxEPD2_3C.h>
#include <ctype.h>
#include <stdlib.h>

#define PIN_SCK   2
#define PIN_MOSI  3
#define PIN_MISO  4
#define PIN_CS    1
#define PIN_DC    6
#define PIN_RST   7
#define PIN_BUSY  8

static constexpr uint32_t BUSY_TIMEOUT_MS = 9000;
static constexpr uint32_t BUSY_TIMEOUT_US = BUSY_TIMEOUT_MS * 1000UL;

class GxEPD2_213c_Lab : public GxEPD2_213c
{
public:
  using GxEPD2_213c::GxEPD2_213c;

  void setBusyTimeout(uint32_t us)
  {
    _busy_timeout = us;
  }

  void rawWriteCommand(uint8_t cmd)
  {
    _writeCommand(cmd);
  }

  void rawWriteDataByte(uint8_t data)
  {
    _writeData(data);
  }

  void waitWhileBusyLab(const char *comment)
  {
    _waitWhileBusy(comment);
  }
};

using Display = GxEPD2_3C<GxEPD2_213c_Lab, GxEPD2_213c::HEIGHT>;
static Display display(GxEPD2_213c_Lab(PIN_CS, PIN_DC, PIN_RST, PIN_BUSY));

static int16_t g_offsetX = 0;
static int16_t g_offsetY = 0;
static uint8_t g_rotation = 1;

struct OffsetPair
{
  int16_t x;
  int16_t y;
};

static OffsetPair g_baseOffset[4] = {
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0}
};

static void ensureInit();
static void fullClear();
static void drawDiagnostics();
static void refreshDisplay(bool verbose = true);
static void handleSerial();
static void processCommand(const String &line);
static void showHelp();
static void printStatus();
static void printBaseOffsets();
static bool parseOffsetValues(const String &input, int16_t &outX, int16_t &outY);
static bool parseRotationValue(const String &input, uint8_t &outRotation);
static bool parseBaseCommand(const String &input, uint8_t &outRotation, int16_t &outX, int16_t &outY);
static bool tokenize(const String &input, String *tokens, size_t &count, size_t maxTokens);
static bool parseHexByte(const String &token, uint8_t &outValue);
static long parseSigned(const String &token, int base = 10);
static void commandRaw(const String &args);
static void commandGate(const String &args);
static void commandHScan(const String &args);
static void commandDiagBlock(const String &args);
static void commandWash();
static void commandFullClear();
static void commandContrastCycle();

static void ensureInit()
{
  static bool initialized = false;
  if (initialized) return;
  display.init(115200, true, 20, false);
  display.epd2.setBusyTimeout(BUSY_TIMEOUT_US);
  initialized = true;
}

static void fullClear()
{
  ensureInit();
  display.setRotation(g_rotation);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
  }
  while (display.nextPage());
}

static void drawDiagnostics()
{
  ensureInit();
  display.setRotation(g_rotation);
  const uint16_t w = display.width();
  const uint16_t h = display.height();
  const OffsetPair base = g_baseOffset[g_rotation & 0x03];
  const int16_t ox = base.x + g_offsetX;
  const int16_t oy = base.y + g_offsetY;

  display.setTextColor(GxEPD_BLACK);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.drawFastHLine(ox, (h / 2) + oy, w, GxEPD_BLACK);
    display.drawFastVLine((w / 2) + ox, oy, h, GxEPD_BLACK);
    display.drawRect(ox, oy, w, h, GxEPD_BLACK);
    const int16_t boxSize = 48;
    const int16_t boxX = (w / 2) + ox - (boxSize / 2);
    const int16_t boxY = (h / 2) + oy - (boxSize / 2);
    display.fillRect(boxX, boxY, boxSize, boxSize, GxEPD_RED);
    display.setCursor(10 + ox, 16 + oy);
    display.print(F("Diag GxEPD2_213c"));
    display.setCursor(10 + ox, 36 + oy);
    display.print(F("w="));
    display.print(w);
    display.print(F(" h="));
    display.print(h);
    display.setCursor(10 + ox, h - 24 + oy);
    display.print(F("rot="));
    display.print(g_rotation);
    display.print(F(" busy="));
    display.print(digitalRead(PIN_BUSY));
    display.setCursor(10 + ox, h - 10 + oy);
    display.print(F("off="));
    display.print(g_offsetX);
    display.print(',');
    display.print(g_offsetY);
    display.print(F(" base="));
    display.print(base.x);
    display.print(',');
    display.print(base.y);
  }
  while (display.nextPage());
}

static void refreshDisplay(bool verbose)
{
  fullClear();
  drawDiagnostics();
  if (verbose)
  {
    Serial.println(F("[EPD] display refreshed"));
  }
}

static void showHelp()
{
  Serial.println(F("[HELP] Commands:"));
  Serial.println(F("  h                 - print this help"));
  Serial.println(F("  s                 - show offsets/status"));
  Serial.println(F("  d                 - redraw diagnostics"));
  Serial.println(F("  o <x> <y>         - set user offsets"));
  Serial.println(F("  rot <0-3>         - set rotation"));
  Serial.println(F("  base <rot> <x> <y>- set base offset"));
  Serial.println(F("  rb                - reset base offsets"));
  Serial.println(F("  r                 - reset user offsets"));
  Serial.println(F("  rawcmd <cmd> [data..] (hex)"));
  Serial.println(F("  gate <start> <end> - set gate range (0x45)"));
  Serial.println(F("  hs <start> <end>   - set horizontal range (0x44)"));
  Serial.println(F("  diag <sx> <ex> <sy> <ey> - draw raw block"));
  Serial.println(F("  wash              - white->black conditioning"));
  Serial.println(F("  clear             - full white clear"));
  Serial.println(F("  contrast          - black/white cycle"));
}

static void printBaseOffsets()
{
  Serial.print(F("[BASE]"));
  for (uint8_t rot = 0; rot < 4; ++rot)
  {
    Serial.print(' ');
    Serial.print(rot);
    Serial.print('=');
    Serial.print(g_baseOffset[rot].x);
    Serial.print(',');
    Serial.print(g_baseOffset[rot].y);
  }
  Serial.println();
}

static void printStatus()
{
  Serial.print(F("[STAT] rot="));
  Serial.print(g_rotation);
  Serial.print(F(" offset="));
  Serial.print(g_offsetX);
  Serial.print(',');
  Serial.println(g_offsetY);
  printBaseOffsets();
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

static bool parseRotationValue(const String &input, uint8_t &outRotation)
{
  String trimmed = input;
  trimmed.trim();
  if (trimmed.length() == 0) return false;
  long value = strtol(trimmed.c_str(), nullptr, 10);
  if (value < 0 || value > 3) return false;
  outRotation = static_cast<uint8_t>(value);
  return true;
}

static bool parseBaseCommand(const String &input, uint8_t &outRotation, int16_t &outX, int16_t &outY)
{
  String trimmed = input;
  trimmed.trim();
  int firstSpace = trimmed.indexOf(' ');
  if (firstSpace < 0) return false;
  String rotStr = trimmed.substring(0, firstSpace);
  String rest = trimmed.substring(firstSpace + 1);
  rotStr.trim();
  rest.trim();
  if (!parseRotationValue(rotStr, outRotation)) return false;
  if (rest.length() == 0) return false;
  int secondSpace = rest.indexOf(' ');
  if (secondSpace < 0) return false;
  String xStr = rest.substring(0, secondSpace);
  String yStr = rest.substring(secondSpace + 1);
  xStr.trim();
  yStr.trim();
  if (xStr.length() == 0 || yStr.length() == 0) return false;
  outX = static_cast<int16_t>(xStr.toInt());
  outY = static_cast<int16_t>(yStr.toInt());
  return true;
}

static bool tokenize(const String &input, String *tokens, size_t &count, size_t maxTokens)
{
  count = 0;
  const int len = input.length();
  int idx = 0;
  while (idx < len && count < maxTokens)
  {
    while (idx < len && isspace(static_cast<unsigned char>(input[idx]))) ++idx;
    if (idx >= len) break;
    const int start = idx;
    while (idx < len && !isspace(static_cast<unsigned char>(input[idx]))) ++idx;
    tokens[count++] = input.substring(start, idx);
  }
  return count > 0;
}

static bool parseHexByte(const String &token, uint8_t &outValue)
{
  const long value = strtol(token.c_str(), nullptr, 16);
  if (value < 0x00 || value > 0xFF) return false;
  outValue = static_cast<uint8_t>(value);
  return true;
}

static long parseSigned(const String &token, int base)
{
  return strtol(token.c_str(), nullptr, base);
}

static void commandRaw(const String &args)
{
  String tokens[10];
  size_t count = 0;
  if (!tokenize(args, tokens, count, 10) || count == 0)
  {
    Serial.println(F("[ERR] usage: rawcmd <cmd> [data..] (hex)"));
    return;
  }
  uint8_t cmd;
  if (!parseHexByte(tokens[0], cmd))
  {
    Serial.println(F("[ERR] invalid command byte"));
    return;
  }
  ensureInit();
  display.epd2.rawWriteCommand(cmd);
  for (size_t i = 1; i < count; ++i)
  {
    uint8_t dataByte;
    if (!parseHexByte(tokens[i], dataByte))
    {
      Serial.print(F("[ERR] invalid data byte at index "));
      Serial.println(i - 1);
      return;
    }
    display.epd2.rawWriteDataByte(dataByte);
  }
  Serial.print(F("[CMD] rawcmd 0x"));
  Serial.print(cmd, HEX);
  Serial.print(F(" len="));
  Serial.println(count - 1);
}

static void commandGate(const String &args)
{
  String tokens[2];
  size_t count = 0;
  if (!tokenize(args, tokens, count, 2) || count != 2)
  {
    Serial.println(F("[ERR] usage: gate <start> <end> (decimal)"));
    return;
  }
  const long start = parseSigned(tokens[0]);
  const long end = parseSigned(tokens[1]);
  if (start < 0 ||  start > 65535 || end < 0 || end > 65535 || start > end)
  {
    Serial.println(F("[ERR] invalid gate arguments"));
    return;
  }
  ensureInit();
  display.epd2.rawWriteCommand(0x45);
  display.epd2.rawWriteDataByte(static_cast<uint8_t>(start & 0xFF));
  display.epd2.rawWriteDataByte(static_cast<uint8_t>((start >> 8) & 0xFF));
  display.epd2.rawWriteDataByte(static_cast<uint8_t>(end & 0xFF));
  display.epd2.rawWriteDataByte(static_cast<uint8_t>((end >> 8) & 0xFF));
  Serial.print(F("[CMD] gate range set to "));
  Serial.print(start);
  Serial.print('-');
  Serial.println(end);
}

static void commandHScan(const String &args)
{
  String tokens[2];
  size_t count = 0;
  if (!tokenize(args, tokens, count, 2) || count != 2)
  {
    Serial.println(F("[ERR] usage: hs <start> <end> (decimal, multiples of 8)"));
    return;
  }
  const long start = parseSigned(tokens[0]);
  const long end = parseSigned(tokens[1]);
  if (start < 0 || end < 0 || start > 255 || end > 255 || start > end)
  {
    Serial.println(F("[ERR] invalid horizontal range"));
    return;
  }
  ensureInit();
  display.epd2.rawWriteCommand(0x44);
  display.epd2.rawWriteDataByte(static_cast<uint8_t>(start));
  display.epd2.rawWriteDataByte(static_cast<uint8_t>(end));
  Serial.print(F("[CMD] horizontal range set to "));
  Serial.print(start);
  Serial.print('-');
  Serial.println(end);
}

static void commandDiagBlock(const String &args)
{
  String tokens[4];
  size_t count = 0;
  if (!tokenize(args, tokens, count, 4) || count != 4)
  {
    Serial.println(F("[ERR] usage: diag <sx> <ex> <sy> <ey>"));
    return;
  }
  const long sx = parseSigned(tokens[0]);
  const long ex = parseSigned(tokens[1]);
  const long sy = parseSigned(tokens[2]);
  const long ey = parseSigned(tokens[3]);
  ensureInit();
  display.setRotation(g_rotation);

  const uint16_t w = display.width();
  const uint16_t h = display.height();
  const uint16_t bytesPerRow = w / 8;

  display.epd2.rawWriteCommand(0x24);
  for (uint16_t y = 0; y < h; ++y)
  {
    for (uint16_t xb = 0; xb < bytesPerRow; ++xb)
    {
      const uint16_t xStart = xb * 8;
      const uint16_t xEnd = xStart + 7;
      uint8_t value = 0xFF;
      if (y >= sy && y <= ey && xStart >= sx && xEnd <= ex)
      {
        value = 0x00;
      }
      display.epd2.rawWriteDataByte(value);
    }
  }
  display.epd2.rawWriteCommand(0x26);
  for (uint32_t i = 0; i < (w * h / 8); ++i)
  {
    display.epd2.rawWriteDataByte(0xFF);
  }
  display.epd2.rawWriteCommand(0x22);
  display.epd2.rawWriteDataByte(0xF7);
  display.epd2.rawWriteCommand(0x20);
  display.epd2.waitWhileBusyLab("diag block");
  Serial.print(F("[CMD] diag block drawn x:"));
  Serial.print(sx);
  Serial.print('-');
  Serial.print(ex);
  Serial.print(F(" y:"));
  Serial.print(sy);
  Serial.print('-');
  Serial.println(ey);
}

static void commandWash()
{
  ensureInit();
  Serial.println(F("[CMD] wash (white -> black -> redraw)"));
  display.epd2.clearScreen(0xFF, 0xFF);
  delay(300);
  display.epd2.clearScreen(0x00, 0xFF);
  delay(300);
  refreshDisplay(false);
}

static void commandFullClear()
{
  ensureInit();
  Serial.println(F("[CMD] clear (full white)"));
  display.epd2.clearScreen(0xFF, 0xFF);
  refreshDisplay(false);
}

static void commandContrastCycle()
{
  ensureInit();
  Serial.println(F("[CMD] contrast cycle (white/black/white)"));
  display.epd2.clearScreen(0xFF, 0xFF);
  delay(200);
  display.epd2.clearScreen(0x00, 0xFF);
  delay(200);
  display.epd2.clearScreen(0xFF, 0xFF);
  refreshDisplay(false);
}

static void processCommand(const String &line)
{
  if (line.length() == 0) return;

  String lower = line;
  lower.toLowerCase();
  if (lower.startsWith("rot"))
  {
    uint8_t newRotation = g_rotation;
    if (!parseRotationValue(line.substring(3), newRotation))
    {
      Serial.println(F("[ERR] usage: rot <0-3>"));
      return;
    }
    g_rotation = newRotation;
    ensureInit();
    display.setRotation(g_rotation);
    Serial.print(F("[CMD] rotation set to "));
    Serial.println(g_rotation);
    refreshDisplay(false);
    return;
  }
  if (lower.startsWith("base"))
  {
    uint8_t rotIndex = g_rotation;
    int16_t baseX = 0;
    int16_t baseY = 0;
    if (!parseBaseCommand(line.substring(4), rotIndex, baseX, baseY))
    {
      Serial.println(F("[ERR] usage: base <rot> <x> <y>"));
      return;
    }
    rotIndex &= 0x03;
    g_baseOffset[rotIndex].x = baseX;
    g_baseOffset[rotIndex].y = baseY;
    Serial.print(F("[CMD] base offset updated rot="));
    Serial.print(rotIndex);
    Serial.print(F(" -> "));
    Serial.print(baseX);
    Serial.print(',');
    Serial.println(baseY);
    refreshDisplay(false);
    return;
  }
  if (lower.startsWith("rb"))
  {
    for (uint8_t rot = 0; rot < 4; ++rot)
    {
      g_baseOffset[rot].x = 0;
      g_baseOffset[rot].y = 0;
    }
    Serial.println(F("[CMD] base offsets reset"));
    refreshDisplay(false);
    return;
  }
  if (lower.startsWith("rawcmd"))
  {
    commandRaw(line.substring(6));
    return;
  }
  if (lower.startsWith("gate"))
  {
    commandGate(line.substring(4));
    return;
  }
  if (lower.startsWith("hs"))
  {
    commandHScan(line.substring(2));
    return;
  }
  if (lower.startsWith("diag"))
  {
    commandDiagBlock(line.substring(4));
    return;
  }
  if (lower.startsWith("wash"))
  {
    commandWash();
    return;
  }
  if (lower.startsWith("clear"))
  {
    commandFullClear();
    return;
  }
  if (lower.startsWith("contrast"))
  {
    commandContrastCycle();
    return;
  }

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
      refreshDisplay(false);
      break;
    case 'r':
      g_offsetX = 0;
      g_offsetY = 0;
      Serial.println(F("[CMD] user offsets reset"));
      refreshDisplay(false);
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
      Serial.print(',');
      Serial.println(g_offsetY);
      refreshDisplay(false);
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

  ensureInit();
  display.setRotation(g_rotation);

  Serial.println(F("[EPD] init done (GxEPD2_213c lab mode)"));
  refreshDisplay(false);
  showHelp();
  printStatus();
  Serial.println(F("[NOTE] Full refresh can take >10s on tri-colour panels"));
}

void loop()
{
  handleSerial();
}


