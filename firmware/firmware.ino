/*
 * スマホ → ESP32(M5Stack AtomS3U) → PC キーボード入力 ドングル
 *
 * スマホ（Bluefy で開いた web/index.html）から Bluetooth(BLE) で文章を受け取り、
 * USB キーボードのふりをして PC に打ち込む。PC には何もインストールしない。
 *
 * 【必要なもの】
 *  - Arduino IDE 2.x
 *  - ボード: esp32 by Espressif Systems 3.x（ボード選択は「M5AtomS3U」または「ESP32S3 Dev Module」）
 *      Tools > USB Mode          : USB-OTG (TinyUSB)
 *      Tools > USB CDC On Boot    : Enabled（シリアルログを見たい時）
 *  - ライブラリ: NimBLE-Arduino 2.x（ライブラリマネージャで「NimBLE-Arduino」）
 *
 * 【通信の約束ごと】web/index.html 冒頭のコメントと必ず一致させること
 *  スマホ→ドングル（WRITE）: 先頭1バイトが命令、1回最大20バイト
 *   0x01 合言葉        : [0x01][合言葉UTF-8 最大19バイト]
 *   0x10 送信開始      : [0x10][モード 0=Unicode 1=ローマ字][配列 0=JIS 1=US][速度 0/1/2][総バイト数 uint32 LE]
 *   0x11 本文の断片    : [0x11][通し番号 uint16 LE][本文 最大17バイト]
 *   0x12 送信終わり    : [0x12]
 *   0x13 停止          : [0x13]
 *   0x20 キー1つ       : [0x20][キー番号][配列（省略時JIS）]
 *   0x30 ジグラー      : [0x30][ON=1/OFF=0][間隔秒 uint16 LE]
 *   0x40 状態問い合わせ : [0x40]
 *  ドングル→スマホ（NOTIFY）
 *   0x81 合言葉の結果 / 0x82 断片の受領 / 0x83 進み具合 / 0x84 完了 / 0x85 停止
 *   0x86 ロック状態 / 0x87 ジグラー状態 / 0x88 エラー(1=未認証 2=入力中 3=容量オーバー 4=順番ずれ)
 */

#include <Arduino.h>
#include "USB.h"
#include "USBHIDKeyboard.h"
#include <NimBLEDevice.h>

// ===================== ここは自分用に変える =====================
#define PASSPHRASE   "change-me"     // 合言葉（19バイトまで。スマホで入れるものと同じにする）
#define DEVICE_NAME  "PhoneKB"       // スマホの接続画面に出る名前
#define AUTO_IME     true            // 英数字の前後で IME を自動で切り替える（PCのIMEは普段「あ」のままでOK）
#define TEST_SERIAL  true            // USBシリアルからの打鍵試験を受け付ける（PC側のClaude Codeの自動試験用）
// ================================================================

// ===== 固定UUID（web/index.html と同じ値） =====
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dc4b10"
#define WRITE_UUID   "6e400002-b5a3-f393-e0a9-e50e24dc4b10"
#define NOTIFY_UUID  "6e400003-b5a3-f393-e0a9-e50e24dc4b10"

// ===== AtomS3U のピン =====
#define PIN_BUTTON 41   // 本体ボタン（押すとLOW）
#define PIN_LED    35   // フルカラーLED（SK6812）

// ===== 大きさの決まり =====
#define MAX_BYTES 32000           // 受け取れる本文の最大バイト数（スマホ側と同じ）

// ===== HID キーコード =====
#define K_ENTER     0x28
#define K_ESC       0x29
#define K_BS        0x2A
#define K_TAB       0x2B
#define K_SPACE     0x2C
#define K_CAPSLOCK  0x39
#define K_INSERT    0x49
#define K_DELETE    0x4C
#define K_NUMLOCK   0x53
#define K_KP_PLUS   0x57
#define K_KP_1      0x59          // テンキー1。2〜9は続き、0は 0x62
#define K_KP_0      0x62
#define K_F15       0x6A
#define K_RO        0x87          // JIS「ろ」キー（\ と _）
#define K_HENKAN    0x8A          // 変換（IME ON に割り当てる）
#define K_MUHENKAN  0x8B          // 無変換（IME OFF に割り当てる）
#define K_YEN       0x89          // JIS「￥」キー（\ と |）
#define K_HANKAKU   0x35          // JIS では 半角/全角
#define K_F10       0x43          // IME の合成中は「半角英数に変換」
#define MOD_LSHIFT  0xE1
#define MOD_LALT    0xE2

USBHIDKeyboard Keyboard;

// ===== BLE =====
NimBLECharacteristic* notifyChar = nullptr;
volatile bool bleConnected = false;
volatile bool authed = false;

// ===== 受け取り中の本文 =====
uint8_t  textBuf[MAX_BYTES];
uint32_t textLen = 0;            // 受け取った量
uint32_t textExpected = 0;       // 0x10 で知らされた総量
uint16_t nextSeq = 0;
bool     receiving = false;

// ===== 打ち込みの設定と状態 =====
uint8_t  typeMode = 0;           // 0=Unicode16進 1=ローマ字
uint8_t  layout = 0;             // 0=JIS 1=US
uint8_t  speed = 1;              // 0=ゆっくり 1=ふつう 2=速い
volatile bool startTyping = false;   // BLE側から「打ち始めて」の合図
volatile bool typing = false;
bool testRun = false;               // USBシリアルからの試験打鍵か（BLE未接続でも打つ）
volatile bool stopRequested = false;

// 速度ごとの待ち時間（ミリ秒）: キーを押している時間 / 次のキーまで
const uint8_t HOLD_MS[3] = { 20, 8, 3 };
const uint8_t GAP_MS[3]  = { 20, 8, 3 };

// ===== ロック状態（PCから届く）=====
volatile uint8_t lockBits = 0;   // bit0=Num bit1=Caps bit2=Scroll
volatile bool lockChanged = false;

// IME の扱い: 0=何もしない 1=無変換(OFF)/変換(ON)キーで切り替え 2=F10(半角英数変換)で確定
uint8_t imeMode = AUTO_IME ? 1 : 0;

// ===== ジグラー =====
// スマホがつながっていなくても動く。差した直後から ON（本体ボタンかスマホで OFF にできる）
#define JIG_ON_AT_BOOT true
bool     jigOn = JIG_ON_AT_BOOT;
uint16_t jigSec = 60;
uint32_t jigLast = 0;

// ===== キー操作命令（BLE側で受けて loop で実行）=====
volatile int pendingKey = -1;
volatile uint8_t pendingKeyLayout = 0;

// =====================================================================
//  通知（ドングル → スマホ）
// =====================================================================
void notify(const uint8_t* data, size_t len) {
  if (notifyChar && bleConnected) notifyChar->notify(data, len);
}
void notify1(uint8_t a) { notify(&a, 1); }
void notifyErr(uint8_t code) { uint8_t b[2] = { 0x88, code }; notify(b, 2); }
void notifyLocks() { uint8_t b[2] = { 0x86, lockBits }; notify(b, 2); }
void notifyJig() { uint8_t b[4] = { 0x87, (uint8_t)jigOn, (uint8_t)(jigSec & 0xff), (uint8_t)(jigSec >> 8) }; notify(b, 4); }
void notifyProgress(uint16_t done, uint16_t total) {
  uint8_t b[5] = { 0x83, (uint8_t)(done & 0xff), (uint8_t)(done >> 8), (uint8_t)(total & 0xff), (uint8_t)(total >> 8) };
  notify(b, 5);
}

// =====================================================================
//  LED
// =====================================================================
void led(uint8_t r, uint8_t g, uint8_t b) { rgbLedWrite(PIN_LED, r, g, b); }

// =====================================================================
//  キーを打つ下回り
// =====================================================================
void tapRaw(uint8_t key, bool shift = false) {
  if (shift) Keyboard.pressRaw(MOD_LSHIFT);
  Keyboard.pressRaw(key);
  delay(HOLD_MS[speed]);
  Keyboard.releaseRaw(key);
  if (shift) Keyboard.releaseRaw(MOD_LSHIFT);
  delay(GAP_MS[speed]);
}

// ASCII 1文字 → キーコードとShift。打てない文字は false
bool asciiToKey(char c, uint8_t lay, uint8_t& key, bool& shift) {
  shift = false;
  if (c >= 'a' && c <= 'z') { key = 0x04 + (c - 'a'); return true; }
  if (c >= 'A' && c <= 'Z') { key = 0x04 + (c - 'A'); shift = true; return true; }
  if (c >= '1' && c <= '9') { key = 0x1E + (c - '1'); return true; }
  if (c == '0') { key = 0x27; return true; }
  if (c == ' ') { key = K_SPACE; return true; }
  if (c == '\n') { key = K_ENTER; return true; }
  if (c == '\t') { key = K_TAB; return true; }

  // 記号: 配列ごとの表 { 文字, キー, Shift }
  struct M { char c; uint8_t k; bool s; };
  static const M US[] = {
    {'!',0x1E,1},{'@',0x1F,1},{'#',0x20,1},{'$',0x21,1},{'%',0x22,1},{'^',0x23,1},{'&',0x24,1},{'*',0x25,1},
    {'(',0x26,1},{')',0x27,1},{'-',0x2D,0},{'_',0x2D,1},{'=',0x2E,0},{'+',0x2E,1},{'[',0x2F,0},{'{',0x2F,1},
    {']',0x30,0},{'}',0x30,1},{'\\',0x31,0},{'|',0x31,1},{';',0x33,0},{':',0x33,1},{'\'',0x34,0},{'"',0x34,1},
    {'`',0x35,0},{'~',0x35,1},{',',0x36,0},{'<',0x36,1},{'.',0x37,0},{'>',0x37,1},{'/',0x38,0},{'?',0x38,1},
  };
  static const M JIS[] = {
    {'!',0x1E,1},{'"',0x1F,1},{'#',0x20,1},{'$',0x21,1},{'%',0x22,1},{'&',0x23,1},{'\'',0x24,1},{'(',0x25,1},
    {')',0x26,1},{'-',0x2D,0},{'=',0x2D,1},{'^',0x2E,0},{'~',0x2E,1},{'@',0x2F,0},{'`',0x2F,1},{'[',0x30,0},
    {'{',0x30,1},{']',0x32,0},{'}',0x32,1},{';',0x33,0},{'+',0x33,1},{':',0x34,0},{'*',0x34,1},{',',0x36,0},
    {'<',0x36,1},{'.',0x37,0},{'>',0x37,1},{'/',0x38,0},{'?',0x38,1},{'\\',K_RO,0},{'_',K_RO,1},{'|',K_YEN,1},
  };
  const M* t = (lay == 1) ? US : JIS;
  size_t n = (lay == 1) ? sizeof(US) / sizeof(M) : sizeof(JIS) / sizeof(M);
  for (size_t i = 0; i < n; i++) if (t[i].c == c) { key = t[i].k; shift = t[i].s; return true; }
  return false;
}

bool typeAscii(char c) {
  uint8_t k; bool s;
  if (!asciiToKey(c, layout, k, s)) return false;
  tapRaw(k, s);
  return true;
}

// Unicode16進入力: Alt を押したまま テンキー「+」→ 16進数字 → Alt を離す
// （PC側で EnableHexNumpad の設定が必要。NumLock は ON にしておく）
void typeUnicodeHex(uint16_t cp) {
  Keyboard.pressRaw(MOD_LALT);
  delay(GAP_MS[speed]);
  tapRaw(K_KP_PLUS);
  for (int shiftBits = 12; shiftBits >= 0; shiftBits -= 4) {
    uint8_t d = (cp >> shiftBits) & 0xF;
    if (d == 0) tapRaw(K_KP_0);
    else if (d <= 9) tapRaw(K_KP_1 + d - 1);   // 数字はテンキー
    else tapRaw(0x04 + (d - 10));              // a〜f は文字キー
  }
  Keyboard.releaseRaw(MOD_LALT);
  delay(GAP_MS[speed]);
}

// =====================================================================
//  ローマ字モード用の表（ひらがな → ローマ字）
//  カタカナはひらがなに直してから引く。出力は IME で「ひらがな」になる。
// =====================================================================
struct Kana { uint16_t cp; const char* r; };
static const Kana KANA[] = {
  {0x3041,"xa"},{0x3042,"a"},{0x3043,"xi"},{0x3044,"i"},{0x3045,"xu"},{0x3046,"u"},{0x3047,"xe"},{0x3048,"e"},{0x3049,"xo"},{0x304A,"o"},
  {0x304B,"ka"},{0x304C,"ga"},{0x304D,"ki"},{0x304E,"gi"},{0x304F,"ku"},{0x3050,"gu"},{0x3051,"ke"},{0x3052,"ge"},{0x3053,"ko"},{0x3054,"go"},
  {0x3055,"sa"},{0x3056,"za"},{0x3057,"si"},{0x3058,"zi"},{0x3059,"su"},{0x305A,"zu"},{0x305B,"se"},{0x305C,"ze"},{0x305D,"so"},{0x305E,"zo"},
  {0x305F,"ta"},{0x3060,"da"},{0x3061,"ti"},{0x3062,"di"},{0x3063,"xtu"},{0x3064,"tu"},{0x3065,"du"},{0x3066,"te"},{0x3067,"de"},{0x3068,"to"},{0x3069,"do"},
  {0x306A,"na"},{0x306B,"ni"},{0x306C,"nu"},{0x306D,"ne"},{0x306E,"no"},
  {0x306F,"ha"},{0x3070,"ba"},{0x3071,"pa"},{0x3072,"hi"},{0x3073,"bi"},{0x3074,"pi"},{0x3075,"hu"},{0x3076,"bu"},{0x3077,"pu"},
  {0x3078,"he"},{0x3079,"be"},{0x307A,"pe"},{0x307B,"ho"},{0x307C,"bo"},{0x307D,"po"},
  {0x307E,"ma"},{0x307F,"mi"},{0x3080,"mu"},{0x3081,"me"},{0x3082,"mo"},
  {0x3083,"xya"},{0x3084,"ya"},{0x3085,"xyu"},{0x3086,"yu"},{0x3087,"xyo"},{0x3088,"yo"},
  {0x3089,"ra"},{0x308A,"ri"},{0x308B,"ru"},{0x308C,"re"},{0x308D,"ro"},
  {0x308E,"xwa"},{0x308F,"wa"},{0x3090,"wyi"},{0x3091,"wye"},{0x3092,"wo"},{0x3093,"nn"},{0x3094,"vu"},
  {0x3095,"xka"},{0x3096,"xke"},
  {0x30FC,"-"},   // ー（長音）
  {0x3001,","},   // 、
  {0x3002,"."},   // 。
  {0x300C,"["},   // 「
  {0x300D,"]"},   // 」
  {0x30FB,"/"},   // ・
  {0x3000," "},   // 全角スペース
};

const char* kanaToRomaji(uint16_t cp) {
  if (cp >= 0x30A1 && cp <= 0x30F6) cp -= 0x60;   // カタカナ → ひらがな
  for (size_t i = 0; i < sizeof(KANA) / sizeof(Kana); i++) if (KANA[i].cp == cp) return KANA[i].r;
  return nullptr;
}

// ローマ字モードでは IME の変換途中の文字が長くなりすぎないよう、ときどき Enter で確定する
uint16_t pendingKana = 0;
void confirmKana() { if (pendingKana) { tapRaw(K_ENTER); pendingKana = 0; } }

// =====================================================================
//  受け取った本文を打ち込む（loop から呼ぶ）
// =====================================================================
// UTF-8 を1文字ずつ取り出す。壊れていたら 0xFFFD
uint32_t nextCodepoint(uint32_t& i) {
  uint8_t c = textBuf[i++];
  if (c < 0x80) return c;
  int extra = (c >= 0xF0) ? 3 : (c >= 0xE0) ? 2 : (c >= 0xC0) ? 1 : 0;
  if (extra == 0) return 0xFFFD;
  uint32_t cp = c & (0x3F >> extra);
  for (int k = 0; k < extra; k++) {
    if (i >= textLen || (textBuf[i] & 0xC0) != 0x80) return 0xFFFD;
    cp = (cp << 6) | (textBuf[i++] & 0x3F);
  }
  return cp;
}

uint16_t countChars() {
  uint32_t n = 0;
  for (uint32_t i = 0; i < textLen; i++) if ((textBuf[i] & 0xC0) != 0x80) n++;
  return n > 0xFFFF ? 0xFFFF : n;
}

void typeAll() {
  typing = true;
  stopRequested = false;
  uint16_t total = countChars();
  uint16_t done = 0;
  uint32_t lastNotify = 0;
  pendingKana = 0;
  Serial.printf("[打鍵] 開始 %u文字 モード=%u 配列=%u 速度=%u ime=%u ロック=%u\n", total, typeMode, layout, speed, imeMode, (unsigned)lockBits);

  // Unicode モードでは NumLock が ON でないとテンキーの数字が打てない
  bool numWasOff = false;
  if (typeMode == 0 && !(lockBits & 1)) {
    numWasOff = true;
    tapRaw(K_NUMLOCK);
    delay(50);
  }

  bool inAscii = false;   // imeMode=1: いま IME OFF にして英数字を打っている途中か
  bool asciiRun = false;  // imeMode=2: いま英数字の連続を打っている途中か
  // imeMode=2: 英数字の連続のあとで、F10 で半角英数にして Enter で確定する
  auto flushAscii = [&]() {
    if (asciiRun) { tapRaw(K_F10); tapRaw(K_ENTER); asciiRun = false; }
  };
  uint32_t i = 0;
  while (i < textLen) {
    if (stopRequested || (!bleConnected && !testRun)) break;
    uint32_t cp = nextCodepoint(i);
    if (cp == '\r') { done++; continue; }

    if (typeMode == 0) {
      // ----- Unicode16進モード（Alt＋テンキー。Word・メモ帳では働かないことが分かっている）-----
      if (imeMode == 1) {
        if (cp < 0x80 && !inAscii) { tapRaw(K_MUHENKAN); delay(30); inAscii = true; }
        else if (cp >= 0x80 && inAscii) { tapRaw(K_HENKAN); delay(30); inAscii = false; }
      }
      if (cp < 0x80) {
        if (!typeAscii((char)cp)) Serial.printf("[打鍵] 打てない文字 0x%02X\n", (unsigned)cp);
      } else if (cp <= 0xFFFF) {
        typeUnicodeHex((uint16_t)cp);
      } else {
        Serial.printf("[打鍵] 絵文字など(U+%X)は飛ばします\n", (unsigned)cp);
      }
    } else {
      // ----- ローマ字モード（IME ON のまま、かなをローマ字で打つ。漢字は打てない）-----
      const char* r = (cp <= 0xFFFF) ? kanaToRomaji((uint16_t)cp) : nullptr;
      if (r) {
        flushAscii();
        for (const char* p = r; *p; p++) typeAscii(*p);
        pendingKana++;
        // 句読点のあと、または30文字ごとに確定
        if (cp == 0x3001 || cp == 0x3002 || pendingKana >= 30) confirmKana();
      } else if (cp == '\n') {
        flushAscii();
        confirmKana();
        tapRaw(K_ENTER);
      } else if (cp < 0x80) {
        if (imeMode == 2) {
          if (!asciiRun) { confirmKana(); asciiRun = true; }
          typeAscii((char)cp);
        } else {
          if (imeMode == 1 && !inAscii) { confirmKana(); tapRaw(K_MUHENKAN); delay(30); inAscii = true; }
          typeAscii((char)cp);
          if (imeMode != 1) pendingKana++;
        }
      } else {
        Serial.printf("[打鍵] ローマ字にできない文字(U+%X)は飛ばします\n", (unsigned)cp);
      }
      if (cp >= 0x80 && inAscii) { tapRaw(K_HENKAN); delay(30); inAscii = false; }
    }

    done++;
    if (millis() - lastNotify > 200) { notifyProgress(done, total); lastNotify = millis(); }
  }
  if (typeMode == 1) { flushAscii(); confirmKana(); }
  if (imeMode == 1 && inAscii) { tapRaw(K_HENKAN); delay(30); }   // IME を日本語入力に戻す
  if (numWasOff) tapRaw(K_NUMLOCK);   // NumLock を元に戻す
  Keyboard.releaseAll();

  typing = false;
  textLen = 0; textExpected = 0; receiving = false;
  jigLast = millis();
  if (stopRequested) {
    Serial.println("[打鍵] 停止しました");
    notify1(0x85);
  } else {
    notifyProgress(done, total);
    notify1(0x84);
    Serial.println("[打鍵] 完了");
  }
  stopRequested = false;
}


// =====================================================================
//  試験用: USBシリアル(CDC)から打鍵を起動する（TEST_SERIAL が true のとき）
//  PC側のClaude Codeが、スマホ無しで自動試験をするために使う。
//   送る行: @T <モード> <配列> <速度> <ime> <base64(UTF-8)>\n
//     モード 0=Unicode16進 1=ローマ字 / 配列 0=JIS 1=US / 速度 0〜2 / ime 0〜2
//   返す行: @TOK（受付）/ @TNG <理由> / @TDONE（打ち終わり）
// =====================================================================
#if TEST_SERIAL
int b64val(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}
size_t b64decode(const char* in, uint8_t* out, size_t cap) {
  size_t n = 0; uint32_t acc = 0; int bits = 0;
  for (; *in; in++) {
    int v = b64val(*in);
    if (v < 0) continue;
    acc = (acc << 6) | v; bits += 6;
    if (bits >= 8) { bits -= 8; if (n >= cap) return n; out[n++] = (acc >> bits) & 0xFF; }
  }
  return n;
}
String testLine;
void handleTestLine(const String& line) {
  if (!line.startsWith("@T ")) return;
  if (typing || startTyping) { Serial.println("@TNG busy"); return; }
  int m, l, sp, ime;
  int consumed = 0;
  if (sscanf(line.c_str() + 3, "%d %d %d %d %n", &m, &l, &sp, &ime, &consumed) < 4 || consumed == 0) { Serial.println("@TNG format"); return; }
  typeMode = m ? 1 : 0; layout = l ? 1 : 0; speed = sp > 2 ? 1 : sp; imeMode = ime > 2 ? 1 : ime;
  textLen = b64decode(line.c_str() + 3 + consumed, textBuf, MAX_BYTES);
  testRun = true; startTyping = true;
  Serial.printf("@TOK %u\n", (unsigned)textLen);
}
void pollSerialTest() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleTestLine(testLine); testLine = ""; }
    else if (c != '\r' && testLine.length() < 46000) testLine += c;
  }
}
#endif

// =====================================================================
//  キー1つの命令
// =====================================================================
void doKey(int code, uint8_t lay) {
  switch (code) {
    case 1:  tapRaw(K_ENTER); break;
    case 2:  tapRaw(K_BS); break;
    case 3:  tapRaw(K_TAB); break;
    case 4:  tapRaw(K_HENKAN); break;      // IME ON（変換キー）
    case 5:  tapRaw(K_MUHENKAN); break;    // IME OFF（無変換キー）
    case 6:  tapRaw(K_HANKAKU); break;     // 半角/全角
    case 7:  tapRaw(K_NUMLOCK); break;
    case 8:  tapRaw(K_CAPSLOCK, lay == 0); break;  // JIS では Shift+英数 で CapsLock
    case 9:  tapRaw(K_INSERT); break;
    case 10: tapRaw(K_ESC); break;
    case 11: tapRaw(K_DELETE); break;
    case 12: tapRaw(K_SPACE); break;
    default: Serial.printf("[キー] 知らない番号 %d\n", code); return;
  }
  Serial.printf("[キー] %d\n", code);
}

// =====================================================================
//  BLE の受け取り
// =====================================================================
class WriteCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    std::string v = c->getValue();
    const uint8_t* b = (const uint8_t*)v.data();
    size_t n = v.size();
    if (n == 0) return;

    // 合言葉だけは未認証でも受け付ける
    if (b[0] == 0x01) {
      std::string p((const char*)b + 1, n - 1);
      authed = (p == PASSPHRASE);
      uint8_t r[2] = { 0x81, (uint8_t)(authed ? 1 : 0) };
      notify(r, 2);
      Serial.println(authed ? "[BLE] 合言葉OK" : "[BLE] 合言葉NG");
      return;
    }
    if (!authed) { notifyErr(1); return; }

    switch (b[0]) {
      case 0x10:   // 送信開始
        if (typing || startTyping) { notifyErr(2); return; }
        if (n < 8) return;
        typeMode = b[1]; layout = b[2]; speed = b[3] > 2 ? 1 : b[3];
        textExpected = b[4] | (b[5] << 8) | (b[6] << 16) | ((uint32_t)b[7] << 24);
        if (textExpected > MAX_BYTES) { notifyErr(3); receiving = false; return; }
        textLen = 0; nextSeq = 0; receiving = true;
        Serial.printf("[BLE] 受け取り開始 %u バイト\n", (unsigned)textExpected);
        break;

      case 0x11: { // 本文の断片
        if (!receiving || n < 3) { notifyErr(4); return; }
        uint16_t seq = b[1] | (b[2] << 8);
        if (seq != nextSeq) { notifyErr(4); receiving = false; return; }
        size_t len = n - 3;
        if (textLen + len > MAX_BYTES) { notifyErr(3); receiving = false; return; }
        memcpy(textBuf + textLen, b + 3, len);
        textLen += len;
        nextSeq++;
        uint8_t ack[3] = { 0x82, b[1], b[2] };
        notify(ack, 3);
        break;
      }

      case 0x12:   // 送信終わり → loop で打ち始める
        if (!receiving) { notifyErr(4); return; }
        if (textLen != textExpected) Serial.printf("[BLE] 量が違います 予定%u 実際%u\n", (unsigned)textExpected, (unsigned)textLen);
        receiving = false;
        startTyping = true;
        break;

      case 0x13:   // 停止
        stopRequested = true;
        if (!typing) {
          startTyping = false; receiving = false; textLen = 0;
          notify1(0x85);
        }
        break;

      case 0x20:   // キー1つ
        if (typing) { notifyErr(2); return; }
        if (n >= 2) { pendingKeyLayout = (n >= 3) ? b[2] : 0; pendingKey = b[1]; }
        break;

      case 0x30:   // ジグラー
        if (n >= 4) {
          jigOn = b[1] == 1;
          uint16_t s = b[2] | (b[3] << 8);
          jigSec = constrain(s, 30, 300);
          jigLast = millis();
          notifyJig();
          Serial.printf("[ジグラー] %s %u秒\n", jigOn ? "ON" : "OFF", jigSec);
        }
        break;

      case 0x40:   // 状態問い合わせ
        notifyLocks();
        notifyJig();
        break;
    }
  }
};

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    bleConnected = true; authed = false;
    Serial.println("[BLE] 接続されました");
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    bleConnected = false; authed = false;
    receiving = false;
    if (typing) stopRequested = true;   // 切れたら打つのをやめる
    Serial.printf("[BLE] 切断されました (理由 %d)\n", reason);
    NimBLEDevice::startAdvertising();
  }
};

// =====================================================================
//  PC から届くロック状態（NumLock/CapsLock/ScrollLock のランプ）
// =====================================================================
void onKeyboardEvent(void* arg, esp_event_base_t base, int32_t id, void* data) {
  if (id != ARDUINO_USB_HID_KEYBOARD_LED_EVENT) return;
  arduino_usb_hid_keyboard_event_data_t* d = (arduino_usb_hid_keyboard_event_data_t*)data;
  lockBits = (d->numlock ? 1 : 0) | (d->capslock ? 2 : 0) | (d->scrolllock ? 4 : 0);
  lockChanged = true;
}

// =====================================================================
//  起動
// =====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  led(0, 0, 8);

  Keyboard.onEvent(ARDUINO_USB_HID_KEYBOARD_LED_EVENT, onKeyboardEvent);
  Keyboard.begin();
  USB.begin();

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(3);   // 送信出力（dBm）。近くで使う前提なので控えめ
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  NimBLEService* svc = server->createService(SERVICE_UUID);
  NimBLECharacteristic* wr = svc->createCharacteristic(WRITE_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  wr->setCallbacks(new WriteCB());
  notifyChar = svc->createCharacteristic(NOTIFY_UUID, NIMBLE_PROPERTY::NOTIFY);
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setName(DEVICE_NAME);
  adv->enableScanResponse(true);
  adv->start();

  jigLast = millis();
  Serial.println("[起動] スマホからの接続を待っています（ジグラーは単独で動きます）");
}

// =====================================================================
//  ずっと繰り返す
// =====================================================================
void loop() {
  static uint32_t btnDown = 0;
  static bool btnPrev = false;

  // ----- 打ち込み -----
#if TEST_SERIAL
  pollSerialTest();
#endif
  if (startTyping) {
    startTyping = false;
    led(0, 0, 40);
    typeAll();
    if (testRun) { testRun = false; Serial.println("@TDONE"); }
  }

  // ----- キー1つ -----
  if (pendingKey >= 0) {
    int k = pendingKey; pendingKey = -1;
    doKey(k, pendingKeyLayout);
  }

  // ----- ロック状態をスマホへ -----
  if (lockChanged) { lockChanged = false; notifyLocks(); }

  // ----- 本体ボタン: 押すたびにジグラー ON/OFF -----
  bool btn = digitalRead(PIN_BUTTON) == LOW;
  if (btn && !btnPrev) btnDown = millis();
  if (!btn && btnPrev && millis() - btnDown > 30) {
    jigOn = !jigOn; jigLast = millis();
    notifyJig();
    Serial.printf("[ジグラー] ボタンで %s\n", jigOn ? "ON" : "OFF");
  }
  btnPrev = btn;

  // ----- ジグラー: 決めた間隔で F15 を押す（打ち込み中は休む）-----
  if (jigOn && !typing && millis() - jigLast >= (uint32_t)jigSec * 1000) {
    jigLast = millis();
    tapRaw(K_F15);
    Serial.println("[ジグラー] F15");
  }

  // ----- LED: ジグラーONなら緑点滅、接続中は青、待機中は暗い青 -----
  if (jigOn) {
    bool on = (millis() / 500) % 2;
    led(0, on ? 30 : 0, 0);
  } else if (bleConnected && authed) {
    led(0, 0, 20);
  } else {
    led(0, 0, 3);
  }

  delay(10);
}
