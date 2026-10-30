  /*
  * esc_bridge_v2.ino - Orange Pi (ROS2) <-> STM32H7xx <-> ESC Hobbywing QuicRun WP 10BL60 G2
  * Core STM32duino. Them: console debug qua Serial Monitor + lenh phat xung tho + doi chieu luc chay.
  *
  * ---- Giao thuc nhi phan voi Orange Pi (khong doi so voi ban truoc) ----
  *   [0xAA][0x55][LEN][TYPE][PAYLOAD][CRC8]   CRC8 poly 0x07 tren LEN,TYPE,PAYLOAD
  *   Host -> MCU: 0x01 THROTTLE(int16, uint8 mode) | 0x02 STOP | 0x03 REDUCE | 0x04 PING
  *   MCU -> Host: 0x81 STATUS(int16 cur, int16 tgt, uint8 flags)
  *
  * ---- Console debug (Serial Monitor, chon "Newline", 115200) ----
  *   Go 'h' de xem danh sach lenh. Console chi hoat dong khi KHONG co Orange Pi dang gui khung nhi phan
  *   (khi host im qua 2 s thi console tu mo lai). Khi go lenh, failsafe tam tat cho den khi host gui khung moi.
  */
#include <IWatchdog.h>

HardwareTimer *escTim = nullptr;
uint32_t       escCh  = 0;

  // ===== Cong (DOI CHO PHU HOP BOARD) =====
  constexpr PinName ESC_PIN = PE_9;      // chan tin hieu ESC
  #define LinkSerial Serial              // cong noi Orange Pi. Neu dung Serial1/Serial2... thi doi o day
  constexpr uint32_t LINK_BAUD = 115200;

  // 1 = Serial Monitor va Orange Pi dung CHUNG 1 cong (bench test, host phat hien bang khung 0xAA 0x55)
  // 0 = console debug dung cong rieng 'Serial' (vd USB CDC), cong LinkSerial chi danh cho Orange Pi
  #define DEBUG_SAME_PORT 1
  #if DEBUG_SAME_PORT
    #define DbgPort LinkSerial
  #else
    #define DbgPort Serial
  #endif

  // ===== Cau hinh ESC =====
  constexpr int PWM_MIN     = 1000;
  constexpr int PWM_NEUTRAL = 1500;
  constexpr int PWM_MAX     = 2000;

  // Phia nao la "tien" phu thuoc vao luc ban calibrate ESC bang nut SET (xem huong dan).
  // Co the doi luc chay bang lenh 'z' trong console.
  int fwdLimit = 1800;
  int revLimit = 1200;
  constexpr int THROTTLE_DEADBAND = 30;  // /1000

  constexpr int RAMP_STEP           = 10;
  constexpr int RAMP_DELAY_MS       = 50;
  constexpr int ESC_ARM_DELAY_MS    = 5000;
  constexpr int DIRECTION_SAFE_MS   = 700;
  constexpr int DOUBLE_TAP_MS       = 400;
  constexpr int FAILSAFE_TIMEOUT_MS = 500;
  constexpr int HOST_IDLE_MS        = 2000;   // host im qua thoi gian nay thi console duoc phep
  constexpr int STATUS_PERIOD_MS    = 50;
  constexpr int VERBOSE_PERIOD_MS   = 250;

  inline int minLimit() { return fwdLimit < revLimit ? fwdLimit : revLimit; }
  inline int maxLimit() { return fwdLimit > revLimit ? fwdLimit : revLimit; }
  inline int revDir()   { return (revLimit < PWM_NEUTRAL) ? -1 : 1; }
  inline int tapPWM()   { return PWM_NEUTRAL + revDir() * 200; }

  // ===== Giao thuc =====
  constexpr uint8_t SOF1 = 0xAA, SOF2 = 0x55;
  constexpr uint8_t MAX_PAYLOAD = 16;
  constexpr uint8_t CMD_THROTTLE = 0x01, CMD_STOP = 0x02, CMD_REDUCE = 0x03, CMD_PING = 0x04;
  constexpr uint8_t MSG_STATUS = 0x81;

  // ===== Trang thai =====
  enum Seq { SEQ_NONE, SEQ_DIR_WAIT, SEQ_TAP_BRAKE, SEQ_TAP_NEUTRAL };
void escBegin() {
  TIM_TypeDef *inst = (TIM_TypeDef *)pinmap_peripheral(ESC_PIN, PinMap_PWM);
  escCh  = STM_PIN_CHANNEL(pinmap_function(ESC_PIN, PinMap_PWM));
  escTim = new HardwareTimer(inst);
  escTim->setOverflow(50, HERTZ_FORMAT);                       // 50 Hz
  escTim->setMode(escCh, TIMER_OUTPUT_COMPARE_PWM1, ESC_PIN);
  escTim->setCaptureCompare(escCh, PWM_NEUTRAL, MICROSEC_COMPARE_FORMAT);
  escTim->resume();
}

inline void escWrite(int us) {
  escTim->setCaptureCompare(escCh, us, MICROSEC_COMPARE_FORMAT);
}
  int      curPWM = PWM_NEUTRAL;
  int      tgtPWM = PWM_NEUTRAL;
  int      pendingPWM = PWM_NEUTRAL;
  bool     pendingInstant = true;
  bool     reverseArmed = false;
  bool     failsafe = false;
  bool     hostSeen = false;     // da nhan khung nhi phan hop le tu Orange Pi
  bool     verbose = true;
  Seq      seq = SEQ_NONE;
  uint32_t seqEnd = 0;
  uint32_t lastRxMs = 0, lastRampMs = 0, lastStatusMs = 0, lastVerboseMs = 0;

  // ===== In debug =====
  inline bool dbgEnabled() { return DEBUG_SAME_PORT ? !hostSeen : true; }
  #define DBG_PRINT(x)   do { if (dbgEnabled()) DbgPort.print(x); } while (0)
  #define DBG_PRINTLN(x) do { if (dbgEnabled()) DbgPort.println(x); } while (0)

  const char *seqName(int s) {
    switch (s) {
      case SEQ_DIR_WAIT:    return "DIR_WAIT";
      case SEQ_TAP_BRAKE:   return "TAP_BRAKE";
      case SEQ_TAP_NEUTRAL: return "TAP_NEUTRAL";
      default:              return "-";
    }
  }

  // ===== CRC8 =====
  uint8_t crc8(const uint8_t *d, uint16_t n, uint8_t crc) {
    while (n--) {
      crc ^= *d++;
      for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
  }

  // ===== ESC co ban =====
  void setPWM(int us) {
    us = constrain(us, PWM_MIN, PWM_MAX);
    escWrite(us);
    curPWM = us;
  }

  int dirOf(int pwm) {
    if (pwm < PWM_NEUTRAL) return -1;
    if (pwm > PWM_NEUTRAL) return 1;
    return 0;
  }

  // Phat xung tho: bo qua gioi han, ramp, chuoi cham phanh. Dung de kiem tra ESC.
  void rawPulse(int us) {
    us = constrain(us, PWM_MIN, PWM_MAX);
    seq = SEQ_NONE;
    tgtPWM = us;
    pendingPWM = us;
    setPWM(us);
  }

  // Yeu cau dat ga. An toan khi goi lap lai (heartbeat)
  void request(int pwm, bool instant) {
    pwm = constrain(pwm, minLimit(), maxLimit());
    uint32_t now = millis();
    int nxt = dirOf(pwm);

    // Ve trung tinh: luon huy chuoi dang chay
    if (nxt == 0) {
      seq = SEQ_NONE;
      pendingPWM = PWM_NEUTRAL;
      tgtPWM = PWM_NEUTRAL;
      if (instant || curPWM == tapPWM()) setPWM(PWM_NEUTRAL);
      return;
    }

    pendingPWM = pwm;
    pendingInstant = instant;
    if (seq != SEQ_NONE) return;     // chuoi se tu lay pending khi xong

    int cur = dirOf(curPWM);

    // Dao chieu truc tiep: ve trung tinh va cho
    if (cur != 0 && cur != nxt) {
      DBG_PRINTLN("[SEQ] dao chieu -> ve trung tinh va cho");
      setPWM(PWM_NEUTRAL);
      tgtPWM = PWM_NEUTRAL;
      seq = SEQ_DIR_WAIT;
      seqEnd = now + DIRECTION_SAFE_MS;
      return;
    }

    if (nxt == revDir()) {
      // ESC F/B/R: lan day dau tien sang phia lui chi la PHANH -> cham phanh roi ve trung tinh
      if (!reverseArmed) {
        DBG_PRINTLN("[SEQ] cham phanh truoc khi lui (double-tap)");
        setPWM(tapPWM());
        tgtPWM = PWM_NEUTRAL;
        seq = SEQ_TAP_BRAKE;
        seqEnd = now + DOUBLE_TAP_MS;
        return;
      }
    } else {
      reverseArmed = false;          // di tien thi lan lui sau phai cham lai
    }

    tgtPWM = pwm;
    if (instant) setPWM(pwm);
  }

  void seqUpdate(uint32_t now) {
    if (seq == SEQ_NONE || (int32_t)(now - seqEnd) < 0) return;
    switch (seq) {
      case SEQ_DIR_WAIT:
        seq = SEQ_NONE;
        request(pendingPWM, pendingInstant);
        break;
      case SEQ_TAP_BRAKE:
        setPWM(PWM_NEUTRAL);
        seq = SEQ_TAP_NEUTRAL;
        seqEnd = now + DOUBLE_TAP_MS;
        break;
      case SEQ_TAP_NEUTRAL:
        reverseArmed = true;
        DBG_PRINTLN("[SEQ] xong cham phanh, ESC san sang lui");
        seq = SEQ_NONE;
        request(pendingPWM, pendingInstant);
        break;
      default:
        seq = SEQ_NONE;
        break;
    }
  }

  void rampUpdate(uint32_t now) {
    if (seq != SEQ_NONE || curPWM == tgtPWM) return;
    if (now - lastRampMs < RAMP_DELAY_MS) return;
    lastRampMs = now;

    int next = curPWM;
    if (curPWM < tgtPWM) { next += RAMP_STEP; if (next > tgtPWM) next = tgtPWM; }
    else                 { next -= RAMP_STEP; if (next < tgtPWM) next = tgtPWM; }
    setPWM(next);
  }

  // throttle -1000..1000 -> PWM (duong = tien)
  int throttleToPWM(int t) {
    t = constrain(t, -1000, 1000);
    if (abs(t) < THROTTLE_DEADBAND) return PWM_NEUTRAL;
    if (t > 0) return PWM_NEUTRAL + (long)t * (fwdLimit - PWM_NEUTRAL) / 1000;
    return PWM_NEUTRAL + (long)(-t) * (revLimit - PWM_NEUTRAL) / 1000;
  }

  // ===== Console debug =====
  void printStatus() {
    DbgPort.print("cur="); DbgPort.print(curPWM);
    DbgPort.print(" tgt="); DbgPort.print(tgtPWM);
    
    DbgPort.print(" seq="); DbgPort.print(seqName(seq));
    DbgPort.print(" revArmed="); DbgPort.print(reverseArmed ? 1 : 0);
    DbgPort.print(" failsafe="); DbgPort.print(failsafe ? 1 : 0);
    DbgPort.print(" fwd="); DbgPort.print(fwdLimit);
    DbgPort.print(" rev="); DbgPort.println(revLimit);
  }

  void printHelp() {
    DbgPort.println();
    DbgPort.println("===== LENH (ket thuc bang Enter) =====");
    DbgPort.println("w / x     : tien / lui tu tu (toi gioi han)");
    DbgPort.println("f / r     : tien / lui lap tuc");
    DbgPort.println("t <n>     : ga tu tu, n = -1000..1000 (duong = tien)");
    DbgPort.println("i <n>     : ga lap tuc, n = -1000..1000");
    DbgPort.println("u <us>    : PHAT XUNG THO 1000..2000 us (bo qua moi bao ve) - de test ESC");
    DbgPort.println("d         : giam ga con 1/2");
    DbgPort.println("n         : ve trung tinh tu tu");
    DbgPort.println("s / space : dung lap tuc");
    DbgPort.println("z         : doi cho gioi han tien <-> lui (chi khi dang o trung tinh)");
    DbgPort.println("p         : in trang thai");
    DbgPort.println("v         : bat/tat in trang thai dinh ky");
    DbgPort.println("h         : tro giup");
    DbgPort.println("======================================");
  }

  void runCommand(char *s) {
    hostSeen = false;                // nhuong quyen cho console, tat failsafe
    failsafe = false;
    char cmd = (char)tolower((unsigned char)s[0]);
    bool hasArg = (s[1] != 0);
    long val = hasArg ? atol(s + 1) : 0;

    switch (cmd) {
      case 'h': case '?': printHelp(); break;
      case 's':
        request(PWM_NEUTRAL, true);
        DbgPort.println("DUNG LAP TUC");
        break;
      case 'w': DbgPort.println("TIEN TU TU");   request(fwdLimit, false); break;
      case 'x': DbgPort.println("LUI TU TU");    request(revLimit, false); break;
      case 'f': DbgPort.println("TIEN LAP TUC"); request(fwdLimit, true);  break;
      case 'r': DbgPort.println("LUI LAP TUC");  request(revLimit, true);  break;
      case 'n': DbgPort.println("VE TRUNG TINH TU TU"); request(PWM_NEUTRAL, false); break;
      case 'd': {
        int offset = curPWM - PWM_NEUTRAL;
        request(PWM_NEUTRAL + offset / 2, true);
        DbgPort.println("GIAM GA 1/2");
        break;
      }
      case 't': case 'i':
        if (!hasArg) { DbgPort.println("Thieu tham so, vd: t 300"); break; }
        DbgPort.print("GA "); DbgPort.print(val);
        DbgPort.print(" -> PWM "); DbgPort.println(throttleToPWM((int)val));
        request(throttleToPWM((int)val), cmd == 'i');
        break;
      case 'u':
        if (!hasArg) { DbgPort.println("Thieu tham so, vd: u 1600"); break; }
        DbgPort.print("XUNG THO "); DbgPort.print(constrain((int)val, PWM_MIN, PWM_MAX));
        DbgPort.println(" us");
        rawPulse((int)val);
        break;
      case 'z':
        if (curPWM != PWM_NEUTRAL || seq != SEQ_NONE) {
          DbgPort.println("Chi doi duoc khi dang o trung tinh. Go 's' truoc.");
        } else {
          int tmp = fwdLimit; fwdLimit = revLimit; revLimit = tmp;
          reverseArmed = false;
          DbgPort.print("Da doi: tien="); DbgPort.print(fwdLimit);
          DbgPort.print(" lui="); DbgPort.println(revLimit);
        }
        break;
      case 'p': printStatus(); break;
      case 'v':
        verbose = !verbose;
        DbgPort.println(verbose ? "In dinh ky: BAT" : "In dinh ky: TAT");
        break;
      default:
        DbgPort.print("Lenh khong hop le: "); DbgPort.println(s);
        break;
    }
  }

  char    lineBuf[24];
  uint8_t lineLen = 0;

  void consoleByte(char c) {
    if (c == '\r' || c == '\n') {
      if (lineLen) {
        lineBuf[lineLen] = 0;
        lineLen = 0;
        runCommand(lineBuf);
      }
      return;
    }
    if (c == ' ' && lineLen == 0) {   // phim cach = dung khan cap, khong can Enter
      char stopCmd[2] = {'s', 0};
      runCommand(stopCmd);
      return;
    }
    if (c < 32 || c > 126) return;    // bo ky tu rac
    if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
  }

  inline bool consoleAccepts() {
    return !hostSeen || (millis() - lastRxMs > HOST_IDLE_MS);
  }

  // ===== Xu ly frame nhi phan =====
  void handleFrame(uint8_t type, const uint8_t *p, uint8_t len) {
    switch (type) {
      case CMD_THROTTLE:
        if (len >= 3) {
          int16_t t = (int16_t)(p[0] | (p[1] << 8));
          request(throttleToPWM(t), p[2] == 1);
        }
        break;
      case CMD_STOP:
        request(PWM_NEUTRAL, true);
        break;
      case CMD_REDUCE: {
        int offset = curPWM - PWM_NEUTRAL;
        request(PWM_NEUTRAL + offset / 2, true);
        break;
      }
      case CMD_PING:
      default:
        break;
    }
  }

  enum PState { P_H1, P_H2, P_LEN, P_BODY, P_CRC };
  PState  ps = P_H1;
  uint8_t rxLen = 0, rxIdx = 0, rxBuf[MAX_PAYLOAD + 1];

  void parseByte(uint8_t b) {
    switch (ps) {
      case P_H1:
        if (b == SOF1) ps = P_H2;
  #if DEBUG_SAME_PORT
        else if (consoleAccepts()) consoleByte((char)b);   // byte ASCII di vao console
  #endif
        break;
      case P_H2: ps = (b == SOF2) ? P_LEN : (b == SOF1 ? P_H2 : P_H1); break;
      case P_LEN:
        if (b > MAX_PAYLOAD) { ps = P_H1; break; }
        rxLen = b; rxIdx = 0; ps = P_BODY;
        break;
      case P_BODY:                    // rxBuf[0] = TYPE, con lai = PAYLOAD
        rxBuf[rxIdx++] = b;
        if (rxIdx == rxLen + 1) ps = P_CRC;
        break;
      case P_CRC: {
        uint8_t c = crc8(&rxLen, 1, 0);
        c = crc8(rxBuf, rxLen + 1, c);
        if (c == b) {
          lastRxMs = millis();
          failsafe = false;
          hostSeen = true;
          lineLen = 0;
          handleFrame(rxBuf[0], &rxBuf[1], rxLen);
        }
        ps = P_H1;
        break;
      }
    }
  }

  // ===== Gui STATUS (chi khi co host) =====
  void sendStatus() {
    uint8_t f[10];
    uint8_t flags = (failsafe ? 1 : 0) | (reverseArmed ? 2 : 0) | (seq != SEQ_NONE ? 4 : 0);
    f[0] = SOF1; f[1] = SOF2; f[2] = 5; f[3] = MSG_STATUS;
    f[4] = (uint8_t)(curPWM & 0xFF); f[5] = (uint8_t)((curPWM >> 8) & 0xFF);
    f[6] = (uint8_t)(tgtPWM & 0xFF); f[7] = (uint8_t)((tgtPWM >> 8) & 0xFF);
    f[8] = flags;
    uint8_t c = crc8(&f[2], 1, 0);
    c = crc8(&f[3], 6, c);
    f[9] = c;
    LinkSerial.write(f, sizeof f);
  }

  // ===== Setup / Loop =====
void setup() {
  LinkSerial.begin(LINK_BAUD);
#if !DEBUG_SAME_PORT
  DbgPort.begin(115200);
#endif

  bool wdReset = IWatchdog.isReset(true);   // true = xoa co sau khi doc

  escBegin();
  setPWM(PWM_NEUTRAL);
  IWatchdog.begin(500000);                  // 500 ms, don vi us

  DbgPort.println();
  if (wdReset) DbgPort.println("[WDG] MCU vua bi reset boi watchdog!");
  DbgPort.println("ESC dang khoi tao (5 s). KHONG cham vao dong co.");

  uint32_t t0 = millis();
  while (millis() - t0 < ESC_ARM_DELAY_MS) {   // thay delay() de van nuoi watchdog
    IWatchdog.reload();
    delay(10);
  }
  setPWM(PWM_NEUTRAL);
  DbgPort.println("Khoi tao xong. Go 'h' de xem lenh.");

  lastRxMs = millis();
}
  void loop() {
    uint32_t now = millis();

    while (LinkSerial.available() > 0) parseByte((uint8_t)LinkSerial.read());
  #if !DEBUG_SAME_PORT
    while (DbgPort.available() > 0) consoleByte((char)DbgPort.read());
  #endif

    // Failsafe: chi ap dung khi dang co host Orange Pi. Mat lien lac -> dung ngay
    if (hostSeen && !failsafe && (now - lastRxMs) > FAILSAFE_TIMEOUT_MS) {
      failsafe = true;
      request(PWM_NEUTRAL, true);
      DBG_PRINTLN("[FAILSAFE] mat lien lac voi host -> trung tinh");
    }

    seqUpdate(now);
    rampUpdate(now);

    if (hostSeen && (now - lastStatusMs >= STATUS_PERIOD_MS)) {
      lastStatusMs = now;
      sendStatus();
    }
  IWatchdog.reload();   // chi nuoi o day: neu loop() ket thi MCU se tu reset
    // In trang thai khi co thay doi (toi da 4 lan/giay)
    static int lastCur = -1, lastTgt = -1;
    static Seq lastSeq = SEQ_NONE;
    if (verbose && dbgEnabled() && (now - lastVerboseMs >= VERBOSE_PERIOD_MS)) {
      if (curPWM != lastCur || tgtPWM != lastTgt || seq != lastSeq) {
        lastVerboseMs = now;
        lastCur = curPWM; lastTgt = tgtPWM; lastSeq = seq;
        printStatus();
      }
    }
  }
