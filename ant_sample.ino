/**
   注意:商用利用するときはAnt+の大元さんにお布施必要
*/
#include <SoftwareSerial.h>
#include <SakuraIO.h>
#include <time.h>
#include <TinyGPS++.h>

// PINアサイン
const int RTS_PIN        =  2;
const int ANT_TX_PIN     = 10;
const int ANT_RX_PIN     = 11;
const int GPS_TX_PIN     =  9;
const int GPS_RX_PIN     =  8;
const int CHAN_EST_OK    =  5;
const int SAKURA_EST_OK  =  7;

// URATレート
const int ANT_BAUD_RATE = 9600;
const int GPS_BAUD_RATE = 9600;
volatile int state = 1;

// 各センサ
SoftwareSerial antSerial(ANT_TX_PIN, ANT_RX_PIN);
SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN);
TinyGPSPlus gps;

int state_counter;
boolean clear_to_send = false;

// 速度計算用
double old_time = 0;
double old_rev = 0;
double old_sp = 0;
double old_hr = 0;
const int BIKE_CIR = 2096; // 自転車ホイール径(一般的な700cの場合)[cm]

// データ受信トライ回数
const int TRY_RECIEVE_ANT_CNT = 10;
const int TRY_RECIEVE_GPS_CNT = 3;

// チャネル設定用
int done_channnel_establish = 0;

// Ant+メッセージ構造体
typedef struct ant_message {
  byte msg_id;
  byte msg_length;
  byte data[100];
  byte chk_sum;
} ant_message_t;
ant_message_t* recieve_message;

int rate = 1;

// さくらのIoT
SakuraIO_I2C sakuraio;

/**
   RTS信号の立ち上がり検知
*/
void isr_ant() {
  state = 1;
}

/**
  1バイトデータ送信
  @return check_sum
*/
byte writeByte(byte sig, byte check_sum) {
  antSerial.write(sig);
  check_sum ^= sig;
  return check_sum;
}

/**
    メッセージを送信する
*/
void sendSignal(byte msg_id, byte argc, ...) {
  byte i;
  byte check_sum = 0x00;
  va_list list;
  va_start(list, argc);

  // 同期信号
  check_sum = writeByte((byte)0xA4, check_sum);
  // メッセージ長
  check_sum = writeByte(argc, check_sum);
  // 送信するメッセージID
  check_sum = writeByte(msg_id, check_sum);
  // メッセージ長だけデータを送信
  for (i = 0; i < argc; i++) {
    check_sum = writeByte((va_arg(list, unsigned int)), check_sum);
  }

  va_end(list);

  // チェックサム送信
  writeByte(check_sum, check_sum);
}

/**
   Ant+メッセージ構造のパケットを受信
*/
ant_message_t readPackets() {
  // 読み込んでいるパケットの位置を記憶(1バイトずつ処理を進める)
  ant_message_t r_message;

  int read_statement = 0;
  int data_num = 0;
  byte chk_sum = (byte)0xA4;
  byte signal;

  if (antSerial.available()) {
    // 先頭の信号が同期信号でなければ処理を中断
    signal = antSerial.read();
    if (signal != (byte)0xA4) {
      r_message.msg_length = 0;
      return r_message;
    }

    // 読めるだけ読む
    while (antSerial.available()) {
      signal = antSerial.read();
      //Serial.println(signal, HEX);
      if (read_statement == 0) {
        // メッセージ長
        r_message.msg_length = signal;
      } else if (read_statement == 1) {
        // メッセージID(タイプ)
        r_message.msg_id = signal;
      } else if (read_statement < r_message.msg_length + 2) {
        r_message.data[data_num++] = signal;
      } else {
        r_message.chk_sum = signal;
        // 自前で計算したチェックサムとAntモジュールから受け取ったそれが等しいか確認
        if (r_message.chk_sum == chk_sum) {
          return r_message;
        } else {
          r_message.msg_length  = 0;
          return r_message;
        }
      }
      //Serial.println(signal, HEX);
      read_statement++;
      chk_sum ^= signal;
    }
  }
  r_message.msg_length = 0;
  return r_message;
}

/**
   メッセージから心拍数を取り出す
*/
double getHeartRate(ant_message_t r_message) {
  // ブロードキャストデータ確認
  if (r_message.msg_id == (byte)0x4E) {
    // HRMに設定したチャンネルか確認(0x00)
    if (r_message.data[0] == (byte)0x00) {
      // 心拍数を取り出す(8byte目にbpm単位で心拍数が記録されている)
      return (double)r_message.data[8];
    }
  }
  return -1;
}

/**
   メッセージから速度[km/h]を取り出す
*/
double getSpeed(ant_message_t r_message) {
  double d_time = 0;
  double d_rev  = 0;
  double c_speed = 0;
  // ブロードキャストデータ確認
  if (r_message.msg_id == (byte)0x4E) {
    // HRMに設定したチャンネルか確認(0x01)
    if (r_message.data[0] == (byte)0x01) {
      // 測定時刻, 回転回数を取得
      d_time = (double)((r_message.data[6] << 8) + r_message.data[5]);
      d_rev  = (double)((r_message.data[8] << 8) + r_message.data[7]);
      // 速度の算出(算出式はAnt+公式リファレンス参照)
      if ((d_rev - old_rev) != 0 && (d_time - old_time) != 0) {
        c_speed = (BIKE_CIR * (d_rev - old_rev) * 1024) / ((d_time - old_time) * 1000);
        c_speed = c_speed * 3600 / 1000;
      }
      // 今回測定したデータを一時保存
      // TODO: 2byteしか保存できないので回転数とか時刻がすぐオーバオフローして0に戻るので考慮する
      old_time = d_time;
      old_rev  = d_rev;
      return c_speed;
    }
  }
  return -1;
}

void setup() {
  Serial.begin(9600);
  while (!Serial)
  {
    ;
  }
  // チャネル設定完了確認LED
  pinMode(CHAN_EST_OK, OUTPUT);
  pinMode(SAKURA_EST_OK, OUTPUT);
  // Ant+RTSシグナルピン
  pinMode(RTS_PIN, INPUT);

  // Ant+モジュール側のRTS信号の立ち上がりでArduinoからの送信が完了したか判断する(詳細はnrf24AP2モジュールのリファレンス参照)
  attachInterrupt(0, isr_ant, RISING);
  antSerial.begin(ANT_BAUD_RATE);
  gpsSerial.begin(GPS_BAUD_RATE);
  antSerial.listen();

  // さくらのIoTのモジュールと接続を確認
  while ((sakuraio.getConnectionStatus() & 0x80) != 0x80) {
    delay(1000);
  }
  // さくらのIoT接続完了　LED点灯処理追加
  digitalWrite(SAKURA_EST_OK, HIGH);
  Serial.println("Setup() Complete!");
}

void loop() {
  ant_message_t r_message;
  int i;
  double sp;
  double hr;
  bool isGetHr = false;
  bool isGetSpeed = false;

  if (done_channnel_establish == 1) {
    // センサ情報を取得する(10回トライ)
    for (i = 0; i < TRY_RECIEVE_ANT_CNT; i++) {
      r_message = readPackets();
      if (r_message.msg_length != 0) {
        if (isGetHr == false) {
          hr = getHeartRate(r_message);
          if (hr == -1) {
            hr = old_hr;
          } else {
            old_hr = hr;
            Serial.println(hr);
            sakuraio.enqueueTx(2, (float)hr);
            isGetHr = true;
          }
        }

        if (isGetSpeed == false) {
          sp = getSpeed(r_message);
          if (sp == -1) {
            sp = old_sp;
          } else {
            old_sp = sp;
            Serial.println(sp);
            //sakuraio.enqueueTx(1, (int32_t)sp);
            isGetSpeed = true;
          }
        }
      }
    }

    gpsSerial.listen();
    delay(100);

    int try_read = 0;
    int gps_signal_recieved = false;
    while (!gps_signal_recieved) {
      while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
          if (gps.location.isValid()) {
            sakuraio.enqueueTx(3, (float)(gps.location.lat()));
            sakuraio.enqueueTx(4, (float)(gps.location.lng()));
            gps_signal_recieved = true;
          } else {
            if (try_read == TRY_RECIEVE_GPS_CNT) {
              gps_signal_recieved = true;
            }
            try_read++;
          }
        }
      }
    }

    antSerial.listen();
    delay(100);

    // TODO:キューにデータが存在する場合のみsendするように修正
    uint8_t avail;
    uint8_t queued;
    sakuraio.getTxQueueLength(&avail, &queued);
    //Serial.println(queued);
    if (queued > 0) {
      sakuraio.send();
    }
    sakuraio.clearTx();

  }

  // チャンネル設立
  if (done_channnel_establish == 0) {
    if ((state_counter % 2) == 0) {
      delay(500);
    } else if (state_counter == 3) {
      Serial.println("RESET_SYSTEM()");
      sendSignal(0x4A, 1, 0x00);
      state = 0;
    } else if (state_counter == 5) {
      Serial.println("REQUEST( CAPS )");
      sendSignal(0x4D, 2, 0x00, 0x54);
      state = 0;
      //clear_to_send = false;
    } else if (state_counter == 7) {
      Serial.println("CHANNEL(ASSIGN)");
      sendSignal(0x42, 3, 0x00, 0x00, 0x00);
      state = 0;
    } else if (state_counter == 9) {
      Serial.println("CHANNEL ID(SET)");
      sendSignal(0x51, 5, 0x00, 0x00, 0x00, 0x78, 0x00);
      state = 0;
    } else if (state_counter == 11) {
      Serial.println("NET WORK KEY(SET)");
      sendSignal(0x46, 9, 0x00, 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45);
      state = 0;
    } else if (state_counter == 13) {
      Serial.println("SEARCH TIME OUT(SET)");
      sendSignal(0x44, 2, 0x00, 0x12);
      state = 0;
    } else if (state_counter == 15) {
      Serial.println("RF FREQUENCY(SET)");
      sendSignal(0x45, 2, 0x00, 0x39);
      state = 0;
    } else if (state_counter == 17) {
      Serial.println("Channel Period(SET)");
      sendSignal(0x43, 3, 0x00, (8070 & 0x00FF), ((8070 & 0xFF00) >> 8));
      state = 0;
    } else if (state_counter == 19) {
      Serial.println("OPEN CHANNEL");
      sendSignal(0x4B, 1, 0x00);
      state = 0;
    } else if (state_counter == 21) {
      Serial.println("CHANNEL(ASSIGN)");
      sendSignal(0x42, 3, 0x01, 0x00, 0x00);
      state = 0;
    } else if (state_counter == 23) {
      Serial.println("CHANNEL ID(SET)");
      sendSignal(0x51, 5, 0x01, 0x00, 0x00, 0x79, 0x00);
      state = 0;
    } else if (state_counter == 25) {
      Serial.println("NET WORK KEY(SET)");
      sendSignal(0x46, 9, 0x00, 0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45);
      state = 0;
    } else if (state_counter == 27) {
      Serial.println("SEARCH TIME OUT(SET)");
      sendSignal(0x44, 2, 0x01, 0x12);
      state = 0;
    } else if (state_counter == 29) {
      Serial.println("RF FREQUENCY(SET)");
      sendSignal(0x45, 2, 0x01, 0x39);
      state = 0;
    } else if (state_counter == 31) {
      Serial.println("Channel Period(SET)");
      sendSignal(0x43, 3, 0x01, (8086 & 0x00FF), ((8086 & 0xFF00) >> 8));
      state = 0;
    } else if (state_counter == 33) {
      Serial.println("OPEN CHANNEL");
      sendSignal(0x4B, 1, 0x01);
      state = 0;
      done_channnel_establish = 1;
      digitalWrite(CHAN_EST_OK, HIGH);
    }
    state_counter++;
  }
}




