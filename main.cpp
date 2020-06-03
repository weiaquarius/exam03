#include "mbed.h"
#include "mbed_rpc.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

#define UINT14_MAX        16383

// FXOS8700CQ I2C address
#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1

// FXOS8700CQ internal register addresses
#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7

// GLOBAL VARIABLES
WiFiInterface *wifi;
InterruptIn btn2(SW2);
InterruptIn btn3(SW3);
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;

const char* topic = "Velocity";

Thread mqtt_thread(osPriorityHigh);
EventQueue mqtt_queue;


RawSerial pc(USBTX, USBRX);
RawSerial xbee(D12, D11);
I2C i2c( PTD9,PTD8);

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;
int m_addr = FXOS8700CQ_SLAVE_ADDR1;

float velocity;

void FXOS8700CQ_readRegs(int, uint8_t*, int);
void FXOS8700CQ_writeRegs(uint8_t*, int);
void getAcc(Arguments*, Reply*);
void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);

RPCFunction rpcAcc(&getAcc, "getAcc");
void messageArrived(MQTT::MessageData& md) {
      MQTT::Message &message = md.message;
      char msg[300];
      sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
      printf(msg);
      wait_ms(1000);
      char payload[300];
      sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
      printf(payload);
      ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
      message_num++;
      MQTT::Message message;
      char buff[100];
      sprintf(buff, "QoS0 Hello, Python! #%d", message_num);
      message.qos = MQTT::QOS0;
      message.retained = false;
      message.dup = false;
      message.payload = (void*) buff;
      message.payloadlen = strlen(buff) + 1;
      int rc = client->publish(topic, message);

      printf("rc:  %d\r\n", rc);
      printf("Puslish message: %s\r\n", buff);
}

void close_mqtt() {
      closed = true;
}
int main(){
  // XBee setting
  char xbee_reply[4];
  xbee.baud(9600);
  xbee.printf("+++");
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();

  if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
    pc.printf("enter AT mode.\r\n");
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
  }

  xbee.printf("ATMY 0x258\r\n");
  reply_messange(xbee_reply, "setting MY : 0x258");

  xbee.printf("ATDL 0x158\r\n");
  reply_messange(xbee_reply, "setting DL : 0x158");

  xbee.printf("ATID 0x1\r\n");
  reply_messange(xbee_reply, "setting PAN ID : <0x1>");

  xbee.printf("ATWR\r\n");
  reply_messange(xbee_reply, "write config");

  xbee.printf("ATMY\r\n");
  check_addr(xbee_reply, "MY");

  xbee.printf("ATDL\r\n");
  check_addr(xbee_reply, "DL");

  xbee.printf("ATCN\r\n");
  reply_messange(xbee_reply, "exit AT mode");

  xbee.getc();

  // Enable the FXOS8700Q
  uint8_t data[2] ;
  FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
  data[1] |= 0x01;
  data[0] = FXOS8700Q_CTRL_REG1;
  FXOS8700CQ_writeRegs(data, 2);
  pc.baud(9600);

  // start
  pc.printf("start\r\n");
  t.start(callback(&queue, &EventQueue::dispatch_forever));

  // Setup a serial interrupt function of receiving data from xbee
  wifi = WiFiInterface::get_default_instance();
      if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
      }


      printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
      int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
      if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
      }


      NetworkInterface* net = wifi;
      MQTTNetwork mqttNetwork(net);
      MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

      //TODO: revise host to your ip
      const char* host = "192.168.1.113";
      printf("Connecting to TCP network...\r\n");
      int rc = mqttNetwork.connect(host, 1883);
      if (rc != 0) {
            printf("Connection error.");
            return -1;
      }
      printf("Successfully connected!\r\n");

      MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
      data.MQTTVersion = 3;
      data.clientID.cstring = "Velocity";

      if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
      }
      if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
      }

      mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
      btn2.rise(mqtt_queue.event(&publish_message, &client));
      btn3.rise(&close_mqtt);

      int num = 0;
      while (num != 5) {
            client.yield(100);
            ++num;
      }

      while (1) {
            if (closed) break;
            wait_us(500000);
      }

      printf("Ready to close MQTT Network......\n");

      if ((rc = client.unsubscribe(topic)) != 0) {
            printf("Failed: rc from unsubscribe was %d\n", rc);
      }
      if ((rc = client.disconnect()) != 0) {
      printf("Failed: rc from disconnect was %d\n", rc);
      }

      mqttNetwork.disconnect();
      printf("Successfully closed!\n");

      
  
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
  return 0;
}


void xbee_rx_interrupt(void){
  xbee.attach(NULL, Serial::RxIrq); // detach interrupt
  queue.call(&xbee_rx);
}

void xbee_rx(void){
  char buf[256] = {0};
  char outbuf[256] = {0};
  while(xbee.readable()){
    for (int i=0;; i++) {
      char recv = xbee.getc();
      if ( recv == '\r') {
        break;
      }
      buf[i] = pc.putc(recv);
    }
    RPC::call(buf, outbuf);
    pc.printf("%s\r\n", outbuf);
  }
  xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}

void reply_messange(char *xbee_reply, char *messange){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){
    pc.printf("%s\r\n", messange);
    xbee_reply[0] = '\0';
    xbee_reply[1] = '\0';
    xbee_reply[2] = '\0';
  }
}

void check_addr(char *xbee_reply, char *messenger){
  xbee_reply[0] = xbee.getc();
  xbee_reply[1] = xbee.getc();
  xbee_reply[2] = xbee.getc();
  xbee_reply[3] = xbee.getc();
  pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);

  xbee_reply[0] = '\0';
  xbee_reply[1] = '\0';
  xbee_reply[2] = '\0';
  xbee_reply[3] = '\0';
}

void getAcc(Arguments *in, Reply *out) {

   int16_t acc16;
   float t[3];
   uint8_t res[6];
   FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);

   acc16 = (res[0] << 6) | (res[1] >> 2);
   if (acc16 > UINT14_MAX/2)
      acc16 -= UINT14_MAX;
   t[0] = ((float)acc16) / 4096.0f;

   acc16 = (res[2] << 6) | (res[3] >> 2);
   if (acc16 > UINT14_MAX/2)
      acc16 -= UINT14_MAX;
   t[1] = ((float)acc16) / 4096.0f;

   acc16 = (res[4] << 6) | (res[5] >> 2);
   if (acc16 > UINT14_MAX/2)
      acc16 -= UINT14_MAX;
   t[2] = ((float)acc16) / 4096.0f;

  //  pc.printf("FXOS8700Q ACC: X=%1.4f(%x%x) Y=%1.4f(%x%x) Z=%1.4f(%x%x)",\
  //        t[0], res[0], res[1],\
  //        t[1], res[2], res[3],\
  //        t[2], res[4], res[5]\
  //  );
  velocity = t[0] * 0.1;
  pc.printf("Velocity = %1.4f\n",velocity);
}

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char t = addr;
   i2c.write(m_addr, &t, 1, true);
   i2c.read(m_addr, (char *)data, len);
}

void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}
// #include "mbed.h"
// #include "mbed_rpc.h"
// #include "fsl_port.h"
// #include "fsl_gpio.h"

// #define UINT14_MAX        16383

// // FXOS8700CQ I2C address
// #define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
// #define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
// #define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
// #define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1

// // FXOS8700CQ internal register addresses
// #define FXOS8700Q_STATUS 0x00
// #define FXOS8700Q_OUT_X_MSB 0x01
// #define FXOS8700Q_OUT_Y_MSB 0x03
// #define FXOS8700Q_OUT_Z_MSB 0x05
// #define FXOS8700Q_M_OUT_X_MSB 0x33
// #define FXOS8700Q_M_OUT_Y_MSB 0x35
// #define FXOS8700Q_M_OUT_Z_MSB 0x37
// #define FXOS8700Q_WHOAMI 0x0D
// #define FXOS8700Q_XYZ_DATA_CFG 0x0E
// #define FXOS8700Q_CTRL_REG1 0x2A
// #define FXOS8700Q_M_CTRL_REG1 0x5B
// #define FXOS8700Q_M_CTRL_REG2 0x5C
// #define FXOS8700Q_WHOAMI_VAL 0xC7

// RawSerial pc(USBTX, USBRX);
// RawSerial xbee(D12, D11);
// I2C i2c( PTD9,PTD8);

// EventQueue queue(32 * EVENTS_EVENT_SIZE);
// Thread t;
// int m_addr = FXOS8700CQ_SLAVE_ADDR1;

// float velocity;

// void FXOS8700CQ_readRegs(int, uint8_t*, int);
// void FXOS8700CQ_writeRegs(uint8_t*, int);
// void getAcc(Arguments*, Reply*);
// void xbee_rx_interrupt(void);
// void xbee_rx(void);
// void reply_messange(char *xbee_reply, char *messange);
// void check_addr(char *xbee_reply, char *messenger);

// RPCFunction rpcAcc(&getAcc, "getAcc");

// int main(){
//   // XBee setting
//   char xbee_reply[4];
//   xbee.baud(9600);
//   xbee.printf("+++");
//   xbee_reply[0] = xbee.getc();
//   xbee_reply[1] = xbee.getc();

//   if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K'){
//     pc.printf("enter AT mode.\r\n");
//     xbee_reply[0] = '\0';
//     xbee_reply[1] = '\0';
//   }

//   xbee.printf("ATMY 0x258\r\n");
//   reply_messange(xbee_reply, "setting MY : 0x258");

//   xbee.printf("ATDL 0x158\r\n");
//   reply_messange(xbee_reply, "setting DL : 0x158");

//   xbee.printf("ATID 0x1\r\n");
//   reply_messange(xbee_reply, "setting PAN ID : <0x1>");

//   xbee.printf("ATWR\r\n");
//   reply_messange(xbee_reply, "write config");

//   xbee.printf("ATMY\r\n");
//   check_addr(xbee_reply, "MY");

//   xbee.printf("ATDL\r\n");
//   check_addr(xbee_reply, "DL");

//   xbee.printf("ATCN\r\n");
//   reply_messange(xbee_reply, "exit AT mode");

//   xbee.getc();

//   // Enable the FXOS8700Q
//   uint8_t data[2] ;
//   FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
//   data[1] |= 0x01;
//   data[0] = FXOS8700Q_CTRL_REG1;
//   FXOS8700CQ_writeRegs(data, 2);
//   pc.baud(9600);

//   // start
//   pc.printf("start\r\n");
//   t.start(callback(&queue, &EventQueue::dispatch_forever));

//   // Setup a serial interrupt function of receiving data from xbee
//   xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
// }


// void xbee_rx_interrupt(void){
//   xbee.attach(NULL, Serial::RxIrq); // detach interrupt
//   queue.call(&xbee_rx);
// }

// void xbee_rx(void){
//   char buf[256] = {0};
//   char outbuf[256] = {0};
//   while(xbee.readable()){
//     for (int i=0;; i++) {
//       char recv = xbee.getc();
//       if ( recv == '\r') {
//         break;
//       }
//       buf[i] = pc.putc(recv);
//     }
//     RPC::call(buf, outbuf);
//     pc.printf("%s\r\n", outbuf);
//   }
//   xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
// }

// void reply_messange(char *xbee_reply, char *messange){
//   xbee_reply[0] = xbee.getc();
//   xbee_reply[1] = xbee.getc();
//   xbee_reply[2] = xbee.getc();
//   if(xbee_reply[1] == 'O' && xbee_reply[2] == 'K'){
//     pc.printf("%s\r\n", messange);
//     xbee_reply[0] = '\0';
//     xbee_reply[1] = '\0';
//     xbee_reply[2] = '\0';
//   }
// }

// void check_addr(char *xbee_reply, char *messenger){
//   xbee_reply[0] = xbee.getc();
//   xbee_reply[1] = xbee.getc();
//   xbee_reply[2] = xbee.getc();
//   xbee_reply[3] = xbee.getc();
//   pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);

//   xbee_reply[0] = '\0';
//   xbee_reply[1] = '\0';
//   xbee_reply[2] = '\0';
//   xbee_reply[3] = '\0';
// }

// void getAcc(Arguments *in, Reply *out) {

//    int16_t acc16;
//    float t[3];
//    uint8_t res[6];
//    FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);

//    acc16 = (res[0] << 6) | (res[1] >> 2);
//    if (acc16 > UINT14_MAX/2)
//       acc16 -= UINT14_MAX;
//    t[0] = ((float)acc16) / 4096.0f;

//    acc16 = (res[2] << 6) | (res[3] >> 2);
//    if (acc16 > UINT14_MAX/2)
//       acc16 -= UINT14_MAX;
//    t[1] = ((float)acc16) / 4096.0f;

//    acc16 = (res[4] << 6) | (res[5] >> 2);
//    if (acc16 > UINT14_MAX/2)
//       acc16 -= UINT14_MAX;
//    t[2] = ((float)acc16) / 4096.0f;

//   //  pc.printf("FXOS8700Q ACC: X=%1.4f(%x%x) Y=%1.4f(%x%x) Z=%1.4f(%x%x)",\
//   //        t[0], res[0], res[1],\
//   //        t[1], res[2], res[3],\
//   //        t[2], res[4], res[5]\
//   //  );
//   velocity = t[0] * 0.1;
//   pc.printf("velocity: %1.4f\n",velocity);
// }

// void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
//    char t = addr;
//    i2c.write(m_addr, &t, 1, true);
//    i2c.read(m_addr, (char *)data, len);
// }

// void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
//    i2c.write(m_addr, (char *)data, len);
// }