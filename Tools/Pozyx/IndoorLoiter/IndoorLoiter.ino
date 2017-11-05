
/*
    Indoor Positioning System for ArduPilot
    Send current vehicle position to arducopter via Mavlink protocols
    Receive IPS calculated via RF CC1101 UART protocol 
    SNSH Team - 2017
*/

#include <Pozyx.h>
#include <Pozyx_definitions.h>

#include <SoftwareSerial.h>
#include <Wire.h>

#define NUM_ANCHORS 4
#define MAX_RX_BUFF 255
#define FRAME_START (0xab | 0xAB)

uint16_t anchor_id[5] = { 0x601C, // (0,0)
                          0x6020, // x-axis
                          0x6057, // y-axis
                          0x605E,
                          0x6045};     

int32_t anchors_x[NUM_ANCHORS] = {0,     10000, 0,     10000};    // anchor x-coorindates in mm (horizontal)
int32_t anchors_y[NUM_ANCHORS] = {0,     0,     10000, 10000};    // anchor y-coordinates in mm (vertical)
int32_t heights[NUM_ANCHORS] =   {-1200, -1200, -1200, -1200};    // anchor z-coordinates in mm (1.2m above vehicle's starting altitude)

SoftwareSerial fcboardSerial(10, 11); // rx, tx

#define MSG_HEADER          0x01
#define MSGID_BEACON_CONFIG 0x02
#define MSGID_BEACON_DIST   0x03
#define MSGID_POSITION      0x04

// structure for messages uploaded to ardupilot
union beacon_config_msg {
    struct {
        uint8_t beacon_id;
        uint8_t beacon_count;
        int32_t x;
        int32_t y;
        int32_t z;
    } info;
    uint8_t buf[14];
};
union beacon_distance_msg {
    struct {
        uint8_t beacon_id;
        uint32_t distance;
    } info;
    uint8_t buf[5];
};
union vehicle_position_msg {
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
        int16_t position_error;
    } info;
    uint8_t buf[14];
};
////////////////////////////////////////////////
//Current Vehicle Postion
int32_t pos_x = 0, pos_y = 0, pos_z = 0;
//int32_t distance_t/o_beacon[NUM_ANCHORS] = {0,  0,  0,  0,  0};
int16_t pos_error = 0;

char rx_Buff[MAX_RX_BUFF];
int16_t rx_cnt = 0;
boolean frameComplete = false; 

void setup()
{
    Serial.begin(115200);
    fcboardSerial.begin(115200);
}

void loop()
{
    // slow down counter
    static uint32_t counter = 0;
    counter++;
    if (counter >= 10000) {
        counter = 0;
    }

    // during stage 0 (init) send position and beacon config as quickly as possible
    // during stage 1 send about every 2 seconds
    if (counter == 0) {
        send_beacon_config();
        get_position();
        get_ranges();
    }

    if (frameComplete)
        parse_frame();
}

void print_tab()
{  
    Serial.print("\t");
}

// print coordinates to the serial monitor
void print_coordinates(coordinates_t coor, pos_error_t pos_error)
{  
    Serial.print("Pos x:");
    Serial.print(coor.x);
    print_tab();
    Serial.print("y:");
    Serial.print(coor.y);
    print_tab();
    Serial.print("z:");
    Serial.print(coor.z);
}

// get ranges for each anchor
void get_ranges()
{
    // get range for each anchor
    device_range_t range;
    for (uint8_t i=0; i<NUM_ANCHORS; i++) {
        // send info to ardupilot
        // hacking arduipilot
        send_beacon_distance(i, sqrt((pos_x-anchors_x[i])^2 + (pos_y-anchors_y[i])^2 + (pos_z-heights[i])^2 ));
    }
}

// get position of tag
void get_position()
{
    coordinates_t position;
    pos_error_t _pos_error;

    position.x = pos_x;
    position.y = pos_y;
    position.z = pos_z;
    _pos_error.xy = pos_error;

    print_coordinates(position, _pos_error);
    send_vehicle_position(position, _pos_error);
}

// send all beacon config to ardupilot
void send_beacon_config()
{
    beacon_config_msg msg;
    msg.info.beacon_count = NUM_ANCHORS;
    for (uint8_t i=0; i<NUM_ANCHORS; i++) {
        msg.info.beacon_id = i;
        msg.info.x = anchors_x[i];
        msg.info.y = anchors_y[i];
        msg.info.z = heights[i];
        send_message(MSGID_BEACON_CONFIG, sizeof(msg.buf), msg.buf);
    }
    Serial.println("Sent anchor info");
}

// send a beacon's distance to ardupilot
void send_beacon_distance(uint8_t beacon_id, uint32_t distance_mm)
{
    beacon_distance_msg msg;
    msg.info.beacon_id = beacon_id;
    msg.info.distance = distance_mm;
    send_message(MSGID_BEACON_DIST, sizeof(msg.buf), msg.buf);
}

// send vehicle's position to ardupilot
void send_vehicle_position(coordinates_t& position, pos_error_t& pos_error)
{
    vehicle_position_msg msg;

    // sanity check position
    if (position.x == 0 || position.y == 0) {
        return;
    }

    msg.info.x = position.x;
    msg.info.y = position.y;
    msg.info.z = position.z;
    // msg.info.z = 0;
    msg.info.position_error = pos_error.xy;
    send_message(MSGID_POSITION, sizeof(msg.buf), msg.buf);
}

void send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[])
{
    // sanity check
    if (data_len == 0) {
        return;
    }
    // message is buffer length + 1 (for checksum)
    uint8_t msg_len = data_len+1;

    // calculate checksum and place in last element of array
    uint8_t checksum = 0;
    checksum ^= msg_id;
    checksum ^= msg_len;
    for (uint8_t i=0; i<data_len; i++) {
        checksum = checksum ^ data_buf[i];
    }

    // send message
    int16_t num_sent = 0;
    num_sent += fcboardSerial.write(MSG_HEADER);
    num_sent += fcboardSerial.write(msg_id);
    num_sent += fcboardSerial.write(msg_len);
    num_sent += fcboardSerial.write(data_buf, data_len);
    num_sent += fcboardSerial.write(&checksum, 1);
    fcboardSerial.flush();
}


void parse_frame(){
  
    int8_t checksum = 0;
    Serial.println("Process frame");
    if (rx_Buff[0] == 0xab | 0xAB) { //match frame
      Serial.println("START FRAME");
        //Check SUM
//        for(unsigned int i = 2; i < 10; i++)
//        {
//            int8_t n = rx_Buff[i];
//            while(n) {
//                checksum += n % 2;
//                n >>= 1;
//            }
//        }
//        if (checksum == rx_Buff[10]) {
            //2 bytes Pos X, 2 bytes Pos Y, 2 byte Pos Z, 2 bytes covariance XY
            pos_x = (int16_t) (rx_Buff[2] << 8) | rx_Buff[3];
            pos_y = (int16_t) (rx_Buff[4] << 8) | rx_Buff[5];
            pos_z = (int16_t) (rx_Buff[6] << 8) | rx_Buff[7];
            pos_error = (int16_t) (rx_Buff[8] << 8) | rx_Buff[9];
//        }/
    }

    frameComplete = false;
}

//PARSE Data Postion from CC1101
void serialEvent() {
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      rx_Buff[rx_cnt++] = inChar;
      if (inChar == '\n') {
        frameComplete = true;
        rx_cnt = 0;
      }
    }
  }
