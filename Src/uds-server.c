/*
 * Instrument cluster simulator
 *
 * (c) 2014 Open Garages - Craig Smith <craig@theialabs.com>
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <time.h>
#include <sys/stat.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "uds-server.h"

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_can.h"

#define DEBUG 0
//#define VIN "1G1ZT53826F109149"
//#define VIN "5YJSA1S2FEFA00001"
#define VIN "WAUZZZ8V9FA149850"
//#define VIN "2B3KA43R86H389824"
#define DATA_ALPHA     0
#define DATA_ALPHANUM  1
#define DATA_BINARY    2


/* Globals */
int running = 0;
int verbose = 0;
int no_flow_control = 0;
int keep_spec = 0;
FILE *plogfp = NULL;
char *vin = VIN;
int pending_data;
struct can_frame gm_data_by_id;
uint32_t gm_lastcms = 0;

/* This is for flow control packets */
char gBuffer[255];
int gBufSize;
int gBufLengthRemaining;
int gBufCounter;

extern CAN_HandleTypeDef hcan;

#define sleep(s) {HAL_Delay(s*1000);}

/* Prototypes */
void print_pkt(struct canfd_frame);
void print_bin(unsigned char *, int);

int write_can_frame(struct canfd_frame *frame, int bytes)
{
  uint32_t mb = 0;
  CAN_TxHeaderTypeDef header;
  header.DLC = frame->len;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.ExtId = frame->can_id;
  if (HAL_CAN_AddTxMessage(&hcan, &header, frame->data, &mb) != HAL_OK)
    return -1;
  return bytes;
}


// Simple function to print logging info to screen or to a file
/*void plog(char *fmt, ...) {
  va_list args;
  char buf[2046];
  int len;

  va_start(args, fmt);
  len = vsnprintf(buf, 2045, fmt, args);
  va_end(args);

  if(plogfp) {
    len = fwrite_can_frame(buf, 1, len, plogfp);
  } else {
    printf("%s", buf);
  }
}*/

#define plog printf

void intHandler(int sig) {
  running = 0;
}

// Generates data into a buff and returns it.
uint8_t *gen_data(int scope, int size)
{
  char *charset, *buf;
  unsigned char byte;
  int num;
  int i;
  buf = malloc(size);
  memset(buf,0,size);
  switch(scope) {
    case DATA_ALPHA:
      charset = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
      for(i = 0; i < size; i++) {
        buf[i] = charset[rand() % strlen(charset)];
      }
      break;
    case DATA_ALPHANUM:
      charset = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
      for(i = 0; i < size; i++) {
        num = rand() % strlen(charset);
        byte = charset[num];
        if (DEBUG) printf("DEBUG: random byte[%d] = %d %02X\n", i, num, byte);
        buf[i] = byte;
      }
      break;
    case DATA_BINARY:
      for(i = 0; i < size; i++) {
        buf[i] = rand() % 256;
      }
    default:
      break;
  }
  return (uint8_t*)buf;
}

// If a flow control packet comes in, push out more data
// This isn't fully supported, just a hack at the moment
void flow_control_push_to(int id)
{
  struct canfd_frame frame;
  int nbytes;
  if(no_flow_control) return;
  if(verbose) plog("FC: Flushing ISOTP buffers\n");
  frame.can_id = id;
  while(gBufLengthRemaining > 0) {
    if(gBufLengthRemaining > 7) {
      frame.len = 8;
      frame.data[0] = gBufCounter;
      memcpy(&frame.data[1], gBuffer+(gBufSize-gBufLengthRemaining), 7);
      nbytes = write_can_frame(&frame, CAN_MTU);
      if(nbytes < 0) perror("Write packet (FC)");
      gBufCounter++;
      gBufLengthRemaining -= 7;
    } else {
      frame.len = gBufLengthRemaining + 1;
      frame.data[0] = gBufCounter;
      memcpy(&frame.data[1], gBuffer+(gBufSize-gBufLengthRemaining), CAN_MTU);
      nbytes = write_can_frame(&frame, CAN_MTU);
      if(nbytes < 0) perror("Write packet (FC Final)");
      gBufLengthRemaining = 0;
    }
  }
}

void flow_control_push()
{
  flow_control_push_to(0x7e8);
}

void isotp_send_to(uint8_t *data, int size, int dest)
{
  struct canfd_frame frame;
  int left = size;
  int counter;
  int nbytes;
  if(size > 256) return;
  frame.can_id = dest;
  if(size < 7) {
    frame.len = size + 1;
    frame.data[0] = size;
    memcpy(&frame.data[1], data, size);
    nbytes = write_can_frame(&frame, CAN_MTU);
    if(nbytes < 0) perror("Write packet");
  } else {
    frame.len = 8;
    frame.data[0] = 0x10;
    frame.data[1] = (char)size-1;
    memcpy(&frame.data[2], data, 6);
    nbytes = write_can_frame(&frame, CAN_MTU);
    if(nbytes < 0) perror("Write packet");
    left -= 6;
    counter = 0x21;
    if(no_flow_control) {
      while(left > 0) {
        if(left > 7) {
          frame.len = 8;
          frame.data[0] = counter;
          memcpy(&frame.data[1], data+(size-left), 7);
          write_can_frame(&frame, CAN_MTU);
          counter++;
          left -= 7;
        } else {
          frame.len = left + 1;
          frame.data[0] = counter;
          memcpy(&frame.data[1], data+(size-left), CAN_MTU);
          write_can_frame(&frame, frame.len);
          left = 0;
        }
      }
    } else { // FC
      memcpy(gBuffer, data, size); // Size is restricted to <256
      gBufSize = size;
      gBufLengthRemaining = left;
      gBufCounter = counter;
    }
  }
}

void isotp_send(uint8_t *data, int size)
{
  isotp_send_to(data, size, 0x7e8);
}

/*
 * Some UDS queries requiest periodic data.  This handles those
 */
void handle_pending_data()
{
  struct canfd_frame frame;
  uint32_t currcms; //
  int i, offset, datacnt;
  if(!pending_data) return;

  currcms = HAL_GetTick();

  if(IS_SET(pending_data, PENDING_READ_DATA_BY_ID_GM)) {
    if(gm_data_by_id.data[0] == 0xFE) {
      offset = 1;
    } else {
      offset = 0;
    }
    frame.can_id = gm_data_by_id.can_id;
    frame.len = 8;
    switch(gm_data_by_id.data[2 + offset]) { // Subfunctions
      case 0x02:  // Slow Rate - every 10 sec
        if (currcms - gm_lastcms > 10000) {
          for(i=3; i < gm_data_by_id.data[0]+1; i++) {
            frame.data[0] = gm_data_by_id.data[i];
            for(datacnt=1; datacnt < 8; datacnt++) {
              frame.data[datacnt] = rand() % 255;
            }
            write_can_frame(&frame, CAN_MTU);
            if(verbose > 1) plog("  + Sending GM data (%02X) at a slow rate\n", frame.data[0]);
          }
          gm_lastcms = currcms;
        }
        break;
      case 0x03:  // Medium Rate - every sec
        if (currcms - gm_lastcms > 1000) {
          for(i=3; i < gm_data_by_id.data[0]+1; i++) {
            frame.data[0] = gm_data_by_id.data[i];
            for(datacnt=1; datacnt < 8; datacnt++) {
              frame.data[datacnt] = rand() % 255;
            }
            write_can_frame(&frame, CAN_MTU);
            if(verbose > 1) plog("  + Sending GM data (%02X) at a medium rate\n", frame.data[0]);
          }
          gm_lastcms = currcms;
        }
        break;
      case 0x04:  // Fast Rate - every 50 ms
        if (currcms - gm_lastcms > 50) {
          for(i=3; i < gm_data_by_id.data[0]+1; i++) {
            frame.data[0] = gm_data_by_id.data[i];
            for(datacnt=1; datacnt < 8; datacnt++) {
              frame.data[datacnt] = rand() % 255;
            }
            write_can_frame(&frame, CAN_MTU);
            if(verbose > 1) plog("  + Sending GM data (%02X) at a fast rate\n", frame.data[0]);
          }
          gm_lastcms = currcms;
        }
        break;
      default:
        plog("Unknown subfunction timer\n");
        break;
    }
  } // IS_SET PENDING_READ_DATA_BY_ID_GM
}

void send_dtcs(char total, struct canfd_frame frame)
{
  unsigned char resp[1024];
  char i;
  memset(resp, 0, 1024);
  resp[0] = frame.data[1] + 0x40;
  resp[1] = total; // Total DTCs
  for(i = 0; i <= total*2; i+=2) {
    resp[2+i] = 1;
    resp[2+i+1] = i;
  }
  if(total == 0) {
    isotp_send(resp, 2);
  } else if (total < 3) {
    isotp_send(resp, 2+(total*2));
  } else {
    isotp_send(resp, total*2);
  }
}

unsigned char calc_vin_checksum(char *vin, int size)
{
  char w[17] = { 8, 7, 6, 5, 4, 3, 2, 10, 0, 9, 8, 7, 6, 5, 4, 3, 2 };
  int i;
  int checksum = 0;
  int num = 0;
  for(i=0; i < size; i++) {
    if(vin[i] == 'I' || vin[i] == 'O' || vin[i] == 'Q') {
      num = 0;
    } else {
      if(vin[i] >= '0' && vin[i] <='9') num = vin[i] - '0';
      if(vin[i] >= 'A' && vin[i] <='I') num = (vin[i] - 'A') + 1;
      if(vin[i] >= 'J' && vin[i] <='R') num = (vin[i] - 'J') + 1;
      if(vin[i] >= 'S' && vin[i] <='Z') num = (vin[i] - 'S') + 2;
    }
    checksum += num * w[i];
  }
  checksum = checksum % 11;
  if (checksum == 10) return 'X';
  return ('0' + checksum);
}

void send_error_snfs(struct canfd_frame frame)
{
  uint8_t resp[4];
  if(verbose) plog("Responded with Sub Function Not Supported\n");
  resp[0] = 0x7f;
  resp[1] = frame.data[1];
  resp[2] = 12; // SubFunctionNotSupported
  isotp_send(resp, 3);
}

void send_error_roor(struct canfd_frame frame, int id)
{
  uint8_t resp[4];
  if(verbose) plog("Responded with Sub Function Not Supported\n");
  resp[0] = 0x7f;
  resp[1] = frame.data[1];
  resp[2] = 31; // RequestOutOfRange
  isotp_send_to(resp, 3, id);
}

void generic_OK_resp(struct canfd_frame frame)
{
  uint8_t resp[4];
  if(verbose > 1) plog("Responding with a generic OK message\n");
  resp[0] = frame.data[1] + 0x40;
  resp[1] = frame.data[2];
  resp[2] = 0;
  isotp_send(resp, 3);
}

void generic_OK_resp_to(struct canfd_frame frame, int id)
{
  uint8_t resp[4];
  if(verbose > 1) plog("Responding with a generic OK message\n");
  resp[0] = frame.data[1] + 0x40;
  resp[1] = frame.data[2];
  resp[2] = 0;
  isotp_send_to(resp, 3, id);
}

void handle_current_data(struct canfd_frame frame)
{
  if(verbose) plog("Received Current info request\n");
  uint8_t resp[8];
  switch(frame.data[2]) {
    case 0x00: // Supported PIDs
      if(verbose) plog("Responding with a generic set of PIDs (1-20)\n");
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0xBF;
      resp[3] = 0xBF;
      resp[4] = 0xB9;
      resp[5] = 0x93;
      isotp_send(resp, 6);
      break;
    case 0x01: // MIL & DTC Status
      if(verbose) plog("Responding to MIL and DTC Status request\n");
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0x00;
      resp[3] = 0x07;
      resp[4] = 0xE5;
      resp[5] = 0xE5;
      isotp_send(resp, 6);
      break;
    case 0x20: // More supported PIDs (21-40)
      if(verbose) plog("Responding with PIDs supported (21-40)\n");
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0xBF;
      resp[3] = 0xBF;
      resp[4] = 0xB9;
      resp[5] = 0x93;
      isotp_send(resp, 6);
      break;
    case 0x40: // More supported PIDs (41-60)
      if(verbose) plog("Responding with PIDs supported (41-60)\n");
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0xBF;
      resp[3] = 0xBF;
      resp[4] = 0xB9;
      resp[5] = 0x93;
      isotp_send(resp, 6);
      break;
    case 0x41: // Monitor status this drive cycle
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0;
      resp[3] = 0x0F;
      resp[4] = 0xFF;
      resp[5] = 0x00;
      isotp_send(resp, 6);
      break;
    case 0x60: // More supported PIDs (61-80)
      if(verbose) plog("Responding with PIDs supported (61-80)\n");
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0xBF;
      resp[3] = 0xBF;
      resp[4] = 0xB9;
      resp[5] = 0x93;
      isotp_send(resp, 6);
      break;
    case 0x80: // More supported PIDs (81-100)
      if(verbose) plog("Responding with PIDs supported (81-100)\n");
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0xBF;
      resp[3] = 0xBF;
      resp[4] = 0xB9;
      resp[5] = 0x93;
      isotp_send(resp, 6);
      break;
    case 0xA0:  // More Supported PIDs (101-120)
      if(verbose) plog("Responding with PIDs supported (101-120)\n");
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0xBF;
      resp[3] = 0xBF;
      resp[4] = 0xB9;
      resp[5] = 0x93;
      isotp_send(resp, 6);
      break;
    case 0xC0: // More supported PIDs (121-140)
      if(verbose) plog("Responding with PIDs supported (121-140)\n");
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0xBF;
      resp[3] = 0xBF;
      resp[4] = 0xB9;
      resp[5] = 0x93;
      isotp_send(resp, 6);
      break;
    default:
      if(verbose) plog("Note: Requested unsupported service %02X\n", frame.data[2]);
      break;
  }
}

void handle_vehicle_info(struct canfd_frame frame)
{
  if(verbose) plog("Received Vehicle info request\n");
  uint8_t resp[300];
  switch(frame.data[2]) {
    case 0x00: // Supported PIDs
      if(verbose) plog("Replying with ALL Pids supported\n");
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0x55;
      resp[3] = 0;
      resp[4] = 0;
      resp[5] = 0;
      isotp_send(resp, 6);
      break;
    case 0x02: // Get VIN
      if(verbose) plog("Sending VIN %s\n", vin);
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 1;
      memcpy(&resp[3], vin, strlen(vin));
      isotp_send(resp, 4 + strlen(vin));
      break;
    default:
      break;
  }
}

void handle_pending_codes(struct canfd_frame frame)
{
  if(verbose) plog("Received request for pending trouble codes\n");
  send_dtcs(20, frame);
}

void handle_stored_codes(struct canfd_frame frame)
{
  if(verbose) plog("Received request for stored trouble codes\n");
  send_dtcs(2, frame);
}

// TODO: This is wrong.  Record a real transaction to see the format
void handle_freeze_frame(struct canfd_frame frame)
{
  if(verbose) plog("Received request for freeze frame code\n");
  //send_dtcs(1, frame);
  uint8_t resp[4];
  resp[0] = frame.data[1] + 0x40;
  resp[1] = 0x01;
  resp[2] = 0x01;
  isotp_send(resp, 3);
}

void handle_perm_codes(struct canfd_frame frame)
{
  if(verbose) plog("Received request for permanent trouble codes\n");
  send_dtcs(0, frame);
}

void handle_dsc(struct canfd_frame frame)
{
  //if(verbose) plog("Received DSC Request\n");
  //send_error_snfs(frame);
  if(verbose) plog("Received DSC Request giving VCDS respose\n");
  frame.can_id = 0x77A;
  frame.len = 8;
  frame.data[0] = 0x06;
  frame.data[1] = 0x50;
  frame.data[2] = 0x03;
  frame.data[3] = 0x00;
  frame.data[4] = 0x32;
  frame.data[5] = 0x01;
  frame.data[6] = 0xF4;
  frame.data[7] = 0xAA;
  write_can_frame(&frame, CAN_MTU);
}

/*
  ECU Memory, based on VCDS response for now
 */
void handle_read_data_by_id(struct canfd_frame frame)
{
  if(verbose) plog("Recieved Read Data by ID %02X %02X\n", frame.data[2], frame.data[3]);
  uint8_t resp[120];
  if (frame.data[2] == 0xF1) {
    switch(frame.data[3]) {
      case 0x87:
        if(verbose) plog("Read data by ID 0x87\n");
        resp[0] = frame.data[1] + 0x40;
        resp[1] = frame.data[2];
        resp[2] = 0x87;
        resp[3] = 0x30;
        resp[4] = 0x34;
        resp[5] = 0x45;
        resp[6] = 0x39;
        resp[7] = 0x30;
        resp[8] = 0x36;
        resp[9] = 0x33;
        resp[10] = 0x32;
        resp[11] = 0x33;
        resp[12] = 0x46;
        resp[13] = 0x20; // Note VCDS pads with 55's
        isotp_send_to(resp, 14, 0x77A);
        break;
      case 0x89:
        if(verbose) plog("Read data by ID 0x89\n");
        frame.can_id = 0x7E8;
        frame.len = 8;
        frame.data[0] = 0x07;
        frame.data[1] = 0x62;
        frame.data[2] = 0xF1;
        frame.data[3] = 0x89;
        frame.data[4] = 0x38; //8
        frame.data[5] = 0x34; //4
        frame.data[6] = 0x31; //1
        frame.data[7] = 0x30; //0
        write_can_frame(&frame, CAN_MTU);
        break;
      case 0x9E:
        if(verbose) plog("Read data by ID 0x9E\n");
        resp[0] = frame.data[1] + 0x40;
        resp[1] = frame.data[2];
        resp[2] = 0x45;
        resp[3] = 0x56;
        resp[4] = 0x5F;
        resp[5] = 0x47;
        resp[6] = 0x61;
        resp[7] = 0x74;
        resp[8] = 0x65;
        resp[9] = 0x77;
        resp[10] = 0x45;
        resp[11] = 0x56;
        resp[12] = 0x43;
        resp[13] = 0x6F;
        resp[14] = 0x6E;
        resp[15] = 0x74;
        resp[16] = 0x69;
        resp[17] = 0x00;
        isotp_send(resp, 0x13);
        break;
      case 0xA2:
        if(verbose) plog("Read data by ID 0xA2\n");
        resp[0] = frame.data[1] + 0x40;
        resp[1] = frame.data[2];
        resp[2] = 0xA2;
        resp[3] = 0x30; // 004010
        resp[4] = 0x30;
        resp[5] = 0x34;
        resp[6] = 0x30;
        resp[7] = 0x31;
        resp[8] = 0x30;
        isotp_send(resp, 9);
        break;
      default:
        if(verbose) plog("Not responding to ID %02X\n", frame.data[3]);
        break;
    }
  } else if(frame.data[2] == 0x06) {
    switch(frame.data[3]) {
      case 0x00:
        if(verbose) plog("Read data by ID 0x9E\n");
        resp[0] = frame.data[1] + 0x40;
        resp[1] = frame.data[2];
        resp[2] = 0x02;
        resp[3] = 0x01;
        resp[4] = 0x00;
        resp[5] = 0x17;
        resp[6] = 0x26;
        resp[7] = 0xF2;
        resp[8] = 0x00;
        resp[9] = 0x00;
        resp[10] = 0x5B;
        resp[11] = 0x00;
        resp[12] = 0x12;
        resp[13] = 0x08;
        resp[14] = 0x58;
        resp[15] = 0x00;
        resp[16] = 0x00;
        resp[17] = 0x00;
        resp[18] = 0x00;
        resp[19] = 0x01;
        resp[20] = 0x01;
        resp[21] = 0x01;
        resp[22] = 0x00;
        resp[23] = 0x01;
        resp[24] = 0x00;
        resp[25] = 0x00;
        resp[26] = 0x00;
        resp[27] = 0x00;
        resp[28] = 0x00;
        resp[29] = 0x00;
        resp[30] = 0x00;
        resp[31] = 0x00;
        isotp_send(resp, 0x21);
        break;
      case 0x01:
        if(verbose) plog("Read data by ID 0x01\n");
        send_error_roor(frame, 0x7E8);
        break;
      default:
        if(verbose) plog("Not responding to ID %02X\n", frame.data[3]);
        break;
    }
  } else {
    if(verbose) plog("Unknown read data by ID %02X\n", frame.data[2]);
  }
}

/*
 GM
 */

// Read DID from ID (GM)
// For now we are only setting this up to work with the BCM
// 244   [3]  02 1A 90
void handle_gm_read_did_by_id(struct canfd_frame frame)
{
  if(verbose) plog("Received GM Read DID by ID Request\n");
  uint8_t resp[300];
  char *tracenum = "874602RA51950204";
  switch(frame.data[2]) {
    case 0x90:  // VIN
      if(verbose) plog(" + Requested VIN\n");
      if(verbose) plog("Sending VIN %s\n", vin);
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      memcpy(&resp[2], vin, strlen(vin));
      isotp_send_to(resp, 3 + strlen(vin), 0x644);
      break;
    case 0xA1:  // SDM Primary Key
      if(verbose) plog(" + Requested SDM Primary Key\n");
      if(verbose) plog("Sending SDM Key %04X\n", 0x6966);
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0x69;
      resp[3] = 0x66;
      isotp_send_to(resp, 5, 0x644);
      break;
    case 0xB4:  // Traceability Number
      if(verbose) plog(" + Requested Traceability Number\n");
      if(verbose) plog("Sending Traceabiliity number %s\n", tracenum);
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      memcpy(&resp[2], tracenum, strlen(tracenum));
      isotp_send_to(resp, 3 + strlen(tracenum), 0x644);
      break;
    case 0xB7:  // Software Number
      if(verbose) plog(" + Requested Software Number\n");
      if(verbose) plog("Sending SW # %d\n", 600);
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0x42;
      resp[3] = 0xAA;
      resp[4] = 6;
      resp[5] = 2; // 600
      resp[6] = 0x58;
      isotp_send_to(resp, 6, 0x644);
      break;
    case 0xCB:  // End Model Part #
      if(verbose) plog(" + Requested End Model Part Number\n");
      if(verbose) plog("Sending End Model Part Number %d\n", 15804602);
      resp[0] = frame.data[1] + 0x40;
      resp[1] = frame.data[2];
      resp[2] = 0x00;
      resp[3] = 0xF1;
      resp[4] = 0x28;
      resp[5] = 0xBA;
      isotp_send_to(resp, 6, 0x644);
      break;
    default:
      break;
  }
}

/* GM Read Data via PID */
/* 244   [5]  04 AA 03 02 07 */
/* 544#0738408D8B000200 */
/* 544#02508D8D00000000 */
void handle_gm_read_data_by_id(struct canfd_frame frame)
{
  if(verbose) plog("Received GM Read Data by ID Request\n");
  int offset = 0;
  int i;
  int datacnt;
  char datacpy[8];
  if (frame.data[0] == 0xFE) offset = 1;
  memcpy(&datacpy, &frame.data, 8);
  if(frame.can_id == 0x7e0) {
    frame.can_id = 0x5e8;
  } else {
    frame.can_id = 0x500 + (frame.can_id & 0xFF);
  }
  frame.len = 8;
  switch(frame.data[2 + offset]) { // Subfunctions
    case 0x00:  // Stop
      if(verbose) plog(" + Stop Data Request\n");
      memset(frame.data, 0, 8);
      write_can_frame(&frame, CAN_MTU);
      CLEAR_BIT(pending_data, PENDING_READ_DATA_BY_ID_GM);
      break;
    case 0x01:  // One Response
      if(verbose) plog(" + One Response\n");
      for(i=3; i < datacpy[0]+1; i++) {
        frame.data[0] = datacpy[i];
        for(datacnt=1; datacnt < 8; datacnt++) {
          frame.data[datacnt] = rand() % 256;
        }
        write_can_frame(&frame, CAN_MTU);
        sleep(0.5);
      }
      break;
    case 0x02:  // Slow Rate
      if(verbose) plog(" + Slow Rate\n");
      SET_BIT(pending_data, PENDING_READ_DATA_BY_ID_GM);
      memcpy(&gm_data_by_id, &frame, sizeof(frame));
      break;
    case 0x03:  // Medium Rate
      if(verbose) plog(" + Medium Rate\n");
      SET_BIT(pending_data, PENDING_READ_DATA_BY_ID_GM);
      memcpy(&gm_data_by_id, &frame, sizeof(frame));
      break;
    case 0x04:  // Fast Rate
      if(verbose) plog(" + Fast Rate\n");
      SET_BIT(pending_data, PENDING_READ_DATA_BY_ID_GM);
      memcpy(&gm_data_by_id, &frame, sizeof(frame));
      break;
    default:
      plog("Unknown subfunction timer\n");
      break;
  }
}

/* GM Diag format is either
     101#FE 03 A9 81 52  (Functional addressing: Where FE is the extended address)
     7E0#03 A9 81 52 (no extended addressing)
 */
void handle_gm_read_diag(struct canfd_frame frame) {
  if(verbose) plog("Received GM Read Diagnostic Request\n");
  int offset = 0;
  if(frame.data[0] == 0xFE)
    offset = 1;
  switch(frame.data[2 + offset]) { // Subfunctions
    case UDS_READ_STATUS_BY_MASK:  // Read DTCs by mask
      if(verbose) {
        plog(" + Read DTCs by mask\n");
        if(frame.data[3 + offset] & DTC_SUPPORTED_BY_CALIBRATION) plog("   - Supported By Calibration\n");
        if(frame.data[3 + offset] & DTC_CURRENT_DTC) plog("   - Current DTC\n");
        if(frame.data[3 + offset] & DTC_TEST_NOT_PASSED_SINCE_CLEARED) plog("   - Tests not passed since DTC cleared\n");
        if(frame.data[3 + offset] & DTC_TEST_FAILED_SINCE_CLEARED) plog("   - Tests failed since DTC cleared\n");
        if(frame.data[3 + offset] & DTC_HISTORY) plog("   - DTC History\n");
        if(frame.data[3 + offset] & DTC_TEST_NOT_PASSED_SINCE_POWER) plog("   - Tests not passed since power up\n");
        if(frame.data[3 + offset] & DTC_CURRENT_DTC_SINCE_POWER) plog("   - Tests failed since power up\n");
        if(frame.data[3 + offset] & DTC_WARNING_INDICATOR_STATE) plog("   - Warning Indicator State\n");
      }
      if(frame.can_id == 0x7e0) {
        frame.can_id = 0x5e8;
      } else {
        frame.can_id = 0x500 + (frame.can_id & 0xFF);
      }
      frame.len = 8;
      frame.data[0] = frame.data[2 + offset];
      frame.data[1] = 0;    // DTC 1st byte
      frame.data[2] = 0x30; // DTC 2nd byte
      frame.data[3] = 0;
      frame.data[4] = 0x6F; // Last Test/ This Ignition/ Last Clear bitflag
      frame.data[5] = 0;
      frame.data[6] = 0;
      frame.data[7] = 0;
      write_can_frame(&frame, CAN_MTU);
      sleep(0.2); // Instead of actually processing the FC
      frame.data[1] = 0; // Last frame must be a 0 DTC
      frame.data[2] = 0;
      frame.data[3] = 0;
      frame.data[4] = 0xFF; // Last DTC
      write_can_frame(&frame, CAN_MTU);
      break;
    default:
      if(verbose) plog(" + Unknown subfunction request %02X\n", frame.data[2 + offset]);
      break;
  }
}

/*
  Gateway
 */
void handle_vcds_710(struct canfd_frame frame) {
  if(verbose) plog("Received VCDS 0x710 gateway request\n");
  uint8_t resp[150];
  if(frame.data[0] == 0x30) { // Flow control
    flow_control_push();
    return;
  }
  switch(frame.data[1]) {
    //Pkt: 710#02 10 03 55 55 55 55 55 
    case 0x10: // Diagnostic Session Control
      frame.can_id = 0x77A;
      frame.len = 8;
      frame.data[0] = 0x06;
      frame.data[1] = 0x50;
      frame.data[2] = 0x03;
      frame.data[3] = 0x00;
      frame.data[4] = 0x32;
      frame.data[5] = 0x01;
      frame.data[6] = 0xF4;
      frame.data[7] = 0xAA;
      write_can_frame(&frame, CAN_MTU);
      break;
    case 0x22: // Read Data By Identifier
      if(frame.data[2] == 0xF1) {
        switch(frame.data[3]) {
          case 0x87: // VAG Number
            if(verbose) plog("Read data by ID 0x87\n");
            resp[0] = frame.data[1] + 0x40;
            resp[1] = frame.data[2];
            resp[2] = 0x87;
            resp[3] = 0x35;
            resp[4] = 0x51;
            resp[5] = 0x45;
            resp[6] = 0x39;
            resp[7] = 0x30;
            resp[8] = 0x37;
            resp[9] = 0x35;
            resp[10] = 0x33;
            resp[11] = 0x30;
            resp[12] = 0x43;
            resp[13] = 0x20; // Note normally this would pad with AA's
            isotp_send_to(resp, 14, 0x77A);
            break;
          case 0x89: // VAG Number
            if(verbose) plog("Read data by ID 0x89\n");
            frame.can_id = 0x77A;
            frame.len = 8;
            frame.data[0] = 0x07;
            frame.data[1] = 0x62;
            frame.data[2] = 0xF1;
            frame.data[3] = 0x89;
            frame.data[4] = 0x33; //3
            frame.data[5] = 0x32; //2
            frame.data[6] = 0x30; //0
            frame.data[7] = 0x33; //3
            write_can_frame(&frame, CAN_MTU);
            break;
          case 0x91: // VAG Number
            if(verbose) plog("Read data by ID 0x91\n");
            resp[0] = frame.data[1] + 0x40;
            resp[1] = frame.data[2];
            resp[2] = 0x87;
            resp[3] = 0x35;
            resp[4] = 0x51;
            resp[5] = 0x45;
            resp[6] = 0x39;
            resp[7] = 0x30;
            resp[8] = 0x37;
            resp[9] = 0x35;
            resp[10] = 0x33;
            resp[11] = 0x30;
            resp[12] = 0x41;
            resp[13] = 0x20; // Note normally this would pad with AA's
            isotp_send_to(resp, 14, 0x77A);
            break;
          default:
            if(verbose) plog("NOTE: Read data by unknown ID %02X\n", frame.data[3]);
            resp[0] = frame.data[1] + 0x40;
            resp[1] = frame.data[2];
            resp[2] = 0x87;
            resp[3] = 0x35;
            resp[4] = 0x51;
            resp[5] = 0x45;
            resp[6] = 0x39;
            resp[7] = 0x30;
            resp[8] = 0x37;
            resp[9] = 0x35;
            resp[10] = 0x33;
            resp[11] = 0x30;
            resp[12] = 0x41;
            resp[13] = 0x20; // Note normally this would pad with AA's
            isotp_send_to(resp, 14, 0x77A);
            break;

        }
      } else {
        if (verbose) plog("Unknown read data by Identifier %02X\n", frame.data[2]);
      }
      break;
  }
}

// return Mode/SIDs in english
char *get_mode_str(struct canfd_frame frame) {
  switch(frame.data[1]) {
    case OBD_MODE_SHOW_CURRENT_DATA:
      return "Show current Data";
      break;
    case OBD_MODE_SHOW_FREEZE_FRAME:
      return "Show freeze frame";
      break;
    case OBD_MODE_READ_DTC:
      return "Read DTCs";
      break;
    case OBD_MODE_CLEAR_DTC:
      return "Clear DTCs";
      break;
    case OBD_MODE_TEST_RESULTS_NON_CAN:
      return "Mode Test Results (Non-CAN)";
      break;
    case OBD_MODE_TEST_RESULTS_CAN:
      return "Mode Test Results (CAN)";
      break;
    case OBD_MODE_READ_PENDING_DTC:
      return "Read Pending DTCs";
      break;
    case OBD_MODE_CONTROL_OPERATIONS:
      return "Control Operations";
      break;
    case OBD_MODE_VEHICLE_INFORMATION:
      return "Vehicle Information";
      break;
    case OBD_MODE_READ_PERM_DTC:
      return "Read Permanent DTCs";
      break;
    case UDS_SID_DIAGNOSTIC_CONTROL:
      return "Diagnostic Control";
      break;
    case UDS_SID_ECU_RESET:
      return "ECU Reset";
      break;
    case UDS_SID_CLEAR_DTC:
      return "UDS Clear DTCs";
      break;
    case UDS_SID_READ_DTC:
      return "UDS Read DTCs";
      break;
    case UDS_SID_GM_READ_DID_BY_ID:
      return "Read DID by ID (GM)";
      break;
    case UDS_SID_RESTART_COMMUNICATIONS:
      return "Restore Normal Commnications";
      break;
    case UDS_SID_READ_DATA_BY_ID:
      return "Read DATA By ID";
      break;
    case UDS_SID_READ_MEM_BY_ADDRESS:
      return "Read Memory By Address";
      break;
    case UDS_SID_READ_SCALING_BY_ID:
      return "Read Scalling Data by ID";
      break;
    case UDS_SID_SECURITY_ACCESS:
      return "Security Access";
      break;
    case UDS_SID_COMMUNICATION_CONTROL:
      return "Communication Control";
      break;
    case UDS_SID_READ_DATA_BY_ID_PERIODIC:
      return "Read DATA By ID Periodically";
      break;
    case UDS_SID_DEFINE_DATA_ID:
      return "Define DATA By ID";
      break;
    case UDS_SID_WRITE_DATA_BY_ID:
      return "Write DATA By ID";
      break;
    case UDS_SID_IO_CONTROL_BY_ID:
      return "Input/Output Control By ID";
      break;
    case UDS_SID_ROUTINE_CONTROL:
      return "Routine Control";
      break;
    case UDS_SID_REQUEST_DOWNLOAD:
      return "Request Download";
      break;
    case UDS_SID_REQUEST_UPLOAD:
      return "Request Upload";
      break;
    case UDS_SID_TRANSFER_DATA:
      return "Transfer DATA";
      break;
    case UDS_SID_REQUEST_XFER_EXIT:
      return "Request Transfer Exit";
      break;
    case UDS_SID_REQUEST_XFER_FILE:
      return "Request Transfer File";
      break;
    case UDS_SID_WRITE_MEM_BY_ADDRESS:
      return "Write Memory By Address";
      break;
    case UDS_SID_TESTER_PRESENT:
      return "Tester Present";
      break;
    case UDS_SID_ACCESS_TIMING:
      return "Access Timing";
      break;
    case UDS_SID_SECURED_DATA_TRANS:
      return "Secured DATA Transfer";
      break;
    case UDS_SID_CONTROL_DTC_SETTINGS:
      return "Control DTC Settings";
      break;
    case UDS_SID_RESPONSE_ON_EVENT:
      return "Response On Event";
      break;
    case UDS_SID_LINK_CONTROL:
      return "Link Control";
      break;
    case UDS_SID_GM_PROGRAMMED_STATE:
      return "Programmed State (GM)";
      break;
    case UDS_SID_GM_PROGRAMMING_MODE:
      return "Programming Mode (GM)";
      break;
    case UDS_SID_GM_READ_DIAG_INFO:
      return "Read Diagnostic Information (GM)";
      break;
    case UDS_SID_GM_READ_DATA_BY_ID:
      return "Read DATA By ID (GM)";
      break;
    case UDS_SID_GM_DEVICE_CONTROL:
      return "Device Control (GM)";
      break;
    default:
      printf("Unknown mode/sid (%02X)\n", frame.data[1]);
      return "";
  }
}

// Prints raw packet in ID#DATA format
void print_pkt(struct canfd_frame frame) {
  int i;
  plog("Pkt: %02X#", (unsigned int)frame.can_id);
  for(i = 0; i < frame.len; i++) {
    plog("%02X ", frame.data[i]);
  }
  plog("\n");
}

// Prints binary data in hex format
void print_bin(unsigned char *bin, int size) {
  int i;
  for(i = 0; i < size; i++) {
    plog("%02X ", bin[i]);
  }
  plog("\n");
}

// Handles the incomming CAN Packets
// Each ID that deals with specific controllers a note is
// given where that info came from.  There could be a lot of overlap
// and exceptions here. -- Craig
void handle_pkt(struct canfd_frame frame)
{
  if(DEBUG) print_pkt(frame);
  switch(frame.can_id) {
    case 0x243: // EBCM / GM / Chevy Malibu 2006
      switch(frame.data[1]) {
        case UDS_SID_TESTER_PRESENT:
          if(verbose > 1) plog("Received TesterPresent\n");
          generic_OK_resp_to(frame, 0x643);
          break;
        case UDS_SID_GM_READ_DIAG_INFO:
          handle_gm_read_diag(frame);
          break;
        default:
          if(verbose) print_pkt(frame);
          if(verbose) plog("Unhandled mode/sid: %s\n", get_mode_str(frame));
          break;
      }
      break;
    case 0x244: // Body Control Module / GM / Chevy Malibu 2006
      if(frame.data[0] == 0x30) { // Flow control
        flow_control_push_to(0x644);
        return;
      }
      switch(frame.data[1]) {
        case UDS_SID_TESTER_PRESENT:
          if(verbose > 1) plog("Received TesterPresent\n");
          generic_OK_resp_to(frame, 0x644);
          break;
        case UDS_SID_GM_READ_DIAG_INFO:
          //handle_gm_read_diag(frame);
          break;
        case UDS_SID_GM_READ_DATA_BY_ID:
          //handle_gm_read_data_by_id(frame);
          break;
        case UDS_SID_GM_READ_DID_BY_ID:
          //handle_gm_read_did_by_id(frame);
          break;
        default:
          if(verbose) print_pkt(frame);
          if(verbose) plog("Unhandled mode/sid: %s\n", get_mode_str(frame));
          break;
      }
      break;
    case 0x24A: // Power Steering / GM / Chevy Malibu 2006
      switch(frame.data[1]) {
        default:
          if(verbose) print_pkt(frame);
          if(verbose) plog("Unhandled mode/sid: %s\n", get_mode_str(frame));
          break;
      }
      break;
    case 0x350: // Unsure.  Seen RTRs to this when requesting VIN
      if (frame.can_id & CAN_RTR_FLAG) {
        if (verbose) plog("Received a RTR at ID %02X\n", (unsigned int)frame.can_id);
      }
      break;
    case 0x710: // VCDS
      if(verbose) print_pkt(frame);
      //handle_vcds_710(frame);
      break;
    case 0x7df:
    case 0x7e0:  // Sometimes flow control comes here
      if(verbose) print_pkt(frame);
      if(frame.data[0] == 0x30 && gBufLengthRemaining > 0) flow_control_push();
      if(frame.data[0] == 0 || frame.len == 0) return;
      if(frame.data[0] > frame.len) return;
      switch (frame.data[1]) {
        /*case OBD_MODE_SHOW_CURRENT_DATA:
          handle_current_data(frame);
          break;
        case OBD_MODE_SHOW_FREEZE_FRAME:
          handle_freeze_frame(frame);
          break;
        case OBD_MODE_READ_DTC:
          handle_stored_codes(frame);
          break;
        case OBD_MODE_READ_PENDING_DTC:
          handle_pending_codes(frame);
          break;
        case OBD_MODE_VEHICLE_INFORMATION:
          handle_vehicle_info(frame);
          break;
        case OBD_MODE_READ_PERM_DTC:
          handle_perm_codes(frame);
          break;
        case UDS_SID_DIAGNOSTIC_CONTROL: // DSC
          handle_dsc(frame);
          break;
        case UDS_SID_READ_DATA_BY_ID:
          handle_read_data_by_id(frame);
          break;
        case UDS_SID_TESTER_PRESENT:
          if(verbose > 1) plog("Received TesterPresent\n");
          generic_OK_resp(frame);
          break;
        case UDS_SID_GM_READ_DIAG_INFO:
          handle_gm_read_diag(frame);
          break;
        default:
          //if(verbose) plog("Unhandled mode/sid: %02X\n", frame.data[1]);
          if(verbose) plog("Unhandled mode/sid: %s\n", get_mode_str(frame));
          break;*/
      }
      break;
    default:
      if (DEBUG) print_pkt(frame);
      if (DEBUG) plog("DEBUG: missed ID %02X\n", (unsigned int)frame.can_id);
      break;
  }
}
