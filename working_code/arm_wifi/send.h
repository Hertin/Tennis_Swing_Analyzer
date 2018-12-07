#include <Wire.h>

#define NUMFRAME 10
#define FRAMELEN 6
#define MAXNUMFRAME 20 
#define HEADBUFLEN 4
#define HEADERLEN 6
#define STARTTHRESH (0.5)
#define NUMATTEMPT 100

int data_cframe = 0;
byte seq_num = 7;

const int ledPin =  13;

byte header[HEADERLEN] = {0xee, 0xee, 0xee, 0xee, 0x00, 0x00};

byte header_buf[HEADBUFLEN] = {0xff, 0xff, 0xff, 0xff}; // header for frame

bool send_head_single_attempt() {
	  Serial.write(header, sizeof(header));
    delay(10);

    if (Serial.available()) {
        byte x = Serial.read();
        if (x == byte(0xad+seq_num)) {
            // data sucessfully sent
            digitalWrite(ledPin, LOW);
            seq_num ++;
            
            Serial.println("header received");
            return true;
        }
    }
    return false;
}

bool send_head(int num_frame) {
	  bool is_successful;
    byte num_frame_byte = (byte) num_frame;
    byte checksum = num_frame_byte + seq_num;
    header[3] = seq_num;
    header[4] = num_frame_byte;
    header[5] = ~(checksum);

    for (int _ = 0; _ < NUMATTEMPT; _ ++) {
        is_successful = send_head_single_attempt();
    	if (is_successful) return true;

    }

    return false;
}

bool send_frame_single_attempt(byte* data_buffer, int frame_size, byte checksum) {
	  Serial.write(header_buf, HEADBUFLEN);
    Serial.write(data_buffer, frame_size);
    Serial.write(byte(checksum));

    // wait for feedback from another
    delay(30);

    if (Serial.available()) {
        byte x = Serial.read();
        if (x == checksum) {
            // data sucessfully sent
            return true;
        }
    }
    return false;
}

bool send_frame(byte* data_buffer, int frame_size, int frame_index) {
    byte checksum = 0;
    bool is_successful;
    
    for (int i = 0; i < frame_size; i ++) {
        checksum += data_buffer[i];
    }
    checksum += frame_index; // add frame index to checksum
    float* data_buffer_ptr = (float*) data_buffer;

	int _;
    for (_ = 0; _ < NUMATTEMPT; _ ++) {
        is_successful = send_frame_single_attempt(data_buffer, frame_size, checksum);
        if (is_successful) {
            return true;
        }
    }

    return false;
}

bool send_data(float data_buffer[][FRAMELEN], int num_frame) {

    bool is_successful;
    
    is_successful = send_head(num_frame);
    if (!is_successful) return false;
    
    
    for (int i = 0; i < num_frame; i ++) {
        is_successful = send_frame((byte*) data_buffer[i], FRAMELEN*sizeof(float), i);
        if (!is_successful) return false;
        
    }

    delay(200);
    return true;
}
