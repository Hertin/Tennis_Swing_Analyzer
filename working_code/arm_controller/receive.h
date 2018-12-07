#include <Wire.h>

#define BUFSIZE 1000
#define DATALEN 29
#define SENSORDATALEN 25
#define RAWDATALEN 24
#define HEADERLEN 6
#define MAXNUMFRAME 20
#define NUMATTEMPT 100
#define SEQNUMRANGE 256

byte databuf[BUFSIZE];
byte sensordata[SENSORDATALEN];
int eidx = 0;
int sidx = 0;
byte sensor_data_frames[MAXNUMFRAME][RAWDATALEN];
byte frame_index;
byte seq_num;
byte prev_checksum = 0xff;
// byte seq_num = 7;


bool get_header(byte* num_frame,byte* state_checksum) {
    // read serial buf into databuf
    while (Serial1.available() > 0) {
        databuf[eidx] = Serial1.read();
        eidx = (eidx + 1) % BUFSIZE;
    }
    // declare local varables
    int valid_len = (eidx + BUFSIZE - sidx) % BUFSIZE;
    int i;
    byte num_frame_byte, checksum, seq_num_temp;

    if (valid_len >= HEADERLEN) {
        for (i = 0; i < valid_len; i ++) {
            if (databuf[(i + sidx) % BUFSIZE] == 0xee) {
                break;
            }
        }
        sidx = (i + sidx) % BUFSIZE;
    }
    // recalculate valid data length from the first 0xff
    valid_len = (eidx + BUFSIZE - sidx) % BUFSIZE;

    if (valid_len >= HEADERLEN) {
        // if the data is long enough to read, skip all the preceeding 0xff
        for (i = 0; i < valid_len; i ++) {
            if (databuf[(i + sidx) % BUFSIZE] != 0xee) {
                break;
            }
        }
        sidx = (i + sidx) % BUFSIZE;

        // read data length bytes of data
        seq_num_temp = databuf[sidx];
        sidx = (1 + sidx) % BUFSIZE;
        num_frame_byte = databuf[sidx];
        sidx = (1 + sidx) % BUFSIZE;
        checksum = databuf[sidx];
        sidx = (1 + sidx) % BUFSIZE;
        *state_checksum=checksum;
        if (checksum != byte(~(num_frame_byte+seq_num_temp))) {
            // data corrupted and skip
            Serial1.write((byte) (0xad+seq_num));
            return false;
        } else {
            seq_num = seq_num_temp;
            Serial1.write((byte) (0xad+seq_num));
            seq_num ++;
            *num_frame = num_frame_byte;
            return true;
        }
    }
    return false;
}

bool get_frame_single_attempt(byte frame_index) {
    // read serial buf into databuf
    while (Serial1.available() > 0) {
        databuf[eidx] = Serial1.read();
        eidx = (eidx + 1) % BUFSIZE;
    }
    // declare local varables
    int valid_len = (eidx + BUFSIZE - sidx) % BUFSIZE;
    int i;
    // find the leading 0xff
    if (valid_len >= DATALEN) {
        for (i = 0; i < valid_len; i ++) {
            if (databuf[(i + sidx) % BUFSIZE] == 0xff) {
                break;
            }
        }
        sidx = (i + sidx) % BUFSIZE;
    }
    // recalculate valid data length from the first 0xff
    valid_len = (eidx + BUFSIZE - sidx) % BUFSIZE;

    if (valid_len >= DATALEN) {
        // if the data is long enough to read, skip all the preceeding 0xff
        for (i = 0; i < valid_len; i ++) {
            if (databuf[(i + sidx) % BUFSIZE] != 0xff) {
                break;
            }
        }
        sidx = (i + sidx) % BUFSIZE;

        // read data length bytes of data
        for (i = 0; i < SENSORDATALEN; i ++) {
            sensordata[i] = databuf[(sidx + i) % BUFSIZE];
        }
        sidx = (i + sidx) % BUFSIZE;

        // calculate checksum
        byte checksum = 0;
        for (i = 0; i < RAWDATALEN; i ++) {
            checksum += sensordata[i];
        }
        // add frame index to checksum
        checksum += frame_index;

        if (checksum != sensordata[SENSORDATALEN - 1]) {
            // data corrupted and skip
            Serial1.write(prev_checksum);
        } else {
//            for (int _ = 0; _ < 10; _ ++)
              Serial1.write(checksum);

            prev_checksum = checksum;
            memcpy(sensor_data_frames[frame_index], sensordata, RAWDATALEN);

            return true;
        }
    
    }
    return false;
}

bool get_frame(byte frame_index) {

    bool is_successful;
    for (int _ = 0; _ < NUMATTEMPT; _ ++) {

        is_successful = get_frame_single_attempt(frame_index);
        if (is_successful) {
          
            return true;
        }
        delay(10);
    }
    return false;
}
