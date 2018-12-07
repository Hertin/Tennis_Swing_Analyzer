#include <Wire.h>
#include <Adafruit_L3GD20.h> // gyro
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "send.h"

enum state_t {
    not_started,
    started
};

Adafruit_L3GD20 gyro;
Adafruit_LIS3DH lis = Adafruit_LIS3DH();



float detect_buf[NUMFRAME][FRAMELEN];
float data_buf[MAXNUMFRAME][FRAMELEN];

state_t state = not_started;

int detect_sidx = 0;

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial.println("setup");

        pinMode(ledPin, OUTPUT);
        // Try to initialise and warn if we couldn't detect the chip
        if (!gyro.begin(gyro.L3DS20_RANGE_500DPS)) {
            Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
            while (1);
        }
        // Try to initialize accelerometer
        Serial.println("LIS3DH test!");
        if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
            Serial.println("Couldnt start");
            while (1);
        }
        Serial.println("LIS3DH found!");
        lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
        memset(detect_buf, 0, sizeof(detect_buf));
    delay(1000);
}

void inner_product(float* A, float* B, float* out, int dim) {
    float result = 0.;
    for (int i = 0; i < dim; i++) {
        result += A[i] * B[i];
    }
    *out = result;
}
float weight[NUMFRAME * FRAMELEN] = {
    -2.13304766e-03,  3.08515642e-05, -4.62394409e-04, -2.27734449e-04,
    -1.69856247e-04, -2.58512919e-04, -3.47820079e-04,  2.46600368e-04,
    1.50669396e-03, -1.33127959e-04, -7.22702007e-05,  7.97154109e-05,
    -2.53365161e-03, -4.45162785e-04,  1.44580420e-04,  8.37445120e-05,
    7.38756239e-05,  1.21285897e-04, -1.67585043e-03,  5.63334634e-04,
    2.85814621e-04, -2.86303759e-05,  4.42275481e-05, -1.65334412e-04,
    1.34047585e-03,  1.82538161e-03,  2.33918332e-04, -1.59824899e-04,
    1.95857322e-05, -2.01980559e-04,  1.10839614e-03, -2.74764746e-03,
    -4.18425276e-03, -3.80907140e-06,  1.56871860e-04,  1.53334146e-04,
    3.84160403e-03,  1.27130033e-03, -2.61865776e-03, -4.44843978e-05,
    -4.85383832e-05, -2.87002479e-05, -4.17461088e-03, -5.83865822e-03,
    2.86233319e-03, -9.62689134e-05, -1.26305536e-04, -2.93737751e-04,
    -7.68704615e-04,  1.15181632e-03, -5.22868070e-03,  2.13197548e-04,
    1.81593182e-05,  9.35036005e-04, -1.77914112e-03, -7.24628143e-03,
    3.97028820e-03,  8.73993656e-05,  4.11233280e-04, -3.83729803e-04
};

float bias = 0.014249906630799142;



void loop() {
    int cid;
    float prob = 0.;
    float inner_prod;
    bool is_started;
    sensors_event_t event;
    lis.getEvent(&event);
    gyro.read();

    // update detect_buf
    detect_buf[detect_sidx][0] = event.acceleration.x;
    detect_buf[detect_sidx][1] = event.acceleration.y;
    detect_buf[detect_sidx][2] = event.acceleration.z;
    detect_buf[detect_sidx][3] = gyro.data.x;
    detect_buf[detect_sidx][4] = gyro.data.y;
    detect_buf[detect_sidx][5] = gyro.data.z;

    for (int i = 0; i < NUMFRAME; i++) {
        cid = (detect_sidx + i + 1) % NUMFRAME;
        inner_product(detect_buf[cid], &weight[i * FRAMELEN], &inner_prod, FRAMELEN);
        prob += inner_prod;
    }
    prob += bias;

    is_started = prob > STARTTHRESH;

    if (state == not_started && is_started) {
        // detect start of swing
        Serial.println("<<start of swing");
        send_data(data_buf,0);
        state = started;
        for (int i = 0; i < NUMFRAME; i ++) {
            cid = (detect_sidx + i + 1) % NUMFRAME;
            memcpy(&data_buf[i], &detect_buf[cid], FRAMELEN * sizeof(float));
        }
        data_cframe = NUMFRAME;
    } else if (state == started && is_started) {
        // during the swing
        cid = (detect_sidx + NUMFRAME) % NUMFRAME;
        memcpy(&data_buf[data_cframe], &detect_buf[cid], FRAMELEN * sizeof(float));
        data_cframe ++;
    } else if (state == started && (!is_started || data_cframe == MAXNUMFRAME)) {
        // end of the swing
        cid = (detect_sidx + NUMFRAME) % NUMFRAME;
        memcpy(&data_buf[data_cframe], &detect_buf[cid], FRAMELEN * sizeof(float));
        data_cframe ++;

        
        delay(100);

        lis.getEvent(&event);
        gyro.read();
        data_buf[data_cframe][0] = event.acceleration.x;
        data_buf[data_cframe][1] = event.acceleration.y;
        data_buf[data_cframe][2] = event.acceleration.z;
        data_buf[data_cframe][3] = gyro.data.x;
        data_buf[data_cframe][4] = gyro.data.y;
        data_buf[data_cframe][5] = gyro.data.z;
        data_cframe ++;

        delay(100);

        lis.getEvent(&event);
        gyro.read();
        data_buf[data_cframe][0] = event.acceleration.x;
        data_buf[data_cframe][1] = event.acceleration.y;
        data_buf[data_cframe][2] = event.acceleration.z;
        data_buf[data_cframe][3] = gyro.data.x;
        data_buf[data_cframe][4] = gyro.data.y;
        data_buf[data_cframe][5] = gyro.data.z;
        data_cframe ++;

        
        // print out the data in the frame
        for (int i = 0; i < data_cframe; i ++) {
            float * data_ptr;
            data_ptr = (float *) data_buf[i];
            for (int j = 0; j < FRAMELEN; j ++) {
                Serial.print(data_ptr[j]);
                Serial.print(" ");
            }
            Serial.println();
        }

        bool is_successful = send_data(data_buf, data_cframe);
        if (is_successful) {
            Serial.println("data_sent");
        }
        
        // this function now only sends out the data_cframe
        memset(detect_buf, 0, sizeof(detect_buf));
        state = not_started;

        data_cframe = 0;
        delay(500);
    }
    detect_sidx = (detect_sidx + 1) % NUMFRAME;
    delay(100);
}
