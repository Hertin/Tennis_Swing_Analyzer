#include <Wire.h>
#include <Adafruit_L3GD20.h> // gyro
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <arduinoFFT.h>
#include "receive.h"
#include "pitches.h"

// macro for arm data recording
#define NUMFRAME 10
#define FRAMELEN 6
#define MAXNUMFRAME 20
#define MAXNEWNUMFRAME 50
#define ACCOFFSET 0
#define GYROFFSET (FRAMELEN)

/* class to handle fft
 */
arduinoFFT FFT = arduinoFFT();
const uint16_t nsamples = 32;
double vReal[nsamples];
double vImag[nsamples];

// parameter for swing classification
float clf_weight[nsamples * FRAMELEN * 2] = {
    -2.45705282e-06,  6.26850286e-06,  3.58735826e-06,
     6.98904779e-05, -2.24643145e-05, -6.29071450e-05,
    -1.92764906e-06,  3.30401325e-06,  7.38820768e-06,
    -5.07172633e-05, -8.84155497e-05, -2.68257103e-05,
     1.55584261e-07, -4.84445275e-06,  2.34149538e-06,
    -4.89897405e-05,  1.44470303e-05, -3.09089228e-06,
    -4.16284972e-06,  2.88550075e-06, -1.69372411e-06,
     1.32696803e-05,  9.79396841e-05,  4.14883806e-06,
     1.90349918e-07,  3.05852365e-06, -2.59296859e-06,
     3.56205172e-05,  6.06182895e-06,  9.14530973e-06,
     5.33605779e-06, -5.94338194e-06, -3.94039588e-06,
    -5.53556191e-05, -1.00649174e-04, -1.11350427e-05,
     1.38464463e-06, -1.69706488e-06,  7.03407074e-07,
    -7.02404729e-06, -1.88604420e-05, -2.52491005e-05,
    -3.45560010e-06,  3.96122042e-06,  8.93095956e-06,
     2.93550957e-05,  4.33452092e-05,  2.28541745e-05,
    -1.36932961e-06, -7.27152479e-07, -4.99349485e-07,
    -3.54861995e-05,  4.15951257e-05,  1.54847410e-06,
    -8.32555807e-07,  3.30707536e-06, -5.79179097e-06,
     2.71664380e-06,  5.37993316e-05, -2.97253959e-05,
    -7.16648429e-07,  1.09953971e-06,  2.06300091e-06,
     4.62749301e-05, -7.06440941e-05,  6.85167002e-06,
     1.81382915e-06, -4.05428411e-06,  1.60211779e-06,
    -4.59236383e-05, -7.06870245e-05,  2.25148373e-05,
     9.47545502e-07, -2.75491973e-07,  8.67485928e-08,
    -4.08493980e-05,  3.09643193e-05, -2.64275166e-05,
    -9.63791142e-07,  3.65278656e-06,  1.39937768e-06,
     2.41782762e-05,  6.66778158e-05, -1.39382059e-06,
    -7.71800936e-08, -7.88980493e-07, -1.56807371e-06,
    -4.87852090e-06,  2.47551955e-05,  1.08141782e-05,
     2.60321988e-10, -2.22153106e-07, -1.82555333e-06,
    -1.14570243e-05, -1.67849543e-05,  7.09181433e-07,
    -1.86928647e-06,  6.68366057e-07,  1.50283226e-06,
    -5.94395763e-06, -5.70921223e-05, -5.73135962e-06,
     4.66485740e-07,  1.81256163e-08,  1.69101815e-06,
    -3.53103316e-06,  5.52997948e-06,  9.87989195e-06,
     6.77493587e-07, -6.59686383e-07,  5.43139532e-07,
    -9.61287774e-06,  2.88804883e-05, -7.69274599e-06,
    -1.18834206e-06,  2.17405834e-06, -4.01956274e-07,
    -9.10790153e-06,  3.84281363e-06, -7.90014267e-06,
     2.46936703e-07,  2.20097328e-07, -1.28760119e-07,
     1.89579935e-05,  6.31301004e-06,  2.77406108e-06,
     1.06096760e-06, -1.27329115e-06, -4.05100726e-07,
    -3.11611342e-06,  7.95842459e-06,  2.98964061e-06,
    -6.45220401e-07, -7.94424277e-08,  2.47044680e-07,
    -7.91301242e-06, -2.88512798e-05, -3.06138186e-06,
    -7.68185171e-07,  9.71188173e-07,  7.43145190e-07,
     1.40181560e-05, -7.57134096e-06,  1.01157325e-05,
     4.37443678e-07,  2.84683177e-07,  3.85792606e-07,
    -1.38254031e-05,  1.02224130e-05, -1.10205215e-05,
     2.03692831e-07,  9.33646101e-07, -2.10739774e-07,
    -1.48722184e-05,  8.58170936e-06, -1.16315722e-05,
     3.72269116e-07, -5.32345639e-07, -1.01818625e-06,
     4.93754862e-06,  6.58969367e-06,  5.72324879e-06,
    -2.03469897e-07, -7.86194413e-07,  4.18461210e-07,
     5.13137123e-06,  6.20471168e-06,  7.37429459e-06,
    -6.29684520e-07,  3.13021926e-07,  1.76338089e-06,
     9.15183098e-06, -1.14547333e-05, -3.95660454e-06,
    -3.65550269e-07,  7.84551840e-07, -8.77915456e-09,
     5.19273390e-06, -6.22450105e-06,  1.58077412e-07,
    -7.56953690e-07, -3.38543842e-07,  3.86146920e-07,
    -3.60174705e-06, -6.83767605e-06, -2.41323195e-06,
    -7.02704819e-09,  1.25915209e-06,  5.34949425e-08,
    -1.25938806e-05,  6.06376313e-06, -2.18794533e-06,
     8.58583585e-07,  2.33389657e-07, -9.87587142e-07,
    -1.82646190e-05,  2.05963766e-05, -3.77192833e-06,
    -1.44742890e-07, -1.31555367e-06,  1.04709639e-07,
     1.43825989e-05,  3.14793475e-06,  4.22690886e-06,
    -7.56953690e-07, -3.38543842e-07,  3.86146920e-07,
    -3.60174705e-06, -6.83767605e-06, -2.41323195e-06,
    -7.02704819e-09,  1.25915209e-06,  5.34949425e-08,
    -1.25938806e-05,  6.06376313e-06, -2.18794533e-06,
    -6.29684520e-07,  3.13021926e-07,  1.76338089e-06,
     9.15183098e-06, -1.14547333e-05, -3.95660454e-06,
    -3.65550269e-07,  7.84551840e-07, -8.77915456e-09,
     5.19273390e-06, -6.22450105e-06,  1.58077412e-07,
     3.72269116e-07, -5.32345639e-07, -1.01818625e-06,
     4.93754862e-06,  6.58969367e-06,  5.72324879e-06,
    -2.03469897e-07, -7.86194413e-07,  4.18461210e-07,
     5.13137123e-06,  6.20471168e-06,  7.37429459e-06,
     4.37443678e-07,  2.84683177e-07,  3.85792606e-07,
    -1.38254031e-05,  1.02224130e-05, -1.10205215e-05,
     2.03692831e-07,  9.33646101e-07, -2.10739774e-07,
    -1.48722184e-05,  8.58170936e-06, -1.16315722e-05,
    -6.45220401e-07, -7.94424277e-08,  2.47044680e-07,
    -7.91301242e-06, -2.88512798e-05, -3.06138186e-06,
    -7.68185171e-07,  9.71188173e-07,  7.43145190e-07,
     1.40181560e-05, -7.57134096e-06,  1.01157325e-05,
     2.46936703e-07,  2.20097328e-07, -1.28760119e-07,
     1.89579935e-05,  6.31301004e-06,  2.77406108e-06,
     1.06096760e-06, -1.27329115e-06, -4.05100726e-07,
    -3.11611342e-06,  7.95842459e-06,  2.98964061e-06,
     6.77493587e-07, -6.59686383e-07,  5.43139532e-07,
    -9.61287774e-06,  2.88804883e-05, -7.69274599e-06,
    -1.18834206e-06,  2.17405834e-06, -4.01956274e-07,
    -9.10790153e-06,  3.84281363e-06, -7.90014267e-06,
    -1.86928647e-06,  6.68366057e-07,  1.50283226e-06,
    -5.94395763e-06, -5.70921223e-05, -5.73135962e-06,
     4.66485740e-07,  1.81256163e-08,  1.69101815e-06,
    -3.53103316e-06,  5.52997948e-06,  9.87989195e-06,
    -7.71800936e-08, -7.88980493e-07, -1.56807371e-06,
    -4.87852090e-06,  2.47551955e-05,  1.08141782e-05,
     2.60321988e-10, -2.22153106e-07, -1.82555333e-06,
    -1.14570243e-05, -1.67849543e-05,  7.09181433e-07,
     9.47545502e-07, -2.75491973e-07,  8.67485928e-08,
    -4.08493980e-05,  3.09643193e-05, -2.64275166e-05,
    -9.63791142e-07,  3.65278656e-06,  1.39937768e-06,
     2.41782762e-05,  6.66778158e-05, -1.39382059e-06,
    -7.16648429e-07,  1.09953971e-06,  2.06300091e-06,
     4.62749301e-05, -7.06440941e-05,  6.85167002e-06,
     1.81382915e-06, -4.05428411e-06,  1.60211779e-06,
    -4.59236383e-05, -7.06870245e-05,  2.25148373e-05,
    -1.36932961e-06, -7.27152479e-07, -4.99349485e-07,
    -3.54861995e-05,  4.15951257e-05,  1.54847410e-06,
    -8.32555807e-07,  3.30707536e-06, -5.79179097e-06,
     2.71664380e-06,  5.37993316e-05, -2.97253959e-05,
     1.38464463e-06, -1.69706488e-06,  7.03407074e-07,
    -7.02404729e-06, -1.88604420e-05, -2.52491005e-05,
    -3.45560010e-06,  3.96122042e-06,  8.93095956e-06,
     2.93550957e-05,  4.33452092e-05,  2.28541745e-05,
     1.90349918e-07,  3.05852365e-06, -2.59296859e-06,
     3.56205172e-05,  6.06182895e-06,  9.14530973e-06,
     5.33605779e-06, -5.94338194e-06, -3.94039588e-06,
    -5.53556191e-05, -1.00649174e-04, -1.11350427e-05,
     1.55584261e-07, -4.84445275e-06,  2.34149538e-06,
    -4.89897405e-05,  1.44470303e-05, -3.09089228e-06,
    -4.16284972e-06,  2.88550075e-06, -1.69372411e-06,
     1.32696803e-05,  9.79396841e-05,  4.14883806e-06  
};

float clf_bias = -0.58477614;
// array to store fft of time series
float data_fft[nsamples * FRAMELEN * 2];

enum state_t {
    not_started,
    wait_racket
};

/* buffer for arm data recording, most recent 20 frames
 * the buffer is constantly rolling out the oldest data 
 */
float arm_buf[MAXNUMFRAME][FRAMELEN];
int arm_sidx = 0;

// led pin to output debug sinal
const int ledPin =  13;

// two-state FSM to receive and collect data
state_t state = not_started;

/* buffer to contain the data used to classify the swing, 
 * copied from the newest data in arm_buf once in wait_racket state
 */
float arm_new_buf[MAXNEWNUMFRAME][FRAMELEN];

/* record the current checksum and previous checksum
 */
byte cur_state_checksum, new_state_checksum;
byte cframe_idx;

/* sensor variables to poll data from sensor
 */
Adafruit_L3GD20 gyro;
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    Serial1.begin(9600);

    // setup sensor
    Serial.println("setup");

    pinMode(ledPin, OUTPUT);
    // Try to initialise and warn if we couldn't detect the chip
    if (!gyro.begin(gyro.L3DS20_RANGE_500DPS)) {
        Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
        while (1)
            Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    }
    // Try to initialize accelerometer
    Serial.println("LIS3DH test!");
    if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
        Serial.println("Couldnt start");
        while (1)
            Serial.println("Couldnt start");
    }
    Serial.println("LIS3DH found!");
    lis.setRange(LIS3DH_RANGE_16_G);   // 2, 4, 8 or 16 G!
    memset(arm_buf, 0, sizeof(arm_buf));
    delay(1000);
}

/* helper function to calculate inner product of two 1-D arrays
 * @param: A, B are the two 1-D array
 * @param: out is the inner product of A and B
 * @param: dim is the dimension of two array
 */
void inner_product(float* A, float* B, float* out, int dim) {
    float result = 0.;
    for (int i = 0; i < dim; i++) {
        result += A[i] * B[i];
    }
    *out = result;
}

void loop() {
    // regularly collect data
    int cid;
    bool is_successful;
    float inner_prod;
    float prob;
    int idx;

    sensors_event_t event;
    lis.getEvent(&event);
    gyro.read();

    // update arm_buf at frequency of 10 Hz
    arm_buf[arm_sidx][0] = event.acceleration.x;
    arm_buf[arm_sidx][1] = event.acceleration.y;
    arm_buf[arm_sidx][2] = event.acceleration.z;
    arm_buf[arm_sidx][3] = gyro.data.x;
    arm_buf[arm_sidx][4] = gyro.data.y;
    arm_buf[arm_sidx][5] = gyro.data.z;

    // check if racket controller send signal
    byte num_frame = 0;

    // try to get_header every loop
    if (get_header(&num_frame, &new_state_checksum)) {
        
        if (num_frame == 0 && state == not_started) {
            state = wait_racket;
            //flush buf
            for (int i = 0; i < NUMFRAME; i ++) {
                cid = (arm_sidx + i + 1) % NUMFRAME;
                memcpy(&arm_new_buf[i], &arm_buf[cid], FRAMELEN * sizeof(float));
            }
            cframe_idx = NUMFRAME;
            cur_state_checksum = new_state_checksum;
        }
        else if (num_frame == 0 && state == wait_racket && cur_state_checksum == new_state_checksum) {
            // increment buf
            memcpy(&arm_new_buf[cframe_idx], &arm_buf[arm_sidx], FRAMELEN * sizeof(float));
            cframe_idx++;
        }

        else if (num_frame != 0 && state == wait_racket) {
            // receive data from racket
            for (frame_index = 0; frame_index < num_frame; frame_index++) {
                is_successful = get_frame(frame_index);
                if (!is_successful) break;
            }
            
            // reset new arm buf and state regardless success or not
            cframe_idx = 0;
            state = not_started;
            Serial.println();
            Serial.println();

            // swing analyzer-----------------------------------
            // calcuate fft of arm data
            for (int iDim = 0; iDim < FRAMELEN; iDim ++) {
                // clear vReal and vImag
                memset(vReal, 0, sizeof(vReal));
                memset(vImag, 0, sizeof(vImag));
                // copy arm_new_buf ot vReal
                for (int iFrame = 0; iFrame < num_frame; iFrame ++) {
                    vReal[iFrame] = arm_new_buf[iFrame][iDim];
                }
                FFT.Compute(vReal, vImag, nsamples, FFT_FORWARD);

                for (int iFrame = 0; iFrame < nsamples; iFrame ++) {
                    idx = iFrame * FRAMELEN * 2 + iDim + ACCOFFSET;
                    data_fft[idx] = vReal[iFrame];
                }
            }
            // calcuate fft of racket data
            for (int iDim = 0; iDim < FRAMELEN; iDim ++) {
                // clear vReal and vImag
                memset(vReal, 0, sizeof(vReal));
                memset(vImag, 0, sizeof(vImag));
                // copy sensor data to vReal
                for (int iFrame = 0; iFrame < num_frame; iFrame ++) {
                    vReal[iFrame] = *(float*)&sensor_data_frames[iFrame][sizeof(float) * iDim];
                }
                FFT.Compute(vReal, vImag, nsamples, FFT_FORWARD);

                for (int iFrame = 0; iFrame < nsamples; iFrame ++) {
                    idx = iFrame * FRAMELEN * 2 + iDim + GYROFFSET;
                    data_fft[idx] = vReal[iFrame];
                }
            }

            inner_product(data_fft, clf_weight, &inner_prod, nsamples * FRAMELEN * 2);
            prob = inner_prod + clf_bias;
            // output result as sound
            play_music(prob > 0);
            // -------------------------------------------------

        }
    } else if (state == wait_racket) {
        // increment buf if receive nothing and waiting for racket data
        memcpy(&arm_new_buf[cframe_idx], &arm_buf[arm_sidx], FRAMELEN * sizeof(float));
        cframe_idx++;
    }
    arm_sidx = (arm_sidx + 1) % MAXNUMFRAME;

    delay(100);

}
