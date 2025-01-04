#include <iostream>
#include <vector>
#include "driver/adc.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include <algorithm>

using namespace std;

#define I2C_MASTER_SCL_IO 22                // SCL Pin Number
#define I2C_MASTER_SDA_IO 21                // SDA Pin Number
#define I2C_MASTER_NUM I2C_NUM_0            // I2C Port Number
#define I2C_MASTER_FREQ_HZ 100000           // I2C Frequency
#define MPU6050_ADDR 0x68                   // MPU6050 Address

//configures I2C communication with MPU6050 and defines read and write
class MPU6050 {
public:
    MPU6050() {
        initialize();
    }

    void initialize(){
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        i2c_param_config(I2C_MASTER_NUM, &conf);
        i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

        uint8_t data[] = {0x6B, 0x00};
        writeBytes(MPU6050_ADDR, data, sizeof(data));
    }

//still figuring this out, accel data registers are 59 to 64, gyro data registers are 67 to 72
//3B = ACCEL_XOUT_H, 3C = ACCEL_XOUT_L, 3D = ACCEL_YOUT_H, 3E = ACCEL_YOUT_L, 3F = ACCEL_ZOUT_H, 40 = ACCEL_ZOUT_L
//43 = GYRO_XOUT_H, 44 = GYRO_XOUT_L, 45 = GYRO_YOUT_H, 46 = GYRO_YOUT_L, 47 = GYRO_ZOUT_H, 48 = GYRO_ZOUT_L
/* 
    void getAccelData(int16_t *data) {
        uint8_t rawData[6];
        readBytes(MPU6050_ADDR, rawData, sizeof(rawData));
        data[0] = (int16_t)(rawData[0] << 8 | rawData[1]);
        data[1] = (int16_t)(rawData[2] << 8 | rawData[3]);
        data[2] = (int16_t)(rawData[4] << 8 | rawData[5]);
    }

    void getGyroData(int16_t *data) {
        uint8_t rawData[6];
        readBytes(MPU6050_ADDR, rawData, sizeof(rawData));
        data[0] = (int16_t)(rawData[0] << 8 | rawData[1]);
        data[1] = (int16_t)(rawData[2] << 8 | rawData[3]);
        data[2] = (int16_t)(rawData[4] << 8 | rawData[5]);
    }
 */
private:
    void writeBytes(uint8_t address, uint8_t *data, size_t length) {            //for sending data to MPU6050 (auto-generated, might not be useful)
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();                           //I2C command link
        i2c_master_start(cmd);                                                  //start
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);    //send address
        i2c_master_write(cmd, data, length, true);                              //send data
        i2c_master_stop(cmd);                                                   //stop
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);   //send command
        i2c_cmd_link_delete(cmd);                                               //end command
    }

    void readBytes(uint8_t address, uint8_t reg, uint8_t *data, size_t length) {    //for reading data from MPU6050
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();                               //I2C command link
        i2c_master_start(cmd);                                                      //start 
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);        //send address
        i2c_master_write_byte(cmd, reg, true);                                      //send register
        i2c_master_start(cmd);                                                      //start       
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);         //send address
        i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);                   //read data (might need update)
        i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);                      //^^
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }
};

//Gesture class
class Gesture {
public:
    string name = ""; //IF YOU WANT TO CHANGE THE DATA TYPE OF THE GESTURE, CHANGE IT HERE
    std::vector<int> fingerStates = {0, 0, 0, 0, 0};
    
    void setGesture(string name, vector<int> fingerStates) {
        this->name = name;
        this->fingerStates = fingerStates;
    }

    vector<int> getFingerStates() {
        return fingerStates;
    }

    string getName() {
        return name;
    }

    string isThis(vector<int> flexion) {
        if (flexion == fingerStates) {
            return name;
        }
        else {
            return "";
        }
    }

/*
THERE SHOULD ALSO BE A SECTION HERE WHICH ACCOUNTS FOR THE GYROSCOPE DATAS WHICH MAY BE A CLASS OF ITS OWN
*/

};

//class for gyro data comparision
//the two key parts to remember is that one gyro will catch wrist orientation and the other will catch hand orientation

//class for one flex sensor
class FlexSensor {
public:
    int rawValue = 0;
    int max = 0;
    int min = 10000;

    void autoCalibrate() {
        if (rawValue > max) max = rawValue;
        if (rawValue < min) min = rawValue;
    }

    int scaleValue(int newMin, int newMax) const {
        int range = max - min;
        if (range == 0) range = 1;
        int newRange = newMax - newMin;
        return (((rawValue - min) * newRange) / range) + newMin;
    }

    //this requires actual calibration depending on user hand flex ranges
    int flexCheck(int scaledVal){
        if(scaledVal > 30 && scaledVal < 70){   //30 and 70 are arbitrary values, should be calibrated
            return 0;                           //this is idle state
        }
        else if(scaledVal < 30){                //30 is arbitrary, should be calibrated
            return -1;                          //this is flexed state
        }
        else{
            return 1;                           //this is extended state
        }
    }
};

extern "C" 

void app_main() {
    vector<FlexSensor> sensors(5);
    vector<int> flexion(5);
    vector<Gesture> gestures(3);

    //RAW FLEX SENSOR DATA FROM ADCs
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);
    adc1_config_width(ADC_WIDTH_BIT_12);

    //ASSIGN GESTURES
    gestures[0].setGesture("Agree", {1, -1, -1, -1, -1});
    gestures[1].setGesture("Rock and Roll", {1, 1, -1, -1, 1});
    gestures[2].setGesture("Bird", {1, -1, 1, -1, -1});

    //DATA NORMALIZATION TO A 0 TO 100 SCALE
    while (true) {
        sensors[0].rawValue = adc1_get_raw(ADC1_CHANNEL_0); // Thumb flex sensor
        sensors[1].rawValue = adc1_get_raw(ADC1_CHANNEL_3); // Index flex sensor
        sensors[2].rawValue = adc1_get_raw(ADC1_CHANNEL_6); // Middle flex sensor
        sensors[3].rawValue = adc1_get_raw(ADC1_CHANNEL_7); // Ring flex sensor
        sensors[4].rawValue = adc1_get_raw(ADC1_CHANNEL_4); // Pinky flex sensor

        vTaskDelay(100 / portTICK_PERIOD_MS);

        for (auto &sensor : sensors) {
            sensor.autoCalibrate();
        }

        //FLEXION CHECK
        vector<string> fingerNames = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
        for (size_t i = 0; i < sensors.size(); ++i) {
            int scaledValue = sensors[i].scaleValue(0, 100);
            flexion[i] = sensors[i].flexCheck(scaledValue);
            cout << fingerNames[i] << " value: " << scaledValue << endl;
        }           
         
        //GESTURE CHECK
        Gesture currentGesture;
        currentGesture.setGesture("current", flexion);
        auto gestCheck = std::find_if(gestures.begin(), gestures.end(), [&](Gesture &g) {
            return g.isThis(flexion) == g.name;
        });

        cout << "Gesture: ";
        if (gestCheck != gestures.end()) {
            cout << gestCheck->name;
        }
        else {
            cout << "No gesture";
        }
        endl(cout);
        
        //PRINT FLEXION VALUES
/*      cout << "Flexion: ";
        for (size_t i = 0; i < flexion.size(); i++) {
            cout << flexion[i] << " ";
        }
        cout << endl; */
    }
}