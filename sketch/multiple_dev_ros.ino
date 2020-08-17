#include <ads_two_axis_hal.h>
#include <ads_two_axis_err.h>
#include <ads_two_axis.h>
#include <ads_two_axis_fw.h>
#include <ads_two_axis_util.h>
#include <ads_two_axis_dfu.h>

#include <ads_two_axis_hal.h>
#include <ads_two_axis_err.h>
#include <ads_two_axis.h>
#include <ads_two_axis_fw.h>
#include <ads_two_axis_util.h>
#include <ads_two_axis_dfu.h>

#include "Arduino.h"
#include "ads_two_axis.h"

//Use the following line if the device has a native USB port, the following line should above #include <ros.h>
#define USE_USBCON

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>



//Set up the ros node and publisher
std_msgs::Float32MultiArray sensor_reading_msg;
ros::Publisher pub_temp("BendingSensorReadings", &sensor_reading_msg);
ros::NodeHandle nh;


#define DEVICE_NUM 4
#define DATA_UPDATED (2^DEVICE_NUM-1)
#define ADS_RESET_PIN       (4)         // Pin number attached to ads reset line.
#define ADS_INTERRUPT_PIN   (6)         // Pin number attached to the ads data ready line. 

// function prototypes
void ads_data_callback(float * sample);
void deadzone_filter(float * sample);
void signal_filter(float * sample);
void parse_serial_port(void);


float ang[DEVICE_NUM][2];
volatile unsigned int current_device_id = 0;
volatile uint8_t newData = 0x00;
volatile bool seq = true;


void signal_filter(float * sample)
{
    static float filter_samples[DEVICE_NUM][2][6];
    uint8_t j = current_device_id;
    for(uint8_t i=0; i<2; i++)
    {
      filter_samples[j][i][5] = filter_samples[j][i][4];
      filter_samples[j][i][4] = filter_samples[j][i][3];
      filter_samples[j][i][3] = (float)sample[i];
      filter_samples[j][i][2] = filter_samples[j][i][1];
      filter_samples[j][i][1] = filter_samples[j][i][0];
  
      // 20 Hz cutoff frequency @ 100 Hz Sample Rate
      filter_samples[j][i][0] = filter_samples[j][i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[j][i][2] + \
        0.20657208382614792f*(filter_samples[j][i][3] + 2*filter_samples[j][i][4] + filter_samples[j][i][5]);   

      sample[i] = filter_samples[j][i][0];
    }
}

void deadzone_filter(float * sample)
{
  static float prev_sample[DEVICE_NUM][2];
  float dead_zone = 0.5f;

  for(uint8_t i=0; i<2; i++)
  {
    if(fabs(sample[i]-prev_sample[current_device_id][i]) > dead_zone)
      prev_sample[current_device_id][i] = sample[i];
    else
      sample[i] = prev_sample[current_device_id][i];
  }
}

void ads_data_callback(float * sample)
{
  // Low pass IIR filter
  signal_filter(sample);

  // Deadzone filter
  deadzone_filter(sample);
  
  ang[current_device_id][0] = sample[0];
  ang[current_device_id][1] = sample[1];
  
  newData |= 0x01<<current_device_id;
}

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_temp);
  sensor_reading_msg.data = (float*)malloc(sizeof(float) * DEVICE_NUM * 2);
  sensor_reading_msg.data_length = DEVICE_NUM * 2;

  

  delay(1000);

  nh.loginfo("Initializing Two Axis sensor");
  
  ads_init_t init;

  init.sps = ADS_100_HZ;
  init.ads_sample_callback = &ads_data_callback;
  init.reset_pin = ADS_RESET_PIN;                 // Pin connected to ADS reset line
  init.datardy_pin = ADS_INTERRUPT_PIN;           // Pin connected to ADS data ready interrupt

  // Initialize ADS hardware abstraction layer, and set the sample rate
  uint8_t device_no = 0;
  for(device_no;device_no<DEVICE_NUM;++device_no){
   
    ads_two_axis_select_device(0);
    int ret_val = ads_two_axis_init(&init);
    delay(100); 
    if(ret_val == ADS_OK)
    {
      nh.loginfo("Two Axis ADS initialization succeeded");
    }
    else
    {      
      char msg_str[80];
      sprintf(msg_str, "Errors! Sensor %d initialization failed with reason: %d", device_no, ret_val);
      nh.logerror(msg_str);
    }

  }
  
  // Start reading data!
  ads_two_axis_run(true, current_device_id);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(seq) {
    newData |= 0x01 << current_device_id;
    ads_two_axis_run(false, current_device_id);
    if(current_device_id == 0) {      
      current_device_id = 1;
    } else if(current_device_id == 1) {
      current_device_id = 0;
    }
    
    ads_two_axis_run(true, current_device_id);
  }
  if(newData& 0x0f == 0x0f)
  {
    nh.loginfo("Got all sensors' readings");
    newData = 0x00;
    uint8_t ang_idx = 0;
    for(ang_idx = 0; ang_idx < DEVICE_NUM*2; ++ang_idx) {
      sensor_reading_msg.data[ang_idx] = ang[ang_idx/2][ang_idx%2];
    }

      pub_temp.publish(&sensor_reading_msg);
    
    
  }
  nh.spinOnce();
//  if(Serial.available())
//  {
//    parse_serial_port();
//  }
}

/* Function parses received characters from the COM port for commands */
void parse_serial_port(void)
{
//    char key = Serial.read();
//    
//    if(key == '0')
//      ads_two_axis_calibrate(ADS_CALIBRATE_FIRST, 0);
//    else if(key == 'f')
//      ads_two_axis_calibrate(ADS_CALIBRATE_FLAT, 90);
//    else if(key == 'p')
//      ads_two_axis_calibrate(ADS_CALIBRATE_PERP, 90);
//    else if(key == 'c')
//      ads_two_axis_calibrate(ADS_CALIBRATE_CLEAR, 0);
//    else if(key == 'r')
//      ads_two_axis_run(true, current_device_id);
//    else if(key == 's')
//      ads_two_axis_run(false,current_device_id);
//    else if(key == 'f')
//      ads_two_axis_set_sample_rate(ADS_200_HZ);
//    else if(key == 'u')
//      ads_two_axis_set_sample_rate(ADS_10_HZ);
//    else if(key == 'n')
//      ads_two_axis_set_sample_rate(ADS_100_HZ);
//    else if(key == 'z') {
//      ads_two_axis_update_device_address(1,(uint8_t)0x14);
//    }
//    else if(key == '1') {     
//      if(current_device_id != 0){
//         ads_two_axis_run(false, current_device_id);
//      }
//        
//      current_device_id = 0;
//      ads_two_axis_run(true, current_device_id);
//    }
//    else if(key == '2') {
//      if(current_device_id != 1){
//         ads_two_axis_run(false, current_device_id);
//      }        
//      current_device_id = 1;
//      ads_two_axis_run(true, current_device_id);
//    }
    
}
