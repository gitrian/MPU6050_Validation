#include <Arduino_impl.h>
#include <math.h>
#define PL(x) Serial.println(x)
#define P(x) Serial.print(x)

#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>

volatile bool dataReady = false;
void interrupt(){
  dataReady = true;
}

double a, b, c, d, e, x, y, z = 0;
long t1, t2, t3 = 0;

unsigned short gyro_rate, gyro_fsr;
unsigned char accel_fsr;
float gyro_sens;
unsigned short accel_sens;
unsigned short rate;
void setup() {
  Serial.begin(115200);
  while(!Serial);
  struct int_param_s params;
  params.pin = 2;
  params.cb = interrupt;
  PL(mpu_init(&params));
  PL(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
  PL(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));
  //PL(mpu_set_gyro_fsr(10));
  //PL(mpu_set_accel_fsr(4));
  PL(mpu_set_sample_rate(50));
  //PL(mpu_set_lpf(42));

  PL(dmp_load_motion_driver_firmware());
  PL(mpu_set_dmp_state(1));
  PL(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO));
  PL(dmp_enable_6x_lp_quat(1));
  PL(dmp_enable_gyro_cal(1));
  PL(dmp_set_fifo_rate(50));
}

short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];
unsigned long sensor_timestamp;

void loop() {
  if(dataReady){
    if (!more) dataReady=false;
    dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

    a=quat[0]/((double)pow(2, 30));
    b=quat[1]/((double)pow(2, 30));
    c=quat[2]/((double)pow(2, 30));
    d=quat[3]/((double)pow(2, 30));

    e=a*a+b*b+c*c+d*d;

    x=atan(2*(a*b+c*d)/(a*a-b*b-c*c+d*d));
    y=-asin(2*(b*d-a*c));
    z=atan(2*(a*d+b*c)/(a*a+b*b-c*c-d*d));

    x=(x/(2*M_PI))*360;
    y=(y/(2*M_PI))*360;
    z=(z/(2*M_PI))*360;

    Serial.print(x);Serial.print("  ");Serial.print(y);Serial.print("  ");Serial.print(z);Serial.print("  ");Serial.print(e);
    PL();
  }
}
