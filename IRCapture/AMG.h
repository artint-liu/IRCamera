//low range of the sensor (this will be blue on the screen)
#define MINTEMP 28

//high range of the sensor (this will be red on the screen)
#define MAXTEMP 40

extern uint16_t camColors[];

Adafruit_AMG88xx amg;
unsigned long delayTime;

#define AMG_COLS 8
#define AMG_ROWS 8
float pixels[AMG_COLS * AMG_ROWS];

#define INTERPOLATED_COLS 24
#define INTERPOLATED_ROWS 24