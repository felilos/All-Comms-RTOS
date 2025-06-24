//This project will in an RTOS manner combine the ability to read a gyroscope for pitch and roll,
//display the pitch and roll on an LCD
//also have the ability to receive a BLE signal, switching an LED on and off
//ALL concurrently

#include "mbed.h"
#include <cmath> //match required for pitch and roll processing
#include <cctype> //using for ble to perform charater checks

// 74HC595 control pins on NUCLEO-F401RE
DigitalOut sr_data(PA_7);  // DS (Serial Data Input)
DigitalOut sr_clock(PA_5);  // SHCP (Shift Clock)
DigitalOut sr_latch(PB_6);  // STCP (Storage Latch Clock)
UnbufferedSerial ble(PA_9, PA_10, 115200); //BLE UART
I2C i2c(PB_9, PB_8); // SDA, SCL for gyro
BufferedSerial pc(USBTX, USBRX, 9600);    // monitor ble commands over USB serial

DigitalOut LED(PB_5); //LED for ble comms

#define LCD_RS (1 << 4)  // RS → Q4 for LCD
#define LCD_E  (1 << 5)  // E  → Q5 for LCD
#define MPU_ADDRESS 0x68 << 1  // mbed uses 8-bit address
#define M_PI 3.14159265358979323846f //define PI as float

float ax, ay, az, gx, gy, gz;
float pitch = 0, roll = 0;
char buffer[64];
char ble_buffer[64];
unsigned char c;

Mutex buffer_mutex; //to sync access to char buffer
Mutex pc_mutex; // using this to lock out the pc serial port for writing

// Shift out a byte to 74HC595
void shift_out(uint8_t val) {
    for (int i = 7; i >= 0; i--) {
        sr_data = (val >> i) & 0x01;
        sr_clock = 1;
        wait_us(1);               // Short enable pulse
        sr_clock = 0;
        wait_us(1);               // Short enable pulse
    }
    sr_latch = 1;
    sr_latch = 0;
}
// Pulse the Enable (E) pin with given data
void lcd_pulse(uint8_t data) {
    shift_out(data | LCD_E);  // E = 1
    wait_us(1);               // Short enable pulse
    shift_out(data & ~LCD_E); // E = 0
    wait_us(50);              // Wait for LCD to latch
}
// Send a 4-bit nibble
void lcd_send_nibble(uint8_t nibble, bool rs) {
    uint8_t data = (nibble & 0x0F);   // Put nibble on Q0–Q3 (D4–D7)
    if (rs) data |= LCD_RS;          // RS high for data
    lcd_pulse(data);
}
// Send full 8-bit command/data in 4-bit mode
void lcd_send_byte(uint8_t byte, bool rs) {
    lcd_send_nibble(byte >> 4, rs);   // High nibble first
    lcd_send_nibble(byte & 0x0F, rs); // Then low nibble
}
// Send LCD command
void lcd_cmd(uint8_t cmd) {
    lcd_send_byte(cmd, false);
}
// Send LCD string
void lcd_data(const char* str) { //points to start of string in bracket attribute
    while (*str) {
        lcd_send_byte((uint8_t)*str, true);  // Send each character as a byte
        str++;  // Move to the next character in the string
        wait_us(2);
        //lcd_send_byte(0x14, false);
        //wait_us(2);
    }
}
// Initialize LCD in 4-bit mode
void lcd_init() {
    thread_sleep_for(40); // Wait after power-up
    lcd_send_nibble(0x03, false); thread_sleep_for(5);
    lcd_send_nibble(0x03, false); wait_us(150);
    lcd_send_nibble(0x03, false); wait_us(150);
    lcd_send_nibble(0x02, false); wait_us(150); // Switch to 4-bit mode
    lcd_cmd(0x28); // Function set: 2 lines, 5x8 font
    lcd_cmd(0x08); // Display OFF
    lcd_cmd(0x01); // Clear display
    thread_sleep_for(2);
    lcd_cmd(0x06); // Entry mode: move right
    lcd_cmd(0x0C); // Display ON, cursor OFF
}



// Helper for reading 16-bit signed value
int16_t read16(uint8_t reg) {
    char data[2];
    i2c.write(MPU_ADDRESS, (char*)&reg, 1, true);
    i2c.read(MPU_ADDRESS, data, 2);
    return (int16_t)((data[0] << 8) | data[1]);
}

// MPU init
void mpu_init() {
    char data[2] = {0x6B, 0x00}; // Wake up MPU
    i2c.write(MPU_ADDRESS, data, 2);
}

// Read accel and gyro
void read_mpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    ax = read16(0x3B) / 16384.0f;
    ay = read16(0x3D) / 16384.0f;
    az = read16(0x3F) / 16384.0f;

    gx = read16(0x43) / 131.0f;
    gy = read16(0x45) / 131.0f;
    gz = read16(0x47) / 131.0f;
}

// Orientation calc (complementary filter)
void compute_orientation(float ax, float ay, float az, float gx, float gy, float dt, float &pitch, float &roll) {
    float accel_pitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 180 / M_PI;
    float accel_roll  = atan2f(-ax, az) * 180 / M_PI;

    pitch = 0.98f * (pitch + gx * dt) + 0.02f * accel_pitch;
    roll  = 0.98f * (roll + gy * dt) + 0.02f * accel_roll;
}

Thread thread1; //gyro thread
void thread_gyro(){
        
     while (1) {
        Timer t;
        t.start();
        read_mpu(ax, ay, az, gx, gy, gz);
        float dt = t.read(); // seconds
        t.reset();
        compute_orientation( ax,  ay, az, gx, gy, dt,  pitch, roll);
        //len = sprintf(buffer, "P:%.2f R:%.2f", pitch,roll);
        buffer_mutex.lock();
        sprintf(buffer, "P:%.2f R:%.2f", pitch,roll);
        buffer_mutex.unlock();
        ThisThread::sleep_for(10ms);
    }
}

Thread thread2; //display pitch & roll on LCD
void thread_lcd(){
    while(true){
        lcd_init();
        ThisThread::sleep_for(1ms);
        buffer_mutex.lock();
        lcd_data(buffer);  // find out here how to print the char array of buffer
        buffer_mutex.unlock();



    }
}

Thread thread3; //for the bluetooth switching an LED
void thread_ble() {
    while (true) {
        if (ble.read(&c, 1) > 0) {
            pc_mutex.lock();
            pc.write(&c, 1);  // write the raw character to USB serial
            pc_mutex.unlock();

            if (c == 'O' || c == '0') {
                LED = 0; // Off (assuming active-low LED)
            } else {
                LED = 1; // On
            }
        }
    }
}

// main() runs in its own thread in the OS
int main()
{
    lcd_init();
    LED = 1;
    ble.set_blocking(false);
    pc.set_blocking(true);
    pc_mutex.lock();
    pc.write("Listening on USART1 (PA9/PA10)...\r\n", 36);
    pc_mutex.unlock();
    thread1.start(thread_gyro);
    thread2.start(thread_lcd);
    thread3.start(thread_ble);
    while (true) {
        ThisThread::sleep_for(1s);

    }
}

