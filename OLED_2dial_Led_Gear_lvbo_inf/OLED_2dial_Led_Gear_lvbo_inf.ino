#include <U8g2lib.h>
#include <math.h>

// 初始化OLED对象
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

// 引脚定义
const int NTC_PIN = 34;        // NTC模拟输入引脚
const int FAN_PWM_PIN = 13;    // 风扇PWM控制引脚

// 按钮引脚定义
const int BUTTON1_PIN = 32;    // 按钮1引脚
const int BUTTON2_PIN = 33;    // 按钮2引脚
const int BUTTON3_PIN = 25;    // 按钮3引脚

// LED引脚定义
const int LED1_PIN = 26;       // LED1引脚
const int LED2_PIN = 27;       // LED2引脚
const int LED3_PIN = 14;       // LED3引脚
const int LED4_PIN = 15;       // LED4引脚

// 旋转挡位开关引脚定义
const int GEAR_OFF_PIN = 19;   // OFF挡引脚
const int GEAR_AUTO_PIN = 18;  // AUTO挡引脚
const int GEAR_LO_PIN = 5;     // LO挡引脚
const int GEAR_HI_PIN = 17;    // HI挡引脚
const int GEAR_MAX_PIN = 16;   // MAX挡引脚

// NTC参数
const int SERIES_RESISTOR = 10000;  // 10K串联电阻
const int NTC_NOMINAL = 10500;      // NTC在25°C时的阻值
const int TEMPERATURE_NOMINAL = 25; // 标称温度
const int B_COEFFICIENT = 3950;     // B系数

// 温度控制参数
const float MIN_TEMP = 40;   // 最低工作温度
const float MAX_TEMP = 60;   // 最高工作温度

// 风扇表盘参数
const int FAN_DIAL_X = 48;         // 风扇表盘X坐标
const int FAN_DIAL_Y = 6;          // 风扇表盘Y坐标
const int FAN_DIAL_WIDTH = 69;     // 风扇表盘宽度
const int FAN_DIAL_HEIGHT = 53;    // 风扇表盘高度
const int FAN_NEEDLE_CENTER_X = 79; // 风扇指针中心X
const int FAN_NEEDLE_CENTER_Y = 37; // 风扇指针中心Y
const int FAN_NEEDLE_LENGTH = 37;   // 风扇指针长度

// 风扇角度范围（弧度）
const float FAN_MIN_ANGLE = 150 * M_PI / 180.0;  // 0%时的角度（150度）
const float FAN_MAX_ANGLE = 360 * M_PI / 180.0;   // 100%时的角度（360度）

// 温度表盘参数
const int TEMP_DIAL_X = 48;         // 温度表盘X坐标
const int TEMP_DIAL_Y = 11;         // 温度表盘Y坐标
const int TEMP_DIAL_WIDTH = 63;     // 温度表盘宽度
const int TEMP_DIAL_HEIGHT = 45;    // 温度表盘高度
const int TEMP_NEEDLE_CENTER_X = 79; // 温度指针中心X
const int TEMP_NEEDLE_CENTER_Y = 42; // 温度指针中心Y
const int TEMP_NEEDLE_TOTAL_LENGTH = 36;   // 温度指针总长度
const int TEMP_NEEDLE_VISIBLE_LENGTH = 14; // 温度指针可见长度

// 温度角度范围（弧度）
const float TEMP_MIN_ANGLE = 180 * M_PI / 180.0;  // 10℃时的角度（180度）
const float TEMP_MAX_ANGLE = 360 * M_PI / 180.0;  // 70℃时的角度（360度）
const float TEMP_MIN_VALUE = 10.0;  // 最小温度值
const float TEMP_MAX_VALUE = 70.0;  // 最大温度值

// 温度滤波参数
const int TEMP_FILTER_SAMPLES = 10;  // 滤波采样点数
const float TEMP_FILTER_ALPHA = 0.2; // 一阶低通滤波系数

// 风扇PWM滤波参数
const int PWM_FILTER_SAMPLES = 5;    // PWM滤波采样点数
const float PWM_FILTER_ALPHA = 0.3;  // PWM一阶低通滤波系数

// 风扇表盘位图数据
static const unsigned char fan_dial_bitmap[] = {
0x00,0x00,0x00,0xFC,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,0x03,0xE0,0x01,0x00,
0x00,0x00,0x00,0x00,0x38,0x00,0x00,0x0E,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,
0x30,0x00,0x00,0x00,0x00,0x80,0x01,0x00,0x00,0xC0,0x00,0x00,0x00,0x00,0x40,0x00,
0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,
0x08,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x10,0x00,
0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,
0x40,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x40,0x00,0x00,0x00,
0x00,0x00,0x00,0x01,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x20,0x00,
0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,
0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x80,
0x09,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x60,0x08,0x00,0x04,0x00,0x00,0x00,0x00,
0x00,0x40,0x08,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x40,0x10,0x00,0x02,0x00,0x00,
0x00,0x00,0x00,0x80,0x10,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x80,0x10,0x00,0x02,
0x00,0x00,0x00,0x00,0x00,0x80,0x20,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x21,
0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x21,0x00,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x21,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x00,0x42,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x42,0x00,0x01,0x00,
0x00,0x00,0x00,0x00,0x00,0x42,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0xFE,0xFF,0xFF,0xFF,0x1F,0x01,0x00,0x00,
0x00,0x02,0x00,0x00,0x00,0x10,0x01,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x10,0x02,
0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x10,0x02,0x00,0x00,0x00,0x02,0x00,0x00,0x00,
0x10,0x02,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x10,0x02,0x00,0x00,0x00,0x02,0x00,
0x00,0x00,0x10,0x04,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x10,0x04,0x00,0x00,0x00,
0x02,0x00,0x00,0x00,0x10,0x08,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x10,0x08,0x00,
0x00,0x00,0x02,0x00,0x00,0x00,0x10,0x10,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x10,
0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x02,0x00,0x00,
0x00,0x10,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x02,
0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0xFE,0xFF,0xFF,0xFF,0x1F
};

// 温度表盘位图数据
static const unsigned char temp_dial_bitmap[] = {
0x00,0x00,0x00,0xFC,0x1F,0x00,0x00,0x00,0x00,0x00,0xC0,0x83,0xE0,0x01,0x00,0x00,
0x00,0x00,0x38,0x80,0x00,0x0E,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x30,0x00,0x00,
0x00,0x80,0x01,0x00,0x00,0xC0,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x01,0x00,
0x00,0x30,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x08,0x00,
0x00,0x04,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x20,0x00,
0x00,0x01,0x00,0x00,0x00,0x00,0x40,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x80,0x00,
0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x04,
0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x08,
0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x10,
0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x30,
0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x20,
0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x40,
0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x40,
0x01,0xC0,0xFF,0xFF,0xFF,0xFF,0x03,0x40,0x01,0x40,0x00,0x00,0x00,0x00,0x02,0x40,
0x01,0x40,0x00,0x00,0x00,0x00,0x02,0x40,0x07,0x40,0x00,0x00,0x00,0x00,0x02,0x70,
0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,
0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,
0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,
0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,
0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,
0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x02,0x00,
0x00,0xC0,0xFF,0xFF,0xFF,0xFF,0x03,0x00
};

// 全局变量
int displayMode = 2; // 0:关闭显示, 1:风扇速度表盘, 2:温度表盘, 3:CPU/GPU信息, 4:网络硬盘信息

// 按钮状态变量
bool button1State = HIGH;
bool button1LastState = HIGH;
bool button2State = HIGH;
bool button2LastState = HIGH;
bool button3State = HIGH;
bool button3LastState = HIGH;

// 温度相关变量
float currentTemperature = 0.0;
float filteredTemperature = 0.0; // 滤波后的温度

// 风扇相关变量
float fanSpeedPercent = 0.0;
float filteredFanSpeedPercent = 0.0; // 滤波后的风扇速度百分比
float targetFanSpeedPercent = 0.0;   // 目标风扇速度（挡位设定）

// 温度滤波数组
float temperatureSamples[TEMP_FILTER_SAMPLES];
int temperatureSampleIndex = 0;

// 风扇速度滤波变量
float fanSpeedSamples[PWM_FILTER_SAMPLES];
int fanSpeedSampleIndex = 0;

// PWM通道配置
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 25000;
const int PWM_RESOLUTION = 8; // 8位分辨率，范围0-255

// ==================== 串口数据接收相关 ====================
// 上位机数据结构
struct PCData {
  float cpuTemp;          // CPU温度 (°C)
  float cpuLoad;          // CPU负载 (%)
  float ramUsage;         // 内存使用率 (%)
  float gpuTemp;          // GPU温度 (°C)
  float gpuLoad;          // GPU负载 (%)
  float downloadSpeed;    // 下载速度 (Mbps)
  float uploadSpeed;      // 上传速度 (Mbps)
  float ssdReadRate;      // SSD读取速率 (MB/s)
  float ssdWriteRate;     // SSD写入速率 (MB/s)
};

PCData pcData = {0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0};
bool pcDataValid = false; // 数据是否有效
unsigned long lastPCDataTime = 0; // 最后接收数据时间
const unsigned long DATA_TIMEOUT = 5000; // 5秒超时

// 串口接收缓冲区
const int RX_BUFFER_SIZE = 128;
char rxBuffer[RX_BUFFER_SIZE];
int rxBufferIndex = 0;
bool receivingData = false;

void setup() {
  // 初始化串口 - 用于接收上位机数据
  Serial.begin(9600);
  
  // 初始化OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  
  // 初始化引脚
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  
  pinMode(GEAR_OFF_PIN, INPUT_PULLUP);
  pinMode(GEAR_AUTO_PIN, INPUT_PULLUP);
  pinMode(GEAR_LO_PIN, INPUT_PULLUP);
  pinMode(GEAR_HI_PIN, INPUT_PULLUP);
  pinMode(GEAR_MAX_PIN, INPUT_PULLUP);
  
  // 初始化风扇PWM (ESP32使用ledc)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN_PWM_PIN, PWM_CHANNEL);
  
  // 初始化温度滤波数组
  for(int i = 0; i < TEMP_FILTER_SAMPLES; i++) {
    temperatureSamples[i] = 0.0;
  }
  
  // 初始化风扇速度滤波数组
  for(int i = 0; i < PWM_FILTER_SAMPLES; i++) {
    fanSpeedSamples[i] = 0.0;
  }
  
  // 读取初始温度并填充滤波数组
  float initialTemp = readRawTemperature();
  for(int i = 0; i < TEMP_FILTER_SAMPLES; i++) {
    temperatureSamples[i] = initialTemp;
  }
  filteredTemperature = initialTemp;
  currentTemperature = initialTemp;
  
  // 初始化风扇速度
  filteredFanSpeedPercent = 0.0;
  targetFanSpeedPercent = 0.0;
  
  // 上电后显示温度界面
  displayMode = 2; // 温度显示模式
  updateDisplay();
  updateLEDs();
  
  Serial.println("ESP32 System initialized");
  Serial.print("Initial Temperature: ");
  Serial.println(initialTemp);
  Serial.println("Waiting for PC data...");
}

// 读取原始温度函数
float readRawTemperature() {
  int adcValue = analogRead(NTC_PIN);
  float voltage = adcValue * (3.3 / 4095.0); // ESP32 ADC为12位，0-4095
  float resistance = SERIES_RESISTOR * (3.3 / voltage - 1.0);
  
  // 使用Steinhart-Hart方程计算温度
  float steinhart = resistance / NTC_NOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                     // ln(R/Ro)
  steinhart /= B_COEFFICIENT;                     // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                    // 倒数
  steinhart -= 273.15;                            // 转换为摄氏度
  
  return steinhart;
}

// 移动平均滤波
float movingAverageFilter(float newSample, float samples[], int &sampleIndex, int sampleCount) {
  // 将新采样值存入数组
  samples[sampleIndex] = newSample;
  sampleIndex = (sampleIndex + 1) % sampleCount;
  
  // 计算平均值
  float sum = 0;
  for(int i = 0; i < sampleCount; i++) {
    sum += samples[i];
  }
  
  return sum / sampleCount;
}

// 一阶低通滤波
float lowPassFilter(float newSample, float lastFiltered, float alpha) {
  if(lastFiltered == 0.0) {
    return newSample;
  }
  
  float filtered = alpha * newSample + (1 - alpha) * lastFiltered;
  return filtered;
}

// 组合滤波：移动平均+一阶低通（用于温度）
float combinedTemperatureFilter(float newSample) {
  static float lastFilteredTemp = 0.0;
  
  // 先用移动平均滤波
  float maFiltered = movingAverageFilter(newSample, temperatureSamples, temperatureSampleIndex, TEMP_FILTER_SAMPLES);
  
  // 再用一阶低通滤波
  float filtered = lowPassFilter(maFiltered, lastFilteredTemp, TEMP_FILTER_ALPHA);
  lastFilteredTemp = filtered;
  
  return filtered;
}

// 风扇速度滤波（用于PWM输出）
float filterFanSpeed(float newSpeed) {
  static float lastFilteredSpeed = 0.0;
  
  // 先用移动平均滤波
  float maFiltered = movingAverageFilter(newSpeed, fanSpeedSamples, fanSpeedSampleIndex, PWM_FILTER_SAMPLES);
  
  // 再用一阶低通滤波
  float filtered = lowPassFilter(maFiltered, lastFilteredSpeed, PWM_FILTER_ALPHA);
  lastFilteredSpeed = filtered;
  
  return filtered;
}

// 计算风扇速度百分比（基于滤波后的温度）
float calculateFanSpeedFromTemperature(float temperature) {
  if (temperature <= MIN_TEMP) {
    return 0.0;
  } else if (temperature >= MAX_TEMP) {
    return 100.0;
  } else {
    // 线性插值
    return (temperature - MIN_TEMP) / (MAX_TEMP - MIN_TEMP) * 100.0;
  }
}

// 控制风扇
void controlFan(float speedPercent) {
  // 对速度百分比进行限制
  if (speedPercent < 0.0) speedPercent = 0.0;
  if (speedPercent > 100.0) speedPercent = 100.0;
  
  int pwmValue = (speedPercent / 100.0) * 255;
  ledcWrite(PWM_CHANNEL, pwmValue);
  
  // 更新当前风扇速度百分比
  fanSpeedPercent = speedPercent;
}

// 获取旋转开关档位
int getGearPosition() {
  if (digitalRead(GEAR_OFF_PIN) == LOW) return 1;   // OFF挡
  if (digitalRead(GEAR_AUTO_PIN) == LOW) return 2;  // AUTO挡
  if (digitalRead(GEAR_LO_PIN) == LOW) return 3;    // LO挡
  if (digitalRead(GEAR_HI_PIN) == LOW) return 4;    // HI挡
  if (digitalRead(GEAR_MAX_PIN) == LOW) return 5;   // MAX挡
  return 0; // 无挡位
}

// 根据挡位控制风扇
void controlFanByGear(int gear) {
  switch(gear) {
    case 1: // OFF挡
      targetFanSpeedPercent = 0.0;
      digitalWrite(LED4_PIN, HIGH);
      break;
    case 2: // AUTO挡 - 使用滤波后的温度计算风扇速度
      targetFanSpeedPercent = calculateFanSpeedFromTemperature(filteredTemperature);
      digitalWrite(LED4_PIN, LOW);
      break;
    case 3: // LO挡
      targetFanSpeedPercent = 20.0;
      digitalWrite(LED4_PIN, LOW);
      break;
    case 4: // HI挡
      targetFanSpeedPercent = 50.0;
      digitalWrite(LED4_PIN, LOW);
      break;
    case 5: // MAX挡
      targetFanSpeedPercent = 100.0;
      digitalWrite(LED4_PIN, LOW);
      break;
    default:
      targetFanSpeedPercent = 0.0;
      digitalWrite(LED4_PIN, LOW);
      break;
  }
  
  // 对目标风扇速度进行滤波
  filteredFanSpeedPercent = filterFanSpeed(targetFanSpeedPercent);
  
  // 控制风扇
  controlFan(filteredFanSpeedPercent);
}

// ==================== 串口数据处理函数 ====================
void readSerialData() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '<') {
      // 开始接收数据
      receivingData = true;
      rxBufferIndex = 0;
      rxBuffer[rxBufferIndex++] = c;
    }
    else if (c == '>' && receivingData) {
      // 结束接收数据
      if (rxBufferIndex < RX_BUFFER_SIZE - 1) {
        rxBuffer[rxBufferIndex++] = c;
        rxBuffer[rxBufferIndex] = '\0'; // 添加字符串结束符
        
        // 解析数据
        parseSerialData();
        
        receivingData = false;
        rxBufferIndex = 0;
      }
    }
    else if (receivingData && rxBufferIndex < RX_BUFFER_SIZE - 1) {
      // 接收数据字符
      rxBuffer[rxBufferIndex++] = c;
    }
  }
}

// 解析串口数据
void parseSerialData() {
  // 检查数据格式是否有效
  if (rxBuffer[0] != '<' || rxBuffer[rxBufferIndex-1] != '>') {
    Serial.println("Invalid data format");
    return;
  }
  
  // 去掉帧头和帧尾
  char dataStr[RX_BUFFER_SIZE];
  int dataLen = rxBufferIndex - 2; // 去掉'<'和'>'
  strncpy(dataStr, rxBuffer + 1, dataLen);
  dataStr[dataLen] = '\0';
  
  // 分割字符串
  char *token = strtok(dataStr, ",");
  int index = 0;
  
  while (token != NULL && index < 9) {
    float value = atof(token);
    
    switch(index) {
      case 0: pcData.cpuTemp = value; break;
      case 1: pcData.cpuLoad = value; break;
      case 2: pcData.ramUsage = value; break;
      case 3: pcData.gpuTemp = value; break;
      case 4: pcData.gpuLoad = value; break;
      case 5: pcData.downloadSpeed = value; break;
      case 6: pcData.uploadSpeed = value; break;
      case 7: pcData.ssdReadRate = value; break;
      case 8: pcData.ssdWriteRate = value; break;
    }
    
    token = strtok(NULL, ",");
    index++;
  }
  
  if (index == 9) {
    // 数据解析成功
    pcDataValid = true;
    lastPCDataTime = millis();
    
    // 调试输出
    Serial.print("PC Data Received: CPU=");
    Serial.print(pcData.cpuTemp);
    Serial.print("C/");
    Serial.print(pcData.cpuLoad);
    Serial.print("%, GPU=");
    Serial.print(pcData.gpuTemp);
    Serial.print("C/");
    Serial.print(pcData.gpuLoad);
    Serial.println("%");
  } else {
    Serial.print("Incomplete data, got ");
    Serial.print(index);
    Serial.println(" fields");
  }
}

// 检查数据超时
void checkDataTimeout() {
  if (pcDataValid && (millis() - lastPCDataTime > DATA_TIMEOUT)) {
    pcDataValid = false;
    Serial.println("PC data timeout");
  }
}

// 绘制数值（通用函数，用于风扇速度和温度）
void drawValue(float value, int x, int y, bool isTemperature = false) {
  char valueStr[10];
  
  if (value >= 99.95) {
    strcpy(valueStr, "100");
    u8g2.setFont(u8g2_font_9x15_tf);
    u8g2.drawStr(x, y, valueStr);
  } else {
    dtostrf(value, 4, 1, valueStr);
    
    char intPart[5];
    char decPart[5];
    
    char *dotPos = strchr(valueStr, '.');
    if (dotPos != NULL) {
      int intLen = dotPos - valueStr;
      strncpy(intPart, valueStr, intLen);
      intPart[intLen] = '\0';
      strcpy(decPart, dotPos + 1);
    } else {
      strcpy(intPart, valueStr);
      strcpy(decPart, "0");
    }
    
    u8g2.setFont(u8g2_font_9x15_tf);
    int intWidth = u8g2.getStrWidth(intPart);
    
    u8g2.drawStr(x, y, intPart);
    
    int dotX = x + intWidth + 1;
    int dotY = y + 10;
    u8g2.drawBox(dotX, dotY, 2, 2);
    
    //u8g2.setFont(u8g2_font_7x13_tf);
    //u8g2.drawStr(dotX + 4, y + 2, decPart);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(dotX + 4, y + 4, decPart);
  }
}

// 绘制风扇表盘和指针
void drawFanDial(float speedPercent) {
  u8g2.firstPage();
  do {
    // 绘制风扇表盘位图
    u8g2.drawXBMP(FAN_DIAL_X, FAN_DIAL_Y, FAN_DIAL_WIDTH, FAN_DIAL_HEIGHT, fan_dial_bitmap);
    
    // 计算风扇指针角度
    float fanAngle = FAN_MIN_ANGLE + (speedPercent / 100.0) * (FAN_MAX_ANGLE - FAN_MIN_ANGLE);
    
    // 计算风扇指针终点坐标
    int fanNeedleEndX = FAN_NEEDLE_CENTER_X + FAN_NEEDLE_LENGTH * cos(fanAngle);
    int fanNeedleEndY = FAN_NEEDLE_CENTER_Y + FAN_NEEDLE_LENGTH * sin(fanAngle);
    
    // 绘制风扇指针
    u8g2.drawLine(FAN_NEEDLE_CENTER_X, FAN_NEEDLE_CENTER_Y, fanNeedleEndX, fanNeedleEndY);
    
    // 绘制固定文本
    u8g2.setFont(u8g2_font_8x13_tf);
    u8g2.drawStr(12, 20, "N1");
    u8g2.drawStr(15, 34, "%");
    
    // 绘制风扇转速数值
    drawValue(speedPercent, 85, 44);
    
  } while (u8g2.nextPage());
}

// 绘制温度表盘和指针
void drawTemperatureDial(float temperature) {
  u8g2.firstPage();
  do {
    // 绘制温度表盘位图
    u8g2.drawXBMP(TEMP_DIAL_X, TEMP_DIAL_Y, TEMP_DIAL_WIDTH, TEMP_DIAL_HEIGHT, temp_dial_bitmap);
    
    // 计算温度指针角度
    float tempAngle;
    if (temperature <= TEMP_MIN_VALUE) {
      tempAngle = TEMP_MIN_ANGLE;
    } else if (temperature >= TEMP_MAX_VALUE) {
      tempAngle = TEMP_MAX_ANGLE;
    } else {
      tempAngle = TEMP_MIN_ANGLE + ((temperature - TEMP_MIN_VALUE) / (TEMP_MAX_VALUE - TEMP_MIN_VALUE)) * (TEMP_MAX_ANGLE - TEMP_MIN_ANGLE);
    }
    
    // 计算温度指针的起点和终点
    int tempNeedleStartX = TEMP_NEEDLE_CENTER_X + (TEMP_NEEDLE_TOTAL_LENGTH - TEMP_NEEDLE_VISIBLE_LENGTH) * cos(tempAngle);
    int tempNeedleStartY = TEMP_NEEDLE_CENTER_Y + (TEMP_NEEDLE_TOTAL_LENGTH - TEMP_NEEDLE_VISIBLE_LENGTH) * sin(tempAngle);
    
    int tempNeedleEndX = TEMP_NEEDLE_CENTER_X + TEMP_NEEDLE_TOTAL_LENGTH * cos(tempAngle);
    int tempNeedleEndY = TEMP_NEEDLE_CENTER_Y + TEMP_NEEDLE_TOTAL_LENGTH * sin(tempAngle);
    
    // 绘制温度指针
    u8g2.drawLine(tempNeedleStartX, tempNeedleStartY, tempNeedleEndX, tempNeedleEndY);
    
    // 绘制固定文本
    u8g2.setFont(u8g2_font_8x13_tf);
    u8g2.drawStr(9, 20, "EGT");
    u8g2.drawStr(18, 34, "C");
    u8g2.drawCircle(16, 36, 1);

    // 绘制温度数值
    drawValue(temperature, 66, 41, true);
    
  } while (u8g2.nextPage());
}

// 绘制CPU/GPU数据界面
void drawCPUGPUScreen() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13_tf);
    
    // 绘制CPU数据
    u8g2.drawStr(53, 8, "CPU");
    u8g2.drawStr(24, 13, "%");
    u8g2.drawStr(116, 13, "C");
    u8g2.drawCircle(114, 15, 1);

    u8g2.drawLine(47, 13, 35, 17);// /
    u8g2.drawLine(81, 13, 93, 17);// "\"

    // 绘制CPU负载
    if (pcDataValid) {
      char cpuLoadStr[10];
      sprintf(cpuLoadStr, "%.0f", pcData.cpuLoad);
      // 计算字符串宽度，右对齐到x=23位置（"%"在x=25）
      int strWidth = u8g2.getStrWidth(cpuLoadStr);
      u8g2.drawStr(23 - strWidth, 13, cpuLoadStr);
    } else {
      int strWidth = u8g2.getStrWidth("--");
      u8g2.drawStr(23 - strWidth, 13, "--");
    }           
    
    // 绘制CPU温度
    if (pcDataValid) {
      char cpuTempStr[10];
      sprintf(cpuTempStr, "%.0f", pcData.cpuTemp);
      u8g2.drawStr(96, 13, cpuTempStr);
    } else {
      u8g2.drawStr(96, 13, "--");
    }    

    // 绘制GPU数据
    u8g2.drawStr(53, 39, "GPU"); // y+11
    u8g2.drawStr(24, 44, "%");
    u8g2.drawStr(116, 44, "C");
    u8g2.drawCircle(114, 46, 1);

    u8g2.drawLine(47, 44, 35, 48); //  /
    u8g2.drawLine(81, 44, 93, 48); // "\"

    
    // 绘制GPU负载
    if (pcDataValid) {
      char gpuLoadStr[10];
      sprintf(gpuLoadStr, "%.0f", pcData.gpuLoad);
      // 计算字符串宽度，右对齐到x=23位置（"%"在x=25）
      int strWidth = u8g2.getStrWidth(gpuLoadStr);
      u8g2.drawStr(23 - strWidth, 44, gpuLoadStr);
    } else {
      int strWidth = u8g2.getStrWidth("--");
      u8g2.drawStr(23 - strWidth, 44, "--");
    }   
    
    // 绘制GPU温度
    if (pcDataValid) {
      char gpuTempStr[10];
      sprintf(gpuTempStr, "%.0f", pcData.gpuTemp);
      u8g2.drawStr(96, 44, gpuTempStr);
    } else {
      u8g2.drawStr(96, 44, "--");
    }    
    
  } while (u8g2.nextPage());
}

// 辅助函数：根据数值大小返回格式化字符串，并设置对应的小数点位置信息
const char* formatValueWithCondition(float value, int& intWidth, int& decimalWidth, bool& hasDecimal) {
  static char buffer[10];
  if (value < 100.0) {
    // 小于100时显示一位小数
    sprintf(buffer, "%.1f", value);
    
    // 找到小数点的位置
    char* dotPos = strchr(buffer, '.');
    if (dotPos) {
      // 计算整数部分和小数部分的宽度
      *dotPos = '\0'; // 临时分割字符串
      intWidth = u8g2.getStrWidth(buffer); // 整数部分宽度
      decimalWidth = u8g2.getStrWidth(dotPos + 1); // 小数部分宽度
      *dotPos = '.'; // 恢复原字符串
      hasDecimal = true;
    } else {
      intWidth = u8g2.getStrWidth(buffer);
      decimalWidth = 0;
      hasDecimal = false;
    }
  } else {
    // 大于等于100时显示整数
    sprintf(buffer, "%.0f", value);
    intWidth = u8g2.getStrWidth(buffer);
    decimalWidth = 0;
    hasDecimal = false;
  }
  return buffer;
}

// 绘制网络硬盘数据界面
void drawNetworkDiskScreen() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_8x13_tf);
    
    // 绘制网络数据（上半部分）
    u8g2.drawStr(53, 8, "D/U");
    
    // 绘制网络速度的斜线
    u8g2.drawLine(47, 13, 35, 17);  // 左边的斜线
    u8g2.drawLine(81, 13, 93, 17);  // 右边的斜线

    // 绘制下载速度（右对齐）
    if (pcDataValid) {
      int intWidth, decimalWidth;
      bool hasDecimal;
      const char* downloadStr = formatValueWithCondition(pcData.downloadSpeed, intWidth, decimalWidth, hasDecimal);
      
      if (hasDecimal && pcData.downloadSpeed < 100.0) {
        // 分开绘制整数部分和小数部分
        char intPart[10], decimalPart[10];
        sprintf(intPart, "%d", (int)pcData.downloadSpeed);
        sprintf(decimalPart, ".%d", (int)((pcData.downloadSpeed - (int)pcData.downloadSpeed) * 10));
        
        // 先绘制整数部分（大字体）
        int totalWidth = intWidth + decimalWidth;
        u8g2.drawStr(29 - totalWidth, 13, intPart);
        
        // 切换到小字体绘制小数部分
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(29 - totalWidth + intWidth, 15, decimalPart);
        
        // 恢复大字体
        u8g2.setFont(u8g2_font_8x13_tf);
      } else {
        // 直接绘制整数
        int strWidth = u8g2.getStrWidth(downloadStr);
        u8g2.drawStr(32 - strWidth, 13, downloadStr);
      }
    } else {
      int strWidth = u8g2.getStrWidth("--");
      u8g2.drawStr(32 - strWidth, 13, "--");
    }
    
    // 绘制上传速度（左对齐）
    if (pcDataValid) {
      int intWidth, decimalWidth;
      bool hasDecimal;
      const char* uploadStr = formatValueWithCondition(pcData.uploadSpeed, intWidth, decimalWidth, hasDecimal);
      
      if (hasDecimal && pcData.uploadSpeed < 100.0) {
        // 分开绘制整数部分和小数部分
        char intPart[10], decimalPart[10];
        sprintf(intPart, "%d", (int)pcData.uploadSpeed);
        sprintf(decimalPart, ".%d", (int)((pcData.uploadSpeed - (int)pcData.uploadSpeed) * 10));
        
        // 计算总宽度
        int totalWidth = intWidth + decimalWidth;
        
        // 对于10-99.9的范围，使用不同的对齐方式
        int startX;
        if (pcData.uploadSpeed >= 10.0 && pcData.uploadSpeed < 100.0) {
          // 10-99.9范围：向右移动，不超过原始位置
          startX = 112 - intWidth; // 整数部分从97-intWidth开始
          
          // 先绘制整数部分（大字体）
          u8g2.drawStr(startX, 13, intPart);
          
          // 切换到小字体绘制小数部分
          u8g2.setFont(u8g2_font_6x10_tf);
          u8g2.drawStr(startX + intWidth, 16, decimalPart);
        } else {
          // 小于10或大于等于100的情况使用原来的对齐方式
          startX = 112 - totalWidth;
          
          // 先绘制整数部分（大字体）
          u8g2.drawStr(startX, 13, intPart);
          
          // 切换到小字体绘制小数部分
          u8g2.setFont(u8g2_font_6x10_tf);
          u8g2.drawStr(startX + intWidth, 16, decimalPart);
        }
        
        // 恢复大字体
        u8g2.setFont(u8g2_font_8x13_tf);
      } else {
        // 直接绘制整数（>=100）
        //int strWidth = u8g2.getStrWidth(uploadStr);
        u8g2.drawStr(96, 13, uploadStr);
      }
    } else {
      u8g2.drawStr(96, 13, "--");
    }

    // 绘制硬盘数据（下半部分）
    u8g2.drawStr(53, 39, "R/W");
    
    // 绘制硬盘读写速度的斜线
    u8g2.drawLine(47, 44, 35, 48);  // 左边的斜线
    u8g2.drawLine(81, 44, 93, 48);  // 右边的斜线

    // 绘制读取速度（左对齐）
    if (pcDataValid) {
      int intWidth, decimalWidth;
      bool hasDecimal;
      const char* readStr = formatValueWithCondition(pcData.ssdReadRate, intWidth, decimalWidth, hasDecimal);
      
      if (hasDecimal && pcData.ssdReadRate < 100.0) {
        // 分开绘制整数部分和小数部分
        char intPart[10], decimalPart[10];
        sprintf(intPart, "%d", (int)pcData.ssdReadRate);
        sprintf(decimalPart, ".%d", (int)((pcData.ssdReadRate - (int)pcData.ssdReadRate) * 10));
        
        // 计算总宽度
        int totalWidth = intWidth + decimalWidth;
        
        // 先绘制整数部分（大字体）
        u8g2.drawStr(29 - totalWidth, 44, intPart);
        
        // 切换到小字体绘制小数部分
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(29 - totalWidth + intWidth, 46, decimalPart);
        
        // 恢复大字体
        u8g2.setFont(u8g2_font_8x13_tf);
      } else {
        // 直接绘制整数
        int strWidth = u8g2.getStrWidth(readStr);
        u8g2.drawStr(32 - strWidth, 44, readStr);
      }
    } else {
      int strWidth = u8g2.getStrWidth("--");
      u8g2.drawStr(32 - strWidth, 44, "--");
    }
    
    // 绘制写入速度（左对齐）
    if (pcDataValid) {
      int intWidth, decimalWidth;
      bool hasDecimal;
      const char* writeStr = formatValueWithCondition(pcData.ssdWriteRate, intWidth, decimalWidth, hasDecimal);
      
      if (hasDecimal && pcData.ssdWriteRate < 100.0) {
        // 分开绘制整数部分和小数部分
        char intPart[10], decimalPart[10];
        sprintf(intPart, "%d", (int)pcData.ssdWriteRate);
        sprintf(decimalPart, ".%d", (int)((pcData.ssdWriteRate - (int)pcData.ssdWriteRate) * 10));
        
        // 计算总宽度
        int totalWidth = intWidth + decimalWidth;
        
        // 对于10-99.9的范围，使用不同的对齐方式
        int startX;
        if (pcData.ssdWriteRate >= 10.0 && pcData.ssdWriteRate < 100.0) {
          // 10-99.9范围：向右移动，不超过原始位置
          startX = 112 - intWidth; // 整数部分从97-intWidth开始
          
          // 先绘制整数部分（大字体）
          u8g2.drawStr(startX, 44, intPart);
          
          // 切换到小字体绘制小数部分
          u8g2.setFont(u8g2_font_6x10_tf);
          u8g2.drawStr(startX + intWidth, 46, decimalPart);
        } else {
          // 小于10或大于等于100的情况使用原来的对齐方式
          startX = 112 - totalWidth;
          
          // 先绘制整数部分（大字体）
          u8g2.drawStr(startX, 44, intPart);
          
          // 切换到小字体绘制小数部分
          u8g2.setFont(u8g2_font_6x10_tf);
          u8g2.drawStr(startX + intWidth, 46, decimalPart);
        }
        
        // 恢复大字体
        u8g2.setFont(u8g2_font_8x13_tf);
      } else {
        // 直接绘制整数（>=100）
        //int strWidth = u8g2.getStrWidth(writeStr);
        u8g2.drawStr(96, 44, writeStr);
      }
    } else {
      u8g2.drawStr(96, 44, "--");
    }

    // 绘制单位
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(53, 18, "Mbps");
    u8g2.drawStr(53, 49, "MB/s");    

  } while (u8g2.nextPage());
}

// 关闭显示屏
void turnOffDisplay() {
  u8g2.setPowerSave(true);
}

// 开启显示屏
void turnOnDisplay() {
  u8g2.setPowerSave(false);
}

// 更新显示
void updateDisplay() {
  switch(displayMode) {
    case 0: // 关闭显示
      turnOffDisplay();
      break;
    case 1: // 风扇速度表盘
      turnOnDisplay();
      drawFanDial(filteredFanSpeedPercent);
      break;
    case 2: // 温度表盘
      turnOnDisplay();
      drawTemperatureDial(filteredTemperature);
      break;
    case 3: // CPU/GPU信息界面
      turnOnDisplay();
      drawCPUGPUScreen();
      break;
    case 4: // 网络硬盘信息界面
      turnOnDisplay();
      drawNetworkDiskScreen();
      break;
  }
}

// 更新LED状态
void updateLEDs() {
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  
  switch(displayMode) {
    case 1:
      digitalWrite(LED1_PIN, HIGH);
      break;
    case 2:
      digitalWrite(LED2_PIN, HIGH);
      break;
    case 3:
    case 4:
      digitalWrite(LED3_PIN, HIGH); // CPU/GPU和网络硬盘界面都点亮LED3
      break;
  }
}

// 检测按钮按下
void checkButtons() {
  // 读取按钮状态
  button1State = digitalRead(BUTTON1_PIN);
  button2State = digitalRead(BUTTON2_PIN);
  button3State = digitalRead(BUTTON3_PIN);
  
  // 检测按钮1的下降沿（按下）
  if (button1State == LOW && button1LastState == HIGH) {
    delay(50);
    if (digitalRead(BUTTON1_PIN) == LOW) {
      displayMode = (displayMode == 1) ? 0 : 1;
      updateDisplay();
      updateLEDs();
    }
  }
  
  // 检测按钮2的下降沿（按下）
  if (button2State == LOW && button2LastState == HIGH) {
    delay(50);
    if (digitalRead(BUTTON2_PIN) == LOW) {
      displayMode = (displayMode == 2) ? 0 : 2;
      updateDisplay();
      updateLEDs();
    }
  }
  
  // 检测按钮3的下降沿（按下）- 修改后的逻辑
  if (button3State == LOW && button3LastState == HIGH) {
    delay(50);
    if (digitalRead(BUTTON3_PIN) == LOW) {
      // 按钮3新逻辑：
      // 如果当前不是数据界面（模式3或4），则切换到CPU/GPU界面（模式3）
      // 如果当前是CPU/GPU界面（模式3），则切换到网络硬盘界面（模式4）
      // 如果当前是网络硬盘界面（模式4），则切换回CPU/GPU界面（模式3）
      if (displayMode == 3) {
        // 当前是CPU/GPU界面，切换到网络硬盘界面
        displayMode = 4;
      } else if (displayMode == 4) {
        // 当前是网络硬盘界面，切换回CPU/GPU界面
        displayMode = 3;
      } else {
        // 当前不是数据界面，切换到CPU/GPU界面
        displayMode = 3;
      }
      
      updateDisplay();
      updateLEDs();
    }
  }
  
  button1LastState = button1State;
  button2LastState = button2State;
  button3LastState = button3State;
}

void loop() {
  // 读取串口数据
  readSerialData();
  
  // 检查数据超时
  checkDataTimeout();
  
  // 读取原始温度
  currentTemperature = readRawTemperature();
  
  // 对温度进行滤波
  filteredTemperature = combinedTemperatureFilter(currentTemperature);
  
  // 获取旋转开关档位
  int gear = getGearPosition();
  
  // 根据挡位控制风扇
  controlFanByGear(gear);
  
  // 检测按钮状态
  checkButtons();
  
  // 更新显示
  updateDisplay();
  
  // 串口输出调试信息
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) {
    lastPrintTime = millis();
    
    Serial.print("Raw Temp: ");
    Serial.print(currentTemperature, 2);
    Serial.print("°C, Filtered Temp: ");
    Serial.print(filteredTemperature, 2);
    Serial.print("°C, Fan Speed: ");
    Serial.print(filteredFanSpeedPercent, 1);
    Serial.print("%, Display Mode: ");
    Serial.print(displayMode);
    Serial.print(", Gear: ");
    Serial.print(gear);
    
    if (pcDataValid) {
      Serial.print(", PC Data: CPU=");
      Serial.print(pcData.cpuTemp, 0);
      Serial.print("C/");
      Serial.print(pcData.cpuLoad, 0);
      Serial.print("% GPU=");
      Serial.print(pcData.gpuTemp, 0);
      Serial.print("C/");
      Serial.print(pcData.gpuLoad, 0);
      Serial.print("%");
      Serial.print(", Net: D=");
      Serial.print(pcData.downloadSpeed, 0);
      Serial.print("/U=");
      Serial.print(pcData.uploadSpeed, 0);
      Serial.print(" Mbps, Disk: R=");
      Serial.print(pcData.ssdReadRate, 0);
      Serial.print("/W=");
      Serial.print(pcData.ssdWriteRate, 0);
      Serial.print(" MB/s");
    } else {
      Serial.print(", No PC Data");
    }
    
    Serial.println();
  }
  
  delay(50);
}