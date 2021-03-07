#include <memory.h>
#include <algorithm>
#include "ATM0130.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

ATM0130::ATM0130(gpio_num_t dcPin, gpio_num_t resetPin)
: fig_color(0)
, char_color(0)
, char_color_bg(0)
, char_x(0)
, char_y(0)
, dcPin_(dcPin)
, resetPin_(resetPin)
, nQue_(0)
, posQue_(0)
, trans_(nullptr)
{
  setFigColor(0x0000);
  setCharColor(0xFFFF);
  setCharColorBG(0x00);
  memset(buffer_, 0, sizeof(buffer_));
}

static void transfer_callback(spi_transaction_t *t)
{
    if (t->user != nullptr) {
      gpio_set_level((gpio_num_t)((uint32_t)t->user & 0xFF), ((uint32_t)t->user & 0xFF00) != 0 ? 1 : 0);
    }
}

void ATM0130::writeReg(uint8_t data)
{
  if (nQue_ >= transSize_) {
    queResults();
  }
  auto index = posQue_ % transSize_;
  memset(trans_ + index, 0, sizeof(spi_transaction_t));
  trans_[index].flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  trans_[index].length = 8;
  trans_[index].tx_data[0] = data;
  trans_[index].user = (void*)(dcPin_);
  auto ret = spi_device_queue_trans(device_, trans_ + index, portMAX_DELAY);
  assert(ret == ESP_OK);
  nQue_++;
  posQue_++;
}

void ATM0130::writeData(size_t len, uint8_t *buffer)
{
  if (len <= 4) {
    writeData(len, buffer[0], buffer[1], buffer[2], buffer[3]);
  }
  else {
    if (nQue_ >= transSize_) {
      queResults();
    }
    auto index = posQue_ % transSize_;
    memset(trans_ + index, 0, sizeof(spi_transaction_t));
    trans_[index].length = 8 * len;
    trans_[index].tx_buffer = buffer;
    trans_[index].user = (void*)(0x100 | dcPin_);
		auto ret = spi_device_queue_trans(device_, trans_ + index, portMAX_DELAY);
		assert(ret == ESP_OK);
    nQue_++;
    posQue_++;
  }
}

void ATM0130::writeData(size_t len, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
  if (nQue_ >= transSize_) {
    queResults();
  }
  if (len <= 4) {
    auto index = posQue_ % transSize_;
    memset(trans_ + index, 0, sizeof(spi_transaction_t));
    trans_[index].length = 8 * len;
    trans_[index].flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans_[index].tx_data[0] = d1;
    trans_[index].tx_data[1] = d2;
    trans_[index].tx_data[2] = d3;
    trans_[index].tx_data[3] = d4;
    trans_[index].user = (void*)(0x100 | dcPin_);
		auto ret = spi_device_queue_trans(device_, trans_ + index, portMAX_DELAY);
		assert(ret == ESP_OK);
    nQue_++;
    posQue_++;
  }
}

void ATM0130::writeData(uint8_t data)
{
  writeData(1, data, 0, 0, 0);
}

void ATM0130::start()
{
  queResults();
  auto ret = spi_device_acquire_bus(device_, portMAX_DELAY);
  assert(ret == ESP_OK);
}

void ATM0130::queResults(int pos)
{
	spi_transaction_t* rtrans;
	esp_err_t ret;
	for (; nQue_ > pos; nQue_--) {
		ret = spi_device_get_trans_result(device_, &rtrans, portMAX_DELAY);
		assert(ret == ESP_OK);
	}
  if (pos == 0) {
    posQue_ = 0;
  }
}

void ATM0130::end()
{
  queResults();
  spi_device_release_bus(device_);
}

void ATM0130::begin(int mosi, int miso, int sclk, int cs, int freq)
{
  gpio_config_t io_conf = {
      1ULL << dcPin_ | 1ULL << resetPin_,
      GPIO_MODE_OUTPUT,
      GPIO_PULLUP_DISABLE,
      GPIO_PULLDOWN_DISABLE,
      GPIO_INTR_DISABLE
  };
  gpio_config(&io_conf);

  resetLCD();
  // SPIの初期化
  spi_bus_config_t buscfg = { 
    .mosi_io_num = mosi,
    .miso_io_num = miso,
    .sclk_io_num = sclk, 
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = sizeof(uint16_t) * bufferSize_,
    .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MOSI,
    .intr_flags = 0
  };

  esp_err_t ret = spi_bus_initialize(VSPI_HOST, &buscfg, 2);
  assert(ret == ESP_OK);

  auto freqw = spi_get_actual_clock(APB_CLK_FREQ, freq, 0);
  spi_device_interface_config_t devcfg = {
      .command_bits = 0, .address_bits = 0, .dummy_bits = 0,
      .mode = 0, // SPI mode
      .duty_cycle_pos = 0, .cs_ena_pretrans = 0, .cs_ena_posttrans = 0,
      .clock_speed_hz = freqw,
      .input_delay_ns = 0,
      .spics_io_num = cs, //CS pin
      .flags = 0,
      .queue_size = transSize_, //We want to be able to queue 7 transactions at a time
      .pre_cb = transfer_callback,
      .post_cb = nullptr
  };
  ret = spi_bus_add_device(VSPI_HOST, &devcfg, &device_);
  trans_ = (spi_transaction_t*)heap_caps_malloc(transSize_ * sizeof(spi_transaction_t), MALLOC_CAP_DEFAULT);
  for (int i = 0; i < bufferNum_; i++) {
    buffer_[i] = (uint16_t*)heap_caps_malloc(bufferSize_ * sizeof(uint16_t), MALLOC_CAP_DMA);
  }

  {
    AutoLocker<ATM0130> lock(*this, 100);
    writeReg(0x11);
  }
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0x36);  //MADCTL
    writeData(0x00);
    //MY=0
    //MX=0
    //MV=0
    //ML=0
    //RGB=0
    //MH=0
    writeReg(0x3A);
    writeData(0x55); //65K color , 16bit / pixel

    ////--------------------------------ST7789V Frame rate
    writeReg(0xb2);
    writeData(4, 0x0c, 0x0c, 0x0c, 0x33);
    writeData(0x33);
  }
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0xb7);
    writeData(0x75);
  }
  ////---------------------------------ST7789V Power
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0xc2);
    writeData(0x01);
  }
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0xc3);
    writeData(0x10);
  }
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0xc4);
    writeData(0x20);
  }
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0xc6);
    writeData(0x0f);
    writeReg(0xb0);
    writeData(2, 0x00, 0xf0, 0, 0);//RRRR RGGGG GGGB BBBB
  }
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0xD0);
    writeData(2, 0xA4, 0xA1, 0, 0);
  }
  ////--------------------------------ST7789V gamma
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0x21);
  }
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0xbb);
    writeData(0x3b);
  }
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0xE0);    //Set Gamma
    writeData(4, 0xF0, 0x0b, 0x11, 0x0e);
    writeData(4, 0x0d, 0x19, 0x36, 0x33);
    writeData(4, 0x4b, 0x07, 0x14, 0x14);
    writeData(2, 0x2c, 0x2e, 0, 0);
  }
  {
    AutoLocker<ATM0130> lock(*this, 2);
    writeReg(0xE1);    //Set Gamma
    writeData(4, 0xF0, 0x0d, 0x12, 0x0b);
    writeData(4, 0x09, 0x03, 0x32, 0x44);
    writeData(4, 0x48, 0x39, 0x16, 0x16);
    writeData(2, 0x2d, 0x30, 0, 0);
    writeReg(0x2A);
    writeData(4, 0x00, 0x00, 0x00, 0xEF);
    writeReg(0x2B);
    writeData(4, 0x00, 0x00, 0x00, 0xEF);
    writeReg(0x29);    //Display on
  }
  {
    AutoLocker<ATM0130> lock(*this);
    writeReg(0x2c);
  }
}

void ATM0130::setFigColor(uint8_t r, uint8_t g, uint8_t b)
{
  fig_color = convRGB(r, g, b);
}

void ATM0130::setFigColor(uint16_t c)
{
  fig_color = c;
}

void ATM0130::setCharPlace(uint8_t x, uint8_t y)
{
  char_x = x;
  char_y = y;
}

void ATM0130::setCharColor(uint8_t r, uint8_t g, uint8_t b)
{
  char_color = convRGB(r, g, b);
}

void ATM0130::setCharColor(uint16_t c)
{
  char_color = c;
}

void ATM0130::setCharColorBG(uint8_t r, uint8_t g, uint8_t b)
{
  char_color_bg = convRGB(r, g, b);
}

void ATM0130::setCharColorBG(uint16_t c)
{
  char_color_bg = c;
}

void ATM0130::print(char ch)
{
  if (char_x > 235) {
    char_x = 0;
    char_y += 8;
  }
  if (char_y > 232) {
    char_x = 0;
    char_y = 0;
  }

  if (ch == '\n') {
    char_x = 0;
    char_y += 8;
  }
  else {
    setCharQueue(ch);
    writeCharQueue();
    char_x += 6;
  }
}

void ATM0130::print(const std::string& str)
{
  std::for_each(str.begin(), str.end(), [this](char x){print(x);});
}

void ATM0130::resetLCD(void)
{
  gpio_set_level(resetPin_, 1);
  vTaskDelay(20/portTICK_PERIOD_MS);
  gpio_set_level(resetPin_, 0);
  vTaskDelay(20/portTICK_PERIOD_MS);
  gpio_set_level(resetPin_, 1);
  vTaskDelay(20/portTICK_PERIOD_MS);
}

void ATM0130::setWindow(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
  writeReg(0x2A);
  writeData(4, 0x00, x, 0x00, x + width - 1);
  writeReg(0x2B);
  writeData(4, 0x00, y, 0x00, y + height - 1);
  writeReg(0x2c);
}

void ATM0130::putPixel(uint16_t color)
{
  writeData(2, (uint8_t*)& color);
}

void ATM0130::drawRectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
  int loop = width * height;
  AutoLocker<ATM0130> lock(*this);
  setWindow(x, y, width, height);
  int index = 0;
  for (int i = 0; i < loop; index++) {
    auto buffer = buffer_[index % bufferNum_];
    int j = 0;
    for (; j < bufferSize_ && i < loop; j++, i++) {
      buffer[j] = fig_color;
    }
    queResults(bufferNum_ - 2);
    writeData(j * sizeof(uint16_t), (uint8_t*)buffer);
  }
}

void ATM0130::setCharQueue(uint8_t c)
{
  if ((c >= 0x20) && (c <= 0x7E)) {
    c -= 0x20;
    for (uint8_t i = 0; i < 5; i++) {
      char_queue[i] = chars[5 * c + i];
    }
  }
  else {
    for (uint8_t i = 0; i < 5; i++) {
      char_queue [i] = 0xFF;
    }
  }
}

void ATM0130::writeCharQueue()
{
  AutoLocker<ATM0130> lock(*this);
  int counter = 0;
  setWindow(char_x, char_y, 6, 8);
  for (uint8_t i = 0; i < 5; i++) {
    for (uint8_t j = 0; j < 8; j++, counter++) {
      if ((char_queue[i] & (0x80 >> j)) > 0) {
        putPixel(char_color);
      }
      else {
        putPixel(char_color_bg);
      }
      if (counter % 5 == 4) {
        putPixel(char_color_bg);
      }
    }
  }
}

uint16_t ATM0130::convRGB(uint8_t red, uint8_t green, uint8_t blue)
{
  uint16_t color = 0;
  color = blue >> 3;
  color |= ((green & 0xFC) << 3);
  color |= ((red & 0xF8) << 8);
  return color;
}
