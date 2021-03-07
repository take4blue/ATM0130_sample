#include <driver/spi_master.h>
#include <stdint.h>
#include <string>
#include <esp32/rom/ets_sys.h>

template <typename T>
class AutoLocker {
private:
  T& target_;
  int delayTime_;
public:
  AutoLocker(T& target, int delayTime = 0) : target_(target), delayTime_(delayTime) {
    target_.start();
  }
  ~AutoLocker() {
    target_.end();
    ets_delay_us(delayTime_ * 1000);
  }
};

class ATM0130 {
  public:
    ATM0130(uint8_t pin_dat_cmd, uint8_t pin_reset);
    void begin(void);

    void setFigColor(uint8_t r, uint8_t g, uint8_t b);
    void setFigColor(uint16_t c);
    void drawRectangle(uint8_t x, uint8_t y, uint8_t width, uint8_t height) ;

    void setCharPlace(uint8_t x, uint8_t y);
    void setCharColor(uint8_t r, uint8_t g, uint8_t b);
    void setCharColor(uint16_t c);
    void setCharColorBG(uint8_t r, uint8_t g, uint8_t b);
    void setCharColorBG(uint16_t c);
    void print(char ch);
    void print(const std::string& str);

    void start();
    void end();

  private:
    spi_device_handle_t device_;

    void writeReg(uint8_t data);
    void writeData(uint8_t data);
    void writeData(size_t len, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
    void writeData(size_t len, uint8_t *buffer);
    void resetLCD(void);
    void setWindow(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
    void putPixel(uint16_t color) ;

    void queResults(int pos = 0);

    void setCharQueue(uint8_t c) ;
    void writeCharQueue() ;
    uint16_t convRGB(uint8_t red, uint8_t green, uint8_t blue) ;

    static const int transSize_ = 7;
    static const int bufferSize_ = 512;
    static const int bufferNum_ = 3;
    static const int freq_ = 1000000;

    uint16_t fig_color, char_color, char_color_bg;
    uint8_t char_x, char_y;
    uint8_t dat_cmd, reset;
    bool is_reset_inv;
    uint8_t nQue_;
    uint8_t posQue_;
    spi_transaction_t *trans_;
    uint16_t *buffer_[bufferNum_];

    uint8_t char_queue[5];
    volatile uint8_t chars[475] = {
      0x00, 0x00, 0x00, 0x00, 0x00,  0x21, 0x08, 0x40, 0x10, 0x00,  0x52, 0x94, 0x00, 0x00, 0x00,  0x52, 0x95, 0xF5, 0x7D, 0x4A,  0x23, 0xE8, 0xE2, 0xF8, 0x80,  0xC6, 0x44, 0x44, 0x4C, 0x60,  0x64, 0xA8, 0x8A, 0xC9, 0xA0,  0x61, 0x10, 0x00, 0x00, 0x00,  0x11, 0x10, 0x84, 0x10, 0x40,  0x41, 0x04, 0x21, 0x11, 0x00,  0x01, 0x2A, 0xEA, 0x90, 0x00,  0x01, 0x08, 0xE2, 0x10, 0x00,  0x00, 0x00, 0x06, 0x11, 0x00,  0x00, 0x01, 0xF0, 0x00, 0x00,  0x00, 0x00, 0x00, 0x63, 0x00,  0x00, 0x44, 0x44, 0x40, 0x00,  0x74, 0x67, 0x5C, 0xC5, 0xC0,  0x23, 0x08, 0x42, 0x11, 0xC0,  0x74, 0x42, 0x22, 0x23, 0xE0,  0xF8, 0x88, 0x20, 0xC5, 0xC0,
      0x11, 0x95, 0x2F, 0x88, 0x40,  0xFC, 0x3C, 0x10, 0xC5, 0xC0,  0x32, 0x11, 0xE8, 0xC5, 0xC0,  0xF8, 0x44, 0x44, 0x21, 0x00,  0x74, 0x62, 0xE8, 0xC5, 0xC0,  0x74, 0x62, 0xF0, 0x89, 0x80,  0x03, 0x18, 0x06, 0x30, 0x00,  0x03, 0x18, 0x06, 0x11, 0x00,  0x11, 0x11, 0x04, 0x10, 0x40,  0x00, 0x3E, 0x0F, 0x80, 0x00,  0x41, 0x04, 0x11, 0x11, 0x00,  0x74, 0x42, 0x22, 0x00, 0x80,  0x74, 0x42, 0xDA, 0xD5, 0xC0,  0x74, 0x63, 0x1F, 0xC6, 0x20,  0xF4, 0x63, 0xE8, 0xC7, 0xC0,  0x74, 0x61, 0x08, 0x45, 0xC0,  0xE4, 0xA3, 0x18, 0xCB, 0x80,  0xFC, 0x21, 0xE8, 0x43, 0xE0,  0xFC, 0x21, 0xE8, 0x42, 0x00,  0x74, 0x61, 0x78, 0xC5, 0xE0,
      0x8C, 0x63, 0xF8, 0xC6, 0x20,  0x71, 0x08, 0x42, 0x11, 0xC0,  0x38, 0x84, 0x21, 0x49, 0x80,  0x8C, 0xA9, 0x8A, 0x4A, 0x20,  0x84, 0x21, 0x08, 0x43, 0xE0,  0x8E, 0xEB, 0x58, 0xC6, 0x20,  0x8C, 0x73, 0x59, 0xC6, 0x20,  0x74, 0x63, 0x18, 0xC5, 0xC0,  0xF4, 0x63, 0xE8, 0x42, 0x00,  0x74, 0x63, 0x1A, 0xC9, 0xA0,  0xF4, 0x63, 0xEA, 0x4A, 0x20,  0x74, 0x20, 0xE0, 0x87, 0xC0,  0xF9, 0x08, 0x42, 0x10, 0x80,  0x8C, 0x63, 0x18, 0xC5, 0xC0,  0x8C, 0x63, 0x18, 0xA8, 0x80,  0x8C, 0x63, 0x5A, 0xD5, 0x40,  0x8C, 0x54, 0x45, 0x46, 0x20,  0x8C, 0x62, 0xA2, 0x10, 0x80,  0xF8, 0x44, 0x44, 0x43, 0xE0,  0x72, 0x10, 0x84, 0x21, 0xC0,
      0x8A, 0xBE, 0x4F, 0x90, 0x80,  0x70, 0x84, 0x21, 0x09, 0xC0,  0x22, 0xA2, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x03, 0xE0,  0x41, 0x04, 0x00, 0x00, 0x00,  0x00, 0x1C, 0x17, 0xC5, 0xE0,  0x84, 0x2D, 0x98, 0xC7, 0xC0,  0x00, 0x1D, 0x08, 0x45, 0xC0,  0x08, 0x5B, 0x38, 0xC5, 0xE0,  0x00, 0x1D, 0x1F, 0xC1, 0xC0,  0x32, 0x51, 0xC4, 0x21, 0x00,  0x03, 0xE3, 0x17, 0x85, 0xC0,  0x84, 0x2D, 0x98, 0xC6, 0x20,  0x20, 0x18, 0x42, 0x11, 0xC0,  0x10, 0x0C, 0x21, 0x49, 0x80,  0x84, 0x25, 0x4C, 0x52, 0x40,  0x61, 0x08, 0x42, 0x11, 0xC0,  0x00, 0x35, 0x5A, 0xC6, 0x20,  0x00, 0x2D, 0x98, 0xC6, 0x20,  0x00, 0x1D, 0x18, 0xC5, 0xC0,
      0x00, 0x3D, 0x1F, 0x42, 0x00,  0x00, 0x1B, 0x37, 0x84, 0x20,  0x00, 0x2D, 0x98, 0x42, 0x00,  0x00, 0x1D, 0x07, 0x07, 0xC0,  0x42, 0x38, 0x84, 0x24, 0xC0,  0x00, 0x23, 0x18, 0xCD, 0xA0,  0x00, 0x23, 0x18, 0xA8, 0x80,  0x00, 0x23, 0x1A, 0xD5, 0x40,  0x00, 0x22, 0xA2, 0x2A, 0x20,  0x00, 0x23, 0x17, 0x85, 0xC0,  0x00, 0x3E, 0x22, 0x23, 0xE0,  0x11, 0x08, 0x82, 0x10, 0x40,  0x21, 0x08, 0x42, 0x10, 0x80,  0x41, 0x08, 0x22, 0x11, 0x00,  0x00, 0x11, 0x51, 0x00, 0x00,
    };
};
