#include <driver/gpio.h>
#include "ATM0130.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define delay(x) vTaskDelay(x/portTICK_PERIOD_MS)

//ピンの割り当て (D/C , /RES)
ATM0130 myATM0130 = ATM0130(GPIO_NUM_16, GPIO_NUM_17);

void probe()
{
  gpio_set_level(GPIO_NUM_2, 1);
  gpio_set_level(GPIO_NUM_2, 0);
}

void setup() {
  //初期化
  gpio_config_t io_conf = {
      1ULL << GPIO_NUM_2,
      GPIO_MODE_OUTPUT,
      GPIO_PULLUP_DISABLE,
      GPIO_PULLDOWN_DISABLE,
      GPIO_INTR_DISABLE
  };
  gpio_config(&io_conf);
  probe();
  myATM0130.begin();
}

void colorBar() {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  for (uint8_t i = 0; i < 8; i++) {
    g = 255 * (1 - ((i & 0x04) >> 2));
    r = 255 * (1 - ((i & 0x02) >> 1));
    b = 255 * (1 - ((i & 0x01)));
    myATM0130.setFigColor(r, g, b);
    myATM0130.drawRectangle(i * 30, 0, 30, 180);
  }
  for (uint8_t i = 0; i < 240; i++) {
    myATM0130.setFigColor(i, i, i);
    myATM0130.drawRectangle(i, 180, 1, 60);
  }
}

void loop() {
  //全体を塗りつぶし
  //図形の色を指定(r,g,b)=(255,255,255)
  myATM0130.setFigColor(255, 255, 255);
  //(x,y)=(0,0)から(width,height)=(240,240)で塗りつぶし
  myATM0130.drawRectangle(0, 0, 240, 240);

  //カラーバーを表示
  probe();
  colorBar();

  //文字出力
  //文字の位置を(x,y)=(0,160)に設定
  myATM0130.setCharPlace(0, 160);
  //文字の色を(r,g,b)=(0,0,0)に設定
  myATM0130.setCharColor(0, 0, 0);
  //文字の背景色を(r,g,b)=(255,255,255)に設定
  myATM0130.setCharColorBG(255, 255, 255);
  //テキスト出力
  myATM0130.print("ATM0130B3 TEST\n");
  myATM0130.print("http://akizukidenshi.com/\n");

  // 5秒待つ
  delay(5000);
}


extern "C" void app_main()
{
    setup();
    for (;;) {
        loop();
    }
}