#include <errno.h>
#include <string.h>
#include <math.h>
#include "colors.h"
#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/rand32.h>

#define STRIP_NODE              DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS        DT_PROP(DT_ALIAS(led_strip), chain_length)

#define BT_UART_DEVICE_NODE 		DT_ALIAS(bt_module)
#define PC_UART_CONSOLE_NODE		DT_ALIAS(pc_console)

#define DELAY_TIME K_MSEC(30)

//#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }
/*
static const struct led_rgb colors[] = {
        RGB(0x0f, 0x00, 0x00), 
        RGB(0x00, 0x0f, 0x00), 
        RGB(0x00, 0x00, 0x0f), 
};
*/
static const struct led_rgb black = RGB(0x00, 0x00, 0x00);

struct led_rgb pixels[STRIP_NUM_PIXELS];

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static const struct device *const bt_uart_dev = DEVICE_DT_GET(BT_UART_DEVICE_NODE);
static const struct device *const pc_uart_dev = DEVICE_DT_GET(PC_UART_CONSOLE_NODE);

unsigned int letter, position;

#define MSG_SIZE 32
/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

int scroll_text_length;
int current_char;

#define MAXLETTER 70
#define NRFONTCHR 43
#define SCREENDELAY 100
#define INITIALSPEED 30
#define COLUMNS 32 
#define ROWS 8

// boolean
bool speedupSW, slowdownSW, stopScrolling;
bool scroll_to_left = true;
char scrollText[MAXLETTER];

#define CHRWIDTH 8
#define SCREENWIDTH 32
#define SCREENHEIGHT 8

static int charColor;
const char font_chars[NRFONTCHR]={'0','1','2','3','4','5','6','7','8','9',
	                         '`','~','.',' ','-',':','/',
                                 'a','b','c','d','e','f','g','h','i','j','k','l','m',
                                 'n','o','p','q','r','s','t','u','v','w','x','y','z'};

//const int charsMatrix[NRLETTERS][CHRWIDTH]={
const int font_bitmap8[NRFONTCHR][8]={
{0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C}, // 0
{0x18, 0x38, 0x78, 0x18, 0x18, 0x18, 0x18, 0x7E}, // 1
{0x3C, 0x66, 0x46, 0x06, 0x0C, 0x18, 0x30, 0x7E}, // 2
{0x7E, 0x06, 0x0C, 0x18, 0x0C, 0x06, 0x06, 0x7C}, // 3
{0x06, 0x0E, 0x16, 0x26, 0x7E, 0x06, 0x06, 0x06}, // 4
{0x7E, 0x60, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C}, // 5
{0x3C, 0x66, 0x60, 0x60, 0x7C, 0x66, 0x66, 0x3C}, // 6
{0x7E, 0x06, 0x06, 0x0C, 0x18, 0x18, 0x18, 0x18}, // 7
{0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x66, 0x3C}, // 8
{0x3C, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x66, 0x3C}, // 9
{0x00, 0x24, 0x5A, 0x99, 0x81, 0x42, 0x24, 0x18}, // `heart
{0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C}, // ~smile
{0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00}, // .
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // space
{0x00, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x00, 0x00}, // -
{0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00}, // :
{0x01, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0}, // /
{0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F, 0x00}, // a
{0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x1E, 0x00}, // b
{0x00, 0x00, 0x0E, 0x10, 0x10, 0x11, 0x0E, 0x00}, // c
{0x01, 0x01, 0x0D, 0x13, 0x11, 0x11, 0x0F, 0x00}, // d
{0x00, 0x00, 0x0E, 0x11, 0x1E, 0x10, 0x0E, 0x00}, // e
{0x06, 0x09, 0x08, 0x1C, 0x08, 0x08, 0x08, 0x00}, // f
{0x00, 0x0f, 0x11, 0x11, 0x0F, 0x01, 0x0E, 0x00}, // g
{0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x11, 0x00}, // h
{0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x0E, 0x00}, // i
{0x02, 0x00, 0x06, 0x02, 0x02, 0x12, 0x0C, 0x00}, // j
{0x10, 0x10, 0x12, 0x14, 0x18, 0x14, 0x12, 0x00}, // k
{0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E, 0x00}, // l
{0x00, 0x00, 0x1A, 0x15, 0x15, 0x11, 0x11, 0x00}, // m
{0x00, 0x00, 0x16, 0x19, 0x11, 0x11, 0x11, 0x00}, // n
{0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x00}, // o
{0x00, 0x00, 0x1E, 0x11, 0x1E, 0x10, 0x10, 0x00}, // p
{0x00, 0x00, 0x0D, 0x13, 0x0F, 0x01, 0x01, 0x00}, // q
{0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10, 0x00}, // r
{0x00, 0x00, 0x0E, 0x10, 0x0E, 0x01, 0x1E, 0x00}, // s
{0x08, 0x08, 0x1C, 0x08, 0x08, 0x09, 0x06, 0x00}, // t
{0x00, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0D, 0x00}, // u
{0x00, 0x00, 0x11, 0x11, 0x11, 0x0A, 0x04, 0x00}, // v
{0x00, 0x00, 0x11, 0x11, 0x15, 0x15, 0x0A, 0x00}, // w
{0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x00}, // x
{0x00, 0x00, 0x11, 0x11, 0x0F, 0x01, 0x0E, 0x00}, // y
{0x00, 0x00, 0x1F, 0x02, 0x04, 0x08, 0x1F, 0x00}  // z
};
void set_pixel(int x, int y, const struct led_rgb color)
{
    int cursor;
    if ((x % 2) == 0)
      cursor = ((SCREENWIDTH - x)*SCREENHEIGHT) - 1 - ((SCREENHEIGHT-1) - y);
    else
      cursor = ((SCREENWIDTH - x)*SCREENHEIGHT) - 1 - y;
    memcpy(&pixels[cursor], &color, sizeof(struct led_rgb));
}

struct led_rgb get_pixel(int x, int y)
{
    int cursor;
    if ((x % 2) == 0)
      cursor = ((SCREENWIDTH - x)*SCREENHEIGHT) - 1 - ((SCREENHEIGHT-1) - y);
    else
      cursor = ((SCREENWIDTH - x)*SCREENHEIGHT) - 1 - y;
    return pixels[cursor];
}

void clearScreen()
{
    for (int x = 0; x < COLUMNS - 1; x++) 
        for (int y = 0; y < ROWS; y++) 
	   set_pixel(x, y, black);
}

void scrollMatrix(bool scroll_to_left)
{
    if (scroll_to_left)
//      clearScreen();
      for (int x = 0; x < COLUMNS - 1; x++) 
        for (int y = 0; y < ROWS; y++) {
	  set_pixel(x, y, get_pixel(x+1, y));
//          memcpy(&pixels[cursor_to], &pixels[cursor_from], sizeof(struct led_rgb));
        }
}

void addNewRow(bool scroll_to_left)
{ 
    //int colorText = sys_rand32_get() % 87;
    if (scroll_to_left) {
      for ( int i = 0; i < CHRWIDTH; i++ ) {
        if (font_bitmap8[current_char][i] & (1 << ( CHRWIDTH - position - 1))) 
	  set_pixel(SCREENWIDTH-1, i, colors[charColor]);
	else
	  set_pixel(SCREENWIDTH-1, i, black); 
      }
    }
}

void main(void)
{
        int rc;
	bool stopScrolling = false; 

     	//strcpy(scrollText,"0123456789/:-. `~abcdefghijklmnopqrstuvwxyz");
     	strcpy(scrollText,"open tech school / embedded projects / www.protektwar.net / ");
	scroll_text_length = strlen(scrollText);
	position = 0;
	letter = 0;
        for (int i = 0; i < NRFONTCHR; i++)
           if (scrollText[letter] == font_chars[i])
              current_char = i;

        if (device_is_ready(strip)) {
                LOG_INF("Found LED strip device %s", strip->name);
        } else {
                LOG_ERR("LED strip device %s is not ready", strip->name);
                return;
        }

        if (!device_is_ready(bt_uart_dev)) {
                printk("BT UART device is not ready!");
                LOG_INF("BT UART device is not ready!");
                return;
        }
	else {
                printk("Found BT UART device!");
                LOG_INF("Found BT UART device!");
	}
	
        if (!device_is_ready(bt_uart_dev)) {
                printk("PC UART device is not ready!");
                LOG_INF("PC UART device is not ready!");
                return;
        } 
	else {
                printk("Found PC UART device!");
                LOG_INF("Found PC UART device!");
	}

        /* configure interrupt and callback to receive data */
/*        int ret = uart_irq_callback_user_data_set(pc_uart_dev, serial_cb, NULL);

        if (ret < 0) {
                if (ret == -ENOTSUP) {
                        printk("Interrupt-driven UART API support not enabled\n");
                } else if (ret == -ENOSYS) {
                        printk("UART device does not support interrupt-driven API\n");
                } else {
                        printk("Error setting UART callback: %d\n", ret);
                }   
                return;
        }   
        uart_irq_rx_enable(uart_dev);


        print_uart("Hello! I'm your echo bot.\r\n");
        print_uart("Tell me something and press enter:\r\n");
*/	

        while (1) {
              /*if (speedUP) {
	          speedUP = false;
		  stopScrolling = false;
		  speed = speed - 1;
		  if (speed < 1) 
		    speed = 1;
		}
                if (slowDOWN) {
		  slowDOWN = false;
		  speed = speed + 1;
		  if (speed > MAXSPEED) {
		    stopScrolling = true;
		    speed = MAXSPEED;
		  }
		} */
		if (!stopScrolling) {
                  position++;
		  if (!(position % 8)) {
                    charColor = sys_rand32_get() % 87;
                    position = 0;
		    letter++;
		    if (letter == scroll_text_length)
		      letter = 0;
		    current_char = 0;
		    for (int i = 0; i < NRFONTCHR; i++)
		      if (scrollText[letter] == font_chars[i])
		        current_char = i;
		  }
		  scrollMatrix(scroll_to_left);
		  addNewRow(scroll_to_left);
		}

                rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

                if (rc) {
                        LOG_ERR("couldn't update strip: %d", rc);
                }

                k_sleep(DELAY_TIME);
        }
}

