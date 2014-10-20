#include <stdio.h>
#include <unistd.h>
#include <usb.h>
#include <math.h>
#include <AL/al.h>
#include <AL/alc.h>
#include <SDL/SDL.h>
#include "chuck_fft.h"
#include  <sys/types.h>
#include  <signal.h>
#include  <sys/ipc.h>
#include  <sys/shm.h>

#define FALSE 0

//#define ENABLE_FANBUS

#ifdef ENABLE_FANBUS
#include "serial_port.h"
#include "fanbus.h"
serial_port ports[2];
fanbus bus1;

typedef void sigfunc(int) sigfunc *signal(int, sigfunc*); 

static int red_leds[] = { 0x10, 0x13, 0x16, 0x19 };
static int grn_leds[] = { 0x11, 0x14, 0x17, 0x1A };
static int blu_leds[] = { 0x12, 0x15, 0x18, 0x1B };
#endif // ENABLE_FANBUS

static struct usb_device *find_device(uint16_t vendor, uint16_t product);

static int init_keyboard();

static void update_keyboard();

static void set_led( int x, int y, int r, int g, int b );

void  SIGINT_handler(int);   
void  SIGQUIT_handler(int); 


static const int SPEED = 30;
static const bool wasd_white = false;
static const bool arrows_white = false;
static const bool moba_lights = false;

static const double red_min_bright = 0;
static const double red_max_bright = 7;
static const double green_min_bright = 0;
static const double green_max_bright = 7;
static const double blue_min_bright = 0;
static const double blue_max_bright = 7;



char red_val[144];
char grn_val[144];
char blu_val[144];
char wasd [4][2] = { {12, 3}, { 10, 4}, { 12,4}, {15,4}}; 

char data_pkt[5][64] = { 0 };

unsigned char position_map[] = {
  137,  8, 20,255,
        0, 12, 24, 36, 48, 60, 72, 84, 96,108,120,132,  6, 18, 30, 42, 32, 44, 56, 68,255,
        1, 13, 25, 37, 49, 61, 73, 85, 97,109,121,133,  7, 31, 54, 66, 78, 80, 92,104,116,255,
        2, 14, 26, 38, 50, 62, 74, 86, 98,110,122,134, 90,102, 43, 55, 67,  9, 21, 33,128,255,
        3, 15, 27, 39, 51, 63, 75, 87, 99,111,123,135,126, 57, 69, 81,128,255,
        4, 28, 40, 52, 64, 76, 88,100,112,124,136, 79,103, 93,105,117,140,255,
        5, 17, 29, 53, 89,101,113, 91,115,127,139,129,141,140,255,
};

float size_map[] = {
        -15.5, 1, 1, -2.5, 1, -2, 0,
        1, -.5, 1, 1, 1, 1, -.75, 1, 1, 1, 1, -.75, 1, 1, 1, 1, -.5, 1, 1, 1, -.5, 1, 1, 1, 1, 0,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, -.5, 1, 1, 1, -.5, 1, 1, 1, 1, 0,
        1.5, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1.5, -.5, 1, 1, 1, -.5, 1, 1, 1, 1, 0,
        1.75, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2.25, -4, 1, 1, 1, 1, 0,
        2.25, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2.75, -1.5, 1, -1.5, 1, 1, 1, 1, 0,
        1.5, 1, 1.25, 6.5, 1.25, 1, 1, 1.5, -.5, 1, 1, 1, -.5, 2, 1, 1, 0,
};

unsigned char led_matrix[7][92];

unsigned char led_waveform[7][92];

struct usb_device *dev;
struct usb_dev_handle *handle;

float normalizeFFT(float fftin)
{
    if(fftin > 0)
    {
        return 20.0f*log10(fftin);
    }
    else
    {
        return 0;
    }
}

void sig_handler(int signo)
{
  if (signo == SIGINT) exit(1);
}
void update(int signo)
{
  if (signo == SIGINT) exit(1);
}
static void fix_top_buttons(){
    int led_1 = led_matrix[0][62];
    int led_2 = led_matrix[0][81];
    int led_3 = led_matrix[1][79];
    int led_4 = led_matrix[1][81];

    red_val[led_3] = red_val[led_matrix[1][57]];
    grn_val[led_3] = grn_val[led_matrix[1][57]];
    blu_val[led_3] = blu_val[led_matrix[1][57]];

    red_val[led_4] = red_val[led_matrix[1][55]];
    grn_val[led_4] = grn_val[led_matrix[1][55]];
    blu_val[led_4] = blu_val[led_matrix[1][55]];

    red_val[led_1] = red_val[led_matrix[1][47]];
    grn_val[led_1] = grn_val[led_matrix[1][47]];
    blu_val[led_1] = blu_val[led_matrix[1][47]];
    
    red_val[led_2] = red_val[led_matrix[1][51]];
    grn_val[led_2] = grn_val[led_matrix[1][51]];
    blu_val[led_2] = blu_val[led_matrix[1][51]];



}


int main(int argc, char *argv[])
{
    if (signal(SIGINT, sig_handler) == SIG_ERR)
        printf("\ncan't catch SIGINT\n"); 
    float amplitude = 10.0f;
    unsigned char red, grn, blu;
    unsigned char buffer[256];
    float fft[256];
    unsigned char charfft[256];
    float win[256];
    float fftavg;
    int level;
    SDL_Surface* wavs = NULL;
    SDL_Surface* screen = NULL;
    SDL_Event event;

    //SDL_Init( SDL_INIT_EVERYTHING );
    //screen = SDL_SetVideoMode( 256, 256, 32, SDL_HWSURFACE );
    //wavs = SDL_SetVideoMode( 256, 256, 32, SDL_HWSURFACE );
    //SDL_WM_SetCaption("FanBus Audio Visualizer", NULL);

    ALCdevice *device = alcCaptureOpenDevice(NULL, 8000, AL_FORMAT_MONO8, 256);
    alcCaptureStart(device);

    hanning(win, 256);

#ifdef ENABLE_FANBUS
#ifdef WIN32
    if( ports[0].serial_open("COM2", 38400) == FALSE )
#else
    if( ports[0].serial_open("/dev/ttyS4", 38400) == FALSE )
#endif
    {
        return 0;
    }

    bus1.fanbus_set_port(&ports[0]);
#endif

    init_keyboard();

    while(1)
    {
            for(int j = 0; j < 92; j++)
            {
            for(int i = 0; i < 256; i++)
            {
         
                charfft[i] = 0;
                fft[i] = 0;
            }

            alcCaptureSamples(device, (ALCvoid *)buffer, 256);
            level = 0;
            for(int i = 0; i < 256; i++)
            {
                level += abs((int)buffer[i]-128);
                fft[i] = (buffer[i]-128)*amplitude;
            }

            rfft(fft, 256, 1);
            apply_window(fft, win, 256);

            for(int i = 0; i < 256; i+=2)
            {
                float fftmag = sqrt((fft[i]*fft[i])+(fft[i+1]*fft[i+1]));
                fftavg += fftmag;
                charfft[i] = (unsigned char)fftmag;
                charfft[i+1] = charfft[i];
            }
            fftavg /= 10;
            for(int i = 0; i < 256; i++)
            {
            }
            if(event.type == SDL_QUIT)
            {
                return 0;
            }
            SDL_Delay(20);

#ifdef ENABLE_FANBUS
            for(int i = 0; i < 4; i++)
            {
                red = 64 * ( sin( ( ( ( j + 23 * i ) / 92.0f ) *6.28 )                ) + 1 );
                grn = 64 * ( sin( ( ( ( j + 23 * i ) / 92.0f ) *6.28 ) + ( 6.28 / 3 ) ) + 1 );
                blu = 64 * ( sin( ( ( ( j + 23 * i ) / 92.0f ) *6.28 ) - ( 6.28 / 3 ) ) + 1 );

                bus1.fanbus_write(red_leds[i], 0x02, red);
                bus1.fanbus_write(grn_leds[i], 0x02, grn);
                bus1.fanbus_write(blu_leds[i], 0x02, blu);
                printf("fanbus");
                bus1.fanbus_write(0x0C, 0x02, 0x01);
            }
#endif

            for(int x = 0; x < 91; x++)
            {
                for(int y = 0; y < 7; y++)
                {
                    double red_multiplier = (red_max_bright - red_min_bright + 1)/3.0;
                    double green_multiplier = (green_max_bright - green_min_bright + 1)/3.0;
                    double blue_multiplier = (blue_max_bright - blue_min_bright + 1)/3.0;
                    red =  (1.5f * ( sin( ( x / 92.0f ) * 2 * 3.14f ) + 1 ))* red_multiplier + red_min_bright;
                    grn = (1.5f * ( sin( ( ( x / 92.0f ) * 2 * 3.14f ) - ( 6.28f / 3 ) ) + 1 ))* green_multiplier+ green_min_bright;
                    blu = (1.5f * ( sin( ( ( x / 92.0f ) * 2 * 3.14f ) + ( 6.28f / 3 ) ) + 1 ))* blue_multiplier + blue_min_bright;
                    set_led( ( x + j ) % 92, y, red, grn, blu );
                }
            }
            if (wasd_white){
                for (int i=0; i<4; i++){
                    set_led( wasd[i][0], wasd[i][1], 0x07, 0x07, 0x07);
                }
            }
            if (arrows_white){
                for(int i = 60; i < 75; i++)
                {
                    for(int k = 5; k < 7; k++)
                    {
                        //if( charfft[(int)(i*2.1+1)] > (255/(15 + (i*0.8))) * (7-k) )
                        set_led( i, k, 0x07, 0x07, 0x07);
                    }
                }
            }
            if (moba_lights){
                for(int i = 6; i <26; i++)
                {
                    for(int k = 2; k < 3; k++)
                    {
                        set_led( i, k, 0x00, 0x07, 0x07);
                    }
                }
                for(int i = 6; i <20; i++)
                {
                    for(int k = 3; k < 4; k++)
                    {
                        set_led( i, k, 0x07, 0x07, 0x07);
                    }
                }
                for(int i = 20; i <21; i++)
                {
                    for(int k = 4; k < 5; k++)
                    {
                        set_led( i, k, 0x07, 0x00, 0x00);
                    }
                }
            }
            fix_top_buttons();
            usleep(SPEED*1000);
            update_keyboard();
        }
    }
    return 0;
}

static void set_led( int x, int y, int r, int g, int b )
{
    int led = led_matrix[y][x];

    if(led >= 144)
    {
        return;
    }

    if( r > 7 ) r = 7;
    if( g > 7 ) g = 7;
    if( b > 7 ) b = 7;

    r = 7 - r;
    g = 7 - g;
    b = 7 - b;

    red_val[led] = r;
    grn_val[led] = g;
    blu_val[led] = b;
}


static int init_keyboard()
{
    int status = 0;

    printf("Searching for Corsair K70 RGB keyboard...\n");

    dev = find_device(0x1B1C, 0x1B17);

    if(dev == NULL)
    {
        printf("Corsair K70 RGB keyboard not detected :(\n");
        return 1;
    }
    else
    {
        printf("Corsair K70 RGB keyboard detected successfully :)\n");
    }

    handle = usb_open(dev);

    if(handle == NULL)
    {
        printf("USB device handle did not open :(\n");
        return 1;
    }
    else
    {
        printf("USB device handle opened successfully :)\n");
    }

    status = usb_detach_kernel_driver_np(handle, 3);
    status = usb_claim_interface(handle, 3);

    if(status == 0)
    {
        printf("USB interface claimed successfully :)\n");
    }
    else
    {
        printf("USB interface claim failed with error %d :(\n", status);
//        return 1;
    }

    // Construct XY lookup table
    unsigned char *keys = position_map;
    float *sizes = size_map;
    for (int y = 0; y < 7; y++)
    {
        unsigned char key;
        int size = 0;

        for (int x = 0; x < 92; x++)
        {
            if (size == 0)
            {
                float sizef = *sizes++;
                if (sizef < 0)
                {
                    size = -sizef * 4;
                    key = 255;
                }
                else
                {
                    key = *keys++;
                    size = sizef * 4;
                }
            }

            led_matrix[y][x] = key;
            size--;
        }

        if (*keys++ != 255 || *sizes++ != 0)
        {
            printf("Bad line %d\n", y);
            return 1;
        }
    }

    return 0;
}


static void update_keyboard()
{
    // Perform USB control message to keyboard
    //
    // Request Type:  0x21
    // Request:       0x09
    // Value          0x0300
    // Index:         0x03
    // Size:          64

    data_pkt[0][0] = 0x7F;
    data_pkt[0][1] = 0x01;
    data_pkt[0][2] = 0x3C;

    data_pkt[1][0] = 0x7F;
    data_pkt[1][1] = 0x02;
    data_pkt[1][2] = 0x3C;

    data_pkt[2][0] = 0x7F;
    data_pkt[2][1] = 0x03;
    data_pkt[2][2] = 0x3C;

    data_pkt[3][0] = 0x7F;
    data_pkt[3][1] = 0x04;
    data_pkt[3][2] = 0x24;

        data_pkt[4][0] = 0x07;
        data_pkt[4][1] = 0x27;
        data_pkt[4][4] = 0xD8;
    for(int i = 0; i < 60; i++)
    {
        data_pkt[0][i+4] = red_val[i*2+1] << 4 | red_val[i*2];
    }

    for(int i = 0; i < 12; i++)
    {
        data_pkt[1][i+4] = red_val[i*2+121] << 4 | red_val[i*2+120];
    }

    for(int i = 0; i < 48; i++)
    {
        data_pkt[1][i+16] = grn_val[i*2+1] << 4 | grn_val[i*2];
    }

    for(int i = 0; i < 24; i++)
    {
        data_pkt[2][i+4] = grn_val[i*2+97] << 4 | grn_val[i*2+96];
    }

    for(int i = 0; i < 36; i++)
    {
        data_pkt[2][i+28] = blu_val[i*2+1] << 4 | blu_val[i*2];
    }

    for(int i = 0; i < 36; i++)
    {
        data_pkt[3][i+4] = blu_val[i*2+73] << 4 | blu_val[i*2+72];
    }

    usb_control_msg(handle, 0x21, 0x09, 0x0300, 0x03, data_pkt[0], 64, 1000);
    usb_control_msg(handle, 0x21, 0x09, 0x0300, 0x03, data_pkt[1], 64, 1000);
    usb_control_msg(handle, 0x21, 0x09, 0x0300, 0x03, data_pkt[2], 64, 1000);
    usb_control_msg(handle, 0x21, 0x09, 0x0300, 0x03, data_pkt[3], 64, 1000);
    usb_control_msg(handle, 0x21, 0x09, 0x0300, 0x03, data_pkt[4], 64, 1000);
}


static struct usb_device *find_device(uint16_t vendor, uint16_t product)
{
    struct usb_bus *bus;
    struct usb_device *dev;
    struct usb_bus *busses;

    usb_init();
    usb_find_busses();
    usb_find_devices();

    busses = usb_get_busses();

    for(bus = busses; bus; bus = bus->next)
    {
        for(dev = bus->devices; dev; dev = dev->next)
        {
            if((dev->descriptor.idVendor == vendor) && (dev->descriptor.idProduct == product))
            {
                return dev;
            }
        }
    }

    return NULL;
}
