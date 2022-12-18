#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/uart.h"
#include "esp_system.h"
#include <math.h>
#include "./driver/codec.h"
#include "./driver/aic3204.h"
#include "driver/gpio.h"

#define POWER_BUTTON GPIO_NUM_13
#define MODE_SEL_1 GPIO_NUM_26
#define MODE_SEL_2 GPIO_NUM_25
#define MODE_SEL_3 GPIO_NUM_32

extern "C"{
    void app_main();
    void main_process(void *pvParameters);
}

int32_t sin2Table[101] = {
    0x00000,0x00010,0x00041,0x00091,0x00102,0x00193,0x00244,0x00315,
    0x00405,0x00515,0x00644,0x00791,0x008fd,0x00a87,0x00c2f,0x00df3,
    0x00fd5,0x011d3,0x013ed,0x01622,0x01872,0x01adc,0x01d60,0x01ffc,
    0x022b1,0x0257e,0x02861,0x02b5a,0x02e69,0x0318c,0x034c3,0x0380e,
    0x03b6a,0x03ed8,0x04256,0x045e4,0x04980,0x04d2a,0x050e1,0x054a4,
    0x05872,0x05c4a,0x0602b,0x06414,0x06804,0x06bfa,0x06ff5,0x073f4,
    0x077f6,0x07bfb,0x08000,0x08405,0x0880a,0x08c0c,0x0900b,0x09406,
    0x097fc,0x09bec,0x09fd5,0x0a3b6,0x0a78e,0x0ab5c,0x0af1f,0x0b2d6,
    0x0b680,0x0ba1c,0x0bdaa,0x0c128,0x0c496,0x0c7f2,0x0cb3d,0x0ce74,
    0x0d197,0x0d4a6,0x0d79f,0x0da82,0x0dd4f,0x0e004,0x0e2a0,0x0e524,
    0x0e78e,0x0e9de,0x0ec13,0x0ee2d,0x0f02b,0x0f20d,0x0f3d1,0x0f579,
    0x0f703,0x0f86f,0x0f9bc,0x0faeb,0x0fbfb,0x0fceb,0x0fdbc,0x0fe6d,
    0x0fefe,0x0ff6f,0x0ffbf,0x0fff0,0x10000,
};

void setupGPIOs(void){
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << POWER_BUTTON)|(1ULL << MODE_SEL_1)|(1ULL << MODE_SEL_2)|(1ULL << MODE_SEL_3);
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void app_main(){
    setupGPIOs();
    xTaskCreatePinnedToCore(main_process, "main processing task", 32768, NULL, 5, NULL, 1);
}

int32_t pitch_buf[2048];
int32_t* echo_buf;

int32_t linInterpolationI32(int32_t in[],uint32_t idx){
    uint32_t decimal = idx & 0xffff;
    uint32_t integer = idx >> 16;
    int32_t in1 = in[integer];
    int32_t in2 = in[(integer + 1) & 0x7ff]; 
    return (((int64_t)in1 * (0x10000 - decimal))>>16) + (((int64_t)in2 * decimal)>>16);
}

int32_t pitchShift(float ratio,int32_t in){
    static uint32_t rp1 = 0,rp2 = 0,wp = 0;
    int32_t crossfadelen = 100;
    uint32_t intRatio = (uint32_t)(0x10000 * ratio);
    static uint32_t gain1 = 0x10000;
    static uint32_t gain2 = 0x00000;

    pitch_buf[wp] = in;
    int32_t out1 = linInterpolationI32(pitch_buf,rp1);
    int32_t out2 = linInterpolationI32(pitch_buf,rp2);
    int32_t delta1 = wp - (rp1 >> 16);
    int32_t delta2 = wp - (rp2 >> 16);
    
    if(ratio <= 1.0){
        delta1 = -delta1;
        delta2 = -delta2;
    }

    if(0 <= delta1 && delta1 <= crossfadelen){
        gain1 = sin2Table[delta1];
        gain2 = sin2Table[100-delta1];
    }
    if(0 <= delta2 && delta2 <= crossfadelen){
        gain1 = sin2Table[100-delta2];
        gain2 = sin2Table[delta2];
    }

    int32_t ret = ((int64_t)out1 * gain1) >> 16;
    ret += ((int64_t)out2 * gain2) >>16;

    wp = (wp + 1) & 0x7ff;
    rp1 = (rp1 + intRatio) & 0x7ffffff;
    rp2 = (rp1 + 1024*0x10000) & 0x7ffffff;

    return ret;
}

const uint32_t echo_len = 20000;

int32_t echo(float gain,float dump,int32_t in){
    static uint32_t rp = 1,wp = 0;
    static int32_t dump_buf = 0;
    int32_t ret = echo_buf[rp];
    dump_buf = dump_buf*(1.0 - dump) + ret * dump; 
    ret = dump_buf;
    echo_buf[wp] = (int32_t)ret*gain + in;
    wp = (wp + 1) % echo_len;
    rp = (rp + 1) % echo_len;
    return ret;
}

void main_process(void *pvParameters){
    esp_err_t ret;
    ret = sucodec_init();
    aic3204_set_headphone_volume(AIC3204_BOTH,0);
    aic3204_set_line_out_volume(AIC3204_BOTH,18.0);
    aic3204_set_line_out_mute(AIC3204_BOTH,false);
    aic3204_set_dac_digital_volume(AIC3204_BOTH,0.0);
    aic3204_set_adc_routing(AIC3204_IN1_L,AIC3204_L_P,AIC3204_IN_6db);
    aic3204_set_adc_routing(AIC3204_IN1_R,AIC3204_L_P,AIC3204_IN_6db);
    aic3204_set_micbias(AIC3204_MICBIAS_2V075_2V5,AIC3204_MICBIAS_POWER_LDOIN,true);
    if(ret != ESP_OK)printf("ERR at sucodec_init\n");
    int32_t out_buf[96];

    echo_buf = (int32_t*)heap_caps_malloc(sizeof(int32_t) * echo_len,MALLOC_CAP_SPIRAM);
    if(echo_buf == NULL){
        printf("Error in malloc\n");
        while(1){
            vTaskDelay(100);
        }
    }
    for(int i = 0;i < echo_len;i++){
        echo_buf[i] = 0;
    }

    size_t length = 0;
    size_t ret_size;
    float rate = 2.0;
    
    //just loopback
    while(1){
        //Get Button Status
        int power_button = gpio_get_level(POWER_BUTTON);
        int mode[3];
        mode[0] = gpio_get_level(MODE_SEL_1);
        mode[1] = gpio_get_level(MODE_SEL_2);
        mode[2] = gpio_get_level(MODE_SEL_3);

        if(mode[1] == 1){
            rate = 2.0;
        }
        else{
            rate = 0.6;
        }


        ret = i2s_read(I2S_NUM_0,out_buf,48*2*4,&ret_size,portMAX_DELAY);
        for(int i = 0;i < 48;i++){
            if(mode[0] == 1)out_buf[i*2] = pitchShift(rate,out_buf[i*2]);
            //if(mode[2] == 0)out_buf[i*2] = echo(0.3,0.2,out_buf[i*2]);
        }
        if(power_button == 1){
            for(int i = 0;i < 48;i++){
                out_buf[i*2] = 0;
            }
        }
        ret = i2s_write(I2S_NUM_0,out_buf,48*2*4,&ret_size,portMAX_DELAY);
        if(ret != ESP_OK)printf("ERR at i2s_write\n");
    }
}
