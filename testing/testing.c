
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CFG_SIGNAL_SAMPLES_PER_POINT        16
#define CFG_SIGNAL_POINTS_PER_DIFF          16
#define CFG_SENS_UNIT_INT                   100
#define CFG_SENS_UV_PER_AMPER               66000
#define CFG_ADC_UV_QUANTUM                  4882

#define ABS(x)                              ((x < 0) ? (-x) : x)

typedef int16_t signal;
typedef int32_t signal_diff;


static FILE * fd;
static FILE * fd_avr;
static FILE * fd_diff1;
static FILE * fd_diff2;
static FILE * fd_mul;

static signal g_signal;
static signal g_signal_abs;
static signal g_signal_diff1;
static signal g_signal_diff2;

static bool g_is_data_ready;

static void signal_insert_sample(signal sample)
{
    static signal               signal_acc;
    static uint8_t              signal_index;
    
    signal_acc += sample;
    signal_index++;

    if (signal_index == CFG_SIGNAL_SAMPLES_PER_POINT) {
        signal_index = 0;

        g_signal   = (int32_t)signal_acc
            * (int32_t)(10l * CFG_SENS_UNIT_INT / CFG_SIGNAL_SAMPLES_PER_POINT)
            / (int32_t)(10l * CFG_SENS_UV_PER_AMPER / CFG_ADC_UV_QUANTUM);
        signal_acc = 0;
        g_is_data_ready = true;
    }
}

static signal_diff signal_compute_diff1(
    signal                      signal_sample)
{
    signal_diff                 ret;
    
    static signal               signal_buff[CFG_SIGNAL_POINTS_PER_DIFF];
    static uint8_t              signal_idx;

    signal_buff[signal_idx++] = signal_sample;
    
    if (signal_idx == CFG_SIGNAL_POINTS_PER_DIFF) {
        signal_idx = 0;
    }
    ret = (signal_diff)signal_sample - (signal_diff)signal_buff[signal_idx];

    return (ret);
}

static signal_diff signal_compute_diff2(
    signal_diff                 diff_sample)
{
    signal_diff                 ret;
    
    static signal_diff          diff_buff[CFG_SIGNAL_POINTS_PER_DIFF];
    static uint8_t              diff_idx;

    diff_buff[diff_idx++] = diff_sample;
    
    if (diff_idx == CFG_SIGNAL_POINTS_PER_DIFF) {
        diff_idx = 0;
    }
    ret = diff_sample - diff_buff[diff_idx];

    return (ret);
}

static void signal_process(void)
{
    g_signal_abs   = ABS(g_signal);
    g_signal_diff1 = signal_compute_diff1(g_signal_abs);

    if (g_signal_diff1 < 0) {
        g_signal_diff1 = 0;
    }
    g_signal_diff2 = signal_compute_diff2(g_signal_diff1);
}

int main (int argc, char *argv[]) {
    char *              in_name;
    int16_t             in_name_len;
    char                diff1_name[100];
    char                diff2_name[100];
    char                avr_name[100];
    char                mul_name[100];
    int                 sample_i;
    
    in_name = argv[1];
    
    if ((argc == 1) || (in_name == NULL)) {
        printf("\nINFO: Specify input file name\n");
        
        return (0);
    }
    in_name_len = strlen(in_name);
    strcpy(&diff1_name[0], in_name);
    strcpy(&diff1_name[in_name_len], "_out_diff1");
    strcpy(&diff2_name[0], in_name);
    strcpy(&diff2_name[in_name_len], "_out_diff2");
    strcpy(&avr_name[0], in_name);
    strcpy(&avr_name[in_name_len], "_out_avr");
    strcpy(&mul_name[0], in_name);
    strcpy(&mul_name[in_name_len], "_out_mul");
    printf("\nINFO: Open IN  file: %s\n", in_name);
    fd     = fopen(in_name, "r");
    
    if (fd == NULL) {
        printf("\nERR: Could not open IN file\n");
            
        return (0);
    }
    printf("\nINFO: Open OUT AVR file: %s\n", avr_name);
    fd_avr = fopen(avr_name, "w");
    
    if (fd_avr == NULL) {
        printf("\nERR: Could not open OUT AVR file\n");
        fclose(fd);
        
        return (0);
    }
    printf("\nINFO: Open OUT DIFF1 file: %s\n", diff1_name);
    fd_diff1 = fopen(diff1_name, "w");
    
    if (fd_diff1 == NULL) {
        printf("\nERR: Could not open OUT DIFF1 file\n");
        fclose(fd_avr);
        fclose(fd);
        
        return (0);
    }
    printf("\nINFO: Open OUT DIFF2 file: %s\n", diff1_name);
    fd_diff2 = fopen(diff2_name, "w");
    
    if (fd_diff2 == NULL) {
        printf("\nERR: Could not open OUT DIFF2 file\n");
        fclose(fd_diff1);
        fclose(fd_avr);
        fclose(fd);
        
        return (0);
    }
    printf("\nINFO: Open OUT MUL file: %s\n", diff1_name);
    fd_mul = fopen(mul_name, "w");
    
    if (fd_mul == NULL) {
        printf("\nERR: Could not open OUT MUL file\n");
        fclose(fd_diff2);
        fclose(fd_diff1);
        fclose(fd_avr);
        fclose(fd);
        
        return (0);
    }
    rewind(fd);
    
    while (fscanf(fd, "%d", &sample_i) >= 0) {
        signal_insert_sample(sample_i);
        
        if (g_is_data_ready) {
            static uint32_t     idx;
            int32_t             mul;
        
            g_is_data_ready = false;
            
            signal_process();
            mul = (int32_t)g_signal_diff1 * (int32_t)g_signal_diff2;
            printf("INFO: no: %4d  ::  sample: %6d => diff1: %6d, diff2: %6d, mul: %6d\n", idx++, g_signal, g_signal_diff1, g_signal_diff2, mul);
            fprintf(fd_avr, "%d,\n", g_signal);
            fprintf(fd_diff1, "%d,\n", g_signal_diff1);
            fprintf(fd_diff2, "%d,\n", g_signal_diff2);
            fprintf(fd_mul, "%d,\n", mul);
        }
    }
    fclose(fd_mul);
    fclose(fd_diff2);
    fclose(fd_diff1);
    fclose(fd_avr);
    fclose(fd);
    
    return (0);
}

