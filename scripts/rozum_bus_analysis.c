// Analiza la calidad del bus CAN de Rozum sin mover los servos.
// Mide: fallos y latencia de lectura (SDO), jitter de heartbeats,
// mensajes EMCY (errores CAN de los servos), cambios de estado NMT
// y estabilidad de la tensión de alimentación.
// Uso: ./rozum_bus_analysis <puerto> <segundos> <id> [id...]
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "api.h"

#define MAX_SERVOS 16
#define MAX_SAMPLES 200000

static int emcy_count = 0;
static int nmt_changes = 0;

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static void on_emcy(rr_can_interface_t *iface, int id, uint16_t code, uint8_t reg, uint8_t bits, uint32_t info)
{
    (void)iface;
    emcy_count++;
    printf("  [EMCY] ID %d code=0x%04X reg=0x%02X bits=0x%02X info=0x%X -> %s / %s\n",
           id, code, reg, bits, info, rr_describe_emcy_code(code), rr_describe_emcy_bit(bits));
}

static void on_nmt(rr_can_interface_t *iface, int id, rr_nmt_state_t state)
{
    (void)iface;
    nmt_changes++;
    printf("  [NMT] ID %d -> %s\n", id, rr_describe_nmt(state));
}

static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

typedef struct
{
    int id;
    rr_servo_t *servo;
    int ok, timeout, error;
    double *lat;
    double v_sum, v_sq, v_min, v_max;
} servo_stats_t;

static void print_error_status(servo_stats_t *s, const char *when)
{
    uint32_t count = 0;
    uint8_t errors[ARRAY_ERROR_BITS_SIZE];
    if (rr_read_error_status(s->servo, &count, errors) != RET_OK)
    {
        printf("  ID %d (%s): no se pudo leer el estado de errores\n", s->id, when);
        return;
    }
    printf("  ID %d (%s): %u errores activos\n", s->id, when, count);
    for (uint32_t i = 0; i < count && i < ARRAY_ERROR_BITS_SIZE; i++)
        printf("      - 0x%02X %s\n", errors[i], rr_describe_emcy_bit(errors[i]));
}

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        printf("Uso: %s <puerto> <segundos> <id> [id...]\n", argv[0]);
        return 1;
    }
    const char *port = argv[1];
    double duration_ms = atof(argv[2]) * 1000.0;
    int n = argc - 3 > MAX_SERVOS ? MAX_SERVOS : argc - 3;

    rr_can_interface_t *iface = rr_init_interface(port);
    if (!iface)
    {
        printf("ERROR: no se pudo abrir %s\n", port);
        return 1;
    }
    rr_setup_emcy_callback(iface, on_emcy);
    rr_setup_nmt_callback(iface, on_nmt);

    servo_stats_t s[MAX_SERVOS];
    memset(s, 0, sizeof(s));
    for (int i = 0; i < n; i++)
    {
        s[i].id = atoi(argv[i + 3]);
        s[i].servo = rr_init_servo(iface, s[i].id);
        s[i].lat = malloc(sizeof(double) * MAX_SAMPLES);
        s[i].v_min = 1e9;
        s[i].v_max = -1e9;
        if (!s[i].servo)
        {
            printf("ERROR: el servo %d no responde\n", s[i].id);
            return 1;
        }
    }

    printf("\n== Estado de errores inicial ==\n");
    for (int i = 0; i < n; i++)
        print_error_status(&s[i], "inicio");

    for (int i = 0; i < n; i++)
        rr_servo_clear_hb_stat(s[i].servo);
    emcy_count = 0;
    nmt_changes = 0;

    printf("\n== Midiendo durante %.0f s (eventos en tiempo real abajo) ==\n", duration_ms / 1000.0);
    double t0 = now_ms();
    while (now_ms() - t0 < duration_ms)
    {
        for (int i = 0; i < n; i++)
        {
            float v;
            double t = now_ms();
            rr_ret_status_t st = rr_read_parameter(s[i].servo, APP_PARAM_VOLTAGE_INPUT, &v);
            double dt = now_ms() - t;
            if (st == RET_OK)
            {
                if (s[i].ok < MAX_SAMPLES)
                    s[i].lat[s[i].ok] = dt;
                s[i].ok++;
                s[i].v_sum += v;
                s[i].v_sq += (double)v * v;
                if (v < s[i].v_min) s[i].v_min = v;
                if (v > s[i].v_max) s[i].v_max = v;
            }
            else if (st == RET_TIMEOUT)
            {
                s[i].timeout++;
            }
            else
            {
                s[i].error++;
            }
        }
    }

    printf("\n== Resultados por servo ==\n");
    printf("%-4s %8s %8s %6s %9s %9s %9s %9s %9s %9s %10s\n",
           "ID", "lecturas", "timeout", "error", "lat_min", "lat_med", "lat_p99", "lat_max",
           "hb_min", "hb_max", "V(min-max)");
    for (int i = 0; i < n; i++)
    {
        int64_t hb_min = -1, hb_max = -1;
        rr_servo_get_hb_stat(s[i].servo, &hb_min, &hb_max);
        int m = s[i].ok < MAX_SAMPLES ? s[i].ok : MAX_SAMPLES;
        double med = 0, p99 = 0, mn = 0, mx = 0;
        if (m > 0)
        {
            qsort(s[i].lat, m, sizeof(double), cmp_double);
            mn = s[i].lat[0];
            med = s[i].lat[m / 2];
            p99 = s[i].lat[(int)(m * 0.99)];
            mx = s[i].lat[m - 1];
        }
        printf("%-4d %8d %8d %6d %7.2fms %7.2fms %7.2fms %7.2fms %7lldms %7lldms %5.2f-%5.2f\n",
               s[i].id, s[i].ok, s[i].timeout, s[i].error, mn, med, p99, mx,
               (long long)hb_min, (long long)hb_max, s[i].v_min, s[i].v_max);
        if (s[i].ok > 1)
        {
            double mean = s[i].v_sum / s[i].ok;
            double sd = sqrt(fmax(0, s[i].v_sq / s[i].ok - mean * mean));
            printf("     tensión media %.3f V, desviación %.4f V\n", mean, sd);
        }
    }

    printf("\n== Eventos durante la medida ==\n");
    printf("  EMCY recibidos: %d\n  Cambios NMT: %d\n", emcy_count, nmt_changes);

    printf("\n== Estado de errores final ==\n");
    for (int i = 0; i < n; i++)
        print_error_status(&s[i], "final");

    for (int i = 0; i < n; i++)
        rr_deinit_servo(&s[i].servo);
    rr_deinit_interface(&iface);
    return 0;
}
