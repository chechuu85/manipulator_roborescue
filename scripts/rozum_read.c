// Lee telemetría de los servos Rozum sin moverlos.
// Uso: ./rozum_read /dev/ttyACM0 123 124 125
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "api.h"

static const struct
{
    rr_servo_param_t param;
    const char *name;
} params[] = {
    {APP_PARAM_POSITION, "Posicion (deg)"},
    {APP_PARAM_POSITION_GEAR_360, "Posicion 0-360 (deg)"},
    {APP_PARAM_VELOCITY, "Velocidad (deg/s)"},
    {APP_PARAM_VOLTAGE_INPUT, "Tension (V)"},
    {APP_PARAM_CURRENT_INPUT, "Corriente (A)"},
    {APP_PARAM_TEMPERATURE_ELECTRONICS, "Temp. electronica (C)"},
};

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        printf("Uso: %s <puerto> <id> [id...]\n", argv[0]);
        return 1;
    }

    rr_can_interface_t *iface = rr_init_interface(argv[1]);
    if (!iface)
    {
        printf("ERROR: no se pudo abrir %s\n", argv[1]);
        return 1;
    }
    sleep(1);

    for (int i = 2; i < argc; i++)
    {
        int id = atoi(argv[i]);
        rr_servo_t *servo = rr_init_servo(iface, id);
        if (!servo)
        {
            printf("\n[ID %d] no responde\n", id);
            continue;
        }

        printf("\n[ID %d]\n", id);
        for (size_t p = 0; p < sizeof(params) / sizeof(params[0]); p++)
        {
            float value;
            rr_ret_status_t st = rr_read_parameter(servo, params[p].param, &value);
            if (st == RET_OK)
                printf("  %-22s %10.3f\n", params[p].name, value);
            else
                printf("  %-22s ERROR (%d)\n", params[p].name, st);
        }
        rr_deinit_servo(&servo);
    }

    rr_deinit_interface(&iface);
    return 0;
}
