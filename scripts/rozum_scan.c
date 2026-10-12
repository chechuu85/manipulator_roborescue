// Escanea el bus CAN de Rozum y lista los servos que responden (heartbeat).
// Uso: ./rozum_scan [/dev/ttyACM0] [id_min] [id_max]
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "api.h"

int main(int argc, char **argv)
{
    const char *port = argc > 1 ? argv[1] : "/dev/ttyACM0";
    int id_min = argc > 2 ? atoi(argv[2]) : 1;
    int id_max = argc > 3 ? atoi(argv[3]) : 127;

    printf("Abriendo interfaz %s ...\n", port);
    rr_can_interface_t *iface = rr_init_interface(port);
    if (!iface)
    {
        printf("ERROR: rr_init_interface falló. Puerto inexistente, ocupado o sin permisos.\n");
        return 1;
    }
    printf("Interfaz abierta. Esperando heartbeats (2 s)...\n");
    sleep(2);

    int found = 0;
    for (int id = id_min; id <= id_max; id++)
    {
        rr_nmt_state_t state;
        if (rr_net_get_state(iface, id, &state) == RET_OK && state != RR_NMT_HB_TIMEOUT)
        {
            printf("  Servo ID %3d -> %s\n", id, rr_describe_nmt(state));
            found++;
        }
    }

    if (found == 0)
        printf("No responde ningún servo. Revisa alimentación, CAN_H/CAN_L, terminación 120 Ω y bitrate.\n");
    else
        printf("Servos encontrados: %d\n", found);

    rr_deinit_interface(&iface);
    return found ? 0 : 2;
}
