#include <stdio.h>
#include "kondo_driver/ics.h"

int main(int argc, char **argv)
{
    ICSData ics;
    int product_id = strtol(argv[1], 0, 16);
    int servo_id = atoi(argv[2]);
    int ret;
    fprintf (stderr, "product_id: %x\n", product_id);
    fprintf (stderr, "servo_id: %d\n", servo_id);
    // Initiallize ICS interface
    if (ics_init(&ics, product_id) < 0) {
	fprintf (stderr, "Could not init ICS: %s\n", ics.error);
	exit(0);
    }
    ics.debug = 1;
    // Get servo Id
    ret = ics_get_id (&ics);
    fprintf (stderr, "Current servo ID: %x\n", ret);

    // Set servo Id
    ret = ics_set_id (&ics, servo_id);
    if (servo_id != ret) {
	fprintf (stderr, "Could not set ID.\n");
	exit(0);
    }
    sleep (3);
    fprintf (stderr, "Set servo ID correctly: %d\n", servo_id);
    ret = ics_get_id (&ics);

    return 0;
}	
