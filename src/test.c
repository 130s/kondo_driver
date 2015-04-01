#include <stdio.h>
#include "kondo_driver/ics.h"

int main(int argc, char** argv)
{
    ICSData r;

    if (ics_init(&r, 0x0008) < 0) {
	fprintf (stderr, "Could not init ICS: %s\n", r.error);
    }
    r.debug=1;
    fprintf (stderr, "ID: %d\n", ics_get_id(&r));
    int id = 0;

    if (argc >=2) {
	id = strtol(argv[1], 0, 10);
    }
    ics_set_id(&r, id);
    // sleep (1);
    fprintf (stderr, "ID: %d\n", ics_get_id(&r));

}
