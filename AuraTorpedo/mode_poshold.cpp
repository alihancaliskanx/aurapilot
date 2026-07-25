// AuraTorpedo position hold flight mode
// GPS required
// Jacob Walser August 2016

#include "Torpedo.h"

#if POSHOLD_ENABLED == ENABLED

// poshold_init - initialise PosHold controller
bool ModePoshold::init(bool ignore_checks)
{
    // AURA torpido: POSHOLD holonomik konum tutamaz (torpido yana gidemez, duramaz).
    // Failsafe geri-dusus modu oldugundan ALT_HOLD gibi derinlik tutar; konum (GPS)
    // sarti yok (requires_GPS=false zaten) ki konumsuz da geri-dusus calissin.
    return ModeAlthold::init(ignore_checks);
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void ModePoshold::run()
{
    // AURA torpido: POSHOLD = ALT_HOLD davranisi (derinlik->pitch dis dongusu +
    // dalis oto-gazi + pilot yaw/pitch seyri). Holonomik xy pozisyon tutma
    // kaldirildi — torpido yapamaz; onceki hali derinlik pitch dongusunu (satih
    // tavani/oto-gaz) hic uygulamiyordu, POSHOLD'da derinlik kaybediliyordu.
    control_depth();
    run_pre();
    run_post();
}
#endif  // POSHOLD_ENABLED == ENABLED
