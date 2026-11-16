#ifndef RECEIVER_H
#define RECEIVER_H

#include "bloch.h"

void receiver_setup();
void receiver_loop();
bool receiver_apply_permute(const BlochSphere::permute_t permute);
void receiver_reset_bloch();
void receiver_home();
void receiver_apply_machine_name();

#endif // RECEIVER_H