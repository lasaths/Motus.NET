#include "motus_native.h"

static thread_local const char* g_motus_last_error = "";

extern "C" {

const char* motus_last_error(void)
{
    return g_motus_last_error ? g_motus_last_error : "";
}

}

void motus_set_last_error(const char* msg)
{
    g_motus_last_error = msg;
}
