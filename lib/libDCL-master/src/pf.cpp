#include "pf.h"

void pf(const char *fmt, ... ){
    static char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
    Serial.flush();
}

void pfln(const char *fmt, ... ){
    static char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
    pf();
    Serial.flush();
}

void pf() {
        Serial.print('\n');
}

void pfixs(const char *ptr, size_t n) {
    size_t i;
    for(i = 0;i < n;i++)
        Serial.print(ptr[i]);
    Serial.flush();
}
