// utility header file
#pragma once

#include <Arduino.h>

// Returns 10 to the power of p
extern unsigned long power10(byte p);

// strtoln - Convert a fixed length string (without a terminating NULL) of decimal digits to a long
// Returns 0 and sets *error if non decimal character found
extern uint32_t strtoln(char *str, size_t noofchars, bool *error = NULL);

// hextoln - Convert a fixed length string (without a terminating NULL)
// of n hex digits to a 32bit number.
// Returns 0 and sets *error if non hex character found
extern uint32_t hextoln(char *str, size_t n, bool *error = NULL);

// hextoln_little - Convert a fixed length string (without a terminating NULL)
// of n hex digits to a 32bit number. Little endian style
// Returns 0 ands sets *error if non hex character found or n not 1,2,4,6 or 8
extern uint32_t hextoln_little(char *str, size_t n, bool *error = NULL);

// lntohex - convert 32 bit number ln to an n digit hex string at ptr
extern void lntohex(uint32_t ln, char *ptr, size_t n); 

// lntohex_little - convert 32 bit number ln to an n digit hex string at ptr
// Little endian style
// does nothing if n is odd or > 8
extern void lntohex_little(uint32_t ln, char *ptr, size_t n);

// Splits line using the separator character and puts a ptr to nth string (not null terminated)
// in strptr and the noofchars in the sting in noofcharsptr)
// It returns the number of strings found or 0 if no strings
// ie splitline("456,12,,", ',',1, returns 3 "12" 2
// splitline("456,12,,6789", ',',, returns 4 "6789" 4
// splitline("456,12,,", ',',1, returns 3 "12" 2
extern size_t splitline(const char *line, char separator, int nth, char **strptr, int *noofcharsptr);
