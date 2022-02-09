// Utility routines

#include "utility.h"
#include "pf.h"

unsigned long power10(byte p)
{
  unsigned long Answer = 1;
  int i;

  for(i=0;i<p;i++)
    Answer *= 10;    
  return(Answer);      
}

uint32_t strtoln(const char *str, size_t n, bool *error) {
  int i;
  size_t ret = 0;

  for (i = n-1;i >= 0;i--) {
    if (!isdigit(str[i])) {
      if (error)
        *error = 1;
      return(0);
    }
    ret += (str[i] - '0') * power10(n - i - 1);    
  }
  return(ret);
}

// Returns int for hex character c or -1 if not hex
int hexchartoln(char c) {
  c = toupper(c);
  if ((c >= '0') && (c <= '9'))
    return(c - '0');
  if ((c >= 'A') && (c <= 'F'))
    return(c - 'A' + 10);
  return(-1); 
}

uint32_t hextoln(char *str, size_t n, bool *error) {
  size_t i;
  uint32_t ret = 0;
  int ln;       // needs to signed as hexchartoln returns -1 on error

  for (i = 0;i < n;i++) {
    ln = hexchartoln(str[i]);
    if (ln < 0) {
      if (error)
        *error = 1;
      return(0);
    }
    ret += ln << (4*(n-i-1)); 
  }
  return(ret);
}

uint32_t hextoln_little(char *str, size_t n, bool *error) {
  size_t i;
  uint32_t ln = 0;
  bool error_flg = 0;

  if (n == 1) {
    ln = hextoln(str, 1, &error_flg);
  }
  else if ((n%2) || (n>8))
    goto error_label;
  else {
    for (i = 0;i < n/2;i++) {
      ln += hextoln(str, 2, &error_flg)<<(i*8);
      str += 2;
    }  
  }
  if (error_flg)
    goto error_label;
  return(ln);
error_label:
  if (error)
    *error = 1;
  return(0);
}

void lntohex(uint32_t ln, char *ptr, size_t n) {
  size_t i;
  byte nibble;

  for (i = 0;i < n;i++) {
    nibble = ln & 0xf;
    ptr[n-i-1] = nibble > 9?'A'+nibble-10:'0'+nibble;
    ln >>= 4;
  }
}

void lntohex_little(uint32_t ln, char *ptr, size_t n) {
  size_t i;
  if (n == 2)
    lntohex(ln, ptr, 2);
  else if (n%2 || (n > 8))
    return;
  for(i = 0;i < n/2;i++) {
    lntohex((ln & (0xff<<((i+0)*8)))>>((i+0)*8), ptr+i*2, 2);
    lntohex((ln & (0xff<<((i+1)*8)))>>((i+1)*8), ptr+i*2+2, 2);
  }
}

size_t splitline(const char *line, char separator, int nth, char **strptr, int *noofcharsptr) {
  char _nullchar = 0;
  int i;
  int count = 0;
  int currenti = 0;

  //pf("line %s sep %c nth %i\n", line, separator, nth);
  for (i = 0; 1; i++) {
    //pf("i %i c %hhd currenti %i count %i\n",i, line[i], currenti, count); 
    if ((line[i] == separator) || (line[i] == 0)) {
      if (count == nth) {
        //pf ("found i %i c %c currenti %i count %i\n",i, line[i], currenti, count);
        if ((line[currenti] == 0) || (line[currenti] == separator)) {
          *strptr = &_nullchar;
          *noofcharsptr = 0;
          //pf("empty\n");
        }
        else {
          *strptr = &((char *)line)[currenti];
          *noofcharsptr = i - currenti;
          //pf("line %i strptr %i noof %i\n",line, strptr, *noofcharsptr);
        }
      }
      count++;
      currenti = i+1; 
    }      
    if (line[i] == 0) {
      if (line[i-1] == separator)
        count--;
      break;
    }
  }
  return(count);    
}

//#define MAIN
#ifdef MAIN

#include "pf.h"

void setup()
{
  long hex;
  int i;
  char *strptr;
  const char *line;
  int noofchars, nth;
  // put your setup code here, to run once:
  Serial.begin(115200);
  pf("\nSetup %i byte of meemory free\n", freeMemory());
  hex = hexstrtoln("0123aC", 6);
  pf("0123aC returns 0x%lx\n", hex);
  hex = hexstrtoln("0123ag", 6);
  pf("0123ag returns 0x%lx\n", hex);
  hex = strtoln("01234567", 8);
  pf("01234567 returns %ld\n", hex);
  hex = strtoln("0123 456", 8);
  
  pf("0123 456 returns %ld\n", hex);
  line = "456,12,,"; nth = 0;
  i = splitline(line, ',',nth, &strptr,&noofchars);
  pf("splitline %s %i returns %i :%s: %i\n",line, nth, i, strptr,noofchars); 
  line = "456,12,,"; nth = 1;
  i = splitline(line, ',',nth, &strptr,&noofchars);
  pf("splitline %s %i returns %i :%s: %i\n",line, nth, i, strptr,noofchars);
  line = "456,12,,"; nth = 2;
  i = splitline(line, ',',nth, &strptr,&noofchars);
  pf("splitline %s %i returns %i :%s: %i\n",line, nth, i, strptr,noofchars); 
  line = "456,12,,6789"; nth = 3;
  i = splitline(line, ',',nth , &strptr,&noofchars);
  pf("splitline %s %i returns %i :%s: %i\n",line, nth, i, strptr,noofchars);
  line = "456,12,,"; nth = 3;
  i = splitline(line, ',',nth, &strptr,&noofchars);
  pf("splitline %s %i returns %i :%s: %i\n",line, nth, i, strptr,noofchars);
  pf("Success\n");
} 
void loop()
{
 
}


#endif
