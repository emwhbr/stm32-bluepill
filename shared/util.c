#include <stdio.h>

#include "util.h"

/////////////////////////////////////////////////////////////

void util_hex_dump(const char *desc, void *addr, int len)
{
   int i;
   unsigned char buff[17];
   unsigned char *pc = (unsigned char*)addr;

   // output description
   if (desc != NULL)
   {
      printf ("%s:\n", desc);
      fflush(stdout);
   }

   // process every byte in the data
   for (i = 0; i < len; i++)
   {
      // multiple of 16 means new line (with line offset).

      if ((i % 16) == 0)
      {
         // Just don't print ASCII for the zeroth line
         if (i != 0)
         {
            printf("  %s\n", buff);
            fflush(stdout);
         }

         // output the offset.
         printf("  %04x ", i);
         fflush(stdout);
      }

      // now the hex code for the specific character
      printf(" %02x", pc[i]);
      fflush(stdout);

      // and store a printable ASCII character for later
      if ((pc[i] < 0x20) || (pc[i] > 0x7e))
      {
         buff[i % 16] = '.';
      } else {
         buff[i % 16] = pc[i];
      }

      buff[(i % 16) + 1] = '\0';
   }

   // pad out last line if not exactly 16 characters
   while ((i % 16) != 0)
   {
      printf("   ");
      fflush(stdout);
      i++;
   }

   // and print the final ASCII bit.
   printf("  %s\n", buff);
   fflush(stdout);
}
