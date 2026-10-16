/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <samd.h>
#include <stddef.h>
#include <errno.h>

/* How much free space to keep between the stack and the heap by malloc.
 * Can be changed by the application at any time. */
size_t __malloc_margin = 64;

void *_sbrk(int incr)
{
  extern char end; /* End of global variables, defined by the linker */
  static char *brkval = &end ;
  char *prev_brkval = brkval;

  if (incr > 0 && brkval + incr + __malloc_margin > (char *)__get_MSP()) {
    /* Heap and stack collision */
    errno = ENOMEM;
    return (void*) -1;
  }

  brkval += incr ;
  return (void*) prev_brkval;
}
