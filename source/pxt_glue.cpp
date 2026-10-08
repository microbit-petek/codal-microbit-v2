#include "pxt_glue.h"

Buffer mkBuffer(void * const data, int length)
{
  if (length <= 0)
  {
    return NULL;
  }

  Buffer buffer = (Buffer)malloc(sizeof(BufferInner) + length);
  if (data)
  {
    memcpy(buffer->data, data, length);
  }
  else
  {
    memset(buffer->data, 0, length);
  }
  buffer->length = length;
  return buffer;
}
