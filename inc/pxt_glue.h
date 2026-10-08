#include "platform_includes.h"

struct BufferInner {
  uint8_t length;
  uint8_t data[];
};

typedef BufferInner * Buffer;

Buffer mkBuffer(void * const data, int length);
