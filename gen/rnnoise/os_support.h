/* Minimal replacement for the Opus os_support.h that RNNoise's generic vector code expects. */
#ifndef ELNOISE_OS_SUPPORT_H
#define ELNOISE_OS_SUPPORT_H
#include <string.h>
#define OPUS_CLEAR(dst, n) memset((dst), 0, (n) * sizeof(*(dst)))
#define OPUS_COPY(dst, src, n) memcpy((dst), (src), (n) * sizeof(*(dst)))
#define OPUS_MOVE(dst, src, n) memmove((dst), (src), (n) * sizeof(*(dst)))
#endif
