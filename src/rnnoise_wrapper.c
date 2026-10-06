/* SPDX-License-Identifier: BSD-3-Clause (RNNoise) - small glue file for elNoise.
 *
 * RNNoise v0.2 ships two trained models with the same network shape: "full" and "little" (lighter).
 * The library only exposes the full one through rnnoise_create(), so this translation unit includes
 * denoise.c and adds a second constructor that initialises a state with the little weights.
 * (denoise.c is compiled through this file, not separately.) */
#include "denoise.c"

extern const WeightArray rnnoise_arrays_light[];
extern int init_rnnoise_light (RNNoise *model, const WeightArray *arrays);

DenoiseState *elnoise_create_light (void)
{
  DenoiseState *st = (DenoiseState *) malloc (rnnoise_get_size ());
  if (st == NULL)
    return NULL;
  memset (st, 0, sizeof (*st));
  if (init_rnnoise_light (&st->model, rnnoise_arrays_light) != 0)
  {
    free (st);
    return NULL;
  }
  st->arch = rnn_select_arch ();
  return st;
}
