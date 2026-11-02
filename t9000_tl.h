/* t9000_tl.h - a link's recent busy intervals, for the T9000 network cycle
 * model (t9000_group.h, t4_fabric.c).
 *
 * Nodes and routers run as host threads, so packets reach a link in host
 * order, which is not cycle order.  A link that only kept "free from cycle
 * X" had to either queue an early-stamped latecomer behind packets it
 * really preceded, or add its bytes to X and make every later packet wait
 * for time the link had free; repeated, that inflated one 8-node all-gather
 * by up to half.  Keeping the last few transfers instead lets a latecomer
 * take the first gap at or after its ready cycle, and in-order traffic is
 * placed exactly as before: after the last transfer, or at its own ready
 * cycle if that is later. */
#ifndef T9000_TL_H
#define T9000_TL_H

#include <stdint.h>
#include <string.h>

#define T9K_TL 16

typedef struct {
    uint64_t s[T9K_TL], e[T9K_TL];      /* by start, not overlapping */
    int      n;
} t9k_tl_t;

/* Where a transfer of dur cycles ready at `ready` starts: in the first gap
 * at or after `ready` among the recorded transfers.  Nothing is recorded. */
static inline uint64_t t9k_tl_fit(const t9k_tl_t *tl, uint64_t ready,
                                  uint64_t dur)
{
    uint64_t t = ready;
    int i;

    for (i = 0; i < tl->n; i++) {
        if (t + dur <= tl->s[i])
            break;                      /* fits in the gap before it */
        if (t < tl->e[i])
            t = tl->e[i];
    }
    return t;
}

/* Record a transfer that starts at `start` (from t9k_tl_fit); the oldest
 * goes when the record is full. */
static inline void t9k_tl_add(t9k_tl_t *tl, uint64_t start, uint64_t dur)
{
    int i = tl->n;

    while (i > 0 && tl->s[i - 1] > start)
        i--;
    if (tl->n == T9K_TL) {
        if (i == 0)
            return;                     /* older than all it remembers */
        memmove(&tl->s[0], &tl->s[1], (size_t)(i - 1) * sizeof tl->s[0]);
        memmove(&tl->e[0], &tl->e[1], (size_t)(i - 1) * sizeof tl->e[0]);
        i--;
    } else {
        memmove(&tl->s[i + 1], &tl->s[i], (size_t)(tl->n - i) * sizeof tl->s[0]);
        memmove(&tl->e[i + 1], &tl->e[i], (size_t)(tl->n - i) * sizeof tl->e[0]);
        tl->n++;
    }
    tl->s[i] = start;
    tl->e[i] = start + dur;
}

static inline uint64_t t9k_tl_place(t9k_tl_t *tl, uint64_t ready, uint64_t dur)
{
    uint64_t t = t9k_tl_fit(tl, ready, dur);

    t9k_tl_add(tl, t, dur);
    return t;
}

#endif /* T9000_TL_H */
