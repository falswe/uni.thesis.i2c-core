#include "txn.h"

#include <stddef.h>
#include <string.h>

enum { S_IDLE, S_WAIT, S_RUN };

const char *txn_outcome_name(txn_outcome_t outcome)
{
  switch (outcome) {
    case TXN_SUCCESS:
      return "Success";
    case TXN_ARBITRATION_LOST:
      return "ArbitrationLost";
    case TXN_ADDR_NAK:
      return "AddrNak";
    case TXN_DATA_NAK:
      return "DataNak";
    case TXN_ABORTED:
      return "Aborted";
    case TXN_TIMEOUT:
      return "Timeout";
    case TXN_BUSY:
      return "Busy";
    case TXN_NONE:
    default:
      return "Unknown";
  }
}

static uint32_t now(const txn_t *t)
{
  return t->pf->now_us(t->pf->ctx);
}

static void start_head(txn_t *t, uint32_t at)
{
  memset(&t->rep, 0, sizeof(t->rep));
  t->rep.last = TXN_BUSY;
  t->rep.queue_us = at - t->q[t->head].t_submit;
  t->t_head = at;
  t->t_next = at;
  t->state = S_WAIT;
}

static void finish(txn_t *t, txn_outcome_t outcome, uint32_t at)
{
  const txn_entry_t e = t->q[t->head];
  txn_report_t r = t->rep;

  r.outcome = outcome;
  r.head_us = at - t->t_head;

  t->head = (uint8_t)((t->head + 1u) % TXN_QUEUE_DEPTH);
  t->count--;
  t->state = S_IDLE;
  if (t->count != 0u) {
    start_head(t, at);
  }

  /* Last, so that the callback may submit the next transfer. */
  if (e.done != NULL) {
    e.done(e.user, &e.xfer, &r);
  }
}

void txn_init(txn_t *t, const txn_platform_t *pf, const txn_params_t *prm)
{
  memset(t, 0, sizeof(*t));
  t->pf = pf;
  t->prm = *prm;
  t->state = S_IDLE;
}

void txn_set_params(txn_t *t, const txn_params_t *prm)
{
  t->prm = *prm;
}

void txn_set_policy(txn_t *t, txn_policy_fn policy, void *ctx)
{
  t->policy = policy;
  t->policy_ctx = ctx;
}

bool txn_submit(txn_t *t, const txn_xfer_t *xfer, txn_done_fn done, void *user)
{
  if (t->count >= TXN_QUEUE_DEPTH) {
    return false;
  }

  const uint32_t at = now(t);
  const uint8_t slot = (uint8_t)((t->head + t->count) % TXN_QUEUE_DEPTH);
  t->q[slot].xfer = *xfer;
  t->q[slot].done = done;
  t->q[slot].user = user;
  t->q[slot].t_submit = at;
  t->count++;

  if (t->count == 1u) {
    start_head(t, at);
  }
  return true;
}

uint8_t txn_pending(const txn_t *t)
{
  return t->count;
}

bool txn_idle(const txn_t *t)
{
  return t->count == 0u && !t->stale;
}

static bool expired(const txn_t *t, uint32_t at)
{
  return at - t->t_head >= t->prm.budget_us;
}

/* Evaluate the attempt in flight; leaves S_RUN when it has an outcome. */
static void poll_attempt(txn_t *t, uint32_t at)
{
  bool on_bus = false;
  const txn_outcome_t outcome = t->pf->attempt_poll(t->pf->ctx, &on_bus);

  switch (outcome) {
    case TXN_NONE:
      /* F never cuts an attempt that is on the bus (4.3). */
      if (!expired(t, at) || on_bus) {
        return;
      }
      if (t->pf->attempt_withdraw != NULL && t->pf->attempt_withdraw(t->pf->ctx)) {
        t->rep.last = TXN_BUSY;
        finish(t, TXN_TIMEOUT, at);
      } else if (at - t->t_head - t->prm.budget_us >= t->prm.stuck_us) {
        t->stale = true;
        t->rep.last = TXN_BUSY;
        finish(t, TXN_TIMEOUT, at);
      }
      return;

    case TXN_ARBITRATION_LOST:
      t->rep.arbitration_lost++;
      t->rep.last = outcome;
      if (expired(t, at)) {
        finish(t, TXN_TIMEOUT, at);
      } else {
        t->state = S_WAIT;
        t->t_next = at;
      }
      return;

    case TXN_ADDR_NAK:
      t->rep.addr_nak++;
      t->rep.last = outcome;
      if (expired(t, at)) {
        finish(t, TXN_TIMEOUT, at);
      } else {
        t->state = S_WAIT;
        t->t_next = at + t->prm.backoff_us;
      }
      return;

    case TXN_SUCCESS:
    case TXN_DATA_NAK:
    case TXN_ABORTED:
      t->rep.last = outcome;
      finish(t, outcome, at);
      return;

    default:
      /* Not an attempt outcome (section 3); treat it as a local abort. */
      t->rep.last = TXN_ABORTED;
      finish(t, TXN_ABORTED, at);
      return;
  }
}

/* Arm the next attempt of the head transfer if the rules allow it. */
static void try_start(txn_t *t, uint32_t at)
{
  if (expired(t, at)) {
    finish(t, TXN_TIMEOUT, at);
    return;
  }
  if (t->stale || (int32_t)(at - t->t_next) < 0) {
    return;
  }
  if (t->policy != NULL && !t->policy(t->policy_ctx, at)) {
    return;
  }
  if (!t->pf->attempt_start(t->pf->ctx, &t->q[t->head].xfer)) {
    return;
  }
  t->rep.attempts++;
  t->state = S_RUN;
}

void txn_step(txn_t *t)
{
  const uint32_t at = now(t);
  bool on_bus = false;

  /* A late completion of a retired attempt belongs to no transfer (4.4). */
  if (t->stale && t->pf->attempt_poll(t->pf->ctx, &on_bus) != TXN_NONE) {
    t->stale = false;
  }

  if (t->state == S_RUN) {
    poll_attempt(t, at);
  }
  /* Same step: re-arm after ArbitrationLost, or arm the next head at once. */
  if (t->state == S_WAIT) {
    try_start(t, at);
  }
}
