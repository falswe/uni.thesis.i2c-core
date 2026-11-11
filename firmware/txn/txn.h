/*
 * Transaction layer for a participant on a shared multi-controller I2C bus.
 *
 * Implements docs/transaction_layer.md sections 3 and 4 with the baseline
 * access policy of section 5.1 (re-arm and retry): a bounded FIFO queue, the
 * budget F from the head of the queue, retry after ArbitrationLost, back-off
 * after AddrNak, final DataNak/Aborted, and the report of section 3.
 *
 * The module is chip-agnostic. The platform arms and polls single attempts
 * through txn_platform_t; everything runs in the caller's main loop
 * (txn_step), no interrupt context is involved.
 */
#ifndef TXN_H
#define TXN_H

#include <stdbool.h>
#include <stdint.h>

#ifndef TXN_QUEUE_DEPTH
#define TXN_QUEUE_DEPTH 8u
#endif

typedef enum {
  TXN_NONE = 0,         /* attempt still running */
  TXN_SUCCESS,
  TXN_ARBITRATION_LOST,
  TXN_ADDR_NAK,
  TXN_DATA_NAK,
  TXN_ABORTED,
  TXN_TIMEOUT,          /* final outcome only */
  TXN_BUSY,             /* full queue; as last attempt outcome: no attempt got onto the bus */
} txn_outcome_t;

/* Canonical status token of docs/protocol.md. */
const char *txn_outcome_name(txn_outcome_t outcome);

/* One transfer: an optional write, then an optional read after a repeated START. */
typedef struct {
  uint8_t addr7;
  const uint8_t *wr;
  uint16_t wr_len;
  uint8_t *rd;
  uint16_t rd_len;
} txn_xfer_t;

typedef struct {
  txn_outcome_t outcome;      /* final outcome */
  txn_outcome_t last;         /* last attempt outcome, TXN_BUSY if no attempt got onto the bus */
  uint16_t attempts;          /* attempts armed */
  uint16_t arbitration_lost;  /* attempts that lost arbitration */
  uint16_t addr_nak;          /* attempts NAKed at the address */
  uint32_t queue_us;          /* submission to head of the queue */
  uint32_t head_us;           /* head of the queue to final outcome */
} txn_report_t;

typedef void (*txn_done_fn)(void *user, const txn_xfer_t *xfer, const txn_report_t *report);

typedef struct {
  /* Arm one attempt; the peripheral waits for the free bus in hardware (4.2.1).
   * Return false if the platform cannot arm now, e.g. while it serves a
   * transfer as target; the layer asks again on a later step. */
  bool (*attempt_start)(void *ctx, const txn_xfer_t *xfer);
  /* TXN_NONE while the attempt runs, else its outcome. *on_bus tells whether
   * the START has left the peripheral (the controller competes or owns the bus). */
  txn_outcome_t (*attempt_poll)(void *ctx, bool *on_bus);
  /* Withdraw an armed START that has not left the peripheral (4.3). NULL if
   * the platform cannot do that atomically (platform_capabilities.md C2). */
  bool (*attempt_withdraw)(void *ctx);
  uint32_t (*now_us)(void *ctx);
  void *ctx;
} txn_platform_t;

typedef struct {
  uint32_t budget_us;   /* F */
  uint32_t backoff_us;  /* t_backoff */
  /* An armed START that cannot be withdrawn runs when the bus becomes free
   * (4.3). If it is still armed stuck_us after F, the bus is taken as not
   * becoming free: the transfer ends with Timeout (last Busy) and the attempt
   * is retired before the next one starts (4.4). */
  uint32_t stuck_us;
} txn_params_t;

/* May an attempt start now? NULL selects the baseline policy (5.1): always. */
typedef bool (*txn_policy_fn)(void *ctx, uint32_t now_us);

typedef struct {
  txn_xfer_t xfer;
  txn_done_fn done;
  void *user;
  uint32_t t_submit;
} txn_entry_t;

typedef struct {
  const txn_platform_t *pf;
  txn_params_t prm;
  txn_policy_fn policy;
  void *policy_ctx;
  txn_entry_t q[TXN_QUEUE_DEPTH];
  uint8_t head;
  uint8_t count;
  uint8_t state;
  bool stale;           /* an attempt of an earlier transfer is still armed */
  uint32_t t_head;
  uint32_t t_next;      /* earliest start of the next attempt (back-off) */
  txn_report_t rep;
} txn_t;

void txn_init(txn_t *t, const txn_platform_t *pf, const txn_params_t *prm);
void txn_set_params(txn_t *t, const txn_params_t *prm);
void txn_set_policy(txn_t *t, txn_policy_fn policy, void *ctx);

/* Queue a transfer. Returns false if the queue is full (Busy, 4.1). The
 * buffers of xfer must stay valid until done is called from txn_step, and
 * after a Timeout with last Busy until txn_idle() is true again: the retired
 * attempt still runs when the bus becomes free (stuck_us). */
bool txn_submit(txn_t *t, const txn_xfer_t *xfer, txn_done_fn done, void *user);

/* Advance the layer; call from the main loop as often as possible. */
void txn_step(txn_t *t);

/* Transfers queued, including the one in progress. */
uint8_t txn_pending(const txn_t *t);

/* No transfer queued and no earlier attempt still armed. */
bool txn_idle(const txn_t *t);

#endif /* TXN_H */
