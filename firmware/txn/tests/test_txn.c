/* Host unit tests of the transaction layer against a scripted fake platform.
 * Built and run by host/unit_tests/test_txn.py. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "txn.h"

#define NEVER 0xFFFFFFFFu

static int failures;
static const char *current;

#define CHECK(cond)                                                         \
  do {                                                                      \
    if (!(cond)) {                                                          \
      printf("FAIL %s:%d %s: %s\n", __FILE__, __LINE__, current, #cond);    \
      failures++;                                                           \
    }                                                                       \
  } while (0)

/* --- Fake platform ------------------------------------------------------ */

typedef struct {
  txn_outcome_t outcome;
  uint32_t on_bus_after;  /* after arming, once the bus is free */
  uint32_t duration;      /* on the bus until the outcome */
} attempt_t;

static struct {
  uint32_t now;
  uint32_t bus_free_at;   /* NEVER: another participant holds the bus */
  bool ready;
  bool can_withdraw;
  attempt_t script[64];
  int n_script;
  int next;
  bool active;
  attempt_t cur;
  uint32_t t_armed;
  uint32_t t_on_bus;
  int starts;
  uint32_t start_at[64];
  int start_while_active;
} fk;

static bool fk_on_bus(void)
{
  if (!fk.active || fk.bus_free_at == NEVER || fk.now < fk.bus_free_at) {
    return false;
  }
  if (fk.t_on_bus == NEVER) {
    const uint32_t earliest = fk.t_armed + fk.cur.on_bus_after;
    const uint32_t from = fk.bus_free_at > fk.t_armed ? fk.bus_free_at : fk.t_armed;
    const uint32_t at = from > earliest ? from : earliest;
    if (fk.now < at) {
      return false;
    }
    fk.t_on_bus = at;
  }
  return true;
}

static bool fk_start(void *ctx, const txn_xfer_t *xfer)
{
  (void)ctx;
  (void)xfer;
  if (fk.active) {
    fk.start_while_active++;
    return false;
  }
  if (!fk.ready) {
    return false;
  }
  if (fk.next < fk.n_script) {
    fk.cur = fk.script[fk.next++];
  } else {
    fk.cur = (attempt_t){TXN_SUCCESS, 0u, 10u};
  }
  fk.active = true;
  fk.t_armed = fk.now;
  fk.t_on_bus = NEVER;
  if (fk.starts < 64) {
    fk.start_at[fk.starts] = fk.now;
  }
  fk.starts++;
  return true;
}

static txn_outcome_t fk_poll(void *ctx, bool *on_bus)
{
  (void)ctx;
  *on_bus = fk_on_bus();
  if (*on_bus && fk.now >= fk.t_on_bus + fk.cur.duration) {
    fk.active = false;
    return fk.cur.outcome;
  }
  return TXN_NONE;
}

static bool fk_withdraw(void *ctx)
{
  (void)ctx;
  if (!fk.can_withdraw || !fk.active || fk_on_bus()) {
    return false;
  }
  fk.active = false;
  return true;
}

static uint32_t fk_now(void *ctx)
{
  (void)ctx;
  return fk.now;
}

static const txn_platform_t pf_no_withdraw = {fk_start, fk_poll, NULL, fk_now, NULL};
static const txn_platform_t pf_withdraw = {fk_start, fk_poll, fk_withdraw, fk_now, NULL};

/* --- Harness ------------------------------------------------------------ */

typedef struct {
  int id;
  txn_report_t rep;
} done_t;

static done_t done_log[64];
static int n_done;

static void on_done(void *user, const txn_xfer_t *xfer, const txn_report_t *rep)
{
  (void)xfer;
  done_log[n_done].id = (int)(intptr_t)user;
  done_log[n_done].rep = *rep;
  n_done++;
}

static txn_t t;
static uint8_t payload[2] = {0x11, 0x22};
static const txn_xfer_t xfer = {0x60, payload, 2, NULL, 0};

static void setup(const char *name, const txn_platform_t *pf, uint32_t budget, uint32_t backoff, uint32_t stuck)
{
  current = name;
  memset(&fk, 0, sizeof(fk));
  fk.ready = true;
  fk.bus_free_at = 0u;
  n_done = 0;
  const txn_params_t prm = {budget, backoff, stuck};
  txn_init(&t, pf, &prm);
}

static void script(attempt_t a)
{
  fk.script[fk.n_script++] = a;
}

static bool submit(int id)
{
  return txn_submit(&t, &xfer, on_done, (void *)(intptr_t)id);
}

static void run_until(uint32_t end)
{
  while (fk.now < end) {
    txn_step(&t);
    fk.now++;
  }
  txn_step(&t);
}

/* --- Tests -------------------------------------------------------------- */

static void test_success_first_attempt(void)
{
  setup("success_first_attempt", &pf_no_withdraw, 1000, 100, 1000);
  CHECK(submit(1));
  run_until(100);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_SUCCESS);
  CHECK(done_log[0].rep.last == TXN_SUCCESS);
  CHECK(done_log[0].rep.attempts == 1);
  CHECK(done_log[0].rep.queue_us == 0);
  CHECK(done_log[0].rep.head_us == 10);
  CHECK(txn_idle(&t));
}

static void test_arbitration_lost_is_retried_at_once(void)
{
  setup("arbitration_lost_retried", &pf_no_withdraw, 1000, 100, 1000);
  script((attempt_t){TXN_ARBITRATION_LOST, 0, 20});
  script((attempt_t){TXN_SUCCESS, 0, 10});
  CHECK(submit(1));
  run_until(200);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_SUCCESS);
  CHECK(done_log[0].rep.attempts == 2);
  CHECK(done_log[0].rep.arbitration_lost == 1);
  CHECK(fk.start_at[1] <= fk.start_at[0] + 20 + 1);
}

static void test_addr_nak_backs_off_until_budget(void)
{
  setup("addr_nak_backoff", &pf_no_withdraw, 1000, 100, 1000);
  for (int i = 0; i < 20; i++) {
    script((attempt_t){TXN_ADDR_NAK, 0, 10});
  }
  CHECK(submit(1));
  run_until(2000);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_TIMEOUT);
  CHECK(done_log[0].rep.last == TXN_ADDR_NAK);
  CHECK(done_log[0].rep.attempts == 10);
  CHECK(done_log[0].rep.addr_nak == 10);
  CHECK(done_log[0].rep.head_us == 1000);
  for (int i = 1; i < fk.starts; i++) {
    CHECK(fk.start_at[i] - fk.start_at[i - 1] == 110);
  }
  CHECK(fk.start_at[fk.starts - 1] < 1000);
}

static void test_data_nak_is_final(void)
{
  setup("data_nak_final", &pf_no_withdraw, 1000, 100, 1000);
  script((attempt_t){TXN_DATA_NAK, 0, 10});
  CHECK(submit(1));
  run_until(500);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_DATA_NAK);
  CHECK(done_log[0].rep.attempts == 1);
  CHECK(fk.starts == 1);
}

static void test_aborted_is_final(void)
{
  setup("aborted_final", &pf_no_withdraw, 1000, 100, 1000);
  script((attempt_t){TXN_ABORTED, 0, 10});
  CHECK(submit(1));
  run_until(500);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_ABORTED);
  CHECK(fk.starts == 1);
}

static void test_full_queue_is_busy(void)
{
  setup("full_queue_busy", &pf_no_withdraw, 1000, 100, 1000);
  fk.bus_free_at = NEVER;
  for (int i = 0; i < (int)TXN_QUEUE_DEPTH; i++) {
    CHECK(submit(i));
  }
  CHECK(!submit(99));
  CHECK(txn_pending(&t) == TXN_QUEUE_DEPTH);
  run_until(10);
  CHECK(!submit(99));
  CHECK(fk.starts == 1);
}

static void test_fifo_and_budget_from_head(void)
{
  setup("fifo_budget_from_head", &pf_no_withdraw, 1000, 100, 1000);
  for (int i = 0; i < 3; i++) {
    script((attempt_t){TXN_SUCCESS, 0, 600});
    CHECK(submit(i));
  }
  run_until(3000);
  CHECK(n_done == 3);
  for (int i = 0; i < 3; i++) {
    CHECK(done_log[i].id == i);
    CHECK(done_log[i].rep.outcome == TXN_SUCCESS);
  }
  CHECK(done_log[1].rep.queue_us == 600);
  CHECK(done_log[2].rep.queue_us == 1200);
}

static void test_armed_start_without_withdraw(void)
{
  setup("armed_start_without_withdraw", &pf_no_withdraw, 1000, 100, 500);
  fk.bus_free_at = NEVER;
  CHECK(submit(1));
  run_until(1499);
  CHECK(n_done == 0);
  run_until(1500);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_TIMEOUT);
  CHECK(done_log[0].rep.last == TXN_BUSY);
  CHECK(done_log[0].rep.attempts == 1);
  CHECK(!txn_idle(&t));

  /* The retired attempt keeps the platform until the bus is free (4.4). */
  CHECK(submit(2));
  run_until(1900);
  CHECK(fk.starts == 1);
  fk.bus_free_at = 2000;
  run_until(2100);
  CHECK(n_done == 2);
  CHECK(done_log[1].id == 2);
  CHECK(done_log[1].rep.outcome == TXN_SUCCESS);
  CHECK(done_log[1].rep.attempts == 1);
  CHECK(fk.starts == 2);
  CHECK(fk.start_at[1] >= 2010);
  CHECK(fk.start_while_active == 0);
  CHECK(txn_idle(&t));
}

static void test_armed_start_with_withdraw(void)
{
  setup("armed_start_with_withdraw", &pf_withdraw, 1000, 100, 500);
  fk.bus_free_at = NEVER;
  fk.can_withdraw = true;
  CHECK(submit(1));
  run_until(1000);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_TIMEOUT);
  CHECK(done_log[0].rep.last == TXN_BUSY);
  CHECK(done_log[0].rep.head_us == 1000);
  CHECK(txn_idle(&t));
}

static void test_budget_never_cuts_an_attempt_on_the_bus(void)
{
  setup("budget_never_cuts_on_bus", &pf_withdraw, 1000, 100, 0);
  fk.can_withdraw = true;
  fk.bus_free_at = 900;
  script((attempt_t){TXN_SUCCESS, 0, 300});
  CHECK(submit(1));
  run_until(2000);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_SUCCESS);
  CHECK(done_log[0].rep.head_us == 1200);
}

static void test_arbitration_lost_after_budget_times_out(void)
{
  setup("arbitration_lost_after_budget", &pf_no_withdraw, 1000, 100, 1000);
  fk.bus_free_at = 900;
  script((attempt_t){TXN_ARBITRATION_LOST, 0, 200});
  CHECK(submit(1));
  run_until(2000);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_TIMEOUT);
  CHECK(done_log[0].rep.last == TXN_ARBITRATION_LOST);
  CHECK(fk.starts == 1);
}

static void test_platform_not_ready(void)
{
  setup("platform_not_ready", &pf_no_withdraw, 1000, 100, 1000);
  fk.ready = false;
  CHECK(submit(1));
  run_until(1000);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_TIMEOUT);
  CHECK(done_log[0].rep.last == TXN_BUSY);
  CHECK(done_log[0].rep.attempts == 0);
}

static uint32_t allow_from;

static bool policy_from(void *ctx, uint32_t now_us)
{
  (void)ctx;
  return now_us >= allow_from;
}

static void test_policy_hook_defers_start(void)
{
  setup("policy_hook_defers_start", &pf_no_withdraw, 1000, 100, 1000);
  allow_from = 500;
  txn_set_policy(&t, policy_from, NULL);
  CHECK(submit(1));
  run_until(1000);
  CHECK(n_done == 1);
  CHECK(fk.start_at[0] == 500);
  CHECK(done_log[0].rep.outcome == TXN_SUCCESS);
}

static void resubmit(void *user, const txn_xfer_t *x, const txn_report_t *rep)
{
  on_done(user, x, rep);
  if ((int)(intptr_t)user < 3) {
    CHECK(txn_submit(&t, &xfer, resubmit, (void *)((intptr_t)user + 1)));
  }
}

static void test_submit_from_callback(void)
{
  setup("submit_from_callback", &pf_no_withdraw, 1000, 100, 1000);
  CHECK(txn_submit(&t, &xfer, resubmit, (void *)(intptr_t)1));
  run_until(500);
  CHECK(n_done == 3);
  CHECK(done_log[2].id == 3);
  CHECK(done_log[2].rep.outcome == TXN_SUCCESS);
  CHECK(txn_idle(&t));
}

static void test_unexpected_platform_outcome_aborts(void)
{
  setup("unexpected_outcome_aborts", &pf_no_withdraw, 1000, 100, 1000);
  script((attempt_t){TXN_TIMEOUT, 0, 10});
  CHECK(submit(1));
  run_until(100);
  CHECK(n_done == 1);
  CHECK(done_log[0].rep.outcome == TXN_ABORTED);
}

static void test_outcome_names(void)
{
  current = "outcome_names";
  CHECK(strcmp(txn_outcome_name(TXN_SUCCESS), "Success") == 0);
  CHECK(strcmp(txn_outcome_name(TXN_ARBITRATION_LOST), "ArbitrationLost") == 0);
  CHECK(strcmp(txn_outcome_name(TXN_ADDR_NAK), "AddrNak") == 0);
  CHECK(strcmp(txn_outcome_name(TXN_DATA_NAK), "DataNak") == 0);
  CHECK(strcmp(txn_outcome_name(TXN_ABORTED), "Aborted") == 0);
  CHECK(strcmp(txn_outcome_name(TXN_TIMEOUT), "Timeout") == 0);
  CHECK(strcmp(txn_outcome_name(TXN_BUSY), "Busy") == 0);
  CHECK(strcmp(txn_outcome_name(TXN_NONE), "Unknown") == 0);
}

int main(void)
{
  test_success_first_attempt();
  test_arbitration_lost_is_retried_at_once();
  test_addr_nak_backs_off_until_budget();
  test_data_nak_is_final();
  test_aborted_is_final();
  test_full_queue_is_busy();
  test_fifo_and_budget_from_head();
  test_armed_start_without_withdraw();
  test_armed_start_with_withdraw();
  test_budget_never_cuts_an_attempt_on_the_bus();
  test_arbitration_lost_after_budget_times_out();
  test_platform_not_ready();
  test_policy_hook_defers_start();
  test_submit_from_callback();
  test_unexpected_platform_outcome_aborts();
  test_outcome_names();

  printf("%s: %d failure(s)\n", failures ? "FAILED" : "OK", failures);
  return failures ? 1 : 0;
}
