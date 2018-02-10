#include "threads/thread.h"
#include <debug.h>
#include <stddef.h>
#include <random.h>
#include <stdio.h>
#include <string.h>
#include "threads/flags.h"
#include "threads/interrupt.h"
#include "threads/intr-stubs.h"
#include "threads/palloc.h"
#include "threads/switch.h"
#include "threads/synch.h"
#include "threads/vaddr.h"
#include "threads/fixed-point.h"
#ifdef USERPROG
#include "userprog/process.h"
#include "devices/timer.h"
#endif

/* Random value for struct thread's `magic' member.
   Used to detect stack overflow.  See the big comment at the top
   of thread.h for details. */
#define THREAD_MAGIC 0xcd6abf4b

/* List of processes in THREAD_READY state, that is, processes
   that are ready to run but not actually running. */
static struct list ready_list;

/* List of all processes.  Processes are added to this list
   when they are first scheduled and removed when they exit. */
static struct list all_list;

/* List of processes in THREAD_BLOCKED state, that is, processes
   that is going to sleep for specific ticks */
static struct list block_list;

/* Idle thread. */
static struct thread *idle_thread;

/* Initial thread, the thread running init.c:main(). */
static struct thread *initial_thread;

/* Lock used by allocate_tid(). */
static struct lock tid_lock;

/* Stack frame for kernel_thread(). */
struct kernel_thread_frame
  {
    void *eip;                  /* Return address. */
    thread_func *function;      /* Function to call. */
    void *aux;                  /* Auxiliary data for function. */
  };

/* Statistics. */
static long long idle_ticks;    /* # of timer ticks spent idle. */
static long long kernel_ticks;  /* # of timer ticks in kernel threads. */
static long long user_ticks;    /* # of timer ticks in user programs. */

/* Scheduling. */
#define TIME_SLICE 4            /* # of timer ticks to give each thread. */
static unsigned thread_ticks;   /* # of timer ticks since last yield. */

/* If false (default), use round-robin scheduler.
   If true, use multi-level feedback queue scheduler.
   Controlled by kernel command-line option "-o mlfqs". */
bool thread_mlfqs;

static void kernel_thread (thread_func *, void *aux);

static void idle (void *aux UNUSED);
static struct thread *running_thread (void);
static struct thread *next_thread_to_run (void);
static void init_thread (struct thread *, const char *name, int priority);
static bool is_thread (struct thread *) UNUSED;
static void *alloc_frame (struct thread *, size_t size);
static void schedule (void);
void thread_schedule_tail (struct thread *prev);
static tid_t allocate_tid (void);

/** Our Prototypes **/
void decrement_donated_priorities (struct thread *t, struct list *donars);
int update_max_priority (struct thread *t);
void switch_thread_if_needed(struct thread *t);
struct thread* get_max_thread (struct list *list);
void change_thread_priority (struct thread *t, int new_priority);
void update_donated_priority (struct thread *t, int old_priority, int new_priority);
void update_priority (struct thread *t, int old_priority, int new_priority);
bool compare_threads_wakeup_time (const struct list_elem *e_one, const struct list_elem *e_two , void *aux);

/** Advanced scheduler prototypes **/
void set_recent_cpu_and_nice (struct thread *t);
void update_load_avg (void);
void update_all_recent_cpu (void);
void update_thread_recent_cpu (struct thread *t);
void update_current_thread_priority (void);
void update_thread_priority (struct thread *t);
void update_all_priorities (void);

/* Initializes the threading system by transforming the code
   that's currently running into a thread.  This can't work in
   general and it is possible in this case only because loader.S
   was careful to put the bottom of the stack at a page boundary.

   Also initializes the run queue and the tid lock.

   After calling this function, be sure to initialize the page
   allocator before trying to create any threads with
   thread_create().

   It is not safe to call thread_current() until this function
   finishes. */
void
thread_init (void)
{
  ASSERT (intr_get_level () == INTR_OFF);

  lock_init (&tid_lock);
  list_init (&ready_list);
  list_init (&all_list);
  list_init (&block_list);

  /* Set up a thread structure for the running thread. */
  initial_thread = running_thread ();
  init_thread (initial_thread, "main", PRI_DEFAULT);

  initial_thread->status = THREAD_RUNNING;
  initial_thread->tid = allocate_tid ();

  // Initializes load_avg to zero at system boot.
  load_avg = 0;
}

/* Starts preemptive thread scheduling by enabling interrupts.
   Also creates the idle thread. */
void
thread_start (void)
{
  /* Create the idle thread. */
  struct semaphore idle_started;
  sema_init (&idle_started, 0);
  thread_create ("idle", PRI_MIN, idle, &idle_started);

  /* Start preemptive thread scheduling. */
  intr_enable ();

  /* Wait for the idle thread to initialize idle_thread. */
  sema_down (&idle_started);
}

/* Called by the timer interrupt handler at each timer tick.
   Thus, this function runs in an external interrupt context. */
void
thread_tick (void)
{
  struct thread *t = thread_current ();
  if (thread_mlfqs)
  {
    // Increments the recent_cpu of the current thread if its not the idle_thread every tick
    if (t != idle_thread)
      t->recent_cpu = add_fixed_point_integer(t->recent_cpu, 1);

    int64_t ticks = timer_ticks ();

    // every second update load avg value and recalculate recent cpu for all threads
    if (ticks % get_timer_frequency () == 0) // one second passed
    {
      update_load_avg ();
      update_all_recent_cpu ();
      update_all_priorities ();
    }

    if (ticks % 4 == 0) // updates the priority of all threads every fourth tick
    {
      update_current_thread_priority ();
      intr_yield_on_return ();
    }
  }

  /* Update statistics. */
  if (t == idle_thread)
    idle_ticks++;
#ifdef USERPROG
  else if (t->pagedir != NULL)
    user_ticks++;
#endif
  else
    kernel_ticks++;

  /* Enforce preemption. */
  if (++thread_ticks >= TIME_SLICE)
    intr_yield_on_return ();
}

int
get_load_avg_first_term (void)
{
  int x = convert_to_fixed_point (59);
  x = divide_fixed_point_integer (x,60);
  x = multiply_two_fixed_point (x, load_avg);
  return x;
}

int
get_load_avg_second_term (int ready_threads)
{
  int x = convert_to_fixed_point (1);
  x = divide_fixed_point_integer (x, 60);
  x = multiply_fixed_point_integer (x, ready_threads);
  return x;
}

void
update_load_avg ()
{
  int ready_threads = list_size (&ready_list);
  if (thread_current() != idle_thread)
    ready_threads++;

  int first_term = get_load_avg_first_term ();
  int second_term = get_load_avg_second_term (ready_threads);

  load_avg = add_two_fixed_point (first_term, second_term);
}

// recalulates recent cpu for all threads (running, ready, or blocked)
void
update_all_recent_cpu ()
{
  if (list_empty (&all_list))
    return;

  struct list_elem  *e ;
  for (e = list_begin (&all_list); e != list_end (&all_list);
       e = list_next (e))
    {
      struct thread *t = list_entry (e, struct thread, allelem);
      update_thread_recent_cpu (t);
    }
}

// update the recent_cpu of a thread from the equation.
// recent_cpu = (2 * load_avg) / (2 * load_avg + 1) * recent_cpu + nice
void
update_thread_recent_cpu (struct thread *t)
{
  int recent_cpu = t->recent_cpu;
  int x = multiply_fixed_point_integer (load_avg, 2);
  int y = add_fixed_point_integer (x, 1);
  int z = divide_two_fixed_point (x, y);
  int first_term = multiply_two_fixed_point (z, recent_cpu);
  t->recent_cpu = add_fixed_point_integer (first_term, t->nice);
}

// update the priorities of all the threads (running, ready, or blocked)
void
update_all_priorities ()
{
  if (list_empty (&all_list))
    return;

  struct list_elem  *e ;
  for (e = list_begin (&all_list); e != list_end (&all_list);
       e = list_next (e))
    {
      struct thread *t = list_entry (e, struct thread, allelem);
      update_thread_priority(t);
    }
}

void
update_current_thread_priority ()
{
  struct thread *t = thread_current ();
  update_thread_priority(t);
}

void
update_thread_priority (struct thread *t)
{
  int x = convert_to_fixed_point (t->pre_computed_priority);
  int y = divide_two_fixed_point (t->recent_cpu, convert_to_fixed_point (4));
  int priority = subtract_two_fixed_point(x, y);
  t->max_priority = convert_to_integer_towards_zero (priority);
}

/* Prints thread statistics. */
void
thread_print_stats (void)
{
  printf ("Thread: %lld idle ticks, %lld kernel ticks, %lld user ticks\n",
          idle_ticks, kernel_ticks, user_ticks);
}

void
switch_thread_if_needed(struct thread *t)
{
  if (t->max_priority > thread_current ()->max_priority)
      thread_yield ();
}

/* Creates a new kernel thread named NAME with the given initial
   PRIORITY, which executes FUNCTION passing AUX as the argument,
   and adds it to the ready queue.  Returns the thread identifier
   for the new thread, or TID_ERROR if creation fails.

   If thread_start() has been called, then the new thread may be
   scheduled before thread_create() returns.  It could even exit
   before thread_create() returns.  Contrariwise, the original
   thread may run for any amount of time before the new thread is
   scheduled.  Use a semaphore or some other form of
   synchronization if you need to ensure ordering.

   The code provided sets the new thread's `priority' member to
   PRIORITY, but no actual priority scheduling is implemented.
   Priority scheduling is the goal of Problem 1-3. */
tid_t
thread_create (const char *name, int priority,
               thread_func *function, void *aux)
{
  struct thread *t;
  struct kernel_thread_frame *kf;
  struct switch_entry_frame *ef;
  struct switch_threads_frame *sf;
  tid_t tid;
  enum intr_level old_level;

  ASSERT (function != NULL);

  /* Allocate thread. */
  t = palloc_get_page (PAL_ZERO);
  if (t == NULL)
    return TID_ERROR;

  /* Initialize thread. */
  init_thread (t, name, priority);


  tid = t->tid = allocate_tid ();

  /* Prepare thread for first run by initializing its stack.
     Do this atomically so intermediate values for the 'stack'
     member cannot be observed. */
  old_level = intr_disable ();

  /* Stack frame for kernel_thread(). */
  kf = alloc_frame (t, sizeof *kf);
  kf->eip = NULL;
  kf->function = function;
  kf->aux = aux;

  /* Stack frame for switch_entry(). */
  ef = alloc_frame (t, sizeof *ef);
  ef->eip = (void (*) (void)) kernel_thread;

  /* Stack frame for switch_threads(). */
  sf = alloc_frame (t, sizeof *sf);
  sf->eip = switch_entry;
  sf->ebp = 0;

  intr_set_level (old_level);

  /* Add to ready queue. */
  thread_unblock (t);

  switch_thread_if_needed(t);

  return tid;
}

bool
compare_threads_wakeup_time (const struct list_elem *e_one,
                             const struct list_elem *e_two , void *aux)
{
    struct thread *t_one = list_entry (e_one, struct thread, elem);
    struct thread *t_two = list_entry (e_two, struct thread, elem);
    return t_one->wakeup_time < t_two->wakeup_time;
}

/* Puts the current thread to sleep.  It will not be scheduled
   again until awoken by thread_unblock().

   This function must be called with interrupts turned off.  It
   is usually a better idea to use one of the synchronization
   primitives in synch.h. */
void
thread_block (void)
{
  ASSERT (!intr_context ());
  ASSERT (intr_get_level () == INTR_OFF);
  thread_current ()->status = THREAD_BLOCKED;
  schedule ();
}

/* Transitions a blocked thread T to the ready-to-run state.
   This is an error if T is not blocked.  (Use thread_yield() to
   make the running thread ready.)

   This function does not preempt the running thread.  This can
   be important: if the caller had disabled interrupts itself,
   it may expect that it can atomically unblock a thread and
   update other data. */
void
thread_unblock (struct thread *t)
{
  enum intr_level old_level;

  ASSERT (is_thread (t));

  old_level = intr_disable ();
  ASSERT (t->status == THREAD_BLOCKED);
  list_push_back(&ready_list, &t->elem);
  t->status = THREAD_READY;
  intr_set_level (old_level);
}

void
thread_sleep (int64_t ticks)
{
  int64_t start = timer_ticks ();
  ASSERT (intr_get_level () == INTR_ON);
  enum intr_level old_level = intr_disable ();

  struct thread *current_thread = thread_current ();
  current_thread->wakeup_time = start + ticks;
  list_insert_ordered (&block_list, &current_thread->elem, &compare_threads_wakeup_time, NULL);
  thread_block ();

  intr_set_level (old_level);
}

void
thread_wakeup ()
{
  int64_t time_now = timer_ticks ();
  ASSERT (intr_get_level () == INTR_OFF);

  if (list_empty (&block_list))
    return;

  struct list_elem  *e ;
  while (!list_empty (&block_list)) // can pop more than one thread
  {
    e = list_begin (&block_list);
    struct thread *t = list_entry (e, struct thread, elem);
    if (t->wakeup_time <= time_now)
    {
      list_pop_front (&block_list);
      thread_unblock (t);
    }
    else break;
  }
}

/* Returns the name of the running thread. */
const char *
thread_name (void)
{
  return thread_current()->name;
}

/* Returns the running thread.
   This is running_thread() plus a couple of sanity checks.
   See the big comment at the top of thread.h for details. */
struct thread *
thread_current (void)
{
  struct thread *t = running_thread ();

  /* Make sure T is really a thread.
     If either of these assertions fire, then your thread may
     have overflowed its stack.  Each thread has less than 4 kB
     of stack, so a few big automatic arrays or moderate
     recursion can cause stack overflow. */
  ASSERT (is_thread (t));
  ASSERT (t->status == THREAD_RUNNING);

  return t;
}

/* Returns the running thread's tid. */
tid_t
thread_tid (void)
{
  return thread_current ()->tid;
}

/* Deschedules the current thread and destroys it.  Never
   returns to the caller. */
void
thread_exit (void)
{
  ASSERT (!intr_context ());

#ifdef USERPROG
  process_exit ();
#endif

  /* Remove thread from all threads list, set our status to dying,
     and schedule another process.  That process will destroy us
     when it calls thread_schedule_tail(). */
  intr_disable ();
  list_remove (&thread_current()->allelem);
  thread_current ()->status = THREAD_DYING;
  schedule ();
  NOT_REACHED ();
}

/* Yields the CPU.  The current thread is not put to sleep and
   may be scheduled again immediately at the scheduler's whim. */
void
thread_yield (void)
{
  struct thread *cur = thread_current ();
  enum intr_level old_level;

  ASSERT (!intr_context ());

  old_level = intr_disable ();
  if (cur != idle_thread)
    list_push_back(&ready_list, &cur->elem);
  cur->status = THREAD_READY;
  schedule ();
  intr_set_level (old_level);
}

/* Invoke function 'func' on all threads, passing along 'aux'.
   This function must be called with interrupts off. */
void
thread_foreach (thread_action_func *func, void *aux)
{
  struct list_elem *e;

  ASSERT (intr_get_level () == INTR_OFF);

  for (e = list_begin (&all_list); e != list_end (&all_list);
       e = list_next (e))
    {
      struct thread *t = list_entry (e, struct thread, allelem);
      func (t, aux);
    }
}

/* Sets the current thread's priority to NEW_PRIORITY. */
void
thread_set_priority (int new_priority)
{
  if (!thread_mlfqs)
  {
    struct thread *t = thread_current ();
    change_thread_priority (t, new_priority);
    thread_yield();
  }
}

/* Returns the current thread's priority. */
int
thread_get_priority (void)
{
  return thread_current ()->max_priority;
}

/* Sets the current thread's nice value to NICE. */
void
thread_set_nice (int nice)
{
  ASSERT (thread_mlfqs);
  struct thread *t = thread_current ();
  t->nice = nice;
  t->pre_computed_priority = PRI_MAX - 2 * nice;
  update_thread_priority (t);
  thread_yield();
}

/* Returns the current thread's nice value. */
int
thread_get_nice (void)
{
  ASSERT (thread_mlfqs);
  return thread_current ()-> nice;
}

/* Returns 100 times the system load average. */
int
thread_get_load_avg (void)
{
  ASSERT (thread_mlfqs);
  int x = multiply_fixed_point_integer (load_avg, 100);
  return convert_to_integer_towards_nearest (x);
}

/* Returns 100 times the current thread's recent_cpu value. */
int
thread_get_recent_cpu (void)
{
  ASSERT (thread_mlfqs);
  int x = multiply_fixed_point_integer (thread_current ()-> recent_cpu, 100);
  return convert_to_integer_towards_nearest (x);
}

/* Idle thread.  Executes when no other thread is ready to run.

   The idle thread is initially put on the ready list by
   thread_start().  It will be scheduled once initially, at which
   point it initializes idle_thread, "up"s the semaphore passed
   to it to enable thread_start() to continue, and immediately
   blocks.  After that, the idle thread never appears in the
   ready list.  It is returned by next_thread_to_run() as a
   special case when the ready list is empty. */
static void
idle (void *idle_started_ UNUSED)
{
  struct semaphore *idle_started = idle_started_;
  idle_thread = thread_current ();
  sema_up (idle_started);

  for (;;)
    {
      /* Let someone else run. */
      intr_disable ();
      thread_block ();

      /* Re-enable interrupts and wait for the next one.

         The `sti' instruction disables interrupts until the
         completion of the next instruction, so these two
         instructions are executed atomically.  This atomicity is
         important; otherwise, an interrupt could be handled
         between re-enabling interrupts and waiting for the next
         one to occur, wasting as much as one clock tick worth of
         time.

         See [IA32-v2a] "HLT", [IA32-v2b] "STI", and [IA32-v3a]
         7.11.1 "HLT Instruction". */
      asm volatile ("sti; hlt" : : : "memory");
    }
}

/* Function used as the basis for a kernel thread. */
static void
kernel_thread (thread_func *function, void *aux)
{
  ASSERT (function != NULL);

  intr_enable ();       /* The scheduler runs with interrupts off. */
  function (aux);       /* Execute the thread function. */
  thread_exit ();       /* If function() returns, kill the thread. */
}

/* Returns the running thread. */
struct thread *
running_thread (void)
{
  uint32_t *esp;

  /* Copy the CPU's stack pointer into `esp', and then round that
     down to the start of a page.  Because `struct thread' is
     always at the beginning of a page and the stack pointer is
     somewhere in the middle, this locates the curent thread. */
  asm ("mov %%esp, %0" : "=g" (esp));
  return pg_round_down (esp);
}

/* Returns true if T appears to point to a valid thread. */
static bool
is_thread (struct thread *t)
{
  return t != NULL && t->magic == THREAD_MAGIC;
}

/* Does basic initialization of T as a blocked thread named
   NAME. */
static void
init_thread (struct thread *t, const char *name, int priority)
{
  ASSERT (t != NULL);
  ASSERT (PRI_MIN <= priority && priority <= PRI_MAX);
  ASSERT (name != NULL);

  memset (t, 0, sizeof *t);
  t->status = THREAD_BLOCKED;
  strlcpy (t->name, name, sizeof t->name);
  t->stack = (uint8_t *) t + PGSIZE;
  t->priority = priority;
  t->max_priority = priority;
  memset(&t->donated_priorities, 0, sizeof t-> donated_priorities);
  t->donated_priorities[priority]++;
  t->waiting_lock = NULL;
  t->pre_computed_priority = PRI_MAX - 2 * t->nice;
  lock_init (&t->priority_lock);
  set_recent_cpu_and_nice(t);
  t->magic = THREAD_MAGIC;
  list_push_back (&all_list, &t->allelem);
}

/* Set nice and recent_cpu */
void
set_recent_cpu_and_nice (struct thread *t)
{
  // If the thread is the intial thread then its recent_cpu and nice equal zero
  if (t == initial_thread)
  {
    t->recent_cpu = 0;
    t->nice = 0;
  }
  // If the thread is not the intial thread
  // then it inherits the recent_cpu and nice from its parent.
  else
  {
    struct thread *current_thread = thread_current ();
    t->recent_cpu = current_thread->recent_cpu;
    t->nice = current_thread->nice;
  }
}

/* Allocates a SIZE-byte frame at the top of thread T's stack and
   returns a pointer to the frame's base. */
static void *
alloc_frame (struct thread *t, size_t size)
{
  /* Stack data is always allocated in word-size units. */
  ASSERT (is_thread (t));
  ASSERT (size % sizeof (uint32_t) == 0);

  t->stack -= size;
  return t->stack;
}

/*  List need to have at least one element. It returns the thread with the maximum priority.
    Works only with list in thread.c and sync.c */
struct thread *
get_max_thread (struct list *list)
{
  int max_priority = PRI_MIN;

  // loop though list
  struct list_elem *e = list_begin (list);
  struct thread *t = list_entry (e, struct thread, elem);
  while (e != list_end (list))
  {
    struct thread *tmp_thread = list_entry (e, struct thread, elem);
    if (tmp_thread->max_priority > max_priority)
    {
      max_priority = tmp_thread->max_priority;
      t = tmp_thread;
    }
    e = list_next(e);
  }
  return t;
}

struct thread *
remove_max_thread (struct list *list)
{
  struct thread *t = get_max_thread(list);
  list_remove(&t->elem);
  return t;
}

/* Chooses and returns the next thread to be scheduled.  Should
   return a thread from the run queue, unless the run queue is
   empty.  (If the running thread can continue running, then it
   will be in the run queue.)  If the run queue is empty, return
   idle_thread. */
static struct thread *
next_thread_to_run (void)
{
  if (list_empty (&ready_list))
    return idle_thread;
  return remove_max_thread (&ready_list);
}

/* Completes a thread switch by activating the new thread's page
   tables, and, if the previous thread is dying, destroying it.

   At this function's invocation, we just switched from thread
   PREV, the new thread is already running, and interrupts are
   still disabled.  This function is normally invoked by
   thread_schedule() as its final action before returning, but
   the first time a thread is scheduled it is called by
   switch_entry() (see switch.S).

   It's not safe to call printf() until the thread switch is
   complete.  In practice that means that printf()s should be
   added at the end of the function.

   After this function and its caller returns, the thread switch
   is complete. */
void
thread_schedule_tail (struct thread *prev)
{
  struct thread *cur = running_thread ();

  ASSERT (intr_get_level () == INTR_OFF);

  /* Mark us as running. */
  cur->status = THREAD_RUNNING;

  /* Start new time slice. */
  thread_ticks = 0;

#ifdef USERPROG
  /* Activate the new address space. */
  process_activate ();
#endif

  /* If the thread we switched from is dying, destroy its struct
     thread.  This must happen late so that thread_exit() doesn't
     pull out the rug under itself.  (We don't free
     initial_thread because its memory was not obtained via
     palloc().) */
  if (prev != NULL && prev->status == THREAD_DYING && prev != initial_thread)
    {
      ASSERT (prev != cur);
      palloc_free_page (prev);
    }
}

/* Schedules a new process.  At entry, interrupts must be off and
   the running process's state must have been changed from
   running to some other state.  This function finds another
   thread to run and switches to it.

   It's not safe to call printf() until thread_schedule_tail()
   has completed. */
static void
schedule (void)
{
  struct thread *cur = running_thread ();
  struct thread *next = next_thread_to_run ();
  struct thread *prev = NULL;

  ASSERT (intr_get_level () == INTR_OFF);
  ASSERT (cur->status != THREAD_RUNNING);
  ASSERT (is_thread (next));

  if (cur != next)
    prev = switch_threads (cur, next);
  thread_schedule_tail (prev);
}

/* Returns a tid to use for a new thread. */
static tid_t
allocate_tid (void)
{
  static tid_t next_tid = 1;
  tid_t tid;

  lock_acquire (&tid_lock);
  tid = next_tid++;
  lock_release (&tid_lock);

  return tid;
}

/* donate the given priority to the given thread. Applying it
 recursivly if thread t is waiting on a lock */
void
donate_priority (struct thread *t, int priority)
{
  t->donated_priorities[priority]++; // get new priority
  // check if thread is waiting on a lock and current max is less than new priority
  if (t->waiting_lock != NULL && t->max_priority < priority)
    update_donated_priority (t->waiting_lock->holder, t->max_priority, priority);

  if (t->max_priority < priority) t->max_priority = priority;
}

void
change_thread_priority (struct thread *t, int new_priority)
{
  int old_priority = t->priority;
  t->priority = new_priority;
  update_priority(t, old_priority, new_priority);
}

void
update_priority (struct thread *t, int old_priority, int new_priority)
{
  lock_acquire (&t->priority_lock);
  t->donated_priorities[old_priority]--;
  t->donated_priorities[new_priority]++;

  int prev_max_priority = t->max_priority;
  update_max_priority (t);

  if (t->waiting_lock != NULL && t->max_priority != prev_max_priority)
    update_priority (t->waiting_lock->holder, prev_max_priority, t->max_priority);

  lock_release (&t->priority_lock);
}

void
update_donated_priority (struct thread *t, int old_priority, int new_priority)
{
  update_priority(t, old_priority, new_priority);
}

void
remove_donated_priorities (struct thread *t, struct list *donars)
{
  if (!thread_mlfqs)
  {
    decrement_donated_priorities (t, donars);
    update_max_priority (t);
  }
}

void
decrement_donated_priorities (struct thread *t, struct list *donars)
{
  struct list_elem *e1 = list_begin (donars);
  while (e1 != list_end (donars))
  {
    struct thread *t_donar = list_entry (e1, struct thread, elem);
    t->donated_priorities[t_donar->max_priority]--;
    e1 = list_next(e1);
  }
}

int
update_max_priority (struct thread *t)
{
  t->max_priority = t->priority;
  for (int i = 0; i < PRI_DOMAIN; i++) {
    if (t->donated_priorities[i] > 0)
      t->max_priority = t->max_priority > i ? t->max_priority : i;
  }
  return t->max_priority;
}

/* Offset of `stack' member within `struct thread'.
   Used by switch.S, which can't figure it out on its own. */
uint32_t thread_stack_ofs = offsetof (struct thread, stack);
