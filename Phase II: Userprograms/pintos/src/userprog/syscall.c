#include "userprog/syscall.h"
#include <stdio.h>
#include <syscall-nr.h>
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/vaddr.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "userprog/pagedir.h"

// wrapper functions for system call functions
// extract the parameters of from the stack pointer
// and paas them correctly to the system call function
void halt_wrapper (void);
void exit_wrapper (void);
pid_t exec_wrapper (void);
int wait_wrapper (void);
bool create_wrapper (void);
bool remove_wrapper (void);
int open_wrapper (void);
int filesize_wrapper (void);
int read_wrapper (void);
int write_wrapper (void);
void seek_wrapper (void);
unsigned tell_wrapper (void);
void close_wrapper (void);

void validate_address (void *address);
void* get_void_pointer (void*** stack_pointer);
char* get_char_pointer (char*** stack_pointer);
int get_int (int **stack_pointer);
void* stack_pointer;
// struct list all_files;
struct lock file_system_lock;

static void syscall_handler (struct intr_frame *);

void
syscall_init (void)
{
  lock_init (&file_system_lock);
  intr_register_int (0x30, 3, INTR_ON, syscall_handler, "syscall");
}

static void
syscall_handler (struct intr_frame *f UNUSED)
{
  stack_pointer = f->esp;
  validate_address (f->esp);
  int sys_call = get_int ((int**) &stack_pointer);
  switch (sys_call)
  {
    case SYS_HALT:
        halt_wrapper ();
        break;
    case SYS_EXIT:
        exit_wrapper ();
        break;
    case SYS_EXEC:
        f->eax = exec_wrapper ();
        break;
    case SYS_WAIT:
        f->eax = wait_wrapper ();
        break;
    case SYS_CREATE:
        f->eax = create_wrapper ();
        break;
    case SYS_REMOVE:
        f->eax = remove_wrapper ();
        break;
    case SYS_OPEN:
        f->eax = open_wrapper ();
        break;
    case SYS_FILESIZE:
        f->eax = filesize_wrapper ();
        break;
    case SYS_READ:
        f->eax = read_wrapper ();
        break;
    case SYS_WRITE:
        f->eax = write_wrapper ();
        break;
    case SYS_SEEK:
        seek_wrapper ();
        break;
    case SYS_TELL:
        f->eax = tell_wrapper ();
        break;
    case SYS_CLOSE:
        close_wrapper ();
        break;
    default:
      break;
  }
}


void
halt_wrapper (void)
{

  shutdown_power_off();
}

void
exit (int status)
{
  struct thread* child = thread_current ();
  struct thread* parent = get_thread (child->parent_id);
  if (parent != NULL) // update child info of the parent
  {
    struct child_info* child_info_elem = get_child_from_parent (parent, child->tid);
    if (child_info_elem != NULL)
    {
      lock_acquire (&parent->parent_waiting_lock);
      child_info_elem->exit_status = status;
      lock_release (&parent->parent_waiting_lock);
    }
  }

  child->exit_status = status;
  thread_exit ();
}

void
exit_wrapper (void)
{
  int status = get_int ((int**) &stack_pointer);
  exit (status);
}

pid_t
exec_wrapper (void)
{
  char *cmd_line = get_char_pointer ((char***) &stack_pointer);
  // return process_execute (cmd_line);
  int pid = process_execute (cmd_line);
  if (pid == -1) return -1;

  sema_down (&thread_current ()->exec_sema);
  struct child_info* child = get_child_from_parent(thread_current(), pid);
  if (child != NULL && child->is_loaded == false) return -1;
  return pid;
}

int
wait_wrapper (void)
{
  int pid = get_int ((int**) &stack_pointer);
  return process_wait (pid);
}

bool
create_wrapper (void)
{
  char* file = get_char_pointer ((char***) &stack_pointer);
  unsigned initial_size = get_int ((int**) &stack_pointer);
  lock_acquire (&file_system_lock);
  bool create_status = filesys_create (file, initial_size);
  lock_release (&file_system_lock);
  return create_status;
}

bool
remove_wrapper (void)
{
  char* file = get_void_pointer ((char***) &stack_pointer);
  lock_acquire (&file_system_lock);
  bool remove_status = filesys_remove (file);
  lock_release (&file_system_lock);
  return remove_status;
}

int
open_wrapper (void)
{
  struct thread* cur = thread_current ();
  char* file_name = get_char_pointer ((char***) &stack_pointer);
  lock_acquire (&file_system_lock);
  struct file* file = filesys_open (file_name);
  int descriptor_return = -1;
  if (file != NULL)
  {
    struct file_descriptor *file_des = malloc (sizeof (struct file_descriptor));
    file_des->file = file;
    file_des->fid = cur->fid_generator++;
    descriptor_return = file_des->fid;
    list_push_back (&cur->files, &file_des->thread_elem);
    // list_push_back (&all_files, &file_des->elem);
  }
  lock_release (&file_system_lock);
  return descriptor_return;
}

int
filesize_wrapper (void)
{
  struct thread* cur = thread_current ();
  int fid = get_int ((int**) &stack_pointer);
  lock_acquire (&file_system_lock);
  struct file_descriptor* file_des = get_file_descriptor (cur, fid);
  int length = -1;
  if (file_des != NULL)
  {
    struct file* file = file_des->file;
    length = file_length (file);
  }
  lock_release (&file_system_lock);
  return length;
}

int
read_wrapper (void)
{
  int fd = get_int ((int**) &stack_pointer);
  void* buffer = get_void_pointer ((void***) &stack_pointer);
  unsigned length = get_int ((int**) &stack_pointer);
  int ret_value = length;
  lock_acquire (&file_system_lock);

  if (fd == 0) // read from keyboard
  {
    for (int i = 0; i < length; i++)
    {
      uint8_t value = input_getc (); // uint8_t as defined in the library :(
      *((uint8_t*) buffer) = value;
      buffer += sizeof(uint8_t);
    }
  }
  else
  {
    struct file_descriptor* file_desc = get_file_descriptor (thread_current (), fd);
    if (file_desc == NULL) // File not open
      ret_value = -1;
    else
    {
      struct file* file = file_desc->file;
      ret_value = file_read (file, buffer, length);
    }
  }

  lock_release (&file_system_lock);
  return ret_value;
}

int
write_wrapper (void)
{
  int fd = get_int ((int**) &stack_pointer);
  void* buffer = get_void_pointer ((void***) &stack_pointer);
  unsigned length = get_int ((int**) &stack_pointer);
  int ret_value = length;
  lock_acquire (&file_system_lock);

  if (fd == 1)
    putbuf (buffer, length);
  else
  {
    struct file_descriptor* file_desc = get_file_descriptor (thread_current (), fd);
    if (file_desc == NULL) // File not open
      ret_value = -1;
    else // actual write
    {
      struct file* file = file_desc->file;
      ret_value = file_write (file, buffer, length);
    }
  }

  lock_release (&file_system_lock);
  return ret_value;
}

void
seek_wrapper (void)
{
  int fd = get_int ((int**) &stack_pointer);
  unsigned position = get_int ((int**) &stack_pointer);
  lock_acquire (&file_system_lock);
  struct file_descriptor* file_desc = get_file_descriptor (thread_current (), fd);
  if (file_desc != NULL)
  {
    struct file* file = file_desc->file;
    file_seek (file, position);
  }
  lock_release (&file_system_lock);
}

unsigned
tell_wrapper (void)
{
  int fd = get_int ((int**) &stack_pointer);
  int ret_value = 0;
  lock_acquire (&file_system_lock);
  struct file_descriptor* file_desc = get_file_descriptor (thread_current (), fd);
  if (file_desc == NULL) // File not open
    ret_value = 0;
  else // actual write
  {
    struct file* file = file_desc->file;
    ret_value = file_tell (file);
  }
  lock_release (&file_system_lock);
  return ret_value;
}

void
close_wrapper (void)
{
  int fd = get_int ((int**) &stack_pointer);
  struct file_descriptor* file_desc = get_file_descriptor (thread_current (), fd);
  if (file_desc != NULL)
  {
    list_remove (&file_desc->thread_elem);
    file_close (file_desc->file);
    free (file_desc);
  }
}

void*
get_void_pointer (void*** esp)
{
  validate_address (stack_pointer);
  void* ret = **esp;
  (*esp)++;
  validate_address (ret);
  return ret;
}
char*
get_char_pointer (char*** esp)
{
  validate_address (stack_pointer);
  char* ret = **esp;
  (*esp)++;
  validate_address (ret);
  return ret;
}

int
get_int (int **esp)
{
  validate_address (stack_pointer);
  int ret = **esp;
  (*esp)++;
  return ret;
}

void
validate_address (void *address)
{
  if (address == NULL ||
      is_kernel_vaddr (address) /*Accessing kernel address*/ ||
      pagedir_get_page (thread_current ()->pagedir, address) == NULL) exit (-1);
}
