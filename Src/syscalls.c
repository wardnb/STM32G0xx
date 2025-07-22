/**
 * @file syscalls.c
 * @brief System calls for STM32G0xx grblHAL
 */

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#ifndef STDOUT_USART
#define STDOUT_USART 1
#endif

#ifndef STDERR_USART
#define STDERR_USART 1
#endif

#ifndef STDIN_USART
#define STDIN_USART 1
#endif

#undef errno
extern int errno;

/*
 environ
 A pointer to a list of environment variables and their values.
 For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

int _write(int file, char *ptr, int len);
int _close(int file);
int _fstat(int file, struct stat *st);
int _isatty(int file);
int _lseek(int file, int offset, int whence);
int _read(int file, char *ptr, int len);
caddr_t _sbrk(int incr);
int _stat(const char *filepath, struct stat *st);
int _link(char *old, char *new);
int _unlink(char *name);
void _exit(int status);
int _kill(int pid, int sig);
int _getpid(void);

int _write(int file, char *ptr, int len)
{
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        //__io_putchar(*ptr++);
    }
    return len;
}

int _close(int file)
{
    return -1;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    return 1;
}

int _lseek(int file, int offset, int whence)
{
    return 0;
}

int _read(int file, char *ptr, int len)
{
    int DataIdx;

    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        //*ptr++ = __io_getchar();
    }

   return len;
}

caddr_t _sbrk(int incr)
{
    extern char end asm("end");
    extern char _Min_Heap_Size;
    const char * const heap_limit = (char*)&_Min_Heap_Size;
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0)
        heap_end = &end;

    prev_heap_end = heap_end;

    if (heap_end + incr > heap_limit)
    {
        errno = ENOMEM;
        return (caddr_t) -1;
    }

    heap_end += incr;

    return (caddr_t) prev_heap_end;
}

int _stat(const char *filepath, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _link(char *old, char *new)
{
    errno = EMLINK;
    return -1;
}

int _unlink(char *name)
{
    errno = ENOENT;
    return -1;
}

void _exit(int status)
{
    while (1);
}

int _kill(int pid, int sig)
{
    errno = EINVAL;
    return(-1);
}

int _getpid(void)
{
    return 1;
}