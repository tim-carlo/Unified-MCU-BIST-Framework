#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

// tiny C library syscalls

extern char _end; 
extern char __HeapLimit; 

static char *heap_end;

caddr_t _sbrk(int incr)
{
    if (!heap_end)
        heap_end = &_end;

    char *prev = heap_end;

    if (heap_end + incr > &__HeapLimit) {
        errno = ENOMEM;
        return (caddr_t)-1;
    }

    heap_end += incr;
    return (caddr_t)prev;
}
// They are not needed.
// Just provide minimal implementations to satisfy the linker.
int _close(int file) { (void)file; return -1; }
int _fstat(int file, struct stat *st) { (void)st; return 0; }
int _isatty(int file) { return 1; }
int _lseek(int file, int ptr, int dir) { (void)ptr; (void)dir; return 0; }
int _read(int file, char *ptr, int len) { (void)ptr; return 0; }
int _write(int file, const char *ptr, int len) { (void)ptr; return len; }