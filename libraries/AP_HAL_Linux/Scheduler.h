
#ifndef __AP_HAL_LINUX_SCHEDULER_H__
#define __AP_HAL_LINUX_SCHEDULER_H__

#include "AP_HAL_Linux.h"
#include "Semaphores.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <sys/time.h>
#include <pthread.h>

#define LINUX_SCHEDULER_MAX_TIMER_PROCS 10
#define LINUX_SCHEDULER_MAX_IO_PROCS 10

class Linux::Scheduler : public AP_HAL::Scheduler {

typedef void *(*pthread_startroutine_t)(void *);

public:
    Scheduler();

    static Scheduler *from(AP_HAL::Scheduler *scheduler) {
        return static_cast<Scheduler*>(scheduler);
    }

    void     init();
    void     delay(uint16_t ms);
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,
                uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::MemberProc);
    void     register_io_process(AP_HAL::MemberProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    void     begin_atomic();
    void     end_atomic();

    bool     system_initializing();
    void     system_initialized();

    void     reboot(bool hold_in_bootloader);

    void     stop_clock(uint64_t time_usec);

    uint64_t stopped_clock_usec() const { return _stopped_clock_usec; }

private:
    void _timer_handler(int signum);
    void _microsleep(uint32_t usec);

    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;

    AP_HAL::Proc _failsafe;

    bool _initialized;
    volatile bool _timer_pending;

    AP_HAL::MemberProc _timer_proc[LINUX_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_proc;

    AP_HAL::MemberProc _io_proc[LINUX_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;
    volatile bool _in_io_proc;

    volatile bool _timer_event_missed;

    pthread_t _timer_thread_ctx;
    pthread_t _io_thread_ctx;
    pthread_t _rcin_thread_ctx;
    pthread_t _uart_thread_ctx;
    pthread_t _tonealarm_thread_ctx;

    static void *_timer_thread(void* arg);
    static void *_io_thread(void* arg);
    static void *_rcin_thread(void* arg);
    static void *_uart_thread(void* arg);
    static void _run_uarts(void);
    static void *_tonealarm_thread(void* arg);

    void _run_timers(bool called_from_timer_thread);
    void _run_io(void);
    void _create_realtime_thread(pthread_t *ctx, int rtprio, const char *name,
                                 pthread_startroutine_t start_routine);

    uint64_t _stopped_clock_usec;

    Semaphore _timer_semaphore;
    Semaphore _io_semaphore;
};

#endif // CONFIG_HAL_BOARD

#endif // __AP_HAL_LINUX_SCHEDULER_H__
