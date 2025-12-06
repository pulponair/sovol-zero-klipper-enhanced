// Support for gathering acceleration data from LIS2DW chip
//
// Copyright (C) 2023  Zhou.XianMing <zhouxm@biqu3d.com>
// Copyright (C) 2020-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "sensor_bulk.h" // sensor_bulk_report
#include "spicmds.h" // spidev_transfer

#define LIS_AR_DATAX0 0x28
#define LIS_AM_READ   0x80
#define LIS_FIFO_SAMPLES 0x2F

#define BYTES_PER_SAMPLE 6
#define BYTES_PER_BLOCK 48

struct lis2dw {
    struct timer timer;
    uint32_t rest_ticks;
    struct spidev_s *spi;
    uint8_t flags;
    uint8_t fifo_bytes_pending;
    struct sensor_bulk sb;
};

enum {
    LIS_PENDING = 1<<0,
};

static struct task_wake lis2dw_wake;

// Event handler that wakes lis2dw_task() periodically
static uint_fast8_t
lis2dw_event(struct timer *timer)
{
    struct lis2dw *ax = container_of(timer, struct lis2dw, timer);
    ax->flags |= LIS_PENDING;
    sched_wake_task(&lis2dw_wake);
    return SF_DONE;
}

void
command_config_lis2dw(uint32_t *args)
{
    struct lis2dw *ax = oid_alloc(args[0], command_config_lis2dw
                                   , sizeof(*ax));
    ax->timer.func = lis2dw_event;
    ax->spi = spidev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_lis2dw, "config_lis2dw oid=%c spi_oid=%c");

// Helper code to reschedule the lis2dw_event() timer
static void
lis2dw_reschedule_timer(struct lis2dw *ax)
{
    irq_disable();
    ax->timer.waketime = timer_read_time() + ax->rest_ticks;
    sched_add_timer(&ax->timer);
    irq_enable();
}


static void
update_fifo_status(struct lis2dw *ax, uint8_t fifo_status)
{
    if (fifo_status & 0x40)
        ax->sb.possible_overflows++;

    uint_fast8_t pending;
    pending = fifo_status & 0x3F;

    ax->fifo_bytes_pending = pending * BYTES_PER_SAMPLE;
}



static void
query_fifo_status(struct lis2dw *ax)
{
    uint8_t fifo_status = 0;
    uint8_t fifo[2] = { LIS_FIFO_SAMPLES | LIS_AM_READ, 0x00 };
    spidev_transfer(ax->spi, 1, sizeof(fifo), fifo);
    fifo_status = fifo[1];
    update_fifo_status(ax, fifo_status);
}


// Read 8 samples from FIFO via SPI
static void
read_fifo_block_spi(struct lis2dw *ax)
{
    uint8_t msg[BYTES_PER_BLOCK + 1] = {0};
    msg[0] = LIS_AR_DATAX0 | LIS_AM_READ;

    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    memcpy(ax->sb.data, &msg[1], BYTES_PER_BLOCK);
}


static void
read_fifo_block(struct lis2dw *ax, uint8_t oid)
{
    read_fifo_block_spi(ax);
    ax->sb.data_count = BYTES_PER_BLOCK;
    sensor_bulk_report(&ax->sb, oid);
    ax->fifo_bytes_pending -= BYTES_PER_BLOCK;
}




// Query accelerometer data
static void
lis2dw_query(struct lis2dw *ax, uint8_t oid)
{
    if (ax->fifo_bytes_pending < BYTES_PER_BLOCK)
        query_fifo_status(ax);

    if (ax->fifo_bytes_pending >= BYTES_PER_BLOCK)
        read_fifo_block(ax, oid);

    // check if we need to run the task again (more packets in fifo?)
    if (ax->fifo_bytes_pending >= BYTES_PER_BLOCK) {
        // More data in fifo - wake this task again
        sched_wake_task(&lis2dw_wake);
    } else {
        // Sleep until next check time
        ax->flags &= ~LIS_PENDING;
        lis2dw_reschedule_timer(ax);
    }
}

void
command_query_lis2dw(uint32_t *args)
{
    struct lis2dw *ax = oid_lookup(args[0], command_config_lis2dw);

    sched_del_timer(&ax->timer);
    ax->flags = 0;
    if (!args[1])
        // End measurements
        return;

    // Start new measurements query
    ax->rest_ticks = args[1];
    ax->fifo_bytes_pending = 0;
    sensor_bulk_reset(&ax->sb);
    lis2dw_reschedule_timer(ax);
}
DECL_COMMAND(command_query_lis2dw, "query_lis2dw oid=%c rest_ticks=%u");

void
command_query_lis2dw_status(uint32_t *args)
{
    struct lis2dw *ax = oid_lookup(args[0], command_config_lis2dw);
    uint32_t time1 = 0;
    uint32_t time2 = 0;
    uint8_t fifo_status = 0;

    uint8_t fifo[2] = { LIS_FIFO_SAMPLES | LIS_AM_READ, 0x00 };
    time1 = timer_read_time();
    spidev_transfer(ax->spi, 1, sizeof(fifo), fifo);
    time2 = timer_read_time();
    fifo_status = fifo[1];

    update_fifo_status(ax, fifo_status);

    sensor_bulk_status(&ax->sb, args[0], time1, time2-time1
                       , ax->fifo_bytes_pending);
}
DECL_COMMAND(command_query_lis2dw_status, "query_lis2dw_status oid=%c");

void
lis2dw_task(void)
{
    if (!sched_check_wake(&lis2dw_wake))
        return;
    uint8_t oid;
    struct lis2dw *ax;
    foreach_oid(oid, ax, command_config_lis2dw) {
        uint_fast8_t flags = ax->flags;
        if (flags & LIS_PENDING)
            lis2dw_query(ax, oid);
    }
}
DECL_TASK(lis2dw_task);
