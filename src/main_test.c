/**
  * compile : gcc test.c -o test -I/opt/etherlab/include -L/opt/etherlab/lib -lethercat
  */
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>
#include <math.h>

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/*Application Parameters*/
#define TASK_FREQUENCY          1000 /*Hz*/
#define PERIOD_NS               (1000000000 / TASK_FREQUENCY)
#define PULSE_PER_MM            1000.0 /* Adjust based on mechanics */
#define TRAJ_SPEED              20.0   /* mm/s */

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * 1000000000 + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * 1000000000 + (T).tv_nsec)

#define OP_MODE_CSP             8   /*CSP mode*/

typedef enum { SEG_IDLE, SEG_LINE, SEG_CIRCLE } SegType;

typedef struct {
    SegType type;
    double start_x, start_y;
    double end_x, end_y;
    double center_x, center_y; // For circle
    double radius;             // For circle
    double total_dist;         // Length of segment (mm)
    double current_dist;       // Distance covered so far (mm)
} TrajectorySegment;

#define MAX_SEGMENTS 20
static TrajectorySegment segments[MAX_SEGMENTS];
static int current_seg_idx = 0;
static int total_segments = 0;
static int traj_started = 0;

void add_line(double x1, double y1, double x2, double y2) {
    if (total_segments >= MAX_SEGMENTS) return;
    TrajectorySegment *s = &segments[total_segments++];
    s->type = SEG_LINE;
    s->start_x = x1; s->start_y = y1;
    s->end_x = x2;   s->end_y = y2;
    s->total_dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    s->current_dist = 0;
}

void add_circle(double cx, double cy, double r) {
    if (total_segments >= MAX_SEGMENTS) return;
    TrajectorySegment *s = &segments[total_segments++];
    s->type = SEG_CIRCLE;
    s->center_x = cx; s->center_y = cy;
    s->radius = r;
    s->total_dist = 2 * M_PI * r;
    s->current_dist = 0;
    // Assume full circle starting from (cx+r, cy)
    s->start_x = cx + r; s->start_y = cy; 
}

void init_shapes() {
    total_segments = 0;
    // 1. Triangle (0,0) -> (50, 86.6) -> (100, 0) -> (0,0)
    add_line(0, 0, 50, 86.6);
    add_line(50, 86.6, 100, 0);
    add_line(100, 0, 0, 0);
    
    // 2. Square (0,0) -> (0,100) -> (100,100) -> (100,0) -> (0,0)
    add_line(0, 0, 0, 100);
    add_line(0, 100, 100, 100);
    add_line(100, 100, 100, 0);
    add_line(100, 0, 0, 0);

    // 3. Circle Center(50,50) R=50
    // Note: Start point of circle logic needs to match end of previous shape for smooth path.
    // Here we jump for simplicity or assume separate runs.
    add_circle(50, 50, 50);
}

void update_trajectory(double *x, double *y) {
    if (current_seg_idx >= total_segments) return; // Done

    TrajectorySegment *s = &segments[current_seg_idx];
    double step = TRAJ_SPEED / TASK_FREQUENCY; // mm per cycle
    
    s->current_dist += step;
    if (s->current_dist >= s->total_dist) {
        s->current_dist = s->total_dist; // Cap it
        current_seg_idx++; // Move to next next cycle
    }

    double progress = s->current_dist / s->total_dist;

    if (s->type == SEG_LINE) {
        *x = s->start_x + (s->end_x - s->start_x) * progress;
        *y = s->start_y + (s->end_y - s->start_y) * progress;
    } else if (s->type == SEG_CIRCLE) {
        // Simple full circle 0 to 2PI
        double angle = progress * 2 * M_PI;
        *x = s->center_x + s->radius * cos(angle);
        *y = s->center_y + s->radius * sin(angle);
    }
}

/*****************************************************************************/

/*EtherCAT*/
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc1, *sc2;
static ec_slave_config_state_t sc_state1 = {}, sc_state2 = {};

/****************************************************************************/

/*Process Data*/
static uint8_t *domain1_pd = NULL;

#define DM3E1        0,0                        /*Slave 1 address*/
#define DM3E2        0,1                        /*Slave 2 address*/

#define VID_PID           0x00004321,0x00008100   /*Vendor ID, product code*/

/*Offsets for PDO entries*/
static struct{
    unsigned int operation_mode;
    unsigned int ctrl_word;
    unsigned int target_position;
    unsigned int status_word;
    unsigned int current_velocity;
    unsigned int current_position;
    unsigned int mode_display;
}offsets[2];

const static ec_pdo_entry_reg_t domain1_regs[] = {
    {DM3E1, VID_PID, 0x6040, 0, &offsets[0].ctrl_word},
    {DM3E1, VID_PID, 0x6060, 0, &offsets[0].operation_mode },
    {DM3E1, VID_PID, 0x607A, 0, &offsets[0].target_position},
    {DM3E1, VID_PID, 0x6041, 0, &offsets[0].status_word},
    {DM3E1, VID_PID, 0x606C, 0, &offsets[0].current_velocity},
    {DM3E1, VID_PID, 0x6064, 0, &offsets[0].current_position},
    {DM3E1, VID_PID, 0x6061, 0, &offsets[0].mode_display},
    {DM3E2, VID_PID, 0x6040, 0, &offsets[1].ctrl_word},
    {DM3E2, VID_PID, 0x6060, 0, &offsets[1].operation_mode },
    {DM3E2, VID_PID, 0x607A, 0, &offsets[1].target_position},
    {DM3E2, VID_PID, 0x6041, 0, &offsets[1].status_word},
    {DM3E2, VID_PID, 0x606C, 0, &offsets[1].current_velocity},
    {DM3E2, VID_PID, 0x6064, 0, &offsets[1].current_position},
    {DM3E2, VID_PID, 0x6061, 0, &offsets[1].mode_display},
    {}
};

/***************************************************************************/
/*Config PDOs*/
static ec_pdo_entry_info_t device_pdo_entries[] = {
    /*RxPdo 0x1600*/
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8 }, 
    {0x607A, 0x00, 32},
    /*TxPdo 0x1A00*/
    {0x6041, 0x00, 16},
    {0x606C, 0x00, 32},
    {0x6064, 0x00, 32},
    {0x6061, 0x00, 8 }
};

static ec_pdo_info_t device_pdos[] = {
    //RxPdo
    {0x1600, 3, device_pdo_entries + 0 },
    //TxPdo
    {0x1A00, 4, device_pdo_entries + 3 }
};

static ec_sync_info_t device_syncs[] = {
    { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
    { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
    { 2, EC_DIR_OUTPUT, 1, device_pdos + 0, EC_WD_ENABLE },
    { 3, EC_DIR_INPUT, 1, device_pdos + 1, EC_WD_DISABLE },
    { 0xFF}
};

/**************************************************************************/

/*************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    if (ds.working_counter != domain1_state.working_counter)
    {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state)
    {
        printf("Domain1: State %u.\n", ds.wc_state);
    }
    domain1_state = ds;
}

void check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding)
    {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states)
    {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up)
    {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    master_state = ms;
}

/****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s1;
    ecrt_slave_config_state(sc1, &s1);
    if (s1.al_state != sc_state1.al_state)
    {
        printf("slave1: State 0x%02X.\n", s1.al_state);
    }
    if (s1.online != sc_state1.online)
    {
        printf("slave1: %s.\n", s1.online ? "online" : "offline");
    }
    if (s1.operational != sc_state1.operational)
    {
        printf("slave1: %soperational.\n", s1.operational ? "" : "Not ");
    }
    sc_state1 = s1;

    ec_slave_config_state_t s2;
    ecrt_slave_config_state(sc2, &s2);
    if (s2.al_state != sc_state2.al_state)
    {
        printf("slave2: State 0x%02X.\n", s2.al_state);
    }
    if (s2.online != sc_state2.online)
    {
        printf("slave2: %s.\n", s2.online ? "online" : "offline");
    }
    if (s2.operational != sc_state2.operational)
    {
        printf("slave2: %soperational.\n", s2.operational ? "" : "Not ");
    }
    sc_state2 = s2;
}

/*******************************************************************************/

void cyclic_task(struct timespec *wakeup_time)
{
    static uint16_t command[2] = {0x004F, 0x004F};
    static int32_t target_pos[2] = {0, 0};
    static int32_t home_pos[2] = {0, 0};
    static int is_operational[2] = {0, 0};
    uint16_t status;
    int32_t current_pos_cnt[2];
    double target_mm[2] = {0, 0};
    
    static int counter = 0;

    // 将主站当前的系统时间发送给 EtherCAT 协议栈。
    ecrt_master_application_time(master, TIMESPEC2NS(*wakeup_time));

    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    check_domain1_state();

    if (counter % 100 == 0) {
        check_master_state();
        check_slave_config_states();
    }

    int all_slaves_ready = 1;
    // Pass 1: Check if all slaves are operational
    for (int i = 0; i < 2; ++i)
    {
        status = EC_READ_U16(domain1_pd + offsets[i].status_word);
        if ((status & 0x006F) == 0x0027) {
             if (!is_operational[i]) {
                 current_pos_cnt[i] = EC_READ_S32(domain1_pd + offsets[i].current_position);
                 target_pos[i] = current_pos_cnt[i];
                 home_pos[i] = current_pos_cnt[i]; // Store home position
                 is_operational[i] = 1;
                 printf("Slave %d initialized. Home: %d\n", i+1, home_pos[i]);
             }
        } else {
            is_operational[i] = 0;
            all_slaves_ready = 0;
        }
        if (!is_operational[i]) all_slaves_ready = 0;
    }

    // Trajectory Generation
    if (all_slaves_ready && !traj_started) {
        init_shapes();
        traj_started = 1;
        printf("Trajectory started...\n");
    }

    if (traj_started && current_seg_idx < total_segments) {
        update_trajectory(&target_mm[0], &target_mm[1]);
        // Convert mm to pulses relative to home
        target_pos[0] = home_pos[0] + (int32_t)(target_mm[0] * PULSE_PER_MM);
        target_pos[1] = home_pos[1] + (int32_t)(target_mm[1] * PULSE_PER_MM);
    }

    // Pass 2: Execute control logic
    for (int i = 0; i < 2; ++i)
    {
        status = EC_READ_U16(domain1_pd + offsets[i].status_word);
        int8_t mode_disp = EC_READ_S8(domain1_pd + offsets[i].mode_display);
        
        if (counter % 100 == 0) {
             printf("Slave %d: Status=0x%04X, ModeDisp=%d, Target=%d, Act=%d\n", 
                    i+1, status, mode_disp, target_pos[i], 
                    EC_READ_S32(domain1_pd + offsets[i].current_position));
        }

        if ((status & command[i]) == 0x0040)
        {
            EC_WRITE_U16(domain1_pd + offsets[i].ctrl_word, 0x0006);
            EC_WRITE_S8(domain1_pd + offsets[i].operation_mode, OP_MODE_CSP);
            command[i] = 0x006F;
        }
        else if ((status & command[i]) == 0x0021)
        {
            EC_WRITE_U16(domain1_pd + offsets[i].ctrl_word, 0x0007);
            command[i] = 0x006F;
        }
        else if ((status & command[i]) == 0x0023)
        {
            EC_WRITE_U16(domain1_pd + offsets[i].ctrl_word, 0x000f);
            command[i] = 0x006F;
        }
        else if ((status & command[i]) == 0x0027)
        {
            EC_WRITE_S32(domain1_pd + offsets[i].target_position, target_pos[i]);
            EC_WRITE_U16(domain1_pd + offsets[i].ctrl_word, 0x001f);
        }
    }

    // Sync reference clock (DC)
    ecrt_master_sync_reference_clock(master);
    ecrt_master_sync_slave_clocks(master);

    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    
    counter++;
}

/****************************************************************************/

int main(int argc, char **argv)
{
	 printf("Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master)
    {
        exit(EXIT_FAILURE);
    }
    
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        exit(EXIT_FAILURE);
    }
    if (!(sc1 = ecrt_master_slave_config(master, DM3E1, VID_PID)))
    {
        fprintf(stderr, "Failed to get slave configuration for slave1!\n");
        exit(EXIT_FAILURE);
    }
    ecrt_slave_config_dc(sc1, 0x0300, 10000000, 0, 0, 0); // DC: 10ms cycle

    if (!(sc2 = ecrt_master_slave_config(master, DM3E2, VID_PID)))
    {
        fprintf(stderr, "Failed to get slave configuration for slave2!\n");
        exit(EXIT_FAILURE);
    }
    ecrt_slave_config_dc(sc2, 0x0300, 10000000, 0, 0, 0); // DC: 10ms cycle

    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc1, EC_END, device_syncs))
    {
        fprintf(stderr, "Failed to configure slave1 PDOs!\n");
        exit(EXIT_FAILURE);
    }
    if (ecrt_slave_config_pdos(sc2, EC_END, device_syncs))
    {
        fprintf(stderr, "Failed to configure slave2 PDOs!\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("*Success to configuring slave PDOs*\n");
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) 
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        exit(EXIT_FAILURE);
    }
   
    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        exit(EXIT_FAILURE);
    }
    else
    {
        printf("*Master activated*\n");
    }
    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        exit(EXIT_FAILURE);
    }

    printf("*It's working now*\n");

    struct timespec wakeup_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* Start in future */

    while (1) 
    {
        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= 1000000000) {
            wakeup_time.tv_nsec -= 1000000000;
            wakeup_time.tv_sec++;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
        cyclic_task(&wakeup_time);
    }
    return EXIT_SUCCESS;
}

