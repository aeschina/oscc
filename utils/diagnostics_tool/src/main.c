#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <fcntl.h>

#include "oscc.h"
#include "can_protocols/steering_can_protocol.h"

#define COMMANDER_UPDATE_INTERVAL_MICRO (50000)
#define SLEEP_TICK_INTERVAL_MICRO (1000)
#define OSCC_ENABLED ( 1 )
#define OSCC_DISABLED ( 0 )

static int oscc_enabled = OSCC_DISABLED;
static int error_thrown = OSCC_OK;
static double curr_angle = 0.0;

static unsigned long long get_timestamp_micro( )
{
    struct timeval time;

    gettimeofday( &time, NULL );

    return ( time.tv_usec );
}

static unsigned long long get_elapsed_time( unsigned long long timestamp )
{
    unsigned long long now = get_timestamp_micro( );
    unsigned long long elapsed_time = now - timestamp;

    return elapsed_time;
}

/*
 * These callback functions just check the reports for operator overrides. The
 * firmware modules should have disabled themselves, but we will send the
 * command again just to be safe.
 *
 */
static void throttle_callback(oscc_throttle_report_s *report)
{
    if ( report->operator_override )
    {
        printf("Override: Throttle\n");
    }
}

static void steering_callback(oscc_steering_report_s *report)
{
    if ( report->operator_override )
    {
        printf("Override: Steering\n");
    }
}

static void brake_callback(oscc_brake_report_s * report)
{
    if ( report->operator_override )
    {
        printf("Override: Brake\n");
    }
}

static void fault_callback(oscc_fault_report_s *report)
{
    printf("Fault: ");

    if ( report->fault_origin_id == FAULT_ORIGIN_BRAKE )
    {
        printf("Brake\n");
    }
    else if ( report->fault_origin_id == FAULT_ORIGIN_STEERING )
    {
        printf("Steering\n");
    }
    else if ( report->fault_origin_id == FAULT_ORIGIN_THROTTLE )
    {
        printf("Throttle\n");
    }
}

// To cast specific OBD messages, you need to know the structure of the
// data fields and the CAN_ID.
static void obd_callback(struct can_frame *frame)
{
    if ( frame->can_id == KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID )
    {
        kia_soul_obd_steering_wheel_angle_data_s * steering_data = (kia_soul_obd_steering_wheel_angle_data_s*) frame->data;

        curr_angle = steering_data->steering_wheel_angle * KIA_SOUL_OBD_STEERING_ANGLE_SCALAR;
    }
}

// handle interrupt signal
// allow application to break out of main loop
void signal_handler( int signal_number )
{
    if ( signal_number == SIGINT )
    {
        error_thrown = OSCC_ERROR;
    }
}

// initialize connection via CAN to OSCC CAN network
// subscribe to reports and messages via callback functions
int open_and_enable( int channel )
{
    int return_code = OSCC_ERROR;

    if ( oscc_enabled == OSCC_DISABLED )
    {
        oscc_enabled = OSCC_ENABLED;

        return_code = oscc_open( channel );

        if ( return_code != OSCC_ERROR )
        {
            // register callback handlers
            oscc_subscribe_to_obd_messages(obd_callback);
            oscc_subscribe_to_brake_reports(brake_callback);
            oscc_subscribe_to_steering_reports(steering_callback);
            oscc_subscribe_to_throttle_reports(throttle_callback);
            oscc_subscribe_to_fault_reports(fault_callback);

            oscc_enable();
        }
    }
    return ( return_code );
}

// disable and close_and_disable oscc channel
void close_and_disable( int channel )
{
    if ( oscc_enabled == OSCC_ENABLED )
    {
        oscc_disable( );

        oscc_close( channel );

        oscc_enabled = OSCC_DISABLED;
    }
}

int main( int argc, char **argv )
{
    oscc_result_t ret = OSCC_OK;
    unsigned long long update_timestamp = get_timestamp_micro();
    unsigned long long elapsed_time = 0;

    int channel;

    errno = 0;

    if ( argc != 2 || ( channel = atoi( argv[1] ), errno ) != 0 )
    {
        printf( "usage %s channel\n", argv[0] );
        exit( 1 );
    }

    struct sigaction sig;
    sig.sa_handler = signal_handler;
    sigaction( SIGINT, &sig, NULL );

    ret = open_and_enable( channel );

    if ( ret == OSCC_OK )
    {
        static const float max = 1;
        static const float min = -1.0; // uncomment for steering
        // static const float min = 0; // uncomment for throttle/brake

        static double commanded_value = 0.0;
        static double scalar = 0.1;

        // this is a high frequency loop
        while ( ret == OSCC_OK && error_thrown == OSCC_OK )
        {
            elapsed_time = get_elapsed_time( update_timestamp );

            // low frequency loop -- do heavy lifting, send commands, whatever.
            if ( elapsed_time > COMMANDER_UPDATE_INTERVAL_MICRO )
            {             
                // reached limit, change direction
                if (commanded_value > max || commanded_value < min) {
                    scalar *= -1;
                }
                else { // within range, send commanded value and increment
                    printf("Sending command: %f\n", commanded_value);

                    // ret = oscc_publish_steering_torque(commanded_value);
                    // ret = oscc_publish_brake_position(commanded_value);
                    // ret = oscc_publish_throttle_position(commanded_value);
                }

                commanded_value += scalar;

                update_timestamp = get_timestamp_micro();
            }

            // Delay 1 ms to avoid loading the CPU
            (void) usleep( SLEEP_TICK_INTERVAL_MICRO );
        }
        close_and_disable( channel );
    }

    return 0;
}

