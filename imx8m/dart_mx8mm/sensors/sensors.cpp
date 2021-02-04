/*
 * Copyright (C) 2021 ThermaSol, Blake W. Ford
 *
 */

#define LOG_TAG "ThermaSolSensors"

#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <hardware/sensors.h>

#include <deque>
#include <chrono>
#include <algorithm>
using namespace std::chrono;

#define ACCEL_HANDLE SENSORS_HANDLE_BASE
#define TEMP_HANDLE SENSORS_HANDLE_BASE + 1

struct sensorParameters
{
    pthread_mutex_t lock;
    pthread_t* thread = nullptr;
    volatile bool enabled = false;
    volatile uint32_t microsecondsDelay = 0;
};

sensorParameters gTemperature;
sensorParameters gAccelerometer;

pthread_mutex_t gEventLock;
volatile bool gKeepPolling = false;
std::deque<sensors_event_t> gEvents;
struct sensors_poll_device_1 gPollDevice;

float randomize()
{
    return (rand()%10)/100.0f;
}

void* accelerometer(void* parameters)
{
    sensorParameters* accelParameters = (sensorParameters*)parameters;
    while(accelParameters->enabled)
    {
        usleep(accelParameters->microsecondsDelay);

        sensors_event_t event;
        event.version = sizeof(struct sensors_event_t);
        event.type = SENSOR_TYPE_ACCELEROMETER;
        event.sensor = ACCEL_HANDLE;
        event.timestamp = duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();
        event.acceleration.x = 9.80f + randomize(); // 9.81 is ideal, but the Sensor API expects frequent changes
        event.acceleration.y = randomize();
        event.acceleration.z = randomize();
        pthread_mutex_lock(&gEventLock);
        gEvents.push_back(event);
        pthread_mutex_unlock(&gEventLock);
    }

    return nullptr;
}

void* temperature(void* parameters)
{
    sensorParameters* tempParameters = (sensorParameters*)parameters;
    while(tempParameters->enabled)
    {
        usleep(tempParameters->microsecondsDelay);

        sensors_event_t event;
        event.version = sizeof(struct sensors_event_t);
        event.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
        event.sensor = TEMP_HANDLE;
        event.timestamp = duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();
        event.temperature = 32.0f + randomize();
        pthread_mutex_lock(&gEventLock);
        gEvents.push_back(event);
        pthread_mutex_unlock(&gEventLock);
    }

    return nullptr;
}

static int therma_activate(struct sensors_poll_device_t* pollDevice __unused, int handle, int enabled)
{
    sensorParameters* parameters = nullptr;
    switch(handle)
    {
        case ACCEL_HANDLE:
            parameters = &gAccelerometer;
            break;
        case TEMP_HANDLE:
            parameters = &gTemperature;
            break;
        default:
            break;
    }

    if(parameters != nullptr)
    {
        pthread_mutex_lock(&parameters->lock);
        parameters->enabled = false;
        if(parameters->thread != nullptr)
        {
            pthread_join(*parameters->thread, nullptr);
            delete parameters->thread;
            parameters->thread = nullptr;
        }
        if(enabled)
        {
            parameters->thread = new pthread_t();
            parameters->enabled = true;
        }
        pthread_mutex_unlock(&parameters->lock);

        if(enabled)
        {
            switch(handle)
            {
                case ACCEL_HANDLE:
                    pthread_create(parameters->thread, nullptr, accelerometer, parameters);
                    break;
                case TEMP_HANDLE:
                    pthread_create(parameters->thread, nullptr, temperature, parameters);
                    break;
                default:
                    break;
            }
        }
    }

    return 0;
}

static int therma_batch(struct sensors_poll_device_1* a __unused, int handle, int c __unused, int64_t sampling_period_ns, int64_t e __unused)
{
    sensorParameters* parameters = nullptr;
    switch(handle)
    {
        case ACCEL_HANDLE:
            parameters = &gAccelerometer;
            break;
        case TEMP_HANDLE:
            parameters = &gTemperature;
            break;
        default:
            break;
    }

    if(parameters != nullptr)
    {
        pthread_mutex_lock(&parameters->lock);
        parameters->microsecondsDelay = (uint32_t)(sampling_period_ns/1000);
        pthread_mutex_unlock(&parameters->lock);
    }

    return 0;
}

static int therma_flush(struct sensors_poll_device_1* pollDevice __unused, int handle)
{
    sensors_meta_data_event_t complete;
    complete.version = META_DATA_VERSION;
    complete.type = SENSOR_TYPE_META_DATA;
    complete.sensor = 0;
    complete.timestamp = 0;
    complete.meta_data.sensor = handle;
    complete.meta_data.what = META_DATA_FLUSH_COMPLETE;

    pthread_mutex_lock(&gEventLock);
    gEvents.push_back(complete);
    pthread_mutex_unlock(&gEventLock);

    return 0;
}

static int therma_poll(struct sensors_poll_device_t* pollDevice __unused, sensors_event_t* events, int count)
{
    while(gKeepPolling && gEvents.size() == 0)
    {
        usleep(1); // Block
    }

    pthread_mutex_lock(&gEventLock);
    const int available = std::min((int32_t)gEvents.size(), count);
    int toSend = available;
    while(toSend--)
    {
        events[toSend] = gEvents.front();
        gEvents.pop_front();
    }
    pthread_mutex_unlock(&gEventLock);

    return available;
}

static int therma_close(struct hw_device_t* device __unused)
{
    gKeepPolling = false;
    pthread_mutex_destroy(&gEventLock);
    pthread_mutex_destroy(&gAccelerometer.lock);
    pthread_mutex_destroy(&gTemperature.lock);

    return 0;
}

static const struct sensor_t sSensorList[] =
{
    { .name       = "Static Orientation 3-Axis Accelerometer",
      .vendor     = "ThermaSol",
      .version    = 10,
      .handle     = ACCEL_HANDLE,
      .type       = SENSOR_TYPE_ACCELEROMETER,
      .maxRange   = 10.0f,
      .resolution = 0.01f,
      .power      = 0.0f,
      .minDelay   = 10*1000,
      .maxDelay   = 500*1000,
      .fifoReservedEventCount = 0,
      .fifoMaxEventCount =   0,
      .stringType = SENSOR_STRING_TYPE_ACCELEROMETER,
      .requiredPermission = 0,
      .flags = SENSOR_FLAG_CONTINUOUS_MODE,
      .reserved   = {}
    },
    { .name       = "Steam Room Temperature Sensor",
      .vendor     = "ThermaSol",
      .version    = 10,
      .handle     = TEMP_HANDLE,
      .type       = SENSOR_TYPE_AMBIENT_TEMPERATURE,
      .maxRange   = 200.0f,
      .resolution = 0.01f,
      .power      = 0.0f,
      .minDelay   = 4*1000*1000,
      .maxDelay   = 0,
      .fifoReservedEventCount = 0,
      .fifoMaxEventCount =   0,
      .stringType = SENSOR_STRING_TYPE_AMBIENT_TEMPERATURE,
      .requiredPermission = 0,
      .flags = SENSOR_FLAG_ON_CHANGE_MODE,
      .reserved   = {}
    },
};

static int get_thermasol_sensors_list(struct sensors_module_t* module __unused, struct sensor_t const** list)
{
    *list = sSensorList;
    return sizeof(sSensorList)/sizeof(sensor_t);
}

// Implemented, but should never be called
static int therma_delay(struct sensors_poll_device_t* a, int b, int64_t c);
static int open_thermasol_sensors(const struct hw_module_t* module, const char* name __unused, struct hw_device_t** device)
{
    srand(time(nullptr));
    pthread_mutex_init(&gEventLock, nullptr);
    pthread_mutex_init(&gAccelerometer.lock, nullptr);
    pthread_mutex_init(&gTemperature.lock, nullptr);

    gPollDevice.common.tag = HARDWARE_DEVICE_TAG;
    gPollDevice.common.version = SENSORS_DEVICE_API_VERSION_1_3;
    gPollDevice.common.module = (struct hw_module_t*) module;
    gPollDevice.common.close = therma_close;

    gPollDevice.activate = therma_activate;
    gPollDevice.batch = therma_batch;
    gPollDevice.flush = therma_flush;
    gPollDevice.poll = therma_poll;

    gPollDevice.setDelay = therma_delay;

    gKeepPolling = true;

    *device = &gPollDevice.common;

    return 0;
}

static struct hw_module_methods_t thermasol_module_methods =
{
    .open = open_thermasol_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM =
{
    .common =
    {
        .tag = HARDWARE_MODULE_TAG,
        .version_major = 1,
        .version_minor = 3,
        .id = SENSORS_HARDWARE_MODULE_ID,
        .name = "ThermaSol Sensors Module",
        .author = "Blake W. Ford",
        .methods = &thermasol_module_methods,
    },
    .get_sensors_list = get_thermasol_sensors_list
};

static int therma_delay(struct sensors_poll_device_t* a __unused, int b __unused, int64_t c __unused)
{
    return 0;
}
