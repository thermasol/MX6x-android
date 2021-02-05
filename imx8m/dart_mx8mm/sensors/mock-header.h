#include <stdint.h>

#define __unused
#define SENSORS_HANDLE_BASE 0

#define SENSOR_TYPE_ACCELEROMETER 1
#define SENSOR_STRING_TYPE_ACCELEROMETER ""
#define SENSOR_TYPE_AMBIENT_TEMPERATURE 7
#define SENSOR_STRING_TYPE_AMBIENT_TEMPERATURE ""

#define SENSOR_FLAG_CONTINUOUS_MODE 0
#define SENSOR_FLAG_ON_CHANGE_MODE 0

#define META_DATA_FLUSH_COMPLETE 0
#define SENSOR_TYPE_META_DATA 0
#define META_DATA_VERSION 0

struct sensor_t
{
    const char* name;
    const char* vendor;
    int version;
    int handle;
    int type;
    float maxRange;
    float resolution;
    float power;
    int32_t minDelay;
    int64_t maxDelay;
    uint32_t fifoReservedEventCount;
    uint32_t fifoMaxEventCount;
    const char* stringType;
    const char* requiredPermission;
    uint64_t flags;
    void* reserved[2];
};

struct sensors_vec_t
{
    float x;
    float y;
    float z;
};

struct meta_data_event
{
    int32_t what;
    int32_t sensor;
};

struct sensors_event_t
{
    int32_t version;
    int32_t sensor;
    int32_t type;
    int64_t timestamp;

    union
    {
        sensors_vec_t acceleration;
        float temperature;
        meta_data_event meta_data;
    };
};

struct sensors_poll_device_1
{
};

typedef sensors_event_t sensors_meta_data_event_t;