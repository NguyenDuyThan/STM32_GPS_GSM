/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.6 at Fri Mar 31 14:26:17 2017. */

#include "deviceConfig.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

const bool DeviceConfigV2_fixOn_default = false;
const bool DeviceConfigV2_revOn_default = false;
const bool DeviceConfigV2_fixDoor_default = false;
const bool DeviceConfigV2_revDoor_default = false;
const bool DeviceConfigV2_offBuzz_default = false;
const uint32_t DeviceConfigV2_trackDelayTime_default = 5u;
const uint32_t DeviceConfigV2_trackDist_default = 30u;
const uint32_t DeviceConfigV2_stdbyTrackDelayTime_default = 5u;
const bool DeviceConfigV2_onByEngineOn_default = false;
const uint32_t DeviceConfigV2_CAMdelay_default = 10u;
const uint32_t DeviceConfigV2_CAMres_default = 2u;
const uint32_t DeviceConfigV2_stdbyCAMdelay_default = 0u;


const pb_field_t DeviceConfigV2_fields[13] = {
    PB_FIELD(  1, BOOL    , REQUIRED, STATIC  , FIRST, DeviceConfigV2, fixOn, fixOn, &DeviceConfigV2_fixOn_default),
    PB_FIELD(  2, BOOL    , REQUIRED, STATIC  , OTHER, DeviceConfigV2, revOn, fixOn, &DeviceConfigV2_revOn_default),
    PB_FIELD(  3, BOOL    , REQUIRED, STATIC  , OTHER, DeviceConfigV2, fixDoor, revOn, &DeviceConfigV2_fixDoor_default),
    PB_FIELD(  4, BOOL    , REQUIRED, STATIC  , OTHER, DeviceConfigV2, revDoor, fixDoor, &DeviceConfigV2_revDoor_default),
    PB_FIELD(  5, BOOL    , REQUIRED, STATIC  , OTHER, DeviceConfigV2, offBuzz, revDoor, &DeviceConfigV2_offBuzz_default),
    PB_FIELD(  6, UINT32  , REQUIRED, STATIC  , OTHER, DeviceConfigV2, trackDelayTime, offBuzz, &DeviceConfigV2_trackDelayTime_default),
    PB_FIELD(  7, UINT32  , REQUIRED, STATIC  , OTHER, DeviceConfigV2, trackDist, trackDelayTime, &DeviceConfigV2_trackDist_default),
    PB_FIELD(  8, UINT32  , REQUIRED, STATIC  , OTHER, DeviceConfigV2, stdbyTrackDelayTime, trackDist, &DeviceConfigV2_stdbyTrackDelayTime_default),
    PB_FIELD(  9, BOOL    , REQUIRED, STATIC  , OTHER, DeviceConfigV2, onByEngineOn, stdbyTrackDelayTime, &DeviceConfigV2_onByEngineOn_default),
    PB_FIELD( 10, UINT32  , REQUIRED, STATIC  , OTHER, DeviceConfigV2, CAMdelay, onByEngineOn, &DeviceConfigV2_CAMdelay_default),
    PB_FIELD( 11, UINT32  , REQUIRED, STATIC  , OTHER, DeviceConfigV2, CAMres, CAMdelay, &DeviceConfigV2_CAMres_default),
    PB_FIELD( 12, UINT32  , REQUIRED, STATIC  , OTHER, DeviceConfigV2, stdbyCAMdelay, CAMres, &DeviceConfigV2_stdbyCAMdelay_default),
    PB_LAST_FIELD
};

const pb_field_t localConfig_fields[3] = {
    PB_FIELD(  1, UINT32  , OPTIONAL, STATIC  , FIRST, localConfig, GMT, GMT, 0),
    PB_FIELD(  2, BOOL    , OPTIONAL, STATIC  , OTHER, localConfig, DST, GMT, 0),
    PB_LAST_FIELD
};


/* @@protoc_insertion_point(eof) */