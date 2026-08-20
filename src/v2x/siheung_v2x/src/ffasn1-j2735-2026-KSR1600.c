/* Automatically generated file - do not edit */

#include "asn1defs.h"
#include "ffasn1-j2735-2026.h"

const ASN1CType asn1_type_j2735_local_0[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DSRCmsgID,
};

static const j2735DSRCmsgID asn1_value__local_4 = 20;

static const j2735DSRCmsgID asn1_value__local_5 = 18;

static const j2735DSRCmsgID asn1_value__local_6 = 19;

static const j2735DSRCmsgID asn1_value__local_7 = 21;

static const j2735DSRCmsgID asn1_value__local_8 = 22;

static const j2735DSRCmsgID asn1_value__local_9 = 23;

static const j2735DSRCmsgID asn1_value__local_10 = 24;

static const j2735DSRCmsgID asn1_value__local_11 = 25;

static const j2735DSRCmsgID asn1_value__local_12 = 26;

static const j2735DSRCmsgID asn1_value__local_13 = 27;

static const j2735DSRCmsgID asn1_value__local_14 = 28;

static const j2735DSRCmsgID asn1_value__local_15 = 31;

static const j2735DSRCmsgID asn1_value__local_16 = 32;

static const j2735DSRCmsgID asn1_value__local_17 = 50;

static const j2735MESSAGE_ID_AND_TYPE asn1_value__local_3[] = {
{
  (intptr_t)&asn1_value__local_4,
  (intptr_t)asn1_type_j2735BasicSafetyMessage,
},
{
  (intptr_t)&asn1_value__local_5,
  (intptr_t)asn1_type_j2735MapData,
},
{
  (intptr_t)&asn1_value__local_6,
  (intptr_t)asn1_type_j2735SPAT,
},
{
  (intptr_t)&asn1_value__local_7,
  (intptr_t)asn1_type_j2735CommonSafetyRequest,
},
{
  (intptr_t)&asn1_value__local_8,
  (intptr_t)asn1_type_j2735EmergencyVehicleAlert,
},
{
  (intptr_t)&asn1_value__local_9,
  (intptr_t)asn1_type_j2735IntersectionCollision,
},
{
  (intptr_t)&asn1_value__local_10,
  (intptr_t)asn1_type_j2735NMEAcorrections,
},
{
  (intptr_t)&asn1_value__local_11,
  (intptr_t)asn1_type_j2735ProbeDataManagement,
},
{
  (intptr_t)&asn1_value__local_12,
  (intptr_t)asn1_type_j2735ProbeVehicleData,
},
{
  (intptr_t)&asn1_value__local_13,
  (intptr_t)asn1_type_j2735RoadSideAlert,
},
{
  (intptr_t)&asn1_value__local_14,
  (intptr_t)asn1_type_j2735RTCMcorrections,
},
{
  (intptr_t)&asn1_value__local_15,
  (intptr_t)asn1_type_j2735TravelerInformation,
},
{
  (intptr_t)&asn1_value__local_16,
  (intptr_t)asn1_type_j2735PersonalSafetyMessage,
},
{
  (intptr_t)&asn1_value__local_17,
  (intptr_t)asn1_type_j2735TrafficLightStatusMessage,
},
};

static const ASN1CType asn1_constraint__local_2[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735MESSAGE_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_3,
  14,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_1[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_2,
};

const ASN1CType asn1_type_j2735MessageFrame[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  2,
  sizeof(j2735MessageFrame),

  offsetof(j2735MessageFrame, messageId) | 0x0,
  (intptr_t)asn1_type_j2735_local_0,
  0,
  (intptr_t)"messageId",

  offsetof(j2735MessageFrame, value) | 0x0,
  (intptr_t)asn1_type_j2735_local_1,
  0,
  (intptr_t)"value",

};

const ASN1CType asn1_type_j2735MESSAGE_ID_AND_TYPE[] = {
  (ASN1_CTYPE_OBJECT_CLASS << ASN1_CTYPE_SHIFT) | 0x0 | 0x0,
  2,

  0x1,
  (intptr_t)asn1_type_j2735DSRCmsgID,
  0,
  (intptr_t)"&id",

  0x0,
  0,
  0,
  (intptr_t)"&Type",
};

const ASN1CType asn1_type_j2735REG_EXT_ID_AND_TYPE[] = {
  (ASN1_CTYPE_OBJECT_CLASS << ASN1_CTYPE_SHIFT) | 0x0 | 0x0,
  2,

  0x1,
  (intptr_t)asn1_type_j2735RegionId,
  0,
  (intptr_t)"&id",

  0x0,
  0,
  0,
  (intptr_t)"&Type",
};

const ASN1CType asn1_type_j2735_local_18[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_22 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_21[] = {
{
  (intptr_t)&asn1_value__local_22,
  (intptr_t)asn1_type_j2735BasicSafetyMessage_KOR,
},
};

static const ASN1CType asn1_constraint__local_20[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_21,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_19[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_20,
};

const ASN1CType asn1_type_j2735RegionalExtension_1[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_1),

  offsetof(j2735RegionalExtension_1, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_18,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_1, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_19,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_23[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_25[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_24[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_25,
};

const ASN1CType asn1_type_j2735RegionalExtension_2[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_2),

  offsetof(j2735RegionalExtension_2, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_23,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_2, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_24,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_27[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_31 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_30[] = {
{
  (intptr_t)&asn1_value__local_31,
  (intptr_t)asn1_type_j2735EmergencyVehicleAlert_KOR,
},
};

static const ASN1CType asn1_constraint__local_29[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_30,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_28[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_29,
};

const ASN1CType asn1_type_j2735RegionalExtension_3[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_3),

  offsetof(j2735RegionalExtension_3, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_27,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_3, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_28,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_32[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_36 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_35[] = {
{
  (intptr_t)&asn1_value__local_36,
  (intptr_t)asn1_type_j2735IntersectionCollisionAvoidance_KOR,
},
};

static const ASN1CType asn1_constraint__local_34[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_35,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_33[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_34,
};

const ASN1CType asn1_type_j2735RegionalExtension_4[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_4),

  offsetof(j2735RegionalExtension_4, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_32,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_4, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_33,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_37[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_39[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_38[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_39,
};

const ASN1CType asn1_type_j2735RegionalExtension_5[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_5),

  offsetof(j2735RegionalExtension_5, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_37,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_5, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_38,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_41[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_43[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_42[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_43,
};

const ASN1CType asn1_type_j2735RegionalExtension_6[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_6),

  offsetof(j2735RegionalExtension_6, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_41,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_6, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_42,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_45[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_49 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_48[] = {
{
  (intptr_t)&asn1_value__local_49,
  (intptr_t)asn1_type_j2735PersonalSafetyMessage_KOR,
},
};

static const ASN1CType asn1_constraint__local_47[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_48,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_46[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_47,
};

const ASN1CType asn1_type_j2735RegionalExtension_7[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_7),

  offsetof(j2735RegionalExtension_7, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_45,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_7, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_46,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_50[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_52[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_51[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_52,
};

const ASN1CType asn1_type_j2735RegionalExtension_8[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_8),

  offsetof(j2735RegionalExtension_8, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_50,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_8, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_51,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_54[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_58 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_57[] = {
{
  (intptr_t)&asn1_value__local_58,
  (intptr_t)asn1_type_j2735ProbeVehicleData_KOR,
},
};

static const ASN1CType asn1_constraint__local_56[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_57,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_55[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_56,
};

const ASN1CType asn1_type_j2735RegionalExtension_9[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_9),

  offsetof(j2735RegionalExtension_9, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_54,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_9, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_55,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_59[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_63 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_62[] = {
{
  (intptr_t)&asn1_value__local_63,
  (intptr_t)asn1_type_j2735RoadSideAlert_KOR,
},
};

static const ASN1CType asn1_constraint__local_61[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_62,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_60[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_61,
};

const ASN1CType asn1_type_j2735RegionalExtension_10[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_10),

  offsetof(j2735RegionalExtension_10, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_59,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_10, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_60,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_64[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_66[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_65[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_66,
};

const ASN1CType asn1_type_j2735RegionalExtension_11[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_11),

  offsetof(j2735RegionalExtension_11, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_64,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_11, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_65,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_68[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_70[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_69[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_70,
};

const ASN1CType asn1_type_j2735RegionalExtension_12[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_12),

  offsetof(j2735RegionalExtension_12, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_68,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_12, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_69,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_72[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_76 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_75[] = {
{
  (intptr_t)&asn1_value__local_76,
  (intptr_t)asn1_type_j2735TravelerInformation_KOR,
},
};

static const ASN1CType asn1_constraint__local_74[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_75,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_73[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_74,
};

const ASN1CType asn1_type_j2735RegionalExtension_13[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_13),

  offsetof(j2735RegionalExtension_13, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_72,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_13, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_73,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_77[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_79[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_78[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_79,
};

const ASN1CType asn1_type_j2735RegionalExtension_14[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_14),

  offsetof(j2735RegionalExtension_14, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_77,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_14, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_78,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_81[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_83[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_82[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_83,
};

const ASN1CType asn1_type_j2735RegionalExtension_15[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_15),

  offsetof(j2735RegionalExtension_15, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_81,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_15, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_82,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_85[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_87[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_86[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_87,
};

const ASN1CType asn1_type_j2735RegionalExtension_16[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_16),

  offsetof(j2735RegionalExtension_16, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_85,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_16, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_86,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_89[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_93 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_92[] = {
{
  (intptr_t)&asn1_value__local_93,
  (intptr_t)asn1_type_j2735EventDescription_KOR,
},
};

static const ASN1CType asn1_constraint__local_91[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_92,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_90[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_91,
};

const ASN1CType asn1_type_j2735RegionalExtension_17[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_17),

  offsetof(j2735RegionalExtension_17, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_89,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_17, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_90,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_94[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_96[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_95[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_96,
};

const ASN1CType asn1_type_j2735RegionalExtension_18[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_18),

  offsetof(j2735RegionalExtension_18, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_94,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_18, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_95,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_98[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_102 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_101[] = {
{
  (intptr_t)&asn1_value__local_102,
  (intptr_t)asn1_type_j2735GeographicalPath_KOR,
},
};

static const ASN1CType asn1_constraint__local_100[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_101,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_99[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_100,
};

const ASN1CType asn1_type_j2735RegionalExtension_19[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_19),

  offsetof(j2735RegionalExtension_19, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_98,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_19, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_99,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_103[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_105[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_104[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_105,
};

const ASN1CType asn1_type_j2735RegionalExtension_20[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_20),

  offsetof(j2735RegionalExtension_20, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_103,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_20, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_104,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_107[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_109[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_108[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_109,
};

const ASN1CType asn1_type_j2735RegionalExtension_21[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_21),

  offsetof(j2735RegionalExtension_21, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_107,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_21, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_108,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_111[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_113[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_112[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_113,
};

const ASN1CType asn1_type_j2735RegionalExtension_22[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_22),

  offsetof(j2735RegionalExtension_22, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_111,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_22, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_112,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_115[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_117[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_116[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_117,
};

const ASN1CType asn1_type_j2735RegionalExtension_23[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  2,
  sizeof(j2735RegionalExtension_23),

  offsetof(j2735RegionalExtension_23, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_115,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_23, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_116,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_119[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_121[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_120[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_121,
};

const ASN1CType asn1_type_j2735RegionalExtension_24[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_24),

  offsetof(j2735RegionalExtension_24, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_119,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_24, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_120,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_123[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_125[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_124[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_125,
};

const ASN1CType asn1_type_j2735RegionalExtension_25[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_25),

  offsetof(j2735RegionalExtension_25, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_123,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_25, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_124,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_127[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_129[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_128[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_129,
};

const ASN1CType asn1_type_j2735RegionalExtension_26[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_26),

  offsetof(j2735RegionalExtension_26, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_127,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_26, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_128,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_131[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_133[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_132[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_133,
};

const ASN1CType asn1_type_j2735RegionalExtension_27[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_27),

  offsetof(j2735RegionalExtension_27, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_131,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_27, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_132,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_135[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_137[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_136[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_137,
};

const ASN1CType asn1_type_j2735RegionalExtension_28[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_28),

  offsetof(j2735RegionalExtension_28, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_135,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_28, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_136,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_139[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_141[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_140[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_141,
};

const ASN1CType asn1_type_j2735RegionalExtension_29[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  2,
  sizeof(j2735RegionalExtension_29),

  offsetof(j2735RegionalExtension_29, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_139,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_29, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_140,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_143[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_145[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_144[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_145,
};

const ASN1CType asn1_type_j2735RegionalExtension_30[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  2,
  sizeof(j2735RegionalExtension_30),

  offsetof(j2735RegionalExtension_30, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_143,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_30, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_144,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_147[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_149[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_148[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_149,
};

const ASN1CType asn1_type_j2735RegionalExtension_31[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_31),

  offsetof(j2735RegionalExtension_31, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_147,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_31, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_148,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_151[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_153[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_152[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_153,
};

const ASN1CType asn1_type_j2735RegionalExtension_32[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735RegionalExtension_32),

  offsetof(j2735RegionalExtension_32, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_151,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_32, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_152,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_155[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_157[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_156[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_157,
};

const ASN1CType asn1_type_j2735RegionalExtension_33[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_33),

  offsetof(j2735RegionalExtension_33, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_155,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_33, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_156,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_159[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_161[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_160[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_161,
};

const ASN1CType asn1_type_j2735RegionalExtension_34[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  2,
  sizeof(j2735RegionalExtension_34),

  offsetof(j2735RegionalExtension_34, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_159,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_34, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_160,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_163[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_165[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_164[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_165,
};

const ASN1CType asn1_type_j2735RegionalExtension_35[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_35),

  offsetof(j2735RegionalExtension_35, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_163,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_35, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_164,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_167[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_169[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_168[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_169,
};

const ASN1CType asn1_type_j2735RegionalExtension_36[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_36),

  offsetof(j2735RegionalExtension_36, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_167,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_36, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_168,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_171[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_173[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_172[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_173,
};

const ASN1CType asn1_type_j2735RegionalExtension_37[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_37),

  offsetof(j2735RegionalExtension_37, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_171,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_37, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_172,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_175[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_177[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_176[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_177,
};

const ASN1CType asn1_type_j2735RegionalExtension_38[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_38),

  offsetof(j2735RegionalExtension_38, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_175,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_38, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_176,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_179[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_181[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_180[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_181,
};

const ASN1CType asn1_type_j2735RegionalExtension_39[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_39),

  offsetof(j2735RegionalExtension_39, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_179,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_39, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_180,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_183[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const ASN1CType asn1_constraint__local_185[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  0,
  0,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_184[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_185,
};

const ASN1CType asn1_type_j2735RegionalExtension_40[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_40),

  offsetof(j2735RegionalExtension_40, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_183,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_40, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_184,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_187[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_191 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_190[] = {
{
  (intptr_t)&asn1_value__local_191,
  (intptr_t)asn1_type_j2735SupplementalVehicleExtensions_KOR,
},
};

static const ASN1CType asn1_constraint__local_189[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_190,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_188[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_189,
};

const ASN1CType asn1_type_j2735RegionalExtension_41[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_41),

  offsetof(j2735RegionalExtension_41, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_187,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_41, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_188,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_192[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RegionId,
};

static const j2735RegionId asn1_value__local_196 = 4;

static const j2735REG_EXT_ID_AND_TYPE asn1_value__local_195[] = {
{
  (intptr_t)&asn1_value__local_196,
  (intptr_t)asn1_type_j2735VehicleClassification_KOR,
},
};

static const ASN1CType asn1_constraint__local_194[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735REG_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_195,
  1,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_193[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_194,
};

const ASN1CType asn1_type_j2735RegionalExtension_42[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegionalExtension_42),

  offsetof(j2735RegionalExtension_42, regionId) | 0x0,
  (intptr_t)asn1_type_j2735_local_192,
  0,
  (intptr_t)"regionId",

  offsetof(j2735RegionalExtension_42, regExtValue) | 0x0,
  (intptr_t)asn1_type_j2735_local_193,
  0,
  (intptr_t)"regExtValue",

};

const ASN1CType asn1_type_j2735_local_197[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735BSMcoreData,
};

const ASN1CType asn1_type_j2735BasicSafetyMessage[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  3,
  sizeof(j2735BasicSafetyMessage),

  offsetof(j2735BasicSafetyMessage, coreData) | 0x0,
  (intptr_t)asn1_type_j2735_local_197,
  0,
  (intptr_t)"coreData",

  offsetof(j2735BasicSafetyMessage, partII) | 0x8000000,
  (intptr_t)asn1_type_j2735BasicSafetyMessage_1,
  offsetof(j2735BasicSafetyMessage, partII_option),
  (intptr_t)"partII",

  offsetof(j2735BasicSafetyMessage, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735BasicSafetyMessage_2,
  offsetof(j2735BasicSafetyMessage, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735BasicSafetyMessage_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x8,
  sizeof(j2735PartIIcontent_1),
  (intptr_t)asn1_type_j2735PartIIcontent_1,
  0,
};

const ASN1CType asn1_type_j2735BasicSafetyMessage_2[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_1),
  (intptr_t)asn1_type_j2735RegionalExtension_1,
  0,
};

const ASN1CType asn1_type_j2735PARTII_EXT_ID_AND_TYPE[] = {
  (ASN1_CTYPE_OBJECT_CLASS << ASN1_CTYPE_SHIFT) | 0x0 | 0x0,
  2,

  0x1,
  (intptr_t)asn1_type_j2735PartII_Id,
  0,
  (intptr_t)"&id",

  0x0,
  0,
  0,
  (intptr_t)"&Type",
};

const ASN1CType asn1_type_j2735_local_198[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735PartII_Id,
};

static const j2735PartII_Id asn1_value__local_202 = 0;

static const j2735PartII_Id asn1_value__local_203 = 1;

static const j2735PartII_Id asn1_value__local_204 = 2;

static const j2735PARTII_EXT_ID_AND_TYPE asn1_value__local_201[] = {
{
  (intptr_t)&asn1_value__local_202,
  (intptr_t)asn1_type_j2735VehicleSafetyExtensions,
},
{
  (intptr_t)&asn1_value__local_203,
  (intptr_t)asn1_type_j2735SpecialVehicleExtensions,
},
{
  (intptr_t)&asn1_value__local_204,
  (intptr_t)asn1_type_j2735SupplementalVehicleExtensions,
},
};

static const ASN1CType asn1_constraint__local_200[] = {
  ASN1_CCONSTRAINT_TABLE,
  (intptr_t)asn1_type_j2735PARTII_EXT_ID_AND_TYPE,
  1,
  (intptr_t)asn1_value__local_201,
  3,
  1,
  0,
  0x80000000,
  0,
};

const ASN1CType asn1_type_j2735_local_199[] = {
  (ASN1_CTYPE_ANY << ASN1_CTYPE_SHIFT) | 0x200000 | 0x100001,
  (intptr_t)asn1_constraint__local_200,
};

const ASN1CType asn1_type_j2735PartIIcontent_1[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735PartIIcontent_1),

  offsetof(j2735PartIIcontent_1, partII_Id) | 0x0,
  (intptr_t)asn1_type_j2735_local_198,
  0,
  (intptr_t)"partII-Id",

  offsetof(j2735PartIIcontent_1, partII_Value) | 0x0,
  (intptr_t)asn1_type_j2735_local_199,
  0,
  (intptr_t)"partII-Value",

};

const ASN1CType asn1_type_j2735PartII_Id[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x3f,
};

const ASN1CType asn1_type_j2735_local_205[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_206[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_207[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735TemporaryID,
};

const ASN1CType asn1_type_j2735CommonSafetyRequest[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  5,
  sizeof(j2735CommonSafetyRequest),

  offsetof(j2735CommonSafetyRequest, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_205,
  offsetof(j2735CommonSafetyRequest, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735CommonSafetyRequest, msgCnt) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_206,
  offsetof(j2735CommonSafetyRequest, msgCnt_option),
  (intptr_t)"msgCnt",

  offsetof(j2735CommonSafetyRequest, id) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_207,
  offsetof(j2735CommonSafetyRequest, id_option),
  (intptr_t)"id",

  offsetof(j2735CommonSafetyRequest, requests) | 0x0,
  (intptr_t)asn1_type_j2735RequestedItemList,
  0,
  (intptr_t)"requests",

  offsetof(j2735CommonSafetyRequest, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735CommonSafetyRequest_1,
  offsetof(j2735CommonSafetyRequest, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735CommonSafetyRequest_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_2),
  (intptr_t)asn1_type_j2735RegionalExtension_2,
  0,
};

const ASN1CType asn1_type_j2735_local_208[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_209[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735TemporaryID,
};

const ASN1CType asn1_type_j2735_local_210[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735RoadSideAlert,
};

const ASN1CType asn1_type_j2735_local_211[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735ResponseType,
};

const ASN1CType asn1_type_j2735_local_212[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735EmergencyDetails,
};

const ASN1CType asn1_type_j2735_local_213[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735VehicleMass,
};

const ASN1CType asn1_type_j2735_local_214[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735VehicleType,
};

const ASN1CType asn1_type_j2735_local_215[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735VehicleGroupAffected,
};

const ASN1CType asn1_type_j2735_local_216[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735IncidentResponseEquipment,
};

const ASN1CType asn1_type_j2735_local_217[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  (intptr_t)asn1_type_j2735ResponderGroupAffected,
};

const ASN1CType asn1_type_j2735EmergencyVehicleAlert[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  11,
  sizeof(j2735EmergencyVehicleAlert),

  offsetof(j2735EmergencyVehicleAlert, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_208,
  offsetof(j2735EmergencyVehicleAlert, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735EmergencyVehicleAlert, id) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_209,
  offsetof(j2735EmergencyVehicleAlert, id_option),
  (intptr_t)"id",

  offsetof(j2735EmergencyVehicleAlert, rsaMsg) | 0x0,
  (intptr_t)asn1_type_j2735_local_210,
  0,
  (intptr_t)"rsaMsg",

  offsetof(j2735EmergencyVehicleAlert, responseType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_211,
  offsetof(j2735EmergencyVehicleAlert, responseType_option),
  (intptr_t)"responseType",

  offsetof(j2735EmergencyVehicleAlert, details) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_212,
  offsetof(j2735EmergencyVehicleAlert, details_option),
  (intptr_t)"details",

  offsetof(j2735EmergencyVehicleAlert, mass) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_213,
  offsetof(j2735EmergencyVehicleAlert, mass_option),
  (intptr_t)"mass",

  offsetof(j2735EmergencyVehicleAlert, basicType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_214,
  offsetof(j2735EmergencyVehicleAlert, basicType_option),
  (intptr_t)"basicType",

  offsetof(j2735EmergencyVehicleAlert, vehicleType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_215,
  offsetof(j2735EmergencyVehicleAlert, vehicleType_option),
  (intptr_t)"vehicleType",

  offsetof(j2735EmergencyVehicleAlert, responseEquip) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_216,
  offsetof(j2735EmergencyVehicleAlert, responseEquip_option),
  (intptr_t)"responseEquip",

  offsetof(j2735EmergencyVehicleAlert, responderType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_217,
  offsetof(j2735EmergencyVehicleAlert, responderType_option),
  (intptr_t)"responderType",

  offsetof(j2735EmergencyVehicleAlert, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735EmergencyVehicleAlert_1,
  offsetof(j2735EmergencyVehicleAlert, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735EmergencyVehicleAlert_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10000a,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_3),
  (intptr_t)asn1_type_j2735RegionalExtension_3,
  0,
};

const ASN1CType asn1_type_j2735_local_218[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_219[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735TemporaryID,
};

const ASN1CType asn1_type_j2735_local_220[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_221[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735BSMcoreData,
};

const ASN1CType asn1_type_j2735_local_222[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735PathHistory,
};

const ASN1CType asn1_type_j2735_local_223[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735PathPrediction,
};

const ASN1CType asn1_type_j2735_local_224[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735IntersectionReferenceID,
};

const ASN1CType asn1_type_j2735_local_225[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735VehicleEventFlags,
};

const ASN1CType asn1_type_j2735IntersectionCollision[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  10,
  sizeof(j2735IntersectionCollision),

  offsetof(j2735IntersectionCollision, msgCnt) | 0x0,
  (intptr_t)asn1_type_j2735_local_218,
  0,
  (intptr_t)"msgCnt",

  offsetof(j2735IntersectionCollision, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_219,
  0,
  (intptr_t)"id",

  offsetof(j2735IntersectionCollision, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_220,
  offsetof(j2735IntersectionCollision, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735IntersectionCollision, partOne) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_221,
  offsetof(j2735IntersectionCollision, partOne_option),
  (intptr_t)"partOne",

  offsetof(j2735IntersectionCollision, path) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_222,
  offsetof(j2735IntersectionCollision, path_option),
  (intptr_t)"path",

  offsetof(j2735IntersectionCollision, pathPrediction) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_223,
  offsetof(j2735IntersectionCollision, pathPrediction_option),
  (intptr_t)"pathPrediction",

  offsetof(j2735IntersectionCollision, intersectionID) | 0x0,
  (intptr_t)asn1_type_j2735_local_224,
  0,
  (intptr_t)"intersectionID",

  offsetof(j2735IntersectionCollision, laneNumber) | 0x0,
  (intptr_t)asn1_type_j2735ApproachOrLane,
  0,
  (intptr_t)"laneNumber",

  offsetof(j2735IntersectionCollision, eventFlag) | 0x0,
  (intptr_t)asn1_type_j2735_local_225,
  0,
  (intptr_t)"eventFlag",

  offsetof(j2735IntersectionCollision, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735IntersectionCollision_1,
  offsetof(j2735IntersectionCollision, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735IntersectionCollision_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100009,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_4),
  (intptr_t)asn1_type_j2735RegionalExtension_4,
  0,
};

const ASN1CType asn1_type_j2735_local_226[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_227[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735MapData[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  9,
  sizeof(j2735MapData),

  offsetof(j2735MapData, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_226,
  offsetof(j2735MapData, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735MapData, msgIssueRevision) | 0x0,
  (intptr_t)asn1_type_j2735_local_227,
  0,
  (intptr_t)"msgIssueRevision",

  offsetof(j2735MapData, layerType) | 0x8000000,
  (intptr_t)asn1_type_j2735LayerType,
  offsetof(j2735MapData, layerType_option),
  (intptr_t)"layerType",

  offsetof(j2735MapData, layerID) | 0x8000000,
  (intptr_t)asn1_type_j2735LayerID,
  offsetof(j2735MapData, layerID_option),
  (intptr_t)"layerID",

  offsetof(j2735MapData, intersections) | 0x8000000,
  (intptr_t)asn1_type_j2735IntersectionGeometryList,
  offsetof(j2735MapData, intersections_option),
  (intptr_t)"intersections",

  offsetof(j2735MapData, roadSegments) | 0x8000000,
  (intptr_t)asn1_type_j2735RoadSegmentList,
  offsetof(j2735MapData, roadSegments_option),
  (intptr_t)"roadSegments",

  offsetof(j2735MapData, dataParameters) | 0x8000000,
  (intptr_t)asn1_type_j2735DataParameters,
  offsetof(j2735MapData, dataParameters_option),
  (intptr_t)"dataParameters",

  offsetof(j2735MapData, restrictionList) | 0x8000000,
  (intptr_t)asn1_type_j2735RestrictionClassList,
  offsetof(j2735MapData, restrictionList_option),
  (intptr_t)"restrictionList",

  offsetof(j2735MapData, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735MapData_1,
  offsetof(j2735MapData, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735MapData_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_5),
  (intptr_t)asn1_type_j2735RegionalExtension_5,
  0,
};

const ASN1CType asn1_type_j2735_local_228[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735NMEAcorrections[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735NMEAcorrections),

  offsetof(j2735NMEAcorrections, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_228,
  offsetof(j2735NMEAcorrections, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735NMEAcorrections, rev) | 0x8000000,
  (intptr_t)asn1_type_j2735NMEA_Revision,
  offsetof(j2735NMEAcorrections, rev_option),
  (intptr_t)"rev",

  offsetof(j2735NMEAcorrections, msg) | 0x8000000,
  (intptr_t)asn1_type_j2735NMEA_MsgType,
  offsetof(j2735NMEAcorrections, msg_option),
  (intptr_t)"msg",

  offsetof(j2735NMEAcorrections, wdCount) | 0x8000000,
  (intptr_t)asn1_type_j2735ObjectCount,
  offsetof(j2735NMEAcorrections, wdCount_option),
  (intptr_t)"wdCount",

  offsetof(j2735NMEAcorrections, payload) | 0x0,
  (intptr_t)asn1_type_j2735NMEA_Payload,
  0,
  (intptr_t)"payload",

  offsetof(j2735NMEAcorrections, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735NMEAcorrections_1,
  offsetof(j2735NMEAcorrections, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735NMEAcorrections_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_6),
  (intptr_t)asn1_type_j2735RegionalExtension_6,
  0,
};

const ASN1CType asn1_type_j2735_local_229[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735DSecond,
};

const ASN1CType asn1_type_j2735_local_230[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_231[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735TemporaryID,
};

const ASN1CType asn1_type_j2735_local_232[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Position3D,
};

const ASN1CType asn1_type_j2735_local_233[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735PositionalAccuracy,
};

const ASN1CType asn1_type_j2735_local_234[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735Velocity,
};

const ASN1CType asn1_type_j2735_local_235[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735Heading,
};

const ASN1CType asn1_type_j2735_local_236[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735AccelerationSet4Way,
};

const ASN1CType asn1_type_j2735_local_237[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  (intptr_t)asn1_type_j2735PathHistory,
};

const ASN1CType asn1_type_j2735_local_238[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000a,
  (intptr_t)asn1_type_j2735PathPrediction,
};

const ASN1CType asn1_type_j2735PersonalSafetyMessage[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  26,
  sizeof(j2735PersonalSafetyMessage),

  offsetof(j2735PersonalSafetyMessage, basicType) | 0x0,
  (intptr_t)asn1_type_j2735PersonalDeviceUserType,
  0,
  (intptr_t)"basicType",

  offsetof(j2735PersonalSafetyMessage, secMark) | 0x0,
  (intptr_t)asn1_type_j2735_local_229,
  0,
  (intptr_t)"secMark",

  offsetof(j2735PersonalSafetyMessage, msgCnt) | 0x0,
  (intptr_t)asn1_type_j2735_local_230,
  0,
  (intptr_t)"msgCnt",

  offsetof(j2735PersonalSafetyMessage, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_231,
  0,
  (intptr_t)"id",

  offsetof(j2735PersonalSafetyMessage, position) | 0x0,
  (intptr_t)asn1_type_j2735_local_232,
  0,
  (intptr_t)"position",

  offsetof(j2735PersonalSafetyMessage, accuracy) | 0x0,
  (intptr_t)asn1_type_j2735_local_233,
  0,
  (intptr_t)"accuracy",

  offsetof(j2735PersonalSafetyMessage, speed) | 0x0,
  (intptr_t)asn1_type_j2735_local_234,
  0,
  (intptr_t)"speed",

  offsetof(j2735PersonalSafetyMessage, heading) | 0x0,
  (intptr_t)asn1_type_j2735_local_235,
  0,
  (intptr_t)"heading",

  offsetof(j2735PersonalSafetyMessage, accelSet) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_236,
  offsetof(j2735PersonalSafetyMessage, accelSet_option),
  (intptr_t)"accelSet",

  offsetof(j2735PersonalSafetyMessage, pathHistory) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_237,
  offsetof(j2735PersonalSafetyMessage, pathHistory_option),
  (intptr_t)"pathHistory",

  offsetof(j2735PersonalSafetyMessage, pathPrediction) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_238,
  offsetof(j2735PersonalSafetyMessage, pathPrediction_option),
  (intptr_t)"pathPrediction",

  offsetof(j2735PersonalSafetyMessage, propulsion) | 0x8000000,
  (intptr_t)asn1_type_j2735PropelledInformation,
  offsetof(j2735PersonalSafetyMessage, propulsion_option),
  (intptr_t)"propulsion",

  offsetof(j2735PersonalSafetyMessage, useState) | 0x8000000,
  (intptr_t)asn1_type_j2735PersonalDeviceUsageState,
  offsetof(j2735PersonalSafetyMessage, useState_option),
  (intptr_t)"useState",

  offsetof(j2735PersonalSafetyMessage, crossRequest) | 0x8000000,
  (intptr_t)asn1_type_j2735PersonalCrossingRequest,
  offsetof(j2735PersonalSafetyMessage, crossRequest_option),
  (intptr_t)"crossRequest",

  offsetof(j2735PersonalSafetyMessage, crossState) | 0x8000000,
  (intptr_t)asn1_type_j2735PersonalCrossingInProgress,
  offsetof(j2735PersonalSafetyMessage, crossState_option),
  (intptr_t)"crossState",

  offsetof(j2735PersonalSafetyMessage, clusterSize) | 0x8000000,
  (intptr_t)asn1_type_j2735NumberOfParticipantsInCluster,
  offsetof(j2735PersonalSafetyMessage, clusterSize_option),
  (intptr_t)"clusterSize",

  offsetof(j2735PersonalSafetyMessage, clusterRadius) | 0x8000000,
  (intptr_t)asn1_type_j2735PersonalClusterRadius,
  offsetof(j2735PersonalSafetyMessage, clusterRadius_option),
  (intptr_t)"clusterRadius",

  offsetof(j2735PersonalSafetyMessage, eventResponderType) | 0x8000000,
  (intptr_t)asn1_type_j2735PublicSafetyEventResponderWorkerType,
  offsetof(j2735PersonalSafetyMessage, eventResponderType_option),
  (intptr_t)"eventResponderType",

  offsetof(j2735PersonalSafetyMessage, activityType) | 0x8000000,
  (intptr_t)asn1_type_j2735PublicSafetyAndRoadWorkerActivity,
  offsetof(j2735PersonalSafetyMessage, activityType_option),
  (intptr_t)"activityType",

  offsetof(j2735PersonalSafetyMessage, activitySubType) | 0x8000000,
  (intptr_t)asn1_type_j2735PublicSafetyDirectingTrafficSubType,
  offsetof(j2735PersonalSafetyMessage, activitySubType_option),
  (intptr_t)"activitySubType",

  offsetof(j2735PersonalSafetyMessage, assistType) | 0x8000000,
  (intptr_t)asn1_type_j2735PersonalAssistive,
  offsetof(j2735PersonalSafetyMessage, assistType_option),
  (intptr_t)"assistType",

  offsetof(j2735PersonalSafetyMessage, sizing) | 0x8000000,
  (intptr_t)asn1_type_j2735UserSizeAndBehaviour,
  offsetof(j2735PersonalSafetyMessage, sizing_option),
  (intptr_t)"sizing",

  offsetof(j2735PersonalSafetyMessage, attachment) | 0x8000000,
  (intptr_t)asn1_type_j2735Attachment,
  offsetof(j2735PersonalSafetyMessage, attachment_option),
  (intptr_t)"attachment",

  offsetof(j2735PersonalSafetyMessage, attachmentRadius) | 0x8000000,
  (intptr_t)asn1_type_j2735AttachmentRadius,
  offsetof(j2735PersonalSafetyMessage, attachmentRadius_option),
  (intptr_t)"attachmentRadius",

  offsetof(j2735PersonalSafetyMessage, animalType) | 0x8000000,
  (intptr_t)asn1_type_j2735AnimalType,
  offsetof(j2735PersonalSafetyMessage, animalType_option),
  (intptr_t)"animalType",

  offsetof(j2735PersonalSafetyMessage, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735PersonalSafetyMessage_1,
  offsetof(j2735PersonalSafetyMessage, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735PersonalSafetyMessage_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100019,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_7),
  (intptr_t)asn1_type_j2735RegionalExtension_7,
  0,
};

const ASN1CType asn1_type_j2735_local_239[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_240[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735HeadingSlice,
};

const ASN1CType asn1_type_j2735_local_241[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735SecondOfTime,
};

const ASN1CType asn1_type_j2735ProbeDataManagement[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  8,
  sizeof(j2735ProbeDataManagement),

  offsetof(j2735ProbeDataManagement, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_239,
  offsetof(j2735ProbeDataManagement, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735ProbeDataManagement, sample) | 0x0,
  (intptr_t)asn1_type_j2735Sample,
  0,
  (intptr_t)"sample",

  offsetof(j2735ProbeDataManagement, directions) | 0x0,
  (intptr_t)asn1_type_j2735_local_240,
  0,
  (intptr_t)"directions",

  offsetof(j2735ProbeDataManagement, term) | 0x0,
  (intptr_t)asn1_type_j2735ProbeDataManagement_1,
  0,
  (intptr_t)"term",

  offsetof(j2735ProbeDataManagement, snapshot) | 0x0,
  (intptr_t)asn1_type_j2735ProbeDataManagement_2,
  0,
  (intptr_t)"snapshot",

  offsetof(j2735ProbeDataManagement, txInterval) | 0x0,
  (intptr_t)asn1_type_j2735_local_241,
  0,
  (intptr_t)"txInterval",

  offsetof(j2735ProbeDataManagement, dataElements) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleStatusRequestList,
  offsetof(j2735ProbeDataManagement, dataElements_option),
  (intptr_t)"dataElements",

  offsetof(j2735ProbeDataManagement, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735ProbeDataManagement_3,
  offsetof(j2735ProbeDataManagement, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735ProbeDataManagement_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  2,
  sizeof(j2735ProbeDataManagement_1),
  offsetof(j2735ProbeDataManagement_1, choice),
  offsetof(j2735ProbeDataManagement_1, u),
  (intptr_t)asn1_type_j2735TermTime,
  (intptr_t)"termtime",
  (intptr_t)asn1_type_j2735TermDistance,
  (intptr_t)"termDistance",
};

const ASN1CType asn1_type_j2735ProbeDataManagement_2[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  2,
  sizeof(j2735ProbeDataManagement_2),
  offsetof(j2735ProbeDataManagement_2, choice),
  offsetof(j2735ProbeDataManagement_2, u),
  (intptr_t)asn1_type_j2735SnapshotTime,
  (intptr_t)"snapshotTime",
  (intptr_t)asn1_type_j2735SnapshotDistance,
  (intptr_t)"snapshotDistance",
};

const ASN1CType asn1_type_j2735ProbeDataManagement_3[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_8),
  (intptr_t)asn1_type_j2735RegionalExtension_8,
  0,
};

const ASN1CType asn1_type_j2735_local_242[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_243[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735VehicleIdent,
};

const ASN1CType asn1_type_j2735_local_244[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735FullPositionVector,
};

const ASN1CType asn1_type_j2735_local_245[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735VehicleClassification,
};

const ASN1CType asn1_type_j2735ProbeVehicleData[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  7,
  sizeof(j2735ProbeVehicleData),

  offsetof(j2735ProbeVehicleData, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_242,
  offsetof(j2735ProbeVehicleData, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735ProbeVehicleData, segNum) | 0x8000000,
  (intptr_t)asn1_type_j2735ProbeSegmentNumber,
  offsetof(j2735ProbeVehicleData, segNum_option),
  (intptr_t)"segNum",

  offsetof(j2735ProbeVehicleData, probeID) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_243,
  offsetof(j2735ProbeVehicleData, probeID_option),
  (intptr_t)"probeID",

  offsetof(j2735ProbeVehicleData, startVector) | 0x0,
  (intptr_t)asn1_type_j2735_local_244,
  0,
  (intptr_t)"startVector",

  offsetof(j2735ProbeVehicleData, vehicleType) | 0x0,
  (intptr_t)asn1_type_j2735_local_245,
  0,
  (intptr_t)"vehicleType",

  offsetof(j2735ProbeVehicleData, snapshots) | 0x0,
  (intptr_t)asn1_type_j2735ProbeVehicleData_1,
  0,
  (intptr_t)"snapshots",

  offsetof(j2735ProbeVehicleData, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735ProbeVehicleData_2,
  offsetof(j2735ProbeVehicleData, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735ProbeVehicleData_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x1,
  0x20,
  sizeof(j2735Snapshot),
  (intptr_t)asn1_type_j2735Snapshot,
  0,
};

const ASN1CType asn1_type_j2735ProbeVehicleData_2[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_9),
  (intptr_t)asn1_type_j2735RegionalExtension_9,
  0,
};

const ASN1CType asn1_type_j2735_local_246[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_247[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_248[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735_local_249[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Priority,
};

const ASN1CType asn1_type_j2735_local_250[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735HeadingSlice,
};

const ASN1CType asn1_type_j2735_local_251[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735Extent,
};

const ASN1CType asn1_type_j2735_local_252[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735FullPositionVector,
};

const ASN1CType asn1_type_j2735_local_253[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735FurtherInfoID,
};

const ASN1CType asn1_type_j2735RoadSideAlert[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  10,
  sizeof(j2735RoadSideAlert),

  offsetof(j2735RoadSideAlert, msgCnt) | 0x0,
  (intptr_t)asn1_type_j2735_local_246,
  0,
  (intptr_t)"msgCnt",

  offsetof(j2735RoadSideAlert, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_247,
  offsetof(j2735RoadSideAlert, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735RoadSideAlert, typeEvent) | 0x0,
  (intptr_t)asn1_type_j2735_local_248,
  0,
  (intptr_t)"typeEvent",

  offsetof(j2735RoadSideAlert, description) | 0x8000000,
  (intptr_t)asn1_type_j2735RoadSideAlert_1,
  offsetof(j2735RoadSideAlert, description_option),
  (intptr_t)"description",

  offsetof(j2735RoadSideAlert, priority) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_249,
  offsetof(j2735RoadSideAlert, priority_option),
  (intptr_t)"priority",

  offsetof(j2735RoadSideAlert, heading) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_250,
  offsetof(j2735RoadSideAlert, heading_option),
  (intptr_t)"heading",

  offsetof(j2735RoadSideAlert, extent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_251,
  offsetof(j2735RoadSideAlert, extent_option),
  (intptr_t)"extent",

  offsetof(j2735RoadSideAlert, position) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_252,
  offsetof(j2735RoadSideAlert, position_option),
  (intptr_t)"position",

  offsetof(j2735RoadSideAlert, furtherInfoID) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_253,
  offsetof(j2735RoadSideAlert, furtherInfoID_option),
  (intptr_t)"furtherInfoID",

  offsetof(j2735RoadSideAlert, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735RoadSideAlert_2,
  offsetof(j2735RoadSideAlert, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735RoadSideAlert_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x1,
  0x8,
  sizeof(j2735ITIScodes),
  (intptr_t)asn1_type_j2735ITIScodes,
  0,
};

const ASN1CType asn1_type_j2735RoadSideAlert_2[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100009,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_10),
  (intptr_t)asn1_type_j2735RegionalExtension_10,
  0,
};

const ASN1CType asn1_type_j2735_local_254[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_255[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_256[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735FullPositionVector,
};

const ASN1CType asn1_type_j2735_local_257[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735RTCMheader,
};

const ASN1CType asn1_type_j2735_local_258[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735RTCMmessageList,
};

const ASN1CType asn1_type_j2735RTCMcorrections[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  7,
  sizeof(j2735RTCMcorrections),

  offsetof(j2735RTCMcorrections, msgCnt) | 0x0,
  (intptr_t)asn1_type_j2735_local_254,
  0,
  (intptr_t)"msgCnt",

  offsetof(j2735RTCMcorrections, rev) | 0x0,
  (intptr_t)asn1_type_j2735RTCM_Revision,
  0,
  (intptr_t)"rev",

  offsetof(j2735RTCMcorrections, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_255,
  offsetof(j2735RTCMcorrections, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735RTCMcorrections, anchorPoint) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_256,
  offsetof(j2735RTCMcorrections, anchorPoint_option),
  (intptr_t)"anchorPoint",

  offsetof(j2735RTCMcorrections, rtcmHeader) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_257,
  offsetof(j2735RTCMcorrections, rtcmHeader_option),
  (intptr_t)"rtcmHeader",

  offsetof(j2735RTCMcorrections, msgs) | 0x0,
  (intptr_t)asn1_type_j2735_local_258,
  0,
  (intptr_t)"msgs",

  offsetof(j2735RTCMcorrections, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735RTCMcorrections_1,
  offsetof(j2735RTCMcorrections, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735RTCMcorrections_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_11),
  (intptr_t)asn1_type_j2735RegionalExtension_11,
  0,
};

const ASN1CType asn1_type_j2735_local_259[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_260[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735SPAT[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  4,
  sizeof(j2735SPAT),

  offsetof(j2735SPAT, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_259,
  offsetof(j2735SPAT, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735SPAT, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_260,
  offsetof(j2735SPAT, name_option),
  (intptr_t)"name",

  offsetof(j2735SPAT, intersections) | 0x0,
  (intptr_t)asn1_type_j2735IntersectionStateList,
  0,
  (intptr_t)"intersections",

  offsetof(j2735SPAT, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735SPAT_1,
  offsetof(j2735SPAT, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735SPAT_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_12),
  (intptr_t)asn1_type_j2735RegionalExtension_12,
  0,
};

const ASN1CType asn1_type_j2735_local_261[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_262[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735TravelerInformation[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735TravelerInformation),

  offsetof(j2735TravelerInformation, msgCnt) | 0x0,
  (intptr_t)asn1_type_j2735_local_261,
  0,
  (intptr_t)"msgCnt",

  offsetof(j2735TravelerInformation, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_262,
  offsetof(j2735TravelerInformation, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735TravelerInformation, packetID) | 0x8000000,
  (intptr_t)asn1_type_j2735UniqueMSGID,
  offsetof(j2735TravelerInformation, packetID_option),
  (intptr_t)"packetID",

  offsetof(j2735TravelerInformation, urlB) | 0x8000000,
  (intptr_t)asn1_type_j2735URL_Base,
  offsetof(j2735TravelerInformation, urlB_option),
  (intptr_t)"urlB",

  offsetof(j2735TravelerInformation, dataFrames) | 0x0,
  (intptr_t)asn1_type_j2735TravelerDataFrameList,
  0,
  (intptr_t)"dataFrames",

  offsetof(j2735TravelerInformation, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735TravelerInformation_1,
  offsetof(j2735TravelerInformation, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735TravelerInformation_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_13),
  (intptr_t)asn1_type_j2735RegionalExtension_13,
  0,
};

const ASN1CType asn1_type_j2735_local_263[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Longitude,
};

const ASN1CType asn1_type_j2735_local_264[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Latitude,
};

const ASN1CType asn1_type_j2735_local_265[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735TimeInSecond_B16,
};

const ASN1CType asn1_type_j2735_local_266[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735TimeInSecond_B16,
};

const ASN1CType asn1_type_j2735_local_267[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735DDateTime,
};

const ASN1CType asn1_type_j2735TrafficLightStatusMessage[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  9,
  sizeof(j2735TrafficLightStatusMessage),

  offsetof(j2735TrafficLightStatusMessage, traffiLightID) | 0x0,
  (intptr_t)asn1_type_j2735TrafficLightID,
  0,
  (intptr_t)"traffiLightID",

  offsetof(j2735TrafficLightStatusMessage, Long) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_263,
  offsetof(j2735TrafficLightStatusMessage, Long_option),
  (intptr_t)"long",

  offsetof(j2735TrafficLightStatusMessage, lat) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_264,
  offsetof(j2735TrafficLightStatusMessage, lat_option),
  (intptr_t)"lat",

  offsetof(j2735TrafficLightStatusMessage, operationStatus) | 0x0,
  (intptr_t)asn1_type_j2735TrafficLightOperationStatus,
  0,
  (intptr_t)"operationStatus",

  offsetof(j2735TrafficLightStatusMessage, controllerStatus) | 0x0,
  (intptr_t)asn1_type_j2735TrafficLightControllerStatus,
  0,
  (intptr_t)"controllerStatus",

  offsetof(j2735TrafficLightStatusMessage, cycleCounter) | 0x0,
  (intptr_t)asn1_type_j2735_local_265,
  0,
  (intptr_t)"cycleCounter",

  offsetof(j2735TrafficLightStatusMessage, cycleTime) | 0x0,
  (intptr_t)asn1_type_j2735_local_266,
  0,
  (intptr_t)"cycleTime",

  offsetof(j2735TrafficLightStatusMessage, currentTime) | 0x0,
  (intptr_t)asn1_type_j2735_local_267,
  0,
  (intptr_t)"currentTime",

  offsetof(j2735TrafficLightStatusMessage, trafficLightStatus) | 0x0,
  (intptr_t)asn1_type_j2735TrafficLightStatusList,
  0,
  (intptr_t)"trafficLightStatus",

};

const ASN1CType asn1_type_j2735_local_268[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Acceleration,
};

const ASN1CType asn1_type_j2735_local_269[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Acceleration,
};

const ASN1CType asn1_type_j2735AccelerationSet4Way[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  4,
  sizeof(j2735AccelerationSet4Way),

  offsetof(j2735AccelerationSet4Way, Long) | 0x0,
  (intptr_t)asn1_type_j2735_local_268,
  0,
  (intptr_t)"long",

  offsetof(j2735AccelerationSet4Way, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_269,
  0,
  (intptr_t)"lat",

  offsetof(j2735AccelerationSet4Way, vert) | 0x0,
  (intptr_t)asn1_type_j2735VerticalAcceleration,
  0,
  (intptr_t)"vert",

  offsetof(j2735AccelerationSet4Way, yaw) | 0x0,
  (intptr_t)asn1_type_j2735YawRate,
  0,
  (intptr_t)"yaw",

};

const ASN1CType asn1_type_j2735_local_270[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735YawRateConfidence,
};

const ASN1CType asn1_type_j2735_local_271[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735AccelerationConfidence,
};

const ASN1CType asn1_type_j2735_local_272[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735SteeringWheelAngleConfidence,
};

const ASN1CType asn1_type_j2735AccelSteerYawRateConfidence[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  3,
  sizeof(j2735AccelSteerYawRateConfidence),

  offsetof(j2735AccelSteerYawRateConfidence, yawRate) | 0x0,
  (intptr_t)asn1_type_j2735_local_270,
  0,
  (intptr_t)"yawRate",

  offsetof(j2735AccelSteerYawRateConfidence, acceleration) | 0x0,
  (intptr_t)asn1_type_j2735_local_271,
  0,
  (intptr_t)"acceleration",

  offsetof(j2735AccelSteerYawRateConfidence, steeringWheelAngle) | 0x0,
  (intptr_t)asn1_type_j2735_local_272,
  0,
  (intptr_t)"steeringWheelAngle",

};

const ASN1CType asn1_type_j2735_local_273[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735SpeedConfidence,
};

const ASN1CType asn1_type_j2735_local_274[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735ZoneLength,
};

const ASN1CType asn1_type_j2735_local_275[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735RestrictionClassID,
};

const ASN1CType asn1_type_j2735AdvisorySpeed[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735AdvisorySpeed),

  offsetof(j2735AdvisorySpeed, type) | 0x0,
  (intptr_t)asn1_type_j2735AdvisorySpeedType,
  0,
  (intptr_t)"type",

  offsetof(j2735AdvisorySpeed, speed) | 0x8000000,
  (intptr_t)asn1_type_j2735SpeedAdvice,
  offsetof(j2735AdvisorySpeed, speed_option),
  (intptr_t)"speed",

  offsetof(j2735AdvisorySpeed, confidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_273,
  offsetof(j2735AdvisorySpeed, confidence_option),
  (intptr_t)"confidence",

  offsetof(j2735AdvisorySpeed, distance) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_274,
  offsetof(j2735AdvisorySpeed, distance_option),
  (intptr_t)"distance",

  offsetof(j2735AdvisorySpeed, Class) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_275,
  offsetof(j2735AdvisorySpeed, Class_option),
  (intptr_t)"class",

  offsetof(j2735AdvisorySpeed, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735AdvisorySpeed_1,
  offsetof(j2735AdvisorySpeed, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735AdvisorySpeed_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_14),
  (intptr_t)asn1_type_j2735RegionalExtension_14,
  0,
};

const ASN1CType asn1_type_j2735AdvisorySpeedList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x10,
  sizeof(j2735AdvisorySpeed),
  (intptr_t)asn1_type_j2735AdvisorySpeed,
  0,
};

const ASN1CType asn1_type_j2735_local_276[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Offset_B12,
};

const ASN1CType asn1_type_j2735_local_277[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Offset_B10,
};

const ASN1CType asn1_type_j2735AntennaOffsetSet[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  3,
  sizeof(j2735AntennaOffsetSet),

  offsetof(j2735AntennaOffsetSet, antOffsetX) | 0x0,
  (intptr_t)asn1_type_j2735_local_276,
  0,
  (intptr_t)"antOffsetX",

  offsetof(j2735AntennaOffsetSet, antOffsetY) | 0x0,
  (intptr_t)asn1_type_j2735Offset_B09,
  0,
  (intptr_t)"antOffsetY",

  offsetof(j2735AntennaOffsetSet, antOffsetZ) | 0x0,
  (intptr_t)asn1_type_j2735_local_277,
  0,
  (intptr_t)"antOffsetZ",

};

const ASN1CType asn1_type_j2735_local_278[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ApproachID,
};

const ASN1CType asn1_type_j2735_local_279[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735LaneID,
};

const ASN1CType asn1_type_j2735ApproachOrLane[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  2,
  sizeof(j2735ApproachOrLane),
  offsetof(j2735ApproachOrLane, choice),
  offsetof(j2735ApproachOrLane, u),
  (intptr_t)asn1_type_j2735_local_278,
  (intptr_t)"approach",
  (intptr_t)asn1_type_j2735_local_279,
  (intptr_t)"lane",
};

const ASN1CType asn1_type_j2735BrakeSystemStatus[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  6,
  sizeof(j2735BrakeSystemStatus),

  offsetof(j2735BrakeSystemStatus, wheelBrakes) | 0x0,
  (intptr_t)asn1_type_j2735BrakeAppliedStatus,
  0,
  (intptr_t)"wheelBrakes",

  offsetof(j2735BrakeSystemStatus, traction) | 0x0,
  (intptr_t)asn1_type_j2735TractionControlStatus,
  0,
  (intptr_t)"traction",

  offsetof(j2735BrakeSystemStatus, abs) | 0x0,
  (intptr_t)asn1_type_j2735AntiLockBrakeStatus,
  0,
  (intptr_t)"abs",

  offsetof(j2735BrakeSystemStatus, scs) | 0x0,
  (intptr_t)asn1_type_j2735StabilityControlStatus,
  0,
  (intptr_t)"scs",

  offsetof(j2735BrakeSystemStatus, brakeBoost) | 0x0,
  (intptr_t)asn1_type_j2735BrakeBoostApplied,
  0,
  (intptr_t)"brakeBoost",

  offsetof(j2735BrakeSystemStatus, auxBrakes) | 0x0,
  (intptr_t)asn1_type_j2735AuxiliaryBrakeStatus,
  0,
  (intptr_t)"auxBrakes",

};

const ASN1CType asn1_type_j2735_local_280[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_281[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735TemporaryID,
};

const ASN1CType asn1_type_j2735_local_282[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735DSecond,
};

const ASN1CType asn1_type_j2735_local_283[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735Latitude,
};

const ASN1CType asn1_type_j2735_local_284[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Longitude,
};

const ASN1CType asn1_type_j2735_local_285[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735Elevation,
};

const ASN1CType asn1_type_j2735_local_286[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735PositionalAccuracy,
};

const ASN1CType asn1_type_j2735_local_287[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735TransmissionState,
};

const ASN1CType asn1_type_j2735_local_288[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735Speed,
};

const ASN1CType asn1_type_j2735_local_289[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  (intptr_t)asn1_type_j2735Heading,
};

const ASN1CType asn1_type_j2735_local_290[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000a,
  (intptr_t)asn1_type_j2735SteeringWheelAngle,
};

const ASN1CType asn1_type_j2735_local_291[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000b,
  (intptr_t)asn1_type_j2735AccelerationSet4Way,
};

const ASN1CType asn1_type_j2735_local_292[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000c,
  (intptr_t)asn1_type_j2735BrakeSystemStatus,
};

const ASN1CType asn1_type_j2735BSMcoreData[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  14,
  sizeof(j2735BSMcoreData),

  offsetof(j2735BSMcoreData, msgCnt) | 0x0,
  (intptr_t)asn1_type_j2735_local_280,
  0,
  (intptr_t)"msgCnt",

  offsetof(j2735BSMcoreData, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_281,
  0,
  (intptr_t)"id",

  offsetof(j2735BSMcoreData, secMark) | 0x0,
  (intptr_t)asn1_type_j2735_local_282,
  0,
  (intptr_t)"secMark",

  offsetof(j2735BSMcoreData, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_283,
  0,
  (intptr_t)"lat",

  offsetof(j2735BSMcoreData, Long) | 0x0,
  (intptr_t)asn1_type_j2735_local_284,
  0,
  (intptr_t)"long",

  offsetof(j2735BSMcoreData, elev) | 0x0,
  (intptr_t)asn1_type_j2735_local_285,
  0,
  (intptr_t)"elev",

  offsetof(j2735BSMcoreData, accuracy) | 0x0,
  (intptr_t)asn1_type_j2735_local_286,
  0,
  (intptr_t)"accuracy",

  offsetof(j2735BSMcoreData, transmission) | 0x0,
  (intptr_t)asn1_type_j2735_local_287,
  0,
  (intptr_t)"transmission",

  offsetof(j2735BSMcoreData, speed) | 0x0,
  (intptr_t)asn1_type_j2735_local_288,
  0,
  (intptr_t)"speed",

  offsetof(j2735BSMcoreData, heading) | 0x0,
  (intptr_t)asn1_type_j2735_local_289,
  0,
  (intptr_t)"heading",

  offsetof(j2735BSMcoreData, angle) | 0x0,
  (intptr_t)asn1_type_j2735_local_290,
  0,
  (intptr_t)"angle",

  offsetof(j2735BSMcoreData, accelSet) | 0x0,
  (intptr_t)asn1_type_j2735_local_291,
  0,
  (intptr_t)"accelSet",

  offsetof(j2735BSMcoreData, brakes) | 0x0,
  (intptr_t)asn1_type_j2735_local_292,
  0,
  (intptr_t)"brakes",

  offsetof(j2735BSMcoreData, size) | 0x0,
  (intptr_t)asn1_type_j2735VehicleSize,
  0,
  (intptr_t)"size",

};

const ASN1CType asn1_type_j2735_local_293[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735BumperHeight,
};

const ASN1CType asn1_type_j2735_local_294[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735BumperHeight,
};

const ASN1CType asn1_type_j2735BumperHeights[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735BumperHeights),

  offsetof(j2735BumperHeights, front) | 0x0,
  (intptr_t)asn1_type_j2735_local_293,
  0,
  (intptr_t)"front",

  offsetof(j2735BumperHeights, rear) | 0x0,
  (intptr_t)asn1_type_j2735_local_294,
  0,
  (intptr_t)"rear",

};

const ASN1CType asn1_type_j2735_local_295[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Position3D,
};

const ASN1CType asn1_type_j2735Circle[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  3,
  sizeof(j2735Circle),

  offsetof(j2735Circle, center) | 0x0,
  (intptr_t)asn1_type_j2735_local_295,
  0,
  (intptr_t)"center",

  offsetof(j2735Circle, radius) | 0x0,
  (intptr_t)asn1_type_j2735Radius_B12,
  0,
  (intptr_t)"radius",

  offsetof(j2735Circle, units) | 0x0,
  (intptr_t)asn1_type_j2735DistanceUnits,
  0,
  (intptr_t)"units",

};

const ASN1CType asn1_type_j2735_local_296[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735LaneID,
};

const ASN1CType asn1_type_j2735_local_297[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735Angle,
};

const ASN1CType asn1_type_j2735_local_298[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Scale_B12,
};

const ASN1CType asn1_type_j2735_local_299[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735Scale_B12,
};

const ASN1CType asn1_type_j2735ComputedLane[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  7,
  sizeof(j2735ComputedLane),

  offsetof(j2735ComputedLane, referenceLaneId) | 0x0,
  (intptr_t)asn1_type_j2735_local_296,
  0,
  (intptr_t)"referenceLaneId",

  offsetof(j2735ComputedLane, offsetXaxis) | 0x0,
  (intptr_t)asn1_type_j2735ComputedLane_1,
  0,
  (intptr_t)"offsetXaxis",

  offsetof(j2735ComputedLane, offsetYaxis) | 0x0,
  (intptr_t)asn1_type_j2735ComputedLane_2,
  0,
  (intptr_t)"offsetYaxis",

  offsetof(j2735ComputedLane, rotateXY) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_297,
  offsetof(j2735ComputedLane, rotateXY_option),
  (intptr_t)"rotateXY",

  offsetof(j2735ComputedLane, scaleXaxis) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_298,
  offsetof(j2735ComputedLane, scaleXaxis_option),
  (intptr_t)"scaleXaxis",

  offsetof(j2735ComputedLane, scaleYaxis) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_299,
  offsetof(j2735ComputedLane, scaleYaxis_option),
  (intptr_t)"scaleYaxis",

  offsetof(j2735ComputedLane, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735ComputedLane_3,
  offsetof(j2735ComputedLane, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735ComputedLane_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  2,
  sizeof(j2735ComputedLane_1),
  offsetof(j2735ComputedLane_1, choice),
  offsetof(j2735ComputedLane_1, u),
  (intptr_t)asn1_type_j2735DrivenLineOffsetSm,
  (intptr_t)"small",
  (intptr_t)asn1_type_j2735DrivenLineOffsetLg,
  (intptr_t)"large",
};

const ASN1CType asn1_type_j2735ComputedLane_2[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  2,
  sizeof(j2735ComputedLane_2),
  offsetof(j2735ComputedLane_2, choice),
  offsetof(j2735ComputedLane_2, u),
  (intptr_t)asn1_type_j2735DrivenLineOffsetSm,
  (intptr_t)"small",
  (intptr_t)asn1_type_j2735DrivenLineOffsetLg,
  (intptr_t)"large",
};

const ASN1CType asn1_type_j2735ComputedLane_3[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_15),
  (intptr_t)asn1_type_j2735RegionalExtension_15,
  0,
};

const ASN1CType asn1_type_j2735_local_300[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735SpeedandHeadingandThrottleConfidence,
};

const ASN1CType asn1_type_j2735_local_301[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735TimeConfidence,
};

const ASN1CType asn1_type_j2735_local_302[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735PositionConfidenceSet,
};

const ASN1CType asn1_type_j2735_local_303[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735SteeringWheelAngleConfidence,
};

const ASN1CType asn1_type_j2735_local_304[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735HeadingConfidence,
};

const ASN1CType asn1_type_j2735_local_305[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735ThrottleConfidence,
};

const ASN1CType asn1_type_j2735ConfidenceSet[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100004,
  7,
  sizeof(j2735ConfidenceSet),

  offsetof(j2735ConfidenceSet, accelConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735AccelSteerYawRateConfidence,
  offsetof(j2735ConfidenceSet, accelConfidence_option),
  (intptr_t)"accelConfidence",

  offsetof(j2735ConfidenceSet, speedConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_300,
  offsetof(j2735ConfidenceSet, speedConfidence_option),
  (intptr_t)"speedConfidence",

  offsetof(j2735ConfidenceSet, timeConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_301,
  offsetof(j2735ConfidenceSet, timeConfidence_option),
  (intptr_t)"timeConfidence",

  offsetof(j2735ConfidenceSet, posConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_302,
  offsetof(j2735ConfidenceSet, posConfidence_option),
  (intptr_t)"posConfidence",

  offsetof(j2735ConfidenceSet, steerConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_303,
  offsetof(j2735ConfidenceSet, steerConfidence_option),
  (intptr_t)"steerConfidence",

  offsetof(j2735ConfidenceSet, headingConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_304,
  offsetof(j2735ConfidenceSet, headingConfidence_option),
  (intptr_t)"headingConfidence",

  offsetof(j2735ConfidenceSet, throttleConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_305,
  offsetof(j2735ConfidenceSet, throttleConfidence_option),
  (intptr_t)"throttleConfidence",

};

const ASN1CType asn1_type_j2735_local_306[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735LaneID,
};

const ASN1CType asn1_type_j2735_local_307[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735AllowedManeuvers,
};

const ASN1CType asn1_type_j2735ConnectingLane[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735ConnectingLane),

  offsetof(j2735ConnectingLane, lane) | 0x0,
  (intptr_t)asn1_type_j2735_local_306,
  0,
  (intptr_t)"lane",

  offsetof(j2735ConnectingLane, maneuver) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_307,
  offsetof(j2735ConnectingLane, maneuver_option),
  (intptr_t)"maneuver",

};

const ASN1CType asn1_type_j2735_local_308[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735IntersectionReferenceID,
};

const ASN1CType asn1_type_j2735_local_309[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735SignalGroupID,
};

const ASN1CType asn1_type_j2735_local_310[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735RestrictionClassID,
};

const ASN1CType asn1_type_j2735_local_311[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735LaneConnectionID,
};

const ASN1CType asn1_type_j2735Connection[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  5,
  sizeof(j2735Connection),

  offsetof(j2735Connection, connectingLane) | 0x0,
  (intptr_t)asn1_type_j2735ConnectingLane,
  0,
  (intptr_t)"connectingLane",

  offsetof(j2735Connection, remoteIntersection) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_308,
  offsetof(j2735Connection, remoteIntersection_option),
  (intptr_t)"remoteIntersection",

  offsetof(j2735Connection, signalGroup) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_309,
  offsetof(j2735Connection, signalGroup_option),
  (intptr_t)"signalGroup",

  offsetof(j2735Connection, userClass) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_310,
  offsetof(j2735Connection, userClass_option),
  (intptr_t)"userClass",

  offsetof(j2735Connection, connectionID) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_311,
  offsetof(j2735Connection, connectionID_option),
  (intptr_t)"connectionID",

};

const ASN1CType asn1_type_j2735_local_312[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735LaneConnectionID,
};

const ASN1CType asn1_type_j2735_local_313[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735ZoneLength,
};

const ASN1CType asn1_type_j2735_local_314[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735ZoneLength,
};

const ASN1CType asn1_type_j2735ConnectionManeuverAssist[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735ConnectionManeuverAssist),

  offsetof(j2735ConnectionManeuverAssist, connectionID) | 0x0,
  (intptr_t)asn1_type_j2735_local_312,
  0,
  (intptr_t)"connectionID",

  offsetof(j2735ConnectionManeuverAssist, queueLength) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_313,
  offsetof(j2735ConnectionManeuverAssist, queueLength_option),
  (intptr_t)"queueLength",

  offsetof(j2735ConnectionManeuverAssist, availableStorageLength) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_314,
  offsetof(j2735ConnectionManeuverAssist, availableStorageLength_option),
  (intptr_t)"availableStorageLength",

  offsetof(j2735ConnectionManeuverAssist, waitOnStop) | 0x8000000,
  (intptr_t)asn1_type_j2735WaitOnStopline,
  offsetof(j2735ConnectionManeuverAssist, waitOnStop_option),
  (intptr_t)"waitOnStop",

  offsetof(j2735ConnectionManeuverAssist, pedBicycleDetect) | 0x8000000,
  (intptr_t)asn1_type_j2735PedestrianBicycleDetect,
  offsetof(j2735ConnectionManeuverAssist, pedBicycleDetect_option),
  (intptr_t)"pedBicycleDetect",

  offsetof(j2735ConnectionManeuverAssist, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735ConnectionManeuverAssist_1,
  offsetof(j2735ConnectionManeuverAssist, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735ConnectionManeuverAssist_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_16),
  (intptr_t)asn1_type_j2735RegionalExtension_16,
  0,
};

const ASN1CType asn1_type_j2735ConnectsToList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0x1,
  0x10,
  sizeof(j2735Connection),
  (intptr_t)asn1_type_j2735Connection,
  0,
};

const ASN1CType asn1_type_j2735_local_315[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  ASN1_CSTR_IA5String,
  0x1,
  0xff,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735_local_316[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  ASN1_CSTR_IA5String,
  0x1,
  0xff,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735_local_317[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  ASN1_CSTR_IA5String,
  0x1,
  0xff,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735_local_318[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  ASN1_CSTR_IA5String,
  0x1,
  0xff,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735DataParameters[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100006,
  4,
  sizeof(j2735DataParameters),

  offsetof(j2735DataParameters, processMethod) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_315,
  offsetof(j2735DataParameters, processMethod_option),
  (intptr_t)"processMethod",

  offsetof(j2735DataParameters, processAgency) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_316,
  offsetof(j2735DataParameters, processAgency_option),
  (intptr_t)"processAgency",

  offsetof(j2735DataParameters, lastCheckedDate) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_317,
  offsetof(j2735DataParameters, lastCheckedDate_option),
  (intptr_t)"lastCheckedDate",

  offsetof(j2735DataParameters, geoidUsed) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_318,
  offsetof(j2735DataParameters, geoidUsed_option),
  (intptr_t)"geoidUsed",

};

const ASN1CType asn1_type_j2735_local_319[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DYear,
};

const ASN1CType asn1_type_j2735_local_320[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735DSecond,
};

const ASN1CType asn1_type_j2735DDateTime[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  7,
  sizeof(j2735DDateTime),

  offsetof(j2735DDateTime, year) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_319,
  offsetof(j2735DDateTime, year_option),
  (intptr_t)"year",

  offsetof(j2735DDateTime, month) | 0x8000000,
  (intptr_t)asn1_type_j2735DMonth,
  offsetof(j2735DDateTime, month_option),
  (intptr_t)"month",

  offsetof(j2735DDateTime, day) | 0x8000000,
  (intptr_t)asn1_type_j2735DDay,
  offsetof(j2735DDateTime, day_option),
  (intptr_t)"day",

  offsetof(j2735DDateTime, hour) | 0x8000000,
  (intptr_t)asn1_type_j2735DHour,
  offsetof(j2735DDateTime, hour_option),
  (intptr_t)"hour",

  offsetof(j2735DDateTime, minute) | 0x8000000,
  (intptr_t)asn1_type_j2735DMinute,
  offsetof(j2735DDateTime, minute_option),
  (intptr_t)"minute",

  offsetof(j2735DDateTime, second) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_320,
  offsetof(j2735DDateTime, second_option),
  (intptr_t)"second",

  offsetof(j2735DDateTime, offset) | 0x8000000,
  (intptr_t)asn1_type_j2735DOffset,
  offsetof(j2735DDateTime, offset_option),
  (intptr_t)"offset",

};

const ASN1CType asn1_type_j2735_local_321[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITIScodes_1,
};

const ASN1CType asn1_type_j2735_local_322[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735GenericLocations,
};

const ASN1CType asn1_type_j2735DisabledVehicle[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100006,
  2,
  sizeof(j2735DisabledVehicle),

  offsetof(j2735DisabledVehicle, statusDetails) | 0x0,
  (intptr_t)asn1_type_j2735_local_321,
  0,
  (intptr_t)"statusDetails",

  offsetof(j2735DisabledVehicle, locationDetails) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_322,
  offsetof(j2735DisabledVehicle, locationDetails_option),
  (intptr_t)"locationDetails",

};

const ASN1CType asn1_type_j2735_local_323[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735SSPindex,
};

const ASN1CType asn1_type_j2735_local_324[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735LightbarInUse,
};

const ASN1CType asn1_type_j2735_local_325[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735ResponseType,
};

const ASN1CType asn1_type_j2735EmergencyDetails[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735EmergencyDetails),

  offsetof(j2735EmergencyDetails, notUsed) | 0x0,
  (intptr_t)asn1_type_j2735_local_323,
  0,
  (intptr_t)"notUsed",

  offsetof(j2735EmergencyDetails, sirenUse) | 0x0,
  (intptr_t)asn1_type_j2735SirenInUse,
  0,
  (intptr_t)"sirenUse",

  offsetof(j2735EmergencyDetails, lightsUse) | 0x0,
  (intptr_t)asn1_type_j2735_local_324,
  0,
  (intptr_t)"lightsUse",

  offsetof(j2735EmergencyDetails, multi) | 0x0,
  (intptr_t)asn1_type_j2735MultiVehicleResponse,
  0,
  (intptr_t)"multi",

  offsetof(j2735EmergencyDetails, events) | 0x8000000,
  (intptr_t)asn1_type_j2735PrivilegedEvents,
  offsetof(j2735EmergencyDetails, events_option),
  (intptr_t)"events",

  offsetof(j2735EmergencyDetails, responseType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_325,
  offsetof(j2735EmergencyDetails, responseType_option),
  (intptr_t)"responseType",

};

const ASN1CType asn1_type_j2735EnabledLaneList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0x10,
  sizeof(j2735LaneID),
  (intptr_t)asn1_type_j2735LaneID,
  0,
};

const ASN1CType asn1_type_j2735_local_326[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735_local_327[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Priority,
};

const ASN1CType asn1_type_j2735_local_328[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735HeadingSlice,
};

const ASN1CType asn1_type_j2735_local_329[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Extent,
};

const ASN1CType asn1_type_j2735EventDescription[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  6,
  sizeof(j2735EventDescription),

  offsetof(j2735EventDescription, typeEvent) | 0x0,
  (intptr_t)asn1_type_j2735_local_326,
  0,
  (intptr_t)"typeEvent",

  offsetof(j2735EventDescription, description) | 0x8000000,
  (intptr_t)asn1_type_j2735EventDescription_1,
  offsetof(j2735EventDescription, description_option),
  (intptr_t)"description",

  offsetof(j2735EventDescription, priority) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_327,
  offsetof(j2735EventDescription, priority_option),
  (intptr_t)"priority",

  offsetof(j2735EventDescription, heading) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_328,
  offsetof(j2735EventDescription, heading_option),
  (intptr_t)"heading",

  offsetof(j2735EventDescription, extent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_329,
  offsetof(j2735EventDescription, extent_option),
  (intptr_t)"extent",

  offsetof(j2735EventDescription, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735EventDescription_2,
  offsetof(j2735EventDescription, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735EventDescription_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x8,
  sizeof(j2735ITIScodes),
  (intptr_t)asn1_type_j2735ITIScodes,
  0,
};

const ASN1CType asn1_type_j2735EventDescription_2[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_17),
  (intptr_t)asn1_type_j2735RegionalExtension_17,
  0,
};

const ASN1CType asn1_type_j2735_local_330[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DDateTime,
};

const ASN1CType asn1_type_j2735_local_331[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Longitude,
};

const ASN1CType asn1_type_j2735_local_332[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Latitude,
};

const ASN1CType asn1_type_j2735_local_333[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735Elevation,
};

const ASN1CType asn1_type_j2735_local_334[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Heading,
};

const ASN1CType asn1_type_j2735_local_335[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735TransmissionAndSpeed,
};

const ASN1CType asn1_type_j2735_local_336[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735PositionalAccuracy,
};

const ASN1CType asn1_type_j2735_local_337[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735TimeConfidence,
};

const ASN1CType asn1_type_j2735_local_338[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735PositionConfidenceSet,
};

const ASN1CType asn1_type_j2735_local_339[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  (intptr_t)asn1_type_j2735SpeedandHeadingandThrottleConfidence,
};

const ASN1CType asn1_type_j2735FullPositionVector[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  10,
  sizeof(j2735FullPositionVector),

  offsetof(j2735FullPositionVector, utcTime) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_330,
  offsetof(j2735FullPositionVector, utcTime_option),
  (intptr_t)"utcTime",

  offsetof(j2735FullPositionVector, Long) | 0x0,
  (intptr_t)asn1_type_j2735_local_331,
  0,
  (intptr_t)"long",

  offsetof(j2735FullPositionVector, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_332,
  0,
  (intptr_t)"lat",

  offsetof(j2735FullPositionVector, elevation) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_333,
  offsetof(j2735FullPositionVector, elevation_option),
  (intptr_t)"elevation",

  offsetof(j2735FullPositionVector, heading) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_334,
  offsetof(j2735FullPositionVector, heading_option),
  (intptr_t)"heading",

  offsetof(j2735FullPositionVector, speed) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_335,
  offsetof(j2735FullPositionVector, speed_option),
  (intptr_t)"speed",

  offsetof(j2735FullPositionVector, posAccuracy) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_336,
  offsetof(j2735FullPositionVector, posAccuracy_option),
  (intptr_t)"posAccuracy",

  offsetof(j2735FullPositionVector, timeConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_337,
  offsetof(j2735FullPositionVector, timeConfidence_option),
  (intptr_t)"timeConfidence",

  offsetof(j2735FullPositionVector, posConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_338,
  offsetof(j2735FullPositionVector, posConfidence_option),
  (intptr_t)"posConfidence",

  offsetof(j2735FullPositionVector, speedConfidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_339,
  offsetof(j2735FullPositionVector, speedConfidence_option),
  (intptr_t)"speedConfidence",

};

const ASN1CType asn1_type_j2735_local_340[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735LaneID,
};

const ASN1CType asn1_type_j2735_local_341[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735_local_342[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735ApproachID,
};

const ASN1CType asn1_type_j2735_local_343[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735ApproachID,
};

const ASN1CType asn1_type_j2735_local_344[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735AllowedManeuvers,
};

const ASN1CType asn1_type_j2735_local_345[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100006,
  (intptr_t)asn1_type_j2735NodeListXY,
};

const ASN1CType asn1_type_j2735GenericLane[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  10,
  sizeof(j2735GenericLane),

  offsetof(j2735GenericLane, laneID) | 0x0,
  (intptr_t)asn1_type_j2735_local_340,
  0,
  (intptr_t)"laneID",

  offsetof(j2735GenericLane, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_341,
  offsetof(j2735GenericLane, name_option),
  (intptr_t)"name",

  offsetof(j2735GenericLane, ingressApproach) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_342,
  offsetof(j2735GenericLane, ingressApproach_option),
  (intptr_t)"ingressApproach",

  offsetof(j2735GenericLane, egressApproach) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_343,
  offsetof(j2735GenericLane, egressApproach_option),
  (intptr_t)"egressApproach",

  offsetof(j2735GenericLane, laneAttributes) | 0x0,
  (intptr_t)asn1_type_j2735LaneAttributes,
  0,
  (intptr_t)"laneAttributes",

  offsetof(j2735GenericLane, maneuvers) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_344,
  offsetof(j2735GenericLane, maneuvers_option),
  (intptr_t)"maneuvers",

  offsetof(j2735GenericLane, nodeList) | 0x0,
  (intptr_t)asn1_type_j2735_local_345,
  0,
  (intptr_t)"nodeList",

  offsetof(j2735GenericLane, connectsTo) | 0x8000000,
  (intptr_t)asn1_type_j2735ConnectsToList,
  offsetof(j2735GenericLane, connectsTo_option),
  (intptr_t)"connectsTo",

  offsetof(j2735GenericLane, overlays) | 0x8000000,
  (intptr_t)asn1_type_j2735OverlayLaneList,
  offsetof(j2735GenericLane, overlays_option),
  (intptr_t)"overlays",

  offsetof(j2735GenericLane, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735GenericLane_1,
  offsetof(j2735GenericLane, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735GenericLane_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100009,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_18),
  (intptr_t)asn1_type_j2735RegionalExtension_18,
  0,
};

const ASN1CType asn1_type_j2735_local_346[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735_local_347[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735RoadSegmentReferenceID,
};

const ASN1CType asn1_type_j2735_local_348[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Position3D,
};

const ASN1CType asn1_type_j2735_local_349[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735LaneWidth,
};

const ASN1CType asn1_type_j2735_local_350[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735DirectionOfUse,
};

const ASN1CType asn1_type_j2735_local_351[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
};

const ASN1CType asn1_type_j2735_local_352[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735HeadingSlice,
};

const ASN1CType asn1_type_j2735GeographicalPath[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  9,
  sizeof(j2735GeographicalPath),

  offsetof(j2735GeographicalPath, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_346,
  offsetof(j2735GeographicalPath, name_option),
  (intptr_t)"name",

  offsetof(j2735GeographicalPath, id) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_347,
  offsetof(j2735GeographicalPath, id_option),
  (intptr_t)"id",

  offsetof(j2735GeographicalPath, anchor) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_348,
  offsetof(j2735GeographicalPath, anchor_option),
  (intptr_t)"anchor",

  offsetof(j2735GeographicalPath, laneWidth) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_349,
  offsetof(j2735GeographicalPath, laneWidth_option),
  (intptr_t)"laneWidth",

  offsetof(j2735GeographicalPath, directionality) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_350,
  offsetof(j2735GeographicalPath, directionality_option),
  (intptr_t)"directionality",

  offsetof(j2735GeographicalPath, closedPath) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_351,
  offsetof(j2735GeographicalPath, closedPath_option),
  (intptr_t)"closedPath",

  offsetof(j2735GeographicalPath, direction) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_352,
  offsetof(j2735GeographicalPath, direction_option),
  (intptr_t)"direction",

  offsetof(j2735GeographicalPath, description) | 0x8000000,
  (intptr_t)asn1_type_j2735GeographicalPath_1,
  offsetof(j2735GeographicalPath, description_option),
  (intptr_t)"description",

  offsetof(j2735GeographicalPath, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735GeographicalPath_2,
  offsetof(j2735GeographicalPath, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735GeographicalPath_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100007,
  3,
  0,
  sizeof(j2735GeographicalPath_1),
  offsetof(j2735GeographicalPath_1, choice),
  offsetof(j2735GeographicalPath_1, u),
  (intptr_t)asn1_type_j2735OffsetSystem,
  (intptr_t)"path",
  (intptr_t)asn1_type_j2735GeometricProjection,
  (intptr_t)"geometry",
  (intptr_t)asn1_type_j2735ValidRegion,
  (intptr_t)"oldRegion",
};

const ASN1CType asn1_type_j2735GeographicalPath_2[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_19),
  (intptr_t)asn1_type_j2735RegionalExtension_19,
  0,
};

const ASN1CType asn1_type_j2735_local_353[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735HeadingSlice,
};

const ASN1CType asn1_type_j2735_local_354[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Extent,
};

const ASN1CType asn1_type_j2735_local_355[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735LaneWidth,
};

const ASN1CType asn1_type_j2735_local_356[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735Circle,
};

const ASN1CType asn1_type_j2735GeometricProjection[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  5,
  sizeof(j2735GeometricProjection),

  offsetof(j2735GeometricProjection, direction) | 0x0,
  (intptr_t)asn1_type_j2735_local_353,
  0,
  (intptr_t)"direction",

  offsetof(j2735GeometricProjection, extent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_354,
  offsetof(j2735GeometricProjection, extent_option),
  (intptr_t)"extent",

  offsetof(j2735GeometricProjection, laneWidth) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_355,
  offsetof(j2735GeometricProjection, laneWidth_option),
  (intptr_t)"laneWidth",

  offsetof(j2735GeometricProjection, circle) | 0x0,
  (intptr_t)asn1_type_j2735_local_356,
  0,
  (intptr_t)"circle",

  offsetof(j2735GeometricProjection, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735GeometricProjection_1,
  offsetof(j2735GeometricProjection, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735GeometricProjection_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_20),
  (intptr_t)asn1_type_j2735RegionalExtension_20,
  0,
};

const ASN1CType asn1_type_j2735_local_357[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735LaneID,
};

const ASN1CType asn1_type_j2735_local_358[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735ApproachID,
};

const ASN1CType asn1_type_j2735_local_359[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735LaneConnectionID,
};

const ASN1CType asn1_type_j2735IntersectionAccessPoint[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x0,
  3,
  0,
  sizeof(j2735IntersectionAccessPoint),
  offsetof(j2735IntersectionAccessPoint, choice),
  offsetof(j2735IntersectionAccessPoint, u),
  (intptr_t)asn1_type_j2735_local_357,
  (intptr_t)"lane",
  (intptr_t)asn1_type_j2735_local_358,
  (intptr_t)"approach",
  (intptr_t)asn1_type_j2735_local_359,
  (intptr_t)"connection",
};

const ASN1CType asn1_type_j2735_local_360[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735_local_361[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735IntersectionReferenceID,
};

const ASN1CType asn1_type_j2735_local_362[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_363[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735Position3D,
};

const ASN1CType asn1_type_j2735_local_364[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735LaneWidth,
};

const ASN1CType asn1_type_j2735IntersectionGeometry[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  9,
  sizeof(j2735IntersectionGeometry),

  offsetof(j2735IntersectionGeometry, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_360,
  offsetof(j2735IntersectionGeometry, name_option),
  (intptr_t)"name",

  offsetof(j2735IntersectionGeometry, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_361,
  0,
  (intptr_t)"id",

  offsetof(j2735IntersectionGeometry, revision) | 0x0,
  (intptr_t)asn1_type_j2735_local_362,
  0,
  (intptr_t)"revision",

  offsetof(j2735IntersectionGeometry, refPoint) | 0x0,
  (intptr_t)asn1_type_j2735_local_363,
  0,
  (intptr_t)"refPoint",

  offsetof(j2735IntersectionGeometry, laneWidth) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_364,
  offsetof(j2735IntersectionGeometry, laneWidth_option),
  (intptr_t)"laneWidth",

  offsetof(j2735IntersectionGeometry, speedLimits) | 0x8000000,
  (intptr_t)asn1_type_j2735SpeedLimitList,
  offsetof(j2735IntersectionGeometry, speedLimits_option),
  (intptr_t)"speedLimits",

  offsetof(j2735IntersectionGeometry, laneSet) | 0x0,
  (intptr_t)asn1_type_j2735LaneList,
  0,
  (intptr_t)"laneSet",

  offsetof(j2735IntersectionGeometry, preemptPriorityData) | 0x8000000,
  (intptr_t)asn1_type_j2735PreemptPriorityList,
  offsetof(j2735IntersectionGeometry, preemptPriorityData_option),
  (intptr_t)"preemptPriorityData",

  offsetof(j2735IntersectionGeometry, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735IntersectionGeometry_1,
  offsetof(j2735IntersectionGeometry, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735IntersectionGeometry_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_21),
  (intptr_t)asn1_type_j2735RegionalExtension_21,
  0,
};

const ASN1CType asn1_type_j2735IntersectionGeometryList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x1,
  0x20,
  sizeof(j2735IntersectionGeometry),
  (intptr_t)asn1_type_j2735IntersectionGeometry,
  0,
};

const ASN1CType asn1_type_j2735_local_365[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735IntersectionID,
};

const ASN1CType asn1_type_j2735IntersectionReferenceID[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735IntersectionReferenceID),

  offsetof(j2735IntersectionReferenceID, region) | 0x8000000,
  (intptr_t)asn1_type_j2735RoadRegulatorID,
  offsetof(j2735IntersectionReferenceID, region_option),
  (intptr_t)"region",

  offsetof(j2735IntersectionReferenceID, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_365,
  0,
  (intptr_t)"id",

};

const ASN1CType asn1_type_j2735_local_366[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735_local_367[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735IntersectionReferenceID,
};

const ASN1CType asn1_type_j2735_local_368[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_369[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_370[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735DSecond,
};

const ASN1CType asn1_type_j2735_local_371[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735ManeuverAssistList,
};

const ASN1CType asn1_type_j2735IntersectionState[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  10,
  sizeof(j2735IntersectionState),

  offsetof(j2735IntersectionState, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_366,
  offsetof(j2735IntersectionState, name_option),
  (intptr_t)"name",

  offsetof(j2735IntersectionState, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_367,
  0,
  (intptr_t)"id",

  offsetof(j2735IntersectionState, revision) | 0x0,
  (intptr_t)asn1_type_j2735_local_368,
  0,
  (intptr_t)"revision",

  offsetof(j2735IntersectionState, status) | 0x0,
  (intptr_t)asn1_type_j2735IntersectionStatusObject,
  0,
  (intptr_t)"status",

  offsetof(j2735IntersectionState, moy) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_369,
  offsetof(j2735IntersectionState, moy_option),
  (intptr_t)"moy",

  offsetof(j2735IntersectionState, timeStamp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_370,
  offsetof(j2735IntersectionState, timeStamp_option),
  (intptr_t)"timeStamp",

  offsetof(j2735IntersectionState, enabledLanes) | 0x8000000,
  (intptr_t)asn1_type_j2735EnabledLaneList,
  offsetof(j2735IntersectionState, enabledLanes_option),
  (intptr_t)"enabledLanes",

  offsetof(j2735IntersectionState, states) | 0x0,
  (intptr_t)asn1_type_j2735MovementList,
  0,
  (intptr_t)"states",

  offsetof(j2735IntersectionState, maneuverAssistList) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_371,
  offsetof(j2735IntersectionState, maneuverAssistList_option),
  (intptr_t)"maneuverAssistList",

  offsetof(j2735IntersectionState, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735IntersectionState_1,
  offsetof(j2735IntersectionState, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735IntersectionState_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100009,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_22),
  (intptr_t)asn1_type_j2735RegionalExtension_22,
  0,
};

const ASN1CType asn1_type_j2735IntersectionStateList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x20,
  sizeof(j2735IntersectionState),
  (intptr_t)asn1_type_j2735IntersectionState,
  0,
};

const ASN1CType asn1_type_j2735ExitService[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x1,
  0x10,
  sizeof(j2735ExitService_2),
  (intptr_t)asn1_type_j2735ExitService_2,
  0,
};

const ASN1CType asn1_type_j2735_local_372[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735ExitService_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735ExitService_1),
  offsetof(j2735ExitService_1, choice),
  offsetof(j2735ExitService_1, u),
  (intptr_t)asn1_type_j2735_local_372,
  (intptr_t)"itis",
  (intptr_t)asn1_type_j2735ITIStextPhrase,
  (intptr_t)"text",
};

const ASN1CType asn1_type_j2735ExitService_2[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  1,
  sizeof(j2735ExitService_2),

  offsetof(j2735ExitService_2, item) | 0x0,
  (intptr_t)asn1_type_j2735ExitService_1,
  0,
  (intptr_t)"item",

};

const ASN1CType asn1_type_j2735GenericSignage[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x10,
  sizeof(j2735GenericSignage_2),
  (intptr_t)asn1_type_j2735GenericSignage_2,
  0,
};

const ASN1CType asn1_type_j2735_local_373[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735GenericSignage_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735GenericSignage_1),
  offsetof(j2735GenericSignage_1, choice),
  offsetof(j2735GenericSignage_1, u),
  (intptr_t)asn1_type_j2735_local_373,
  (intptr_t)"itis",
  (intptr_t)asn1_type_j2735ITIStextPhrase,
  (intptr_t)"text",
};

const ASN1CType asn1_type_j2735GenericSignage_2[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  1,
  sizeof(j2735GenericSignage_2),

  offsetof(j2735GenericSignage_2, item) | 0x0,
  (intptr_t)asn1_type_j2735GenericSignage_1,
  0,
  (intptr_t)"item",

};

const ASN1CType asn1_type_j2735SpeedLimit[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x1,
  0x10,
  sizeof(j2735SpeedLimit_2),
  (intptr_t)asn1_type_j2735SpeedLimit_2,
  0,
};

const ASN1CType asn1_type_j2735_local_374[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735SpeedLimit_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735SpeedLimit_1),
  offsetof(j2735SpeedLimit_1, choice),
  offsetof(j2735SpeedLimit_1, u),
  (intptr_t)asn1_type_j2735_local_374,
  (intptr_t)"itis",
  (intptr_t)asn1_type_j2735ITIStextPhrase,
  (intptr_t)"text",
};

const ASN1CType asn1_type_j2735SpeedLimit_2[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  1,
  sizeof(j2735SpeedLimit_2),

  offsetof(j2735SpeedLimit_2, item) | 0x0,
  (intptr_t)asn1_type_j2735SpeedLimit_1,
  0,
  (intptr_t)"item",

};

const ASN1CType asn1_type_j2735WorkZone[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x10,
  sizeof(j2735WorkZone_2),
  (intptr_t)asn1_type_j2735WorkZone_2,
  0,
};

const ASN1CType asn1_type_j2735_local_375[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735WorkZone_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735WorkZone_1),
  offsetof(j2735WorkZone_1, choice),
  offsetof(j2735WorkZone_1, u),
  (intptr_t)asn1_type_j2735_local_375,
  (intptr_t)"itis",
  (intptr_t)asn1_type_j2735ITIStextPhrase,
  (intptr_t)"text",
};

const ASN1CType asn1_type_j2735WorkZone_2[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  1,
  sizeof(j2735WorkZone_2),

  offsetof(j2735WorkZone_2, item) | 0x0,
  (intptr_t)asn1_type_j2735WorkZone_1,
  0,
  (intptr_t)"item",

};

const ASN1CType asn1_type_j2735_local_376[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735TrailerWeight,
};

const ASN1CType asn1_type_j2735J1939data[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100013,
  10,
  sizeof(j2735J1939data),

  offsetof(j2735J1939data, tires) | 0x8000000,
  (intptr_t)asn1_type_j2735TireDataList,
  offsetof(j2735J1939data, tires_option),
  (intptr_t)"tires",

  offsetof(j2735J1939data, axles) | 0x8000000,
  (intptr_t)asn1_type_j2735AxleWeightList,
  offsetof(j2735J1939data, axles_option),
  (intptr_t)"axles",

  offsetof(j2735J1939data, trailerWeight) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_376,
  offsetof(j2735J1939data, trailerWeight_option),
  (intptr_t)"trailerWeight",

  offsetof(j2735J1939data, cargoWeight) | 0x8000000,
  (intptr_t)asn1_type_j2735CargoWeight,
  offsetof(j2735J1939data, cargoWeight_option),
  (intptr_t)"cargoWeight",

  offsetof(j2735J1939data, steeringAxleTemperature) | 0x8000000,
  (intptr_t)asn1_type_j2735SteeringAxleTemperature,
  offsetof(j2735J1939data, steeringAxleTemperature_option),
  (intptr_t)"steeringAxleTemperature",

  offsetof(j2735J1939data, driveAxleLocation) | 0x8000000,
  (intptr_t)asn1_type_j2735DriveAxleLocation,
  offsetof(j2735J1939data, driveAxleLocation_option),
  (intptr_t)"driveAxleLocation",

  offsetof(j2735J1939data, driveAxleLiftAirPressure) | 0x8000000,
  (intptr_t)asn1_type_j2735DriveAxleLiftAirPressure,
  offsetof(j2735J1939data, driveAxleLiftAirPressure_option),
  (intptr_t)"driveAxleLiftAirPressure",

  offsetof(j2735J1939data, driveAxleTemperature) | 0x8000000,
  (intptr_t)asn1_type_j2735DriveAxleTemperature,
  offsetof(j2735J1939data, driveAxleTemperature_option),
  (intptr_t)"driveAxleTemperature",

  offsetof(j2735J1939data, driveAxleLubePressure) | 0x8000000,
  (intptr_t)asn1_type_j2735DriveAxleLubePressure,
  offsetof(j2735J1939data, driveAxleLubePressure_option),
  (intptr_t)"driveAxleLubePressure",

  offsetof(j2735J1939data, steeringAxleLubePressure) | 0x8000000,
  (intptr_t)asn1_type_j2735SteeringAxleLubePressure,
  offsetof(j2735J1939data, steeringAxleLubePressure_option),
  (intptr_t)"steeringAxleLubePressure",

};

const ASN1CType asn1_type_j2735TireDataList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x1,
  0x10,
  sizeof(j2735TireData),
  (intptr_t)asn1_type_j2735TireData,
  0,
};

const ASN1CType asn1_type_j2735TireData[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  7,
  sizeof(j2735TireData),

  offsetof(j2735TireData, location) | 0x8000000,
  (intptr_t)asn1_type_j2735TireLocation,
  offsetof(j2735TireData, location_option),
  (intptr_t)"location",

  offsetof(j2735TireData, pressure) | 0x8000000,
  (intptr_t)asn1_type_j2735TirePressure,
  offsetof(j2735TireData, pressure_option),
  (intptr_t)"pressure",

  offsetof(j2735TireData, temp) | 0x8000000,
  (intptr_t)asn1_type_j2735TireTemp,
  offsetof(j2735TireData, temp_option),
  (intptr_t)"temp",

  offsetof(j2735TireData, wheelSensorStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735WheelSensorStatus,
  offsetof(j2735TireData, wheelSensorStatus_option),
  (intptr_t)"wheelSensorStatus",

  offsetof(j2735TireData, wheelEndElectFault) | 0x8000000,
  (intptr_t)asn1_type_j2735WheelEndElectFault,
  offsetof(j2735TireData, wheelEndElectFault_option),
  (intptr_t)"wheelEndElectFault",

  offsetof(j2735TireData, leakageRate) | 0x8000000,
  (intptr_t)asn1_type_j2735TireLeakageRate,
  offsetof(j2735TireData, leakageRate_option),
  (intptr_t)"leakageRate",

  offsetof(j2735TireData, detection) | 0x8000000,
  (intptr_t)asn1_type_j2735TirePressureThresholdDetection,
  offsetof(j2735TireData, detection_option),
  (intptr_t)"detection",

};

const ASN1CType asn1_type_j2735AxleWeightList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x10,
  sizeof(j2735AxleWeightSet),
  (intptr_t)asn1_type_j2735AxleWeightSet,
  0,
};

const ASN1CType asn1_type_j2735AxleWeightSet[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  2,
  sizeof(j2735AxleWeightSet),

  offsetof(j2735AxleWeightSet, location) | 0x8000000,
  (intptr_t)asn1_type_j2735AxleLocation,
  offsetof(j2735AxleWeightSet, location_option),
  (intptr_t)"location",

  offsetof(j2735AxleWeightSet, weight) | 0x8000000,
  (intptr_t)asn1_type_j2735AxleWeight,
  offsetof(j2735AxleWeightSet, weight_option),
  (intptr_t)"weight",

};

const ASN1CType asn1_type_j2735LaneAttributes[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  4,
  sizeof(j2735LaneAttributes),

  offsetof(j2735LaneAttributes, directionalUse) | 0x0,
  (intptr_t)asn1_type_j2735LaneDirection,
  0,
  (intptr_t)"directionalUse",

  offsetof(j2735LaneAttributes, sharedWith) | 0x0,
  (intptr_t)asn1_type_j2735LaneSharing,
  0,
  (intptr_t)"sharedWith",

  offsetof(j2735LaneAttributes, laneType) | 0x0,
  (intptr_t)asn1_type_j2735LaneTypeAttributes,
  0,
  (intptr_t)"laneType",

  offsetof(j2735LaneAttributes, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735RegionalExtension_23,
  offsetof(j2735LaneAttributes, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735_local_377[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735RoadwayCrownAngle,
};

const ASN1CType asn1_type_j2735_local_378[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735RoadwayCrownAngle,
};

const ASN1CType asn1_type_j2735_local_379[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735RoadwayCrownAngle,
};

const ASN1CType asn1_type_j2735LaneDataAttribute[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x0,
  7,
  0,
  sizeof(j2735LaneDataAttribute),
  offsetof(j2735LaneDataAttribute, choice),
  offsetof(j2735LaneDataAttribute, u),
  (intptr_t)asn1_type_j2735DeltaAngle,
  (intptr_t)"pathEndPointAngle",
  (intptr_t)asn1_type_j2735_local_377,
  (intptr_t)"laneCrownPointCenter",
  (intptr_t)asn1_type_j2735_local_378,
  (intptr_t)"laneCrownPointLeft",
  (intptr_t)asn1_type_j2735_local_379,
  (intptr_t)"laneCrownPointRight",
  (intptr_t)asn1_type_j2735MergeDivergeNodeAngle,
  (intptr_t)"laneAngle",
  (intptr_t)asn1_type_j2735SpeedLimitList,
  (intptr_t)"speedLimits",
  (intptr_t)asn1_type_j2735LaneDataAttribute_1,
  (intptr_t)"regional",
};

const ASN1CType asn1_type_j2735LaneDataAttribute_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_24),
  (intptr_t)asn1_type_j2735RegionalExtension_24,
  0,
};

const ASN1CType asn1_type_j2735LaneDataAttributeList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x1,
  0x8,
  sizeof(j2735LaneDataAttribute),
  (intptr_t)asn1_type_j2735LaneDataAttribute,
  0,
};

const ASN1CType asn1_type_j2735LaneList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0xff,
  sizeof(j2735GenericLane),
  (intptr_t)asn1_type_j2735GenericLane,
  0,
};

const ASN1CType asn1_type_j2735LaneTypeAttributes[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  8,
  0,
  sizeof(j2735LaneTypeAttributes),
  offsetof(j2735LaneTypeAttributes, choice),
  offsetof(j2735LaneTypeAttributes, u),
  (intptr_t)asn1_type_j2735LaneAttributes_Vehicle,
  (intptr_t)"vehicle",
  (intptr_t)asn1_type_j2735LaneAttributes_Crosswalk,
  (intptr_t)"crosswalk",
  (intptr_t)asn1_type_j2735LaneAttributes_Bike,
  (intptr_t)"bikeLane",
  (intptr_t)asn1_type_j2735LaneAttributes_Sidewalk,
  (intptr_t)"sidewalk",
  (intptr_t)asn1_type_j2735LaneAttributes_Barrier,
  (intptr_t)"median",
  (intptr_t)asn1_type_j2735LaneAttributes_Striping,
  (intptr_t)"striping",
  (intptr_t)asn1_type_j2735LaneAttributes_TrackedVehicle,
  (intptr_t)"trackedVehicle",
  (intptr_t)asn1_type_j2735LaneAttributes_Parking,
  (intptr_t)"parking",
};

const ASN1CType asn1_type_j2735ManeuverAssistList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10,
  0x1,
  0x10,
  sizeof(j2735ConnectionManeuverAssist),
  (intptr_t)asn1_type_j2735ConnectionManeuverAssist,
  0,
};

const ASN1CType asn1_type_j2735MovementEventList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x10,
  sizeof(j2735MovementEvent),
  (intptr_t)asn1_type_j2735MovementEvent,
  0,
};

const ASN1CType asn1_type_j2735MovementEvent[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  4,
  sizeof(j2735MovementEvent),

  offsetof(j2735MovementEvent, eventState) | 0x0,
  (intptr_t)asn1_type_j2735MovementPhaseState,
  0,
  (intptr_t)"eventState",

  offsetof(j2735MovementEvent, timing) | 0x8000000,
  (intptr_t)asn1_type_j2735TimeChangeDetails,
  offsetof(j2735MovementEvent, timing_option),
  (intptr_t)"timing",

  offsetof(j2735MovementEvent, speeds) | 0x8000000,
  (intptr_t)asn1_type_j2735AdvisorySpeedList,
  offsetof(j2735MovementEvent, speeds_option),
  (intptr_t)"speeds",

  offsetof(j2735MovementEvent, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735MovementEvent_1,
  offsetof(j2735MovementEvent, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735MovementEvent_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_25),
  (intptr_t)asn1_type_j2735RegionalExtension_25,
  0,
};

const ASN1CType asn1_type_j2735MovementList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0x1,
  0xff,
  sizeof(j2735MovementState),
  (intptr_t)asn1_type_j2735MovementState,
  0,
};

const ASN1CType asn1_type_j2735_local_380[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735_local_381[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735SignalGroupID,
};

const ASN1CType asn1_type_j2735_local_382[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735ManeuverAssistList,
};

const ASN1CType asn1_type_j2735MovementState[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  5,
  sizeof(j2735MovementState),

  offsetof(j2735MovementState, movementName) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_380,
  offsetof(j2735MovementState, movementName_option),
  (intptr_t)"movementName",

  offsetof(j2735MovementState, signalGroup) | 0x0,
  (intptr_t)asn1_type_j2735_local_381,
  0,
  (intptr_t)"signalGroup",

  offsetof(j2735MovementState, state_time_speed) | 0x0,
  (intptr_t)asn1_type_j2735MovementEventList,
  0,
  (intptr_t)"state-time-speed",

  offsetof(j2735MovementState, maneuverAssistList) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_382,
  offsetof(j2735MovementState, maneuverAssistList_option),
  (intptr_t)"maneuverAssistList",

  offsetof(j2735MovementState, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735MovementState_1,
  offsetof(j2735MovementState, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735MovementState_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_26),
  (intptr_t)asn1_type_j2735RegionalExtension_26,
  0,
};

const ASN1CType asn1_type_j2735_local_383[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735OffsetLL_B12,
};

const ASN1CType asn1_type_j2735_local_384[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735OffsetLL_B12,
};

const ASN1CType asn1_type_j2735Node_LL_24B[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735Node_LL_24B),

  offsetof(j2735Node_LL_24B, lon) | 0x0,
  (intptr_t)asn1_type_j2735_local_383,
  0,
  (intptr_t)"lon",

  offsetof(j2735Node_LL_24B, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_384,
  0,
  (intptr_t)"lat",

};

const ASN1CType asn1_type_j2735_local_385[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735OffsetLL_B14,
};

const ASN1CType asn1_type_j2735_local_386[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735OffsetLL_B14,
};

const ASN1CType asn1_type_j2735Node_LL_28B[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  2,
  sizeof(j2735Node_LL_28B),

  offsetof(j2735Node_LL_28B, lon) | 0x0,
  (intptr_t)asn1_type_j2735_local_385,
  0,
  (intptr_t)"lon",

  offsetof(j2735Node_LL_28B, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_386,
  0,
  (intptr_t)"lat",

};

const ASN1CType asn1_type_j2735_local_387[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735OffsetLL_B16,
};

const ASN1CType asn1_type_j2735_local_388[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735OffsetLL_B16,
};

const ASN1CType asn1_type_j2735Node_LL_32B[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  2,
  sizeof(j2735Node_LL_32B),

  offsetof(j2735Node_LL_32B, lon) | 0x0,
  (intptr_t)asn1_type_j2735_local_387,
  0,
  (intptr_t)"lon",

  offsetof(j2735Node_LL_32B, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_388,
  0,
  (intptr_t)"lat",

};

const ASN1CType asn1_type_j2735_local_389[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735OffsetLL_B18,
};

const ASN1CType asn1_type_j2735_local_390[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735OffsetLL_B18,
};

const ASN1CType asn1_type_j2735Node_LL_36B[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  2,
  sizeof(j2735Node_LL_36B),

  offsetof(j2735Node_LL_36B, lon) | 0x0,
  (intptr_t)asn1_type_j2735_local_389,
  0,
  (intptr_t)"lon",

  offsetof(j2735Node_LL_36B, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_390,
  0,
  (intptr_t)"lat",

};

const ASN1CType asn1_type_j2735_local_391[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735OffsetLL_B22,
};

const ASN1CType asn1_type_j2735_local_392[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735OffsetLL_B22,
};

const ASN1CType asn1_type_j2735Node_LL_44B[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  2,
  sizeof(j2735Node_LL_44B),

  offsetof(j2735Node_LL_44B, lon) | 0x0,
  (intptr_t)asn1_type_j2735_local_391,
  0,
  (intptr_t)"lon",

  offsetof(j2735Node_LL_44B, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_392,
  0,
  (intptr_t)"lat",

};

const ASN1CType asn1_type_j2735_local_393[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735OffsetLL_B24,
};

const ASN1CType asn1_type_j2735_local_394[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735OffsetLL_B24,
};

const ASN1CType asn1_type_j2735Node_LL_48B[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  2,
  sizeof(j2735Node_LL_48B),

  offsetof(j2735Node_LL_48B, lon) | 0x0,
  (intptr_t)asn1_type_j2735_local_393,
  0,
  (intptr_t)"lon",

  offsetof(j2735Node_LL_48B, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_394,
  0,
  (intptr_t)"lat",

};

const ASN1CType asn1_type_j2735_local_395[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Longitude,
};

const ASN1CType asn1_type_j2735_local_396[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Latitude,
};

const ASN1CType asn1_type_j2735Node_LLmD_64b[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  2,
  sizeof(j2735Node_LLmD_64b),

  offsetof(j2735Node_LLmD_64b, lon) | 0x0,
  (intptr_t)asn1_type_j2735_local_395,
  0,
  (intptr_t)"lon",

  offsetof(j2735Node_LLmD_64b, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_396,
  0,
  (intptr_t)"lat",

};

const ASN1CType asn1_type_j2735_local_397[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Offset_B10,
};

const ASN1CType asn1_type_j2735_local_398[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Offset_B10,
};

const ASN1CType asn1_type_j2735Node_XY_20b[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735Node_XY_20b),

  offsetof(j2735Node_XY_20b, x) | 0x0,
  (intptr_t)asn1_type_j2735_local_397,
  0,
  (intptr_t)"x",

  offsetof(j2735Node_XY_20b, y) | 0x0,
  (intptr_t)asn1_type_j2735_local_398,
  0,
  (intptr_t)"y",

};

const ASN1CType asn1_type_j2735_local_399[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Offset_B11,
};

const ASN1CType asn1_type_j2735_local_400[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Offset_B11,
};

const ASN1CType asn1_type_j2735Node_XY_22b[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  2,
  sizeof(j2735Node_XY_22b),

  offsetof(j2735Node_XY_22b, x) | 0x0,
  (intptr_t)asn1_type_j2735_local_399,
  0,
  (intptr_t)"x",

  offsetof(j2735Node_XY_22b, y) | 0x0,
  (intptr_t)asn1_type_j2735_local_400,
  0,
  (intptr_t)"y",

};

const ASN1CType asn1_type_j2735_local_401[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Offset_B12,
};

const ASN1CType asn1_type_j2735_local_402[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Offset_B12,
};

const ASN1CType asn1_type_j2735Node_XY_24b[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735Node_XY_24b),

  offsetof(j2735Node_XY_24b, x) | 0x0,
  (intptr_t)asn1_type_j2735_local_401,
  0,
  (intptr_t)"x",

  offsetof(j2735Node_XY_24b, y) | 0x0,
  (intptr_t)asn1_type_j2735_local_402,
  0,
  (intptr_t)"y",

};

const ASN1CType asn1_type_j2735_local_403[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Offset_B13,
};

const ASN1CType asn1_type_j2735_local_404[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Offset_B13,
};

const ASN1CType asn1_type_j2735Node_XY_26b[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  2,
  sizeof(j2735Node_XY_26b),

  offsetof(j2735Node_XY_26b, x) | 0x0,
  (intptr_t)asn1_type_j2735_local_403,
  0,
  (intptr_t)"x",

  offsetof(j2735Node_XY_26b, y) | 0x0,
  (intptr_t)asn1_type_j2735_local_404,
  0,
  (intptr_t)"y",

};

const ASN1CType asn1_type_j2735_local_405[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Offset_B14,
};

const ASN1CType asn1_type_j2735_local_406[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Offset_B14,
};

const ASN1CType asn1_type_j2735Node_XY_28b[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  2,
  sizeof(j2735Node_XY_28b),

  offsetof(j2735Node_XY_28b, x) | 0x0,
  (intptr_t)asn1_type_j2735_local_405,
  0,
  (intptr_t)"x",

  offsetof(j2735Node_XY_28b, y) | 0x0,
  (intptr_t)asn1_type_j2735_local_406,
  0,
  (intptr_t)"y",

};

const ASN1CType asn1_type_j2735_local_407[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Offset_B16,
};

const ASN1CType asn1_type_j2735_local_408[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Offset_B16,
};

const ASN1CType asn1_type_j2735Node_XY_32b[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  2,
  sizeof(j2735Node_XY_32b),

  offsetof(j2735Node_XY_32b, x) | 0x0,
  (intptr_t)asn1_type_j2735_local_407,
  0,
  (intptr_t)"x",

  offsetof(j2735Node_XY_32b, y) | 0x0,
  (intptr_t)asn1_type_j2735_local_408,
  0,
  (intptr_t)"y",

};

const ASN1CType asn1_type_j2735NodeAttributeLLList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x1,
  0x8,
  sizeof(j2735NodeAttributeLL),
  (intptr_t)asn1_type_j2735NodeAttributeLL,
  0,
};

const ASN1CType asn1_type_j2735_local_409[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735SegmentAttributeLLList,
};

const ASN1CType asn1_type_j2735_local_410[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735SegmentAttributeLLList,
};

const ASN1CType asn1_type_j2735_local_411[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Offset_B10,
};

const ASN1CType asn1_type_j2735_local_412[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735Offset_B10,
};

const ASN1CType asn1_type_j2735NodeAttributeSetLL[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  7,
  sizeof(j2735NodeAttributeSetLL),

  offsetof(j2735NodeAttributeSetLL, localNode) | 0x8000000,
  (intptr_t)asn1_type_j2735NodeAttributeLLList,
  offsetof(j2735NodeAttributeSetLL, localNode_option),
  (intptr_t)"localNode",

  offsetof(j2735NodeAttributeSetLL, disabled) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_409,
  offsetof(j2735NodeAttributeSetLL, disabled_option),
  (intptr_t)"disabled",

  offsetof(j2735NodeAttributeSetLL, enabled) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_410,
  offsetof(j2735NodeAttributeSetLL, enabled_option),
  (intptr_t)"enabled",

  offsetof(j2735NodeAttributeSetLL, data) | 0x8000000,
  (intptr_t)asn1_type_j2735LaneDataAttributeList,
  offsetof(j2735NodeAttributeSetLL, data_option),
  (intptr_t)"data",

  offsetof(j2735NodeAttributeSetLL, dWidth) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_411,
  offsetof(j2735NodeAttributeSetLL, dWidth_option),
  (intptr_t)"dWidth",

  offsetof(j2735NodeAttributeSetLL, dElevation) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_412,
  offsetof(j2735NodeAttributeSetLL, dElevation_option),
  (intptr_t)"dElevation",

  offsetof(j2735NodeAttributeSetLL, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735NodeAttributeSetLL_1,
  offsetof(j2735NodeAttributeSetLL, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735NodeAttributeSetLL_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_27),
  (intptr_t)asn1_type_j2735RegionalExtension_27,
  0,
};

const ASN1CType asn1_type_j2735_local_413[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735SegmentAttributeXYList,
};

const ASN1CType asn1_type_j2735_local_414[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735SegmentAttributeXYList,
};

const ASN1CType asn1_type_j2735_local_415[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Offset_B10,
};

const ASN1CType asn1_type_j2735_local_416[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735Offset_B10,
};

const ASN1CType asn1_type_j2735NodeAttributeSetXY[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  7,
  sizeof(j2735NodeAttributeSetXY),

  offsetof(j2735NodeAttributeSetXY, localNode) | 0x8000000,
  (intptr_t)asn1_type_j2735NodeAttributeXYList,
  offsetof(j2735NodeAttributeSetXY, localNode_option),
  (intptr_t)"localNode",

  offsetof(j2735NodeAttributeSetXY, disabled) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_413,
  offsetof(j2735NodeAttributeSetXY, disabled_option),
  (intptr_t)"disabled",

  offsetof(j2735NodeAttributeSetXY, enabled) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_414,
  offsetof(j2735NodeAttributeSetXY, enabled_option),
  (intptr_t)"enabled",

  offsetof(j2735NodeAttributeSetXY, data) | 0x8000000,
  (intptr_t)asn1_type_j2735LaneDataAttributeList,
  offsetof(j2735NodeAttributeSetXY, data_option),
  (intptr_t)"data",

  offsetof(j2735NodeAttributeSetXY, dWidth) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_415,
  offsetof(j2735NodeAttributeSetXY, dWidth_option),
  (intptr_t)"dWidth",

  offsetof(j2735NodeAttributeSetXY, dElevation) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_416,
  offsetof(j2735NodeAttributeSetXY, dElevation_option),
  (intptr_t)"dElevation",

  offsetof(j2735NodeAttributeSetXY, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735NodeAttributeSetXY_1,
  offsetof(j2735NodeAttributeSetXY, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735NodeAttributeSetXY_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_28),
  (intptr_t)asn1_type_j2735RegionalExtension_28,
  0,
};

const ASN1CType asn1_type_j2735NodeAttributeXYList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x1,
  0x8,
  sizeof(j2735NodeAttributeXY),
  (intptr_t)asn1_type_j2735NodeAttributeXY,
  0,
};

const ASN1CType asn1_type_j2735NodeListLL[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  1,
  0,
  sizeof(j2735NodeListLL),
  offsetof(j2735NodeListLL, choice),
  offsetof(j2735NodeListLL, u),
  (intptr_t)asn1_type_j2735NodeSetLL,
  (intptr_t)"nodes",
};

const ASN1CType asn1_type_j2735NodeListXY[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x0,
  2,
  0,
  sizeof(j2735NodeListXY),
  offsetof(j2735NodeListXY, choice),
  offsetof(j2735NodeListXY, u),
  (intptr_t)asn1_type_j2735NodeSetXY,
  (intptr_t)"nodes",
  (intptr_t)asn1_type_j2735ComputedLane,
  (intptr_t)"computed",
};

const ASN1CType asn1_type_j2735NodeLL[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  2,
  sizeof(j2735NodeLL),

  offsetof(j2735NodeLL, delta) | 0x0,
  (intptr_t)asn1_type_j2735NodeOffsetPointLL,
  0,
  (intptr_t)"delta",

  offsetof(j2735NodeLL, attributes) | 0x8000000,
  (intptr_t)asn1_type_j2735NodeAttributeSetLL,
  offsetof(j2735NodeLL, attributes_option),
  (intptr_t)"attributes",

};

const ASN1CType asn1_type_j2735NodeOffsetPointLL[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  8,
  sizeof(j2735NodeOffsetPointLL),
  offsetof(j2735NodeOffsetPointLL, choice),
  offsetof(j2735NodeOffsetPointLL, u),
  (intptr_t)asn1_type_j2735Node_LL_24B,
  (intptr_t)"node-LL1",
  (intptr_t)asn1_type_j2735Node_LL_28B,
  (intptr_t)"node-LL2",
  (intptr_t)asn1_type_j2735Node_LL_32B,
  (intptr_t)"node-LL3",
  (intptr_t)asn1_type_j2735Node_LL_36B,
  (intptr_t)"node-LL4",
  (intptr_t)asn1_type_j2735Node_LL_44B,
  (intptr_t)"node-LL5",
  (intptr_t)asn1_type_j2735Node_LL_48B,
  (intptr_t)"node-LL6",
  (intptr_t)asn1_type_j2735Node_LLmD_64b,
  (intptr_t)"node-LatLon",
  (intptr_t)asn1_type_j2735RegionalExtension_29,
  (intptr_t)"regional",
};

const ASN1CType asn1_type_j2735_local_417[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Node_XY_24b,
};

const ASN1CType asn1_type_j2735NodeOffsetPointXY[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  8,
  sizeof(j2735NodeOffsetPointXY),
  offsetof(j2735NodeOffsetPointXY, choice),
  offsetof(j2735NodeOffsetPointXY, u),
  (intptr_t)asn1_type_j2735Node_XY_20b,
  (intptr_t)"node-XY1",
  (intptr_t)asn1_type_j2735Node_XY_22b,
  (intptr_t)"node-XY2",
  (intptr_t)asn1_type_j2735_local_417,
  (intptr_t)"node-XY3",
  (intptr_t)asn1_type_j2735Node_XY_26b,
  (intptr_t)"node-XY4",
  (intptr_t)asn1_type_j2735Node_XY_28b,
  (intptr_t)"node-XY5",
  (intptr_t)asn1_type_j2735Node_XY_32b,
  (intptr_t)"node-XY6",
  (intptr_t)asn1_type_j2735Node_LLmD_64b,
  (intptr_t)"node-LatLon",
  (intptr_t)asn1_type_j2735RegionalExtension_30,
  (intptr_t)"regional",
};

const ASN1CType asn1_type_j2735NodeSetLL[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x2,
  0x3f,
  sizeof(j2735NodeLL),
  (intptr_t)asn1_type_j2735NodeLL,
  0,
};

const ASN1CType asn1_type_j2735NodeSetXY[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x2,
  0x3f,
  sizeof(j2735NodeXY),
  (intptr_t)asn1_type_j2735NodeXY,
  0,
};

const ASN1CType asn1_type_j2735NodeXY[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  2,
  sizeof(j2735NodeXY),

  offsetof(j2735NodeXY, delta) | 0x0,
  (intptr_t)asn1_type_j2735NodeOffsetPointXY,
  0,
  (intptr_t)"delta",

  offsetof(j2735NodeXY, attributes) | 0x8000000,
  (intptr_t)asn1_type_j2735NodeAttributeSetXY,
  offsetof(j2735NodeXY, attributes_option),
  (intptr_t)"attributes",

};

const ASN1CType asn1_type_j2735_local_418[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ObstacleDistance,
};

const ASN1CType asn1_type_j2735_local_419[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735ITIScodes_2,
};

const ASN1CType asn1_type_j2735_local_420[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735GenericLocations,
};

const ASN1CType asn1_type_j2735_local_421[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735DDateTime,
};

const ASN1CType asn1_type_j2735_local_422[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735VerticalAccelerationThreshold,
};

const ASN1CType asn1_type_j2735ObstacleDetection[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100005,
  6,
  sizeof(j2735ObstacleDetection),

  offsetof(j2735ObstacleDetection, obDist) | 0x0,
  (intptr_t)asn1_type_j2735_local_418,
  0,
  (intptr_t)"obDist",

  offsetof(j2735ObstacleDetection, obDirect) | 0x0,
  (intptr_t)asn1_type_j2735ObstacleDirection,
  0,
  (intptr_t)"obDirect",

  offsetof(j2735ObstacleDetection, description) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_419,
  offsetof(j2735ObstacleDetection, description_option),
  (intptr_t)"description",

  offsetof(j2735ObstacleDetection, locationDetails) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_420,
  offsetof(j2735ObstacleDetection, locationDetails_option),
  (intptr_t)"locationDetails",

  offsetof(j2735ObstacleDetection, dateTime) | 0x0,
  (intptr_t)asn1_type_j2735_local_421,
  0,
  (intptr_t)"dateTime",

  offsetof(j2735ObstacleDetection, vertEvent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_422,
  offsetof(j2735ObstacleDetection, vertEvent_option),
  (intptr_t)"vertEvent",

};

const ASN1CType asn1_type_j2735_local_423[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Zoom,
};

const ASN1CType asn1_type_j2735OffsetSystem[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735OffsetSystem),

  offsetof(j2735OffsetSystem, scale) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_423,
  offsetof(j2735OffsetSystem, scale_option),
  (intptr_t)"scale",

  offsetof(j2735OffsetSystem, offset) | 0x0,
  (intptr_t)asn1_type_j2735OffsetSystem_1,
  0,
  (intptr_t)"offset",

};

const ASN1CType asn1_type_j2735_local_424[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  (intptr_t)asn1_type_j2735NodeListXY,
};

const ASN1CType asn1_type_j2735OffsetSystem_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  2,
  sizeof(j2735OffsetSystem_1),
  offsetof(j2735OffsetSystem_1, choice),
  offsetof(j2735OffsetSystem_1, u),
  (intptr_t)asn1_type_j2735_local_424,
  (intptr_t)"xy",
  (intptr_t)asn1_type_j2735NodeListLL,
  (intptr_t)"ll",
};

const ASN1CType asn1_type_j2735OverlayLaneList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x1,
  0x5,
  sizeof(j2735LaneID),
  (intptr_t)asn1_type_j2735LaneID,
  0,
};

const ASN1CType asn1_type_j2735_local_425[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735FullPositionVector,
};

const ASN1CType asn1_type_j2735_local_426[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735GNSSstatus,
};

const ASN1CType asn1_type_j2735PathHistory[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  3,
  sizeof(j2735PathHistory),

  offsetof(j2735PathHistory, initialPosition) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_425,
  offsetof(j2735PathHistory, initialPosition_option),
  (intptr_t)"initialPosition",

  offsetof(j2735PathHistory, currGNSSstatus) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_426,
  offsetof(j2735PathHistory, currGNSSstatus_option),
  (intptr_t)"currGNSSstatus",

  offsetof(j2735PathHistory, crumbData) | 0x0,
  (intptr_t)asn1_type_j2735PathHistoryPointList,
  0,
  (intptr_t)"crumbData",

};

const ASN1CType asn1_type_j2735PathHistoryPointList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x17,
  sizeof(j2735PathHistoryPoint),
  (intptr_t)asn1_type_j2735PathHistoryPoint,
  0,
};

const ASN1CType asn1_type_j2735_local_427[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735OffsetLL_B18,
};

const ASN1CType asn1_type_j2735_local_428[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735OffsetLL_B18,
};

const ASN1CType asn1_type_j2735_local_429[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735TimeOffset,
};

const ASN1CType asn1_type_j2735_local_430[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Speed,
};

const ASN1CType asn1_type_j2735_local_431[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735PositionalAccuracy,
};

const ASN1CType asn1_type_j2735_local_432[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735CoarseHeading,
};

const ASN1CType asn1_type_j2735PathHistoryPoint[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  7,
  sizeof(j2735PathHistoryPoint),

  offsetof(j2735PathHistoryPoint, latOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_427,
  0,
  (intptr_t)"latOffset",

  offsetof(j2735PathHistoryPoint, lonOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_428,
  0,
  (intptr_t)"lonOffset",

  offsetof(j2735PathHistoryPoint, elevationOffset) | 0x0,
  (intptr_t)asn1_type_j2735VertOffset_B12,
  0,
  (intptr_t)"elevationOffset",

  offsetof(j2735PathHistoryPoint, timeOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_429,
  0,
  (intptr_t)"timeOffset",

  offsetof(j2735PathHistoryPoint, speed) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_430,
  offsetof(j2735PathHistoryPoint, speed_option),
  (intptr_t)"speed",

  offsetof(j2735PathHistoryPoint, posAccuracy) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_431,
  offsetof(j2735PathHistoryPoint, posAccuracy_option),
  (intptr_t)"posAccuracy",

  offsetof(j2735PathHistoryPoint, heading) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_432,
  offsetof(j2735PathHistoryPoint, heading_option),
  (intptr_t)"heading",

};

const ASN1CType asn1_type_j2735PathPrediction[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  2,
  sizeof(j2735PathPrediction),

  offsetof(j2735PathPrediction, radiusOfCurve) | 0x0,
  (intptr_t)asn1_type_j2735RadiusOfCurvature,
  0,
  (intptr_t)"radiusOfCurve",

  offsetof(j2735PathPrediction, confidence) | 0x0,
  (intptr_t)asn1_type_j2735Confidence,
  0,
  (intptr_t)"confidence",

};

const ASN1CType asn1_type_j2735_local_433[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Offset_B11,
};

const ASN1CType asn1_type_j2735_local_434[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Angle,
};

const ASN1CType asn1_type_j2735PivotPointDescription[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  3,
  sizeof(j2735PivotPointDescription),

  offsetof(j2735PivotPointDescription, pivotOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_433,
  0,
  (intptr_t)"pivotOffset",

  offsetof(j2735PivotPointDescription, pivotAngle) | 0x0,
  (intptr_t)asn1_type_j2735_local_434,
  0,
  (intptr_t)"pivotAngle",

  offsetof(j2735PivotPointDescription, pivots) | 0x0,
  (intptr_t)asn1_type_j2735PivotingAllowed,
  0,
  (intptr_t)"pivots",

};

const ASN1CType asn1_type_j2735_local_435[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Latitude,
};

const ASN1CType asn1_type_j2735_local_436[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Longitude,
};

const ASN1CType asn1_type_j2735_local_437[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Elevation,
};

const ASN1CType asn1_type_j2735Position3D[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  4,
  sizeof(j2735Position3D),

  offsetof(j2735Position3D, lat) | 0x0,
  (intptr_t)asn1_type_j2735_local_435,
  0,
  (intptr_t)"lat",

  offsetof(j2735Position3D, Long) | 0x0,
  (intptr_t)asn1_type_j2735_local_436,
  0,
  (intptr_t)"long",

  offsetof(j2735Position3D, elevation) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_437,
  offsetof(j2735Position3D, elevation_option),
  (intptr_t)"elevation",

  offsetof(j2735Position3D, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735Position3D_1,
  offsetof(j2735Position3D, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735Position3D_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_31),
  (intptr_t)asn1_type_j2735RegionalExtension_31,
  0,
};

const ASN1CType asn1_type_j2735PositionalAccuracy[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  3,
  sizeof(j2735PositionalAccuracy),

  offsetof(j2735PositionalAccuracy, semiMajor) | 0x0,
  (intptr_t)asn1_type_j2735SemiMajorAxisAccuracy,
  0,
  (intptr_t)"semiMajor",

  offsetof(j2735PositionalAccuracy, semiMinor) | 0x0,
  (intptr_t)asn1_type_j2735SemiMinorAxisAccuracy,
  0,
  (intptr_t)"semiMinor",

  offsetof(j2735PositionalAccuracy, orientation) | 0x0,
  (intptr_t)asn1_type_j2735SemiMajorAxisOrientation,
  0,
  (intptr_t)"orientation",

};

const ASN1CType asn1_type_j2735_local_438[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735PositionConfidence,
};

const ASN1CType asn1_type_j2735PositionConfidenceSet[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735PositionConfidenceSet),

  offsetof(j2735PositionConfidenceSet, pos) | 0x0,
  (intptr_t)asn1_type_j2735_local_438,
  0,
  (intptr_t)"pos",

  offsetof(j2735PositionConfidenceSet, elevation) | 0x0,
  (intptr_t)asn1_type_j2735ElevationConfidence,
  0,
  (intptr_t)"elevation",

};

const ASN1CType asn1_type_j2735PreemptPriorityList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0x1,
  0x20,
  sizeof(j2735SignalControlZone),
  (intptr_t)asn1_type_j2735SignalControlZone,
  0,
};

const ASN1CType asn1_type_j2735SignalControlZone[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  1,
  sizeof(j2735SignalControlZone),

  offsetof(j2735SignalControlZone, zone) | 0x0,
  (intptr_t)asn1_type_j2735RegionalExtension_32,
  0,
  (intptr_t)"zone",

};

const ASN1CType asn1_type_j2735_local_439[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735SSPindex,
};

const ASN1CType asn1_type_j2735PrivilegedEvents[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100004,
  2,
  sizeof(j2735PrivilegedEvents),

  offsetof(j2735PrivilegedEvents, notUsed) | 0x0,
  (intptr_t)asn1_type_j2735_local_439,
  0,
  (intptr_t)"notUsed",

  offsetof(j2735PrivilegedEvents, event) | 0x0,
  (intptr_t)asn1_type_j2735PrivilegedEventFlags,
  0,
  (intptr_t)"event",

};

const ASN1CType asn1_type_j2735PropelledInformation[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10000b,
  3,
  0,
  sizeof(j2735PropelledInformation),
  offsetof(j2735PropelledInformation, choice),
  offsetof(j2735PropelledInformation, u),
  (intptr_t)asn1_type_j2735HumanPropelledType,
  (intptr_t)"human",
  (intptr_t)asn1_type_j2735AnimalPropelledType,
  (intptr_t)"animal",
  (intptr_t)asn1_type_j2735MotorizedPropelledType,
  (intptr_t)"motor",
};

const ASN1CType asn1_type_j2735RegionList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x40,
  sizeof(j2735RegionOffsets),
  (intptr_t)asn1_type_j2735RegionOffsets,
  0,
};

const ASN1CType asn1_type_j2735_local_440[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735OffsetLL_B16,
};

const ASN1CType asn1_type_j2735_local_441[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735OffsetLL_B16,
};

const ASN1CType asn1_type_j2735_local_442[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735OffsetLL_B16,
};

const ASN1CType asn1_type_j2735RegionOffsets[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  3,
  sizeof(j2735RegionOffsets),

  offsetof(j2735RegionOffsets, xOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_440,
  0,
  (intptr_t)"xOffset",

  offsetof(j2735RegionOffsets, yOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_441,
  0,
  (intptr_t)"yOffset",

  offsetof(j2735RegionOffsets, zOffset) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_442,
  offsetof(j2735RegionOffsets, zOffset_option),
  (intptr_t)"zOffset",

};

const ASN1CType asn1_type_j2735_local_443[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Position3D,
};

const ASN1CType asn1_type_j2735_local_444[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Zoom,
};

const ASN1CType asn1_type_j2735RegionPointSet[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  3,
  sizeof(j2735RegionPointSet),

  offsetof(j2735RegionPointSet, anchor) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_443,
  offsetof(j2735RegionPointSet, anchor_option),
  (intptr_t)"anchor",

  offsetof(j2735RegionPointSet, scale) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_444,
  offsetof(j2735RegionPointSet, scale_option),
  (intptr_t)"scale",

  offsetof(j2735RegionPointSet, nodeList) | 0x0,
  (intptr_t)asn1_type_j2735RegionList,
  0,
  (intptr_t)"nodeList",

};

const ASN1CType asn1_type_j2735_local_445[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Velocity,
};

const ASN1CType asn1_type_j2735RegulatorySpeedLimit[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RegulatorySpeedLimit),

  offsetof(j2735RegulatorySpeedLimit, type) | 0x0,
  (intptr_t)asn1_type_j2735SpeedLimitType,
  0,
  (intptr_t)"type",

  offsetof(j2735RegulatorySpeedLimit, speed) | 0x0,
  (intptr_t)asn1_type_j2735_local_445,
  0,
  (intptr_t)"speed",

};

const ASN1CType asn1_type_j2735RequestedItemList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x1,
  0x20,
  sizeof(j2735RequestedItem),
  (intptr_t)asn1_type_j2735RequestedItem,
  0,
};

const ASN1CType asn1_type_j2735_local_446[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  (intptr_t)asn1_type_j2735VehicleID,
};

const ASN1CType asn1_type_j2735_local_447[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735RequestorType,
};

const ASN1CType asn1_type_j2735_local_448[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735_local_449[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735RequestorDescription[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  9,
  sizeof(j2735RequestorDescription),

  offsetof(j2735RequestorDescription, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_446,
  0,
  (intptr_t)"id",

  offsetof(j2735RequestorDescription, type) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_447,
  offsetof(j2735RequestorDescription, type_option),
  (intptr_t)"type",

  offsetof(j2735RequestorDescription, position) | 0x8000000,
  (intptr_t)asn1_type_j2735RequestorPositionVector,
  offsetof(j2735RequestorDescription, position_option),
  (intptr_t)"position",

  offsetof(j2735RequestorDescription, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_448,
  offsetof(j2735RequestorDescription, name_option),
  (intptr_t)"name",

  offsetof(j2735RequestorDescription, routeName) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_449,
  offsetof(j2735RequestorDescription, routeName_option),
  (intptr_t)"routeName",

  offsetof(j2735RequestorDescription, transitStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735TransitVehicleStatus,
  offsetof(j2735RequestorDescription, transitStatus_option),
  (intptr_t)"transitStatus",

  offsetof(j2735RequestorDescription, transitOccupancy) | 0x8000000,
  (intptr_t)asn1_type_j2735TransitVehicleOccupancy,
  offsetof(j2735RequestorDescription, transitOccupancy_option),
  (intptr_t)"transitOccupancy",

  offsetof(j2735RequestorDescription, transitSchedule) | 0x8000000,
  (intptr_t)asn1_type_j2735DeltaTime,
  offsetof(j2735RequestorDescription, transitSchedule_option),
  (intptr_t)"transitSchedule",

  offsetof(j2735RequestorDescription, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735RequestorDescription_1,
  offsetof(j2735RequestorDescription, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735RequestorDescription_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_33),
  (intptr_t)asn1_type_j2735RegionalExtension_33,
  0,
};

const ASN1CType asn1_type_j2735_local_450[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Position3D,
};

const ASN1CType asn1_type_j2735_local_451[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Angle,
};

const ASN1CType asn1_type_j2735_local_452[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735TransmissionAndSpeed,
};

const ASN1CType asn1_type_j2735RequestorPositionVector[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  3,
  sizeof(j2735RequestorPositionVector),

  offsetof(j2735RequestorPositionVector, position) | 0x0,
  (intptr_t)asn1_type_j2735_local_450,
  0,
  (intptr_t)"position",

  offsetof(j2735RequestorPositionVector, heading) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_451,
  offsetof(j2735RequestorPositionVector, heading_option),
  (intptr_t)"heading",

  offsetof(j2735RequestorPositionVector, speed) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_452,
  offsetof(j2735RequestorPositionVector, speed_option),
  (intptr_t)"speed",

};

const ASN1CType asn1_type_j2735_local_453[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735BasicVehicleRole,
};

const ASN1CType asn1_type_j2735_local_454[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735Iso3833VehicleType,
};

const ASN1CType asn1_type_j2735_local_455[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735VehicleType,
};

const ASN1CType asn1_type_j2735RequestorType[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735RequestorType),

  offsetof(j2735RequestorType, role) | 0x0,
  (intptr_t)asn1_type_j2735_local_453,
  0,
  (intptr_t)"role",

  offsetof(j2735RequestorType, subrole) | 0x8000000,
  (intptr_t)asn1_type_j2735RequestSubRole,
  offsetof(j2735RequestorType, subrole_option),
  (intptr_t)"subrole",

  offsetof(j2735RequestorType, request) | 0x8000000,
  (intptr_t)asn1_type_j2735RequestImportanceLevel,
  offsetof(j2735RequestorType, request_option),
  (intptr_t)"request",

  offsetof(j2735RequestorType, iso3883) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_454,
  offsetof(j2735RequestorType, iso3883_option),
  (intptr_t)"iso3883",

  offsetof(j2735RequestorType, hpmsType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_455,
  offsetof(j2735RequestorType, hpmsType_option),
  (intptr_t)"hpmsType",

  offsetof(j2735RequestorType, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735RegionalExtension_34,
  offsetof(j2735RequestorType, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735_local_456[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RestrictionClassID,
};

const ASN1CType asn1_type_j2735RestrictionClassAssignment[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RestrictionClassAssignment),

  offsetof(j2735RestrictionClassAssignment, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_456,
  0,
  (intptr_t)"id",

  offsetof(j2735RestrictionClassAssignment, users) | 0x0,
  (intptr_t)asn1_type_j2735RestrictionUserTypeList,
  0,
  (intptr_t)"users",

};

const ASN1CType asn1_type_j2735RestrictionClassList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0x1,
  0xfe,
  sizeof(j2735RestrictionClassAssignment),
  (intptr_t)asn1_type_j2735RestrictionClassAssignment,
  0,
};

const ASN1CType asn1_type_j2735RestrictionUserTypeList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x10,
  sizeof(j2735RestrictionUserType),
  (intptr_t)asn1_type_j2735RestrictionUserType,
  0,
};

const ASN1CType asn1_type_j2735RestrictionUserType[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x0,
  2,
  0,
  sizeof(j2735RestrictionUserType),
  offsetof(j2735RestrictionUserType, choice),
  offsetof(j2735RestrictionUserType, u),
  (intptr_t)asn1_type_j2735RestrictionAppliesTo,
  (intptr_t)"basicType",
  (intptr_t)asn1_type_j2735RestrictionUserType_1,
  (intptr_t)"regional",
};

const ASN1CType asn1_type_j2735RestrictionUserType_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_35),
  (intptr_t)asn1_type_j2735RegionalExtension_35,
  0,
};

const ASN1CType asn1_type_j2735RoadLaneSetList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0xff,
  sizeof(j2735GenericLane),
  (intptr_t)asn1_type_j2735GenericLane,
  0,
};

const ASN1CType asn1_type_j2735RoadSegmentList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x1,
  0x20,
  sizeof(j2735RoadSegment),
  (intptr_t)asn1_type_j2735RoadSegment,
  0,
};

const ASN1CType asn1_type_j2735RoadSegmentReferenceID[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RoadSegmentReferenceID),

  offsetof(j2735RoadSegmentReferenceID, region) | 0x8000000,
  (intptr_t)asn1_type_j2735RoadRegulatorID,
  offsetof(j2735RoadSegmentReferenceID, region_option),
  (intptr_t)"region",

  offsetof(j2735RoadSegmentReferenceID, id) | 0x0,
  (intptr_t)asn1_type_j2735RoadSegmentID,
  0,
  (intptr_t)"id",

};

const ASN1CType asn1_type_j2735_local_457[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735_local_458[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735RoadSegmentReferenceID,
};

const ASN1CType asn1_type_j2735_local_459[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_460[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735Position3D,
};

const ASN1CType asn1_type_j2735_local_461[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735LaneWidth,
};

const ASN1CType asn1_type_j2735RoadSegment[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  8,
  sizeof(j2735RoadSegment),

  offsetof(j2735RoadSegment, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_457,
  offsetof(j2735RoadSegment, name_option),
  (intptr_t)"name",

  offsetof(j2735RoadSegment, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_458,
  0,
  (intptr_t)"id",

  offsetof(j2735RoadSegment, revision) | 0x0,
  (intptr_t)asn1_type_j2735_local_459,
  0,
  (intptr_t)"revision",

  offsetof(j2735RoadSegment, refPoint) | 0x0,
  (intptr_t)asn1_type_j2735_local_460,
  0,
  (intptr_t)"refPoint",

  offsetof(j2735RoadSegment, laneWidth) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_461,
  offsetof(j2735RoadSegment, laneWidth_option),
  (intptr_t)"laneWidth",

  offsetof(j2735RoadSegment, speedLimits) | 0x8000000,
  (intptr_t)asn1_type_j2735SpeedLimitList,
  offsetof(j2735RoadSegment, speedLimits_option),
  (intptr_t)"speedLimits",

  offsetof(j2735RoadSegment, roadLaneSet) | 0x0,
  (intptr_t)asn1_type_j2735RoadLaneSetList,
  0,
  (intptr_t)"roadLaneSet",

  offsetof(j2735RoadSegment, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735RoadSegment_1,
  offsetof(j2735RoadSegment, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735RoadSegment_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_36),
  (intptr_t)asn1_type_j2735RegionalExtension_36,
  0,
};

const ASN1CType asn1_type_j2735_local_462[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Position3D,
};

const ASN1CType asn1_type_j2735_local_463[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735HeadingSlice,
};

const ASN1CType asn1_type_j2735RoadSignID[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  4,
  sizeof(j2735RoadSignID),

  offsetof(j2735RoadSignID, position) | 0x0,
  (intptr_t)asn1_type_j2735_local_462,
  0,
  (intptr_t)"position",

  offsetof(j2735RoadSignID, viewAngle) | 0x0,
  (intptr_t)asn1_type_j2735_local_463,
  0,
  (intptr_t)"viewAngle",

  offsetof(j2735RoadSignID, mutcdCode) | 0x8000000,
  (intptr_t)asn1_type_j2735MUTCDCode,
  offsetof(j2735RoadSignID, mutcdCode_option),
  (intptr_t)"mutcdCode",

  offsetof(j2735RoadSignID, crc) | 0x8000000,
  (intptr_t)asn1_type_j2735MsgCRC,
  offsetof(j2735RoadSignID, crc_option),
  (intptr_t)"crc",

};

const ASN1CType asn1_type_j2735_local_464[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735GNSSstatus,
};

const ASN1CType asn1_type_j2735RTCMheader[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735RTCMheader),

  offsetof(j2735RTCMheader, status) | 0x0,
  (intptr_t)asn1_type_j2735_local_464,
  0,
  (intptr_t)"status",

  offsetof(j2735RTCMheader, offsetSet) | 0x0,
  (intptr_t)asn1_type_j2735AntennaOffsetSet,
  0,
  (intptr_t)"offsetSet",

};

const ASN1CType asn1_type_j2735RTCMmessageList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10,
  0x1,
  0x5,
  sizeof(j2735RTCMmessage),
  (intptr_t)asn1_type_j2735RTCMmessage,
  0,
};

const ASN1CType asn1_type_j2735_local_465[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735RTCMheader,
};

const ASN1CType asn1_type_j2735_local_466[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735RTCMmessageList,
};

const ASN1CType asn1_type_j2735RTCMPackage[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100008,
  2,
  sizeof(j2735RTCMPackage),

  offsetof(j2735RTCMPackage, rtcmHeader) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_465,
  offsetof(j2735RTCMPackage, rtcmHeader_option),
  (intptr_t)"rtcmHeader",

  offsetof(j2735RTCMPackage, msgs) | 0x0,
  (intptr_t)asn1_type_j2735_local_466,
  0,
  (intptr_t)"msgs",

};

const ASN1CType asn1_type_j2735_local_467[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735_local_468[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735Sample[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  2,
  sizeof(j2735Sample),

  offsetof(j2735Sample, sampleStart) | 0x0,
  (intptr_t)asn1_type_j2735_local_467,
  0,
  (intptr_t)"sampleStart",

  offsetof(j2735Sample, sampleEnd) | 0x0,
  (intptr_t)asn1_type_j2735_local_468,
  0,
  (intptr_t)"sampleEnd",

};

const ASN1CType asn1_type_j2735SegmentAttributeLLList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10,
  0x1,
  0x8,
  sizeof(j2735SegmentAttributeLL),
  (intptr_t)asn1_type_j2735SegmentAttributeLL,
  0,
};

const ASN1CType asn1_type_j2735SegmentAttributeXYList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10,
  0x1,
  0x8,
  sizeof(j2735SegmentAttributeXY),
  (intptr_t)asn1_type_j2735SegmentAttributeXY,
  0,
};

const ASN1CType asn1_type_j2735_local_469[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Position3D,
};

const ASN1CType asn1_type_j2735_local_470[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735LaneWidth,
};

const ASN1CType asn1_type_j2735_local_471[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735DirectionOfUse,
};

const ASN1CType asn1_type_j2735_local_472[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100003,
  (intptr_t)asn1_type_j2735NodeListXY,
};

const ASN1CType asn1_type_j2735ShapePointSet[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  4,
  sizeof(j2735ShapePointSet),

  offsetof(j2735ShapePointSet, anchor) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_469,
  offsetof(j2735ShapePointSet, anchor_option),
  (intptr_t)"anchor",

  offsetof(j2735ShapePointSet, laneWidth) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_470,
  offsetof(j2735ShapePointSet, laneWidth_option),
  (intptr_t)"laneWidth",

  offsetof(j2735ShapePointSet, directionality) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_471,
  offsetof(j2735ShapePointSet, directionality_option),
  (intptr_t)"directionality",

  offsetof(j2735ShapePointSet, nodeList) | 0x0,
  (intptr_t)asn1_type_j2735_local_472,
  0,
  (intptr_t)"nodeList",

};

const ASN1CType asn1_type_j2735_local_473[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  (intptr_t)asn1_type_j2735VehicleID,
};

const ASN1CType asn1_type_j2735_local_474[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_475[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735BasicVehicleRole,
};

const ASN1CType asn1_type_j2735_local_476[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735RequestorType,
};

const ASN1CType asn1_type_j2735SignalRequesterInfo[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  5,
  sizeof(j2735SignalRequesterInfo),

  offsetof(j2735SignalRequesterInfo, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_473,
  0,
  (intptr_t)"id",

  offsetof(j2735SignalRequesterInfo, request) | 0x0,
  (intptr_t)asn1_type_j2735RequestID,
  0,
  (intptr_t)"request",

  offsetof(j2735SignalRequesterInfo, sequenceNumber) | 0x0,
  (intptr_t)asn1_type_j2735_local_474,
  0,
  (intptr_t)"sequenceNumber",

  offsetof(j2735SignalRequesterInfo, role) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_475,
  offsetof(j2735SignalRequesterInfo, role_option),
  (intptr_t)"role",

  offsetof(j2735SignalRequesterInfo, typeData) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_476,
  offsetof(j2735SignalRequesterInfo, typeData_option),
  (intptr_t)"typeData",

};

const ASN1CType asn1_type_j2735SignalRequestList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10,
  0x1,
  0x20,
  sizeof(j2735SignalRequestPackage),
  (intptr_t)asn1_type_j2735SignalRequestPackage,
  0,
};

const ASN1CType asn1_type_j2735_local_477[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_478[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735DSecond,
};

const ASN1CType asn1_type_j2735_local_479[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735DSecond,
};

const ASN1CType asn1_type_j2735SignalRequestPackage[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  5,
  sizeof(j2735SignalRequestPackage),

  offsetof(j2735SignalRequestPackage, request) | 0x0,
  (intptr_t)asn1_type_j2735SignalRequest,
  0,
  (intptr_t)"request",

  offsetof(j2735SignalRequestPackage, minute) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_477,
  offsetof(j2735SignalRequestPackage, minute_option),
  (intptr_t)"minute",

  offsetof(j2735SignalRequestPackage, second) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_478,
  offsetof(j2735SignalRequestPackage, second_option),
  (intptr_t)"second",

  offsetof(j2735SignalRequestPackage, duration) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_479,
  offsetof(j2735SignalRequestPackage, duration_option),
  (intptr_t)"duration",

  offsetof(j2735SignalRequestPackage, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735SignalRequestPackage_1,
  offsetof(j2735SignalRequestPackage, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735SignalRequestPackage_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_37),
  (intptr_t)asn1_type_j2735RegionalExtension_37,
  0,
};

const ASN1CType asn1_type_j2735_local_480[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735IntersectionReferenceID,
};

const ASN1CType asn1_type_j2735_local_481[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100003,
  (intptr_t)asn1_type_j2735IntersectionAccessPoint,
};

const ASN1CType asn1_type_j2735_local_482[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100004,
  (intptr_t)asn1_type_j2735IntersectionAccessPoint,
};

const ASN1CType asn1_type_j2735SignalRequest[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  6,
  sizeof(j2735SignalRequest),

  offsetof(j2735SignalRequest, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_480,
  0,
  (intptr_t)"id",

  offsetof(j2735SignalRequest, requestID) | 0x0,
  (intptr_t)asn1_type_j2735RequestID,
  0,
  (intptr_t)"requestID",

  offsetof(j2735SignalRequest, requestType) | 0x0,
  (intptr_t)asn1_type_j2735PriorityRequestType,
  0,
  (intptr_t)"requestType",

  offsetof(j2735SignalRequest, inBoundLane) | 0x0,
  (intptr_t)asn1_type_j2735_local_481,
  0,
  (intptr_t)"inBoundLane",

  offsetof(j2735SignalRequest, outBoundLane) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_482,
  offsetof(j2735SignalRequest, outBoundLane_option),
  (intptr_t)"outBoundLane",

  offsetof(j2735SignalRequest, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735SignalRequest_1,
  offsetof(j2735SignalRequest, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735SignalRequest_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_38),
  (intptr_t)asn1_type_j2735RegionalExtension_38,
  0,
};

const ASN1CType asn1_type_j2735SignalStatusList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10,
  0x1,
  0x20,
  sizeof(j2735SignalStatus),
  (intptr_t)asn1_type_j2735SignalStatus,
  0,
};

const ASN1CType asn1_type_j2735SignalStatusPackageList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x20,
  sizeof(j2735SignalStatusPackage),
  (intptr_t)asn1_type_j2735SignalStatusPackage,
  0,
};

const ASN1CType asn1_type_j2735_local_483[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  (intptr_t)asn1_type_j2735IntersectionAccessPoint,
};

const ASN1CType asn1_type_j2735_local_484[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  (intptr_t)asn1_type_j2735IntersectionAccessPoint,
};

const ASN1CType asn1_type_j2735_local_485[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_486[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735DSecond,
};

const ASN1CType asn1_type_j2735_local_487[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735DSecond,
};

const ASN1CType asn1_type_j2735SignalStatusPackage[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  8,
  sizeof(j2735SignalStatusPackage),

  offsetof(j2735SignalStatusPackage, requester) | 0x8000000,
  (intptr_t)asn1_type_j2735SignalRequesterInfo,
  offsetof(j2735SignalStatusPackage, requester_option),
  (intptr_t)"requester",

  offsetof(j2735SignalStatusPackage, inboundOn) | 0x0,
  (intptr_t)asn1_type_j2735_local_483,
  0,
  (intptr_t)"inboundOn",

  offsetof(j2735SignalStatusPackage, outboundOn) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_484,
  offsetof(j2735SignalStatusPackage, outboundOn_option),
  (intptr_t)"outboundOn",

  offsetof(j2735SignalStatusPackage, minute) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_485,
  offsetof(j2735SignalStatusPackage, minute_option),
  (intptr_t)"minute",

  offsetof(j2735SignalStatusPackage, second) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_486,
  offsetof(j2735SignalStatusPackage, second_option),
  (intptr_t)"second",

  offsetof(j2735SignalStatusPackage, duration) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_487,
  offsetof(j2735SignalStatusPackage, duration_option),
  (intptr_t)"duration",

  offsetof(j2735SignalStatusPackage, status) | 0x0,
  (intptr_t)asn1_type_j2735PrioritizationResponseStatus,
  0,
  (intptr_t)"status",

  offsetof(j2735SignalStatusPackage, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735SignalStatusPackage_1,
  offsetof(j2735SignalStatusPackage, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735SignalStatusPackage_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_39),
  (intptr_t)asn1_type_j2735RegionalExtension_39,
  0,
};

const ASN1CType asn1_type_j2735_local_488[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735MsgCount,
};

const ASN1CType asn1_type_j2735_local_489[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735IntersectionReferenceID,
};

const ASN1CType asn1_type_j2735SignalStatus[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  4,
  sizeof(j2735SignalStatus),

  offsetof(j2735SignalStatus, sequenceNumber) | 0x0,
  (intptr_t)asn1_type_j2735_local_488,
  0,
  (intptr_t)"sequenceNumber",

  offsetof(j2735SignalStatus, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_489,
  0,
  (intptr_t)"id",

  offsetof(j2735SignalStatus, sigStatus) | 0x0,
  (intptr_t)asn1_type_j2735SignalStatusPackageList,
  0,
  (intptr_t)"sigStatus",

  offsetof(j2735SignalStatus, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735SignalStatus_1,
  offsetof(j2735SignalStatus, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735SignalStatus_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_40),
  (intptr_t)asn1_type_j2735RegionalExtension_40,
  0,
};

const ASN1CType asn1_type_j2735_local_490[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735GrossDistance,
};

const ASN1CType asn1_type_j2735_local_491[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735GrossSpeed,
};

const ASN1CType asn1_type_j2735_local_492[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735GrossDistance,
};

const ASN1CType asn1_type_j2735_local_493[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735GrossSpeed,
};

const ASN1CType asn1_type_j2735SnapshotDistance[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  4,
  sizeof(j2735SnapshotDistance),

  offsetof(j2735SnapshotDistance, distance1) | 0x0,
  (intptr_t)asn1_type_j2735_local_490,
  0,
  (intptr_t)"distance1",

  offsetof(j2735SnapshotDistance, speed1) | 0x0,
  (intptr_t)asn1_type_j2735_local_491,
  0,
  (intptr_t)"speed1",

  offsetof(j2735SnapshotDistance, distance2) | 0x0,
  (intptr_t)asn1_type_j2735_local_492,
  0,
  (intptr_t)"distance2",

  offsetof(j2735SnapshotDistance, speed2) | 0x0,
  (intptr_t)asn1_type_j2735_local_493,
  0,
  (intptr_t)"speed2",

};

const ASN1CType asn1_type_j2735_local_494[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735FullPositionVector,
};

const ASN1CType asn1_type_j2735_local_495[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735VehicleSafetyExtensions,
};

const ASN1CType asn1_type_j2735Snapshot[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  3,
  sizeof(j2735Snapshot),

  offsetof(j2735Snapshot, thePosition) | 0x0,
  (intptr_t)asn1_type_j2735_local_494,
  0,
  (intptr_t)"thePosition",

  offsetof(j2735Snapshot, safetyExt) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_495,
  offsetof(j2735Snapshot, safetyExt_option),
  (intptr_t)"safetyExt",

  offsetof(j2735Snapshot, dataSet) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleStatus,
  offsetof(j2735Snapshot, dataSet_option),
  (intptr_t)"dataSet",

};

const ASN1CType asn1_type_j2735_local_496[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735GrossSpeed,
};

const ASN1CType asn1_type_j2735_local_497[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735SecondOfTime,
};

const ASN1CType asn1_type_j2735_local_498[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735GrossSpeed,
};

const ASN1CType asn1_type_j2735_local_499[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735SecondOfTime,
};

const ASN1CType asn1_type_j2735SnapshotTime[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  4,
  sizeof(j2735SnapshotTime),

  offsetof(j2735SnapshotTime, speed1) | 0x0,
  (intptr_t)asn1_type_j2735_local_496,
  0,
  (intptr_t)"speed1",

  offsetof(j2735SnapshotTime, time1) | 0x0,
  (intptr_t)asn1_type_j2735_local_497,
  0,
  (intptr_t)"time1",

  offsetof(j2735SnapshotTime, speed2) | 0x0,
  (intptr_t)asn1_type_j2735_local_498,
  0,
  (intptr_t)"speed2",

  offsetof(j2735SnapshotTime, time2) | 0x0,
  (intptr_t)asn1_type_j2735_local_499,
  0,
  (intptr_t)"time2",

};

const ASN1CType asn1_type_j2735_local_500[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735EmergencyDetails,
};

const ASN1CType asn1_type_j2735SpecialVehicleExtensions[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  3,
  sizeof(j2735SpecialVehicleExtensions),

  offsetof(j2735SpecialVehicleExtensions, vehicleAlerts) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_500,
  offsetof(j2735SpecialVehicleExtensions, vehicleAlerts_option),
  (intptr_t)"vehicleAlerts",

  offsetof(j2735SpecialVehicleExtensions, description) | 0x8000000,
  (intptr_t)asn1_type_j2735EventDescription,
  offsetof(j2735SpecialVehicleExtensions, description_option),
  (intptr_t)"description",

  offsetof(j2735SpecialVehicleExtensions, trailers) | 0x8000000,
  (intptr_t)asn1_type_j2735TrailerData,
  offsetof(j2735SpecialVehicleExtensions, trailers_option),
  (intptr_t)"trailers",

};

const ASN1CType asn1_type_j2735_local_501[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735HeadingConfidence,
};

const ASN1CType asn1_type_j2735_local_502[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735SpeedConfidence,
};

const ASN1CType asn1_type_j2735_local_503[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735ThrottleConfidence,
};

const ASN1CType asn1_type_j2735SpeedandHeadingandThrottleConfidence[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  3,
  sizeof(j2735SpeedandHeadingandThrottleConfidence),

  offsetof(j2735SpeedandHeadingandThrottleConfidence, heading) | 0x0,
  (intptr_t)asn1_type_j2735_local_501,
  0,
  (intptr_t)"heading",

  offsetof(j2735SpeedandHeadingandThrottleConfidence, speed) | 0x0,
  (intptr_t)asn1_type_j2735_local_502,
  0,
  (intptr_t)"speed",

  offsetof(j2735SpeedandHeadingandThrottleConfidence, throttle) | 0x0,
  (intptr_t)asn1_type_j2735_local_503,
  0,
  (intptr_t)"throttle",

};

const ASN1CType asn1_type_j2735SpeedLimitList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x1,
  0x9,
  sizeof(j2735RegulatorySpeedLimit),
  (intptr_t)asn1_type_j2735RegulatorySpeedLimit,
  0,
};

const ASN1CType asn1_type_j2735SpeedProfileMeasurementList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x1,
  0x14,
  sizeof(j2735SpeedProfileMeasurement),
  (intptr_t)asn1_type_j2735SpeedProfileMeasurement,
  0,
};

const ASN1CType asn1_type_j2735SpeedProfile[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100007,
  1,
  sizeof(j2735SpeedProfile),

  offsetof(j2735SpeedProfile, speedReports) | 0x0,
  (intptr_t)asn1_type_j2735SpeedProfileMeasurementList,
  0,
  (intptr_t)"speedReports",

};

const ASN1CType asn1_type_j2735_local_504[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735BasicVehicleClass,
};

const ASN1CType asn1_type_j2735_local_505[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735VehicleClassification,
};

const ASN1CType asn1_type_j2735SupplementalVehicleExtensions[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  10,
  sizeof(j2735SupplementalVehicleExtensions),

  offsetof(j2735SupplementalVehicleExtensions, classification) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_504,
  offsetof(j2735SupplementalVehicleExtensions, classification_option),
  (intptr_t)"classification",

  offsetof(j2735SupplementalVehicleExtensions, classDetails) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_505,
  offsetof(j2735SupplementalVehicleExtensions, classDetails_option),
  (intptr_t)"classDetails",

  offsetof(j2735SupplementalVehicleExtensions, vehicleData) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleData,
  offsetof(j2735SupplementalVehicleExtensions, vehicleData_option),
  (intptr_t)"vehicleData",

  offsetof(j2735SupplementalVehicleExtensions, weatherReport) | 0x8000000,
  (intptr_t)asn1_type_j2735WeatherReport,
  offsetof(j2735SupplementalVehicleExtensions, weatherReport_option),
  (intptr_t)"weatherReport",

  offsetof(j2735SupplementalVehicleExtensions, weatherProbe) | 0x8000000,
  (intptr_t)asn1_type_j2735WeatherProbe,
  offsetof(j2735SupplementalVehicleExtensions, weatherProbe_option),
  (intptr_t)"weatherProbe",

  offsetof(j2735SupplementalVehicleExtensions, obstacle) | 0x8000000,
  (intptr_t)asn1_type_j2735ObstacleDetection,
  offsetof(j2735SupplementalVehicleExtensions, obstacle_option),
  (intptr_t)"obstacle",

  offsetof(j2735SupplementalVehicleExtensions, status) | 0x8000000,
  (intptr_t)asn1_type_j2735DisabledVehicle,
  offsetof(j2735SupplementalVehicleExtensions, status_option),
  (intptr_t)"status",

  offsetof(j2735SupplementalVehicleExtensions, speedProfile) | 0x8000000,
  (intptr_t)asn1_type_j2735SpeedProfile,
  offsetof(j2735SupplementalVehicleExtensions, speedProfile_option),
  (intptr_t)"speedProfile",

  offsetof(j2735SupplementalVehicleExtensions, theRTCM) | 0x8000000,
  (intptr_t)asn1_type_j2735RTCMPackage,
  offsetof(j2735SupplementalVehicleExtensions, theRTCM_option),
  (intptr_t)"theRTCM",

  offsetof(j2735SupplementalVehicleExtensions, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735SupplementalVehicleExtensions_1,
  offsetof(j2735SupplementalVehicleExtensions, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735SupplementalVehicleExtensions_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100009,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_41),
  (intptr_t)asn1_type_j2735RegionalExtension_41,
  0,
};

const ASN1CType asn1_type_j2735_local_506[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735TimeMark,
};

const ASN1CType asn1_type_j2735_local_507[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735TimeMark,
};

const ASN1CType asn1_type_j2735_local_508[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735TimeMark,
};

const ASN1CType asn1_type_j2735_local_509[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735TimeMark,
};

const ASN1CType asn1_type_j2735_local_510[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735TimeMark,
};

const ASN1CType asn1_type_j2735TimeChangeDetails[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  6,
  sizeof(j2735TimeChangeDetails),

  offsetof(j2735TimeChangeDetails, startTime) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_506,
  offsetof(j2735TimeChangeDetails, startTime_option),
  (intptr_t)"startTime",

  offsetof(j2735TimeChangeDetails, minEndTime) | 0x0,
  (intptr_t)asn1_type_j2735_local_507,
  0,
  (intptr_t)"minEndTime",

  offsetof(j2735TimeChangeDetails, maxEndTime) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_508,
  offsetof(j2735TimeChangeDetails, maxEndTime_option),
  (intptr_t)"maxEndTime",

  offsetof(j2735TimeChangeDetails, likelyTime) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_509,
  offsetof(j2735TimeChangeDetails, likelyTime_option),
  (intptr_t)"likelyTime",

  offsetof(j2735TimeChangeDetails, confidence) | 0x8000000,
  (intptr_t)asn1_type_j2735TimeIntervalConfidence,
  offsetof(j2735TimeChangeDetails, confidence_option),
  (intptr_t)"confidence",

  offsetof(j2735TimeChangeDetails, nextTime) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_510,
  offsetof(j2735TimeChangeDetails, nextTime_option),
  (intptr_t)"nextTime",

};

const ASN1CType asn1_type_j2735_local_511[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735SSPindex,
};

const ASN1CType asn1_type_j2735_local_512[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735PivotPointDescription,
};

const ASN1CType asn1_type_j2735TrailerData[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  3,
  sizeof(j2735TrailerData),

  offsetof(j2735TrailerData, notUsed) | 0x0,
  (intptr_t)asn1_type_j2735_local_511,
  0,
  (intptr_t)"notUsed",

  offsetof(j2735TrailerData, connection) | 0x0,
  (intptr_t)asn1_type_j2735_local_512,
  0,
  (intptr_t)"connection",

  offsetof(j2735TrailerData, units) | 0x0,
  (intptr_t)asn1_type_j2735TrailerUnitDescriptionList,
  0,
  (intptr_t)"units",

};

const ASN1CType asn1_type_j2735TrailerHistoryPointList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10000c,
  0x1,
  0x17,
  sizeof(j2735TrailerHistoryPoint),
  (intptr_t)asn1_type_j2735TrailerHistoryPoint,
  0,
};

const ASN1CType asn1_type_j2735_local_513[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735Angle,
};

const ASN1CType asn1_type_j2735_local_514[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735TimeOffset,
};

const ASN1CType asn1_type_j2735_local_515[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Node_XY_24b,
};

const ASN1CType asn1_type_j2735_local_516[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735VertOffset_B07,
};

const ASN1CType asn1_type_j2735_local_517[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735CoarseHeading,
};

const ASN1CType asn1_type_j2735TrailerHistoryPoint[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  5,
  sizeof(j2735TrailerHistoryPoint),

  offsetof(j2735TrailerHistoryPoint, pivotAngle) | 0x0,
  (intptr_t)asn1_type_j2735_local_513,
  0,
  (intptr_t)"pivotAngle",

  offsetof(j2735TrailerHistoryPoint, timeOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_514,
  0,
  (intptr_t)"timeOffset",

  offsetof(j2735TrailerHistoryPoint, positionOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_515,
  0,
  (intptr_t)"positionOffset",

  offsetof(j2735TrailerHistoryPoint, elevationOffset) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_516,
  offsetof(j2735TrailerHistoryPoint, elevationOffset_option),
  (intptr_t)"elevationOffset",

  offsetof(j2735TrailerHistoryPoint, heading) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_517,
  offsetof(j2735TrailerHistoryPoint, heading_option),
  (intptr_t)"heading",

};

const ASN1CType asn1_type_j2735TrailerUnitDescriptionList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x8,
  sizeof(j2735TrailerUnitDescription),
  (intptr_t)asn1_type_j2735TrailerUnitDescription,
  0,
};

const ASN1CType asn1_type_j2735_local_518[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735VehicleWidth,
};

const ASN1CType asn1_type_j2735_local_519[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735VehicleLength,
};

const ASN1CType asn1_type_j2735_local_520[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735VehicleHeight,
};

const ASN1CType asn1_type_j2735_local_521[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735BumperHeights,
};

const ASN1CType asn1_type_j2735_local_522[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735VehicleHeight,
};

const ASN1CType asn1_type_j2735_local_523[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735PivotPointDescription,
};

const ASN1CType asn1_type_j2735_local_524[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735PivotPointDescription,
};

const ASN1CType asn1_type_j2735_local_525[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  (intptr_t)asn1_type_j2735Offset_B12,
};

const ASN1CType asn1_type_j2735_local_526[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000a,
  (intptr_t)asn1_type_j2735Node_XY_24b,
};

const ASN1CType asn1_type_j2735_local_527[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000b,
  (intptr_t)asn1_type_j2735VertOffset_B07,
};

const ASN1CType asn1_type_j2735TrailerUnitDescription[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  13,
  sizeof(j2735TrailerUnitDescription),

  offsetof(j2735TrailerUnitDescription, isDolly) | 0x0,
  (intptr_t)asn1_type_j2735IsDolly,
  0,
  (intptr_t)"isDolly",

  offsetof(j2735TrailerUnitDescription, width) | 0x0,
  (intptr_t)asn1_type_j2735_local_518,
  0,
  (intptr_t)"width",

  offsetof(j2735TrailerUnitDescription, length) | 0x0,
  (intptr_t)asn1_type_j2735_local_519,
  0,
  (intptr_t)"length",

  offsetof(j2735TrailerUnitDescription, height) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_520,
  offsetof(j2735TrailerUnitDescription, height_option),
  (intptr_t)"height",

  offsetof(j2735TrailerUnitDescription, mass) | 0x8000000,
  (intptr_t)asn1_type_j2735TrailerMass,
  offsetof(j2735TrailerUnitDescription, mass_option),
  (intptr_t)"mass",

  offsetof(j2735TrailerUnitDescription, bumperHeights) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_521,
  offsetof(j2735TrailerUnitDescription, bumperHeights_option),
  (intptr_t)"bumperHeights",

  offsetof(j2735TrailerUnitDescription, centerOfGravity) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_522,
  offsetof(j2735TrailerUnitDescription, centerOfGravity_option),
  (intptr_t)"centerOfGravity",

  offsetof(j2735TrailerUnitDescription, frontPivot) | 0x0,
  (intptr_t)asn1_type_j2735_local_523,
  0,
  (intptr_t)"frontPivot",

  offsetof(j2735TrailerUnitDescription, rearPivot) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_524,
  offsetof(j2735TrailerUnitDescription, rearPivot_option),
  (intptr_t)"rearPivot",

  offsetof(j2735TrailerUnitDescription, rearWheelOffset) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_525,
  offsetof(j2735TrailerUnitDescription, rearWheelOffset_option),
  (intptr_t)"rearWheelOffset",

  offsetof(j2735TrailerUnitDescription, positionOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_526,
  0,
  (intptr_t)"positionOffset",

  offsetof(j2735TrailerUnitDescription, elevationOffset) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_527,
  offsetof(j2735TrailerUnitDescription, elevationOffset_option),
  (intptr_t)"elevationOffset",

  offsetof(j2735TrailerUnitDescription, crumbData) | 0x8000000,
  (intptr_t)asn1_type_j2735TrailerHistoryPointList,
  offsetof(j2735TrailerUnitDescription, crumbData_option),
  (intptr_t)"crumbData",

};

const ASN1CType asn1_type_j2735_local_528[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735TransmissionState,
};

const ASN1CType asn1_type_j2735_local_529[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Velocity,
};

const ASN1CType asn1_type_j2735TransmissionAndSpeed[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  2,
  sizeof(j2735TransmissionAndSpeed),

  offsetof(j2735TransmissionAndSpeed, transmission) | 0x0,
  (intptr_t)asn1_type_j2735_local_528,
  0,
  (intptr_t)"transmission",

  offsetof(j2735TransmissionAndSpeed, speed) | 0x0,
  (intptr_t)asn1_type_j2735_local_529,
  0,
  (intptr_t)"speed",

};

const ASN1CType asn1_type_j2735TravelerDataFrameList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x1,
  0x8,
  sizeof(j2735TravelerDataFrame),
  (intptr_t)asn1_type_j2735TravelerDataFrame,
  0,
};

const ASN1CType asn1_type_j2735_local_530[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735SSPindex,
};

const ASN1CType asn1_type_j2735_local_531[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735DYear,
};

const ASN1CType asn1_type_j2735_local_532[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735MinuteOfTheYear,
};

const ASN1CType asn1_type_j2735_local_533[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735SSPindex,
};

const ASN1CType asn1_type_j2735_local_534[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  (intptr_t)asn1_type_j2735SSPindex,
};

const ASN1CType asn1_type_j2735_local_535[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000a,
  (intptr_t)asn1_type_j2735SSPindex,
};

const ASN1CType asn1_type_j2735TravelerDataFrame[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  13,
  sizeof(j2735TravelerDataFrame),

  offsetof(j2735TravelerDataFrame, notUsed) | 0x0,
  (intptr_t)asn1_type_j2735_local_530,
  0,
  (intptr_t)"notUsed",

  offsetof(j2735TravelerDataFrame, frameType) | 0x0,
  (intptr_t)asn1_type_j2735TravelerInfoType,
  0,
  (intptr_t)"frameType",

  offsetof(j2735TravelerDataFrame, msgId) | 0x0,
  (intptr_t)asn1_type_j2735TravelerDataFrame_1,
  0,
  (intptr_t)"msgId",

  offsetof(j2735TravelerDataFrame, startYear) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_531,
  offsetof(j2735TravelerDataFrame, startYear_option),
  (intptr_t)"startYear",

  offsetof(j2735TravelerDataFrame, startTime) | 0x0,
  (intptr_t)asn1_type_j2735_local_532,
  0,
  (intptr_t)"startTime",

  offsetof(j2735TravelerDataFrame, durationTime) | 0x0,
  (intptr_t)asn1_type_j2735MinutesDuration,
  0,
  (intptr_t)"durationTime",

  offsetof(j2735TravelerDataFrame, priority) | 0x0,
  (intptr_t)asn1_type_j2735SignPrority,
  0,
  (intptr_t)"priority",

  offsetof(j2735TravelerDataFrame, notUsed1) | 0x0,
  (intptr_t)asn1_type_j2735_local_533,
  0,
  (intptr_t)"notUsed1",

  offsetof(j2735TravelerDataFrame, regions) | 0x0,
  (intptr_t)asn1_type_j2735TravelerDataFrame_2,
  0,
  (intptr_t)"regions",

  offsetof(j2735TravelerDataFrame, notUsed2) | 0x0,
  (intptr_t)asn1_type_j2735_local_534,
  0,
  (intptr_t)"notUsed2",

  offsetof(j2735TravelerDataFrame, notUsed3) | 0x0,
  (intptr_t)asn1_type_j2735_local_535,
  0,
  (intptr_t)"notUsed3",

  offsetof(j2735TravelerDataFrame, content) | 0x0,
  (intptr_t)asn1_type_j2735TravelerDataFrame_3,
  0,
  (intptr_t)"content",

  offsetof(j2735TravelerDataFrame, url) | 0x8000000,
  (intptr_t)asn1_type_j2735URL_Short,
  offsetof(j2735TravelerDataFrame, url_option),
  (intptr_t)"url",

};

const ASN1CType asn1_type_j2735_local_536[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735FurtherInfoID,
};

const ASN1CType asn1_type_j2735TravelerDataFrame_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  2,
  sizeof(j2735TravelerDataFrame_1),
  offsetof(j2735TravelerDataFrame_1, choice),
  offsetof(j2735TravelerDataFrame_1, u),
  (intptr_t)asn1_type_j2735_local_536,
  (intptr_t)"furtherInfoID",
  (intptr_t)asn1_type_j2735RoadSignID,
  (intptr_t)"roadSignID",
};

const ASN1CType asn1_type_j2735TravelerDataFrame_2[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x1,
  0x10,
  sizeof(j2735GeographicalPath),
  (intptr_t)asn1_type_j2735GeographicalPath,
  0,
};

const ASN1CType asn1_type_j2735_local_537[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITIScodesAndText,
};

const ASN1CType asn1_type_j2735TravelerDataFrame_3[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000b,
  5,
  sizeof(j2735TravelerDataFrame_3),
  offsetof(j2735TravelerDataFrame_3, choice),
  offsetof(j2735TravelerDataFrame_3, u),
  (intptr_t)asn1_type_j2735_local_537,
  (intptr_t)"advisory",
  (intptr_t)asn1_type_j2735WorkZone,
  (intptr_t)"workZone",
  (intptr_t)asn1_type_j2735GenericSignage,
  (intptr_t)"genericSign",
  (intptr_t)asn1_type_j2735SpeedLimit,
  (intptr_t)"speedLimit",
  (intptr_t)asn1_type_j2735ExitService,
  (intptr_t)"exitService",
};

const ASN1CType asn1_type_j2735_local_538[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735HeadingSlice,
};

const ASN1CType asn1_type_j2735_local_539[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Extent,
};

const ASN1CType asn1_type_j2735ValidRegion[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  3,
  sizeof(j2735ValidRegion),

  offsetof(j2735ValidRegion, direction) | 0x0,
  (intptr_t)asn1_type_j2735_local_538,
  0,
  (intptr_t)"direction",

  offsetof(j2735ValidRegion, extent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_539,
  offsetof(j2735ValidRegion, extent_option),
  (intptr_t)"extent",

  offsetof(j2735ValidRegion, area) | 0x0,
  (intptr_t)asn1_type_j2735ValidRegion_1,
  0,
  (intptr_t)"area",

};

const ASN1CType asn1_type_j2735_local_540[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Circle,
};

const ASN1CType asn1_type_j2735ValidRegion_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  3,
  sizeof(j2735ValidRegion_1),
  offsetof(j2735ValidRegion_1, choice),
  offsetof(j2735ValidRegion_1, u),
  (intptr_t)asn1_type_j2735ShapePointSet,
  (intptr_t)"shapePointSet",
  (intptr_t)asn1_type_j2735_local_540,
  (intptr_t)"circle",
  (intptr_t)asn1_type_j2735RegionPointSet,
  (intptr_t)"regionPointSet",
};

const ASN1CType asn1_type_j2735_local_541[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735BasicVehicleClass,
};

const ASN1CType asn1_type_j2735_local_542[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735BasicVehicleRole,
};

const ASN1CType asn1_type_j2735_local_543[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Iso3833VehicleType,
};

const ASN1CType asn1_type_j2735_local_544[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735VehicleType,
};

const ASN1CType asn1_type_j2735_local_545[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735VehicleGroupAffected,
};

const ASN1CType asn1_type_j2735_local_546[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735IncidentResponseEquipment,
};

const ASN1CType asn1_type_j2735_local_547[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  (intptr_t)asn1_type_j2735ResponderGroupAffected,
};

const ASN1CType asn1_type_j2735_local_548[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735FuelType,
};

const ASN1CType asn1_type_j2735VehicleClassification[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  9,
  sizeof(j2735VehicleClassification),

  offsetof(j2735VehicleClassification, keyType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_541,
  offsetof(j2735VehicleClassification, keyType_option),
  (intptr_t)"keyType",

  offsetof(j2735VehicleClassification, role) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_542,
  offsetof(j2735VehicleClassification, role_option),
  (intptr_t)"role",

  offsetof(j2735VehicleClassification, iso3883) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_543,
  offsetof(j2735VehicleClassification, iso3883_option),
  (intptr_t)"iso3883",

  offsetof(j2735VehicleClassification, hpmsType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_544,
  offsetof(j2735VehicleClassification, hpmsType_option),
  (intptr_t)"hpmsType",

  offsetof(j2735VehicleClassification, vehicleType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_545,
  offsetof(j2735VehicleClassification, vehicleType_option),
  (intptr_t)"vehicleType",

  offsetof(j2735VehicleClassification, responseEquip) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_546,
  offsetof(j2735VehicleClassification, responseEquip_option),
  (intptr_t)"responseEquip",

  offsetof(j2735VehicleClassification, responderType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_547,
  offsetof(j2735VehicleClassification, responderType_option),
  (intptr_t)"responderType",

  offsetof(j2735VehicleClassification, fuelType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_548,
  offsetof(j2735VehicleClassification, fuelType_option),
  (intptr_t)"fuelType",

  offsetof(j2735VehicleClassification, regional) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleClassification_1,
  offsetof(j2735VehicleClassification, regional_option),
  (intptr_t)"regional",

};

const ASN1CType asn1_type_j2735VehicleClassification_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x1,
  0x4,
  sizeof(j2735RegionalExtension_42),
  (intptr_t)asn1_type_j2735RegionalExtension_42,
  0,
};

const ASN1CType asn1_type_j2735_local_549[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735VehicleHeight,
};

const ASN1CType asn1_type_j2735_local_550[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735BumperHeights,
};

const ASN1CType asn1_type_j2735_local_551[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735VehicleMass,
};

const ASN1CType asn1_type_j2735_local_552[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735TrailerWeight,
};

const ASN1CType asn1_type_j2735VehicleData[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  4,
  sizeof(j2735VehicleData),

  offsetof(j2735VehicleData, height) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_549,
  offsetof(j2735VehicleData, height_option),
  (intptr_t)"height",

  offsetof(j2735VehicleData, bumpers) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_550,
  offsetof(j2735VehicleData, bumpers_option),
  (intptr_t)"bumpers",

  offsetof(j2735VehicleData, mass) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_551,
  offsetof(j2735VehicleData, mass_option),
  (intptr_t)"mass",

  offsetof(j2735VehicleData, trailerWeight) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_552,
  offsetof(j2735VehicleData, trailerWeight_option),
  (intptr_t)"trailerWeight",

};

const ASN1CType asn1_type_j2735_local_553[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DescriptiveName,
};

const ASN1CType asn1_type_j2735_local_554[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  ASN1_CSTR_IA5String,
  0x1,
  0x20,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735_local_555[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100003,
  (intptr_t)asn1_type_j2735VehicleID,
};

const ASN1CType asn1_type_j2735_local_556[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735VehicleType,
};

const ASN1CType asn1_type_j2735VehicleIdent[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735VehicleIdent),

  offsetof(j2735VehicleIdent, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_553,
  offsetof(j2735VehicleIdent, name_option),
  (intptr_t)"name",

  offsetof(j2735VehicleIdent, vin) | 0x8000000,
  (intptr_t)asn1_type_j2735VINstring,
  offsetof(j2735VehicleIdent, vin_option),
  (intptr_t)"vin",

  offsetof(j2735VehicleIdent, ownerCode) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_554,
  offsetof(j2735VehicleIdent, ownerCode_option),
  (intptr_t)"ownerCode",

  offsetof(j2735VehicleIdent, id) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_555,
  offsetof(j2735VehicleIdent, id_option),
  (intptr_t)"id",

  offsetof(j2735VehicleIdent, vehicleType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_556,
  offsetof(j2735VehicleIdent, vehicleType_option),
  (intptr_t)"vehicleType",

  offsetof(j2735VehicleIdent, vehicleClass) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleIdent_1,
  offsetof(j2735VehicleIdent, vehicleClass_option),
  (intptr_t)"vehicleClass",

};

const ASN1CType asn1_type_j2735_local_557[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735VehicleGroupAffected,
};

const ASN1CType asn1_type_j2735_local_558[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735ResponderGroupAffected,
};

const ASN1CType asn1_type_j2735_local_559[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735IncidentResponseEquipment,
};

const ASN1CType asn1_type_j2735VehicleIdent_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  3,
  sizeof(j2735VehicleIdent_1),
  offsetof(j2735VehicleIdent_1, choice),
  offsetof(j2735VehicleIdent_1, u),
  (intptr_t)asn1_type_j2735_local_557,
  (intptr_t)"vGroup",
  (intptr_t)asn1_type_j2735_local_558,
  (intptr_t)"rGroup",
  (intptr_t)asn1_type_j2735_local_559,
  (intptr_t)"rEquip",
};

const ASN1CType asn1_type_j2735_local_560[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735TemporaryID,
};

const ASN1CType asn1_type_j2735VehicleID[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x0,
  2,
  sizeof(j2735VehicleID),
  offsetof(j2735VehicleID, choice),
  offsetof(j2735VehicleID, u),
  (intptr_t)asn1_type_j2735_local_560,
  (intptr_t)"entityID",
  (intptr_t)asn1_type_j2735StationID,
  (intptr_t)"stationID",
};

const ASN1CType asn1_type_j2735_local_561[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735VehicleEventFlags,
};

const ASN1CType asn1_type_j2735_local_562[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735PathHistory,
};

const ASN1CType asn1_type_j2735_local_563[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735PathPrediction,
};

const ASN1CType asn1_type_j2735_local_564[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735ExteriorLights,
};

const ASN1CType asn1_type_j2735VehicleSafetyExtensions[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  4,
  sizeof(j2735VehicleSafetyExtensions),

  offsetof(j2735VehicleSafetyExtensions, events) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_561,
  offsetof(j2735VehicleSafetyExtensions, events_option),
  (intptr_t)"events",

  offsetof(j2735VehicleSafetyExtensions, pathHistory) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_562,
  offsetof(j2735VehicleSafetyExtensions, pathHistory_option),
  (intptr_t)"pathHistory",

  offsetof(j2735VehicleSafetyExtensions, pathPrediction) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_563,
  offsetof(j2735VehicleSafetyExtensions, pathPrediction_option),
  (intptr_t)"pathPrediction",

  offsetof(j2735VehicleSafetyExtensions, lights) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_564,
  offsetof(j2735VehicleSafetyExtensions, lights_option),
  (intptr_t)"lights",

};

const ASN1CType asn1_type_j2735_local_565[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735VehicleWidth,
};

const ASN1CType asn1_type_j2735_local_566[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735VehicleLength,
};

const ASN1CType asn1_type_j2735VehicleSize[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000d,
  2,
  sizeof(j2735VehicleSize),

  offsetof(j2735VehicleSize, width) | 0x0,
  (intptr_t)asn1_type_j2735_local_565,
  0,
  (intptr_t)"width",

  offsetof(j2735VehicleSize, length) | 0x0,
  (intptr_t)asn1_type_j2735_local_566,
  0,
  (intptr_t)"length",

};

const ASN1CType asn1_type_j2735_local_567[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0xf,
};

const ASN1CType asn1_type_j2735_local_568[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_569[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_570[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
};

const ASN1CType asn1_type_j2735VehicleStatusRequest[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  5,
  sizeof(j2735VehicleStatusRequest),

  offsetof(j2735VehicleStatusRequest, dataType) | 0x0,
  (intptr_t)asn1_type_j2735VehicleStatusDeviceTypeTag,
  0,
  (intptr_t)"dataType",

  offsetof(j2735VehicleStatusRequest, subType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_567,
  offsetof(j2735VehicleStatusRequest, subType_option),
  (intptr_t)"subType",

  offsetof(j2735VehicleStatusRequest, sendOnLessThenValue) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_568,
  offsetof(j2735VehicleStatusRequest, sendOnLessThenValue_option),
  (intptr_t)"sendOnLessThenValue",

  offsetof(j2735VehicleStatusRequest, sendOnMoreThenValue) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_569,
  offsetof(j2735VehicleStatusRequest, sendOnMoreThenValue_option),
  (intptr_t)"sendOnMoreThenValue",

  offsetof(j2735VehicleStatusRequest, sendAll) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_570,
  offsetof(j2735VehicleStatusRequest, sendAll_option),
  (intptr_t)"sendAll",

};

const ASN1CType asn1_type_j2735VehicleStatusRequestList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x1,
  0x20,
  sizeof(j2735VehicleStatusRequest),
  (intptr_t)asn1_type_j2735VehicleStatusRequest,
  0,
};

const ASN1CType asn1_type_j2735_local_571[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ExteriorLights,
};

const ASN1CType asn1_type_j2735_local_572[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735LightbarInUse,
};

const ASN1CType asn1_type_j2735_local_573[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735BrakeSystemStatus,
};

const ASN1CType asn1_type_j2735_local_574[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735AmbientAirTemperature,
};

const ASN1CType asn1_type_j2735_local_575[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  (intptr_t)asn1_type_j2735AmbientAirPressure,
};

const ASN1CType asn1_type_j2735_local_576[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000d,
  (intptr_t)asn1_type_j2735FullPositionVector,
};

const ASN1CType asn1_type_j2735_local_577[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000f,
  (intptr_t)asn1_type_j2735SpeedandHeadingandThrottleConfidence,
};

const ASN1CType asn1_type_j2735_local_578[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100010,
  (intptr_t)asn1_type_j2735SpeedConfidence,
};

const ASN1CType asn1_type_j2735_local_579[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100012,
  (intptr_t)asn1_type_j2735VehicleIdent,
};

const ASN1CType asn1_type_j2735_local_580[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100015,
  (intptr_t)asn1_type_j2735GNSSstatus,
};

const ASN1CType asn1_type_j2735VehicleStatus[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  22,
  sizeof(j2735VehicleStatus),

  offsetof(j2735VehicleStatus, lights) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_571,
  offsetof(j2735VehicleStatus, lights_option),
  (intptr_t)"lights",

  offsetof(j2735VehicleStatus, lightBar) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_572,
  offsetof(j2735VehicleStatus, lightBar_option),
  (intptr_t)"lightBar",

  offsetof(j2735VehicleStatus, wipers) | 0x8000000,
  (intptr_t)asn1_type_j2735WiperSet,
  offsetof(j2735VehicleStatus, wipers_option),
  (intptr_t)"wipers",

  offsetof(j2735VehicleStatus, brakeStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_573,
  offsetof(j2735VehicleStatus, brakeStatus_option),
  (intptr_t)"brakeStatus",

  offsetof(j2735VehicleStatus, brakePressure) | 0x8000000,
  (intptr_t)asn1_type_j2735BrakeAppliedPressure,
  offsetof(j2735VehicleStatus, brakePressure_option),
  (intptr_t)"brakePressure",

  offsetof(j2735VehicleStatus, roadFriction) | 0x8000000,
  (intptr_t)asn1_type_j2735CoefficientOfFriction,
  offsetof(j2735VehicleStatus, roadFriction_option),
  (intptr_t)"roadFriction",

  offsetof(j2735VehicleStatus, sunData) | 0x8000000,
  (intptr_t)asn1_type_j2735SunSensor,
  offsetof(j2735VehicleStatus, sunData_option),
  (intptr_t)"sunData",

  offsetof(j2735VehicleStatus, rainData) | 0x8000000,
  (intptr_t)asn1_type_j2735RainSensor,
  offsetof(j2735VehicleStatus, rainData_option),
  (intptr_t)"rainData",

  offsetof(j2735VehicleStatus, airTemp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_574,
  offsetof(j2735VehicleStatus, airTemp_option),
  (intptr_t)"airTemp",

  offsetof(j2735VehicleStatus, airPres) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_575,
  offsetof(j2735VehicleStatus, airPres_option),
  (intptr_t)"airPres",

  offsetof(j2735VehicleStatus, steering) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleStatus_1,
  offsetof(j2735VehicleStatus, steering_option),
  (intptr_t)"steering",

  offsetof(j2735VehicleStatus, accelSets) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleStatus_2,
  offsetof(j2735VehicleStatus, accelSets_option),
  (intptr_t)"accelSets",

  offsetof(j2735VehicleStatus, object) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleStatus_3,
  offsetof(j2735VehicleStatus, object_option),
  (intptr_t)"object",

  offsetof(j2735VehicleStatus, fullPos) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_576,
  offsetof(j2735VehicleStatus, fullPos_option),
  (intptr_t)"fullPos",

  offsetof(j2735VehicleStatus, throttlePos) | 0x8000000,
  (intptr_t)asn1_type_j2735ThrottlePosition,
  offsetof(j2735VehicleStatus, throttlePos_option),
  (intptr_t)"throttlePos",

  offsetof(j2735VehicleStatus, speedHeadC) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_577,
  offsetof(j2735VehicleStatus, speedHeadC_option),
  (intptr_t)"speedHeadC",

  offsetof(j2735VehicleStatus, speedC) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_578,
  offsetof(j2735VehicleStatus, speedC_option),
  (intptr_t)"speedC",

  offsetof(j2735VehicleStatus, vehicleData) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleStatus_4,
  offsetof(j2735VehicleStatus, vehicleData_option),
  (intptr_t)"vehicleData",

  offsetof(j2735VehicleStatus, vehicleIdent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_579,
  offsetof(j2735VehicleStatus, vehicleIdent_option),
  (intptr_t)"vehicleIdent",

  offsetof(j2735VehicleStatus, j1939data) | 0x8000000,
  (intptr_t)asn1_type_j2735J1939data,
  offsetof(j2735VehicleStatus, j1939data_option),
  (intptr_t)"j1939data",

  offsetof(j2735VehicleStatus, weatherReport) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleStatus_5,
  offsetof(j2735VehicleStatus, weatherReport_option),
  (intptr_t)"weatherReport",

  offsetof(j2735VehicleStatus, gnssStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_580,
  offsetof(j2735VehicleStatus, gnssStatus_option),
  (intptr_t)"gnssStatus",

};

const ASN1CType asn1_type_j2735_local_581[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735SteeringWheelAngle,
};

const ASN1CType asn1_type_j2735_local_582[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735SteeringWheelAngleConfidence,
};

const ASN1CType asn1_type_j2735_local_583[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735DrivingWheelAngle,
};

const ASN1CType asn1_type_j2735VehicleStatus_1[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000a,
  4,
  sizeof(j2735VehicleStatus_1),

  offsetof(j2735VehicleStatus_1, angle) | 0x0,
  (intptr_t)asn1_type_j2735_local_581,
  0,
  (intptr_t)"angle",

  offsetof(j2735VehicleStatus_1, confidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_582,
  offsetof(j2735VehicleStatus_1, confidence_option),
  (intptr_t)"confidence",

  offsetof(j2735VehicleStatus_1, rate) | 0x8000000,
  (intptr_t)asn1_type_j2735SteeringWheelAngleRateOfChange,
  offsetof(j2735VehicleStatus_1, rate_option),
  (intptr_t)"rate",

  offsetof(j2735VehicleStatus_1, wheels) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_583,
  offsetof(j2735VehicleStatus_1, wheels_option),
  (intptr_t)"wheels",

};

const ASN1CType asn1_type_j2735_local_584[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735AccelerationSet4Way,
};

const ASN1CType asn1_type_j2735_local_585[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735VerticalAccelerationThreshold,
};

const ASN1CType asn1_type_j2735_local_586[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735YawRateConfidence,
};

const ASN1CType asn1_type_j2735_local_587[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735AccelerationConfidence,
};

const ASN1CType asn1_type_j2735VehicleStatus_2[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000b,
  5,
  sizeof(j2735VehicleStatus_2),

  offsetof(j2735VehicleStatus_2, accel4way) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_584,
  offsetof(j2735VehicleStatus_2, accel4way_option),
  (intptr_t)"accel4way",

  offsetof(j2735VehicleStatus_2, vertAccelThres) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_585,
  offsetof(j2735VehicleStatus_2, vertAccelThres_option),
  (intptr_t)"vertAccelThres",

  offsetof(j2735VehicleStatus_2, yawRateCon) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_586,
  offsetof(j2735VehicleStatus_2, yawRateCon_option),
  (intptr_t)"yawRateCon",

  offsetof(j2735VehicleStatus_2, hozAccelCon) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_587,
  offsetof(j2735VehicleStatus_2, hozAccelCon_option),
  (intptr_t)"hozAccelCon",

  offsetof(j2735VehicleStatus_2, confidenceSet) | 0x8000000,
  (intptr_t)asn1_type_j2735ConfidenceSet,
  offsetof(j2735VehicleStatus_2, confidenceSet_option),
  (intptr_t)"confidenceSet",

};

const ASN1CType asn1_type_j2735_local_588[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ObstacleDistance,
};

const ASN1CType asn1_type_j2735_local_589[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Angle,
};

const ASN1CType asn1_type_j2735_local_590[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735DDateTime,
};

const ASN1CType asn1_type_j2735VehicleStatus_3[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000c,
  3,
  sizeof(j2735VehicleStatus_3),

  offsetof(j2735VehicleStatus_3, obDist) | 0x0,
  (intptr_t)asn1_type_j2735_local_588,
  0,
  (intptr_t)"obDist",

  offsetof(j2735VehicleStatus_3, obDirect) | 0x0,
  (intptr_t)asn1_type_j2735_local_589,
  0,
  (intptr_t)"obDirect",

  offsetof(j2735VehicleStatus_3, dateTime) | 0x0,
  (intptr_t)asn1_type_j2735_local_590,
  0,
  (intptr_t)"dateTime",

};

const ASN1CType asn1_type_j2735_local_591[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735VehicleHeight,
};

const ASN1CType asn1_type_j2735_local_592[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735BumperHeights,
};

const ASN1CType asn1_type_j2735_local_593[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735VehicleMass,
};

const ASN1CType asn1_type_j2735_local_594[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735TrailerWeight,
};

const ASN1CType asn1_type_j2735_local_595[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735VehicleType,
};

const ASN1CType asn1_type_j2735VehicleStatus_4[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100011,
  5,
  sizeof(j2735VehicleStatus_4),

  offsetof(j2735VehicleStatus_4, height) | 0x0,
  (intptr_t)asn1_type_j2735_local_591,
  0,
  (intptr_t)"height",

  offsetof(j2735VehicleStatus_4, bumpers) | 0x0,
  (intptr_t)asn1_type_j2735_local_592,
  0,
  (intptr_t)"bumpers",

  offsetof(j2735VehicleStatus_4, mass) | 0x0,
  (intptr_t)asn1_type_j2735_local_593,
  0,
  (intptr_t)"mass",

  offsetof(j2735VehicleStatus_4, trailerWeight) | 0x0,
  (intptr_t)asn1_type_j2735_local_594,
  0,
  (intptr_t)"trailerWeight",

  offsetof(j2735VehicleStatus_4, type) | 0x0,
  (intptr_t)asn1_type_j2735_local_595,
  0,
  (intptr_t)"type",

};

const ASN1CType asn1_type_j2735VehicleStatus_5[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100014,
  5,
  sizeof(j2735VehicleStatus_5),

  offsetof(j2735VehicleStatus_5, isRaining) | 0x0,
  (intptr_t)asn1_type_j2735EssPrecipYesNo,
  0,
  (intptr_t)"isRaining",

  offsetof(j2735VehicleStatus_5, rainRate) | 0x8000000,
  (intptr_t)asn1_type_j2735EssPrecipRate,
  offsetof(j2735VehicleStatus_5, rainRate_option),
  (intptr_t)"rainRate",

  offsetof(j2735VehicleStatus_5, precipSituation) | 0x8000000,
  (intptr_t)asn1_type_j2735EssPrecipSituation,
  offsetof(j2735VehicleStatus_5, precipSituation_option),
  (intptr_t)"precipSituation",

  offsetof(j2735VehicleStatus_5, solarRadiation) | 0x8000000,
  (intptr_t)asn1_type_j2735EssSolarRadiation,
  offsetof(j2735VehicleStatus_5, solarRadiation_option),
  (intptr_t)"solarRadiation",

  offsetof(j2735VehicleStatus_5, friction) | 0x8000000,
  (intptr_t)asn1_type_j2735EssMobileFriction,
  offsetof(j2735VehicleStatus_5, friction_option),
  (intptr_t)"friction",

};

const ASN1CType asn1_type_j2735_local_596[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735AmbientAirTemperature,
};

const ASN1CType asn1_type_j2735_local_597[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735AmbientAirPressure,
};

const ASN1CType asn1_type_j2735WeatherProbe[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100004,
  3,
  sizeof(j2735WeatherProbe),

  offsetof(j2735WeatherProbe, airTemp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_596,
  offsetof(j2735WeatherProbe, airTemp_option),
  (intptr_t)"airTemp",

  offsetof(j2735WeatherProbe, airPressure) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_597,
  offsetof(j2735WeatherProbe, airPressure_option),
  (intptr_t)"airPressure",

  offsetof(j2735WeatherProbe, rainRates) | 0x8000000,
  (intptr_t)asn1_type_j2735WiperSet,
  offsetof(j2735WeatherProbe, rainRates_option),
  (intptr_t)"rainRates",

};

const ASN1CType asn1_type_j2735WeatherReport[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100003,
  6,
  sizeof(j2735WeatherReport),

  offsetof(j2735WeatherReport, isRaining) | 0x0,
  (intptr_t)asn1_type_j2735EssPrecipYesNo,
  0,
  (intptr_t)"isRaining",

  offsetof(j2735WeatherReport, rainRate) | 0x8000000,
  (intptr_t)asn1_type_j2735EssPrecipRate,
  offsetof(j2735WeatherReport, rainRate_option),
  (intptr_t)"rainRate",

  offsetof(j2735WeatherReport, precipSituation) | 0x8000000,
  (intptr_t)asn1_type_j2735EssPrecipSituation,
  offsetof(j2735WeatherReport, precipSituation_option),
  (intptr_t)"precipSituation",

  offsetof(j2735WeatherReport, solarRadiation) | 0x8000000,
  (intptr_t)asn1_type_j2735EssSolarRadiation,
  offsetof(j2735WeatherReport, solarRadiation_option),
  (intptr_t)"solarRadiation",

  offsetof(j2735WeatherReport, friction) | 0x8000000,
  (intptr_t)asn1_type_j2735EssMobileFriction,
  offsetof(j2735WeatherReport, friction_option),
  (intptr_t)"friction",

  offsetof(j2735WeatherReport, roadFriction) | 0x8000000,
  (intptr_t)asn1_type_j2735CoefficientOfFriction,
  offsetof(j2735WeatherReport, roadFriction_option),
  (intptr_t)"roadFriction",

};

const ASN1CType asn1_type_j2735_local_598[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735WiperStatus,
};

const ASN1CType asn1_type_j2735_local_599[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735WiperRate,
};

const ASN1CType asn1_type_j2735_local_600[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735WiperStatus,
};

const ASN1CType asn1_type_j2735_local_601[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735WiperRate,
};

const ASN1CType asn1_type_j2735WiperSet[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  4,
  sizeof(j2735WiperSet),

  offsetof(j2735WiperSet, statusFront) | 0x0,
  (intptr_t)asn1_type_j2735_local_598,
  0,
  (intptr_t)"statusFront",

  offsetof(j2735WiperSet, rateFront) | 0x0,
  (intptr_t)asn1_type_j2735_local_599,
  0,
  (intptr_t)"rateFront",

  offsetof(j2735WiperSet, statusRear) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_600,
  offsetof(j2735WiperSet, statusRear_option),
  (intptr_t)"statusRear",

  offsetof(j2735WiperSet, rateRear) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_601,
  offsetof(j2735WiperSet, rateRear_option),
  (intptr_t)"rateRear",

};

const ASN1CType asn1_type_j2735Acceleration[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xfffff830,
  0x7d1,
};

const ASN1CType asn1_type_j2735AccelerationConfidence[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  8,
  (intptr_t)"unavailable",
  (intptr_t)"accl-100-00",
  (intptr_t)"accl-010-00",
  (intptr_t)"accl-005-00",
  (intptr_t)"accl-001-00",
  (intptr_t)"accl-000-10",
  (intptr_t)"accl-000-05",
  (intptr_t)"accl-000-01",
};

const ASN1CType asn1_type_j2735AdvisorySpeedType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  4,
  0,
  (intptr_t)"none",
  (intptr_t)"greenwave",
  (intptr_t)"ecoDrive",
  (intptr_t)"transit",
};

const ASN1CType asn1_type_j2735AllowedManeuvers[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x3,
  0xc,
  0xc,
};

const ASN1CType asn1_type_j2735AmbientAirPressure[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735AmbientAirTemperature[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xbf,
};

const ASN1CType asn1_type_j2735Angle[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x7080,
};

const ASN1CType asn1_type_j2735AnimalPropelledType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  4,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"otherTypes",
  (intptr_t)"animalMounted",
  (intptr_t)"animalDrawnCarriage",
};

const ASN1CType asn1_type_j2735AnimalType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100018,
  4,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"serviceUse",
  (intptr_t)"pet",
  (intptr_t)"farm",
};

const ASN1CType asn1_type_j2735AntiLockBrakeStatus[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  4,
  (intptr_t)"unavailable",
  (intptr_t)"off",
  (intptr_t)"on",
  (intptr_t)"engaged",
};

const ASN1CType asn1_type_j2735ApproachID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xf,
};

const ASN1CType asn1_type_j2735Attachment[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100016,
  7,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"stroller",
  (intptr_t)"bicycleTrailer",
  (intptr_t)"cart",
  (intptr_t)"wheelchair",
  (intptr_t)"otherWalkAssistAttachments",
  (intptr_t)"pet",
};

const ASN1CType asn1_type_j2735AttachmentRadius[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100017,
  0x0,
  0xc8,
};

const ASN1CType asn1_type_j2735AuxiliaryBrakeStatus[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  4,
  (intptr_t)"unavailable",
  (intptr_t)"off",
  (intptr_t)"on",
  (intptr_t)"reserved",
};

const ASN1CType asn1_type_j2735BasicVehicleClass[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735BasicVehicleRole[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  23,
  0,
  (intptr_t)"basicVehicle",
  (intptr_t)"publicTransport",
  (intptr_t)"specialTransport",
  (intptr_t)"dangerousGoods",
  (intptr_t)"roadWork",
  (intptr_t)"roadRescue",
  (intptr_t)"emergency",
  (intptr_t)"safetyCar",
  (intptr_t)"none-unknown",
  (intptr_t)"truck",
  (intptr_t)"motorcycle",
  (intptr_t)"roadSideSource",
  (intptr_t)"police",
  (intptr_t)"fire",
  (intptr_t)"ambulance",
  (intptr_t)"dot",
  (intptr_t)"transit",
  (intptr_t)"slowMoving",
  (intptr_t)"stopNgo",
  (intptr_t)"cyclist",
  (intptr_t)"pedestrian",
  (intptr_t)"nonMotorized",
  (intptr_t)"military",
};

const ASN1CType asn1_type_j2735BrakeAppliedPressure[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  16,
  (intptr_t)"unavailable",
  (intptr_t)"minPressure",
  (intptr_t)"bkLvl-2",
  (intptr_t)"bkLvl-3",
  (intptr_t)"bkLvl-4",
  (intptr_t)"bkLvl-5",
  (intptr_t)"bkLvl-6",
  (intptr_t)"bkLvl-7",
  (intptr_t)"bkLvl-8",
  (intptr_t)"bkLvl-9",
  (intptr_t)"bkLvl-10",
  (intptr_t)"bkLvl-11",
  (intptr_t)"bkLvl-12",
  (intptr_t)"bkLvl-13",
  (intptr_t)"bkLvl-14",
  (intptr_t)"maxPressure",
};

const ASN1CType asn1_type_j2735BrakeAppliedStatus[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x5,
  0x5,
};

const ASN1CType asn1_type_j2735BrakeBoostApplied[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  3,
  (intptr_t)"unavailable",
  (intptr_t)"off",
  (intptr_t)"on",
};

const ASN1CType asn1_type_j2735BumperHeight[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735CoarseHeading[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xf0,
};

const ASN1CType asn1_type_j2735CoefficientOfFriction[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x0,
  0x32,
};

const ASN1CType asn1_type_j2735Confidence[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xc8,
};

const ASN1CType asn1_type_j2735DDay[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x0,
  0x1f,
};

const ASN1CType asn1_type_j2735DeltaAngle[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0xffffff6a,
  0x96,
};

const ASN1CType asn1_type_j2735DeltaTime[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0xffffff86,
  0x79,
};

const ASN1CType asn1_type_j2735DescriptiveName[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x16,
  ASN1_CSTR_IA5String,
  0x1,
  0x3f,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735DHour[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x0,
  0x1f,
};

const ASN1CType asn1_type_j2735DirectionOfUse[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  4,
  (intptr_t)"unavailable",
  (intptr_t)"forward",
  (intptr_t)"reverse",
  (intptr_t)"both",
};

const ASN1CType asn1_type_j2735DistanceUnits[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  8,
  (intptr_t)"centimeter",
  (intptr_t)"cm2-5",
  (intptr_t)"decimeter",
  (intptr_t)"meter",
  (intptr_t)"kilometer",
  (intptr_t)"foot",
  (intptr_t)"yard",
  (intptr_t)"mile",
};

const ASN1CType asn1_type_j2735DMinute[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x0,
  0x3c,
};

const ASN1CType asn1_type_j2735DMonth[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xc,
};

const ASN1CType asn1_type_j2735DOffset[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0xfffffcb8,
  0x348,
};

const ASN1CType asn1_type_j2735DrivenLineOffsetLg[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735DrivenLineOffsetSm[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0xfffff801,
  0x7ff,
};

const ASN1CType asn1_type_j2735DrivingWheelAngle[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xffffff80,
  0x7f,
};

const ASN1CType asn1_type_j2735DSecond[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735DSRCmsgID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x7fff,
};

const ASN1CType asn1_type_j2735DYear[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xfff,
};

const ASN1CType asn1_type_j2735ElevationConfidence[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  16,
  (intptr_t)"unavailable",
  (intptr_t)"elev-500-00",
  (intptr_t)"elev-200-00",
  (intptr_t)"elev-100-00",
  (intptr_t)"elev-050-00",
  (intptr_t)"elev-020-00",
  (intptr_t)"elev-010-00",
  (intptr_t)"elev-005-00",
  (intptr_t)"elev-002-00",
  (intptr_t)"elev-001-00",
  (intptr_t)"elev-000-50",
  (intptr_t)"elev-000-20",
  (intptr_t)"elev-000-10",
  (intptr_t)"elev-000-05",
  (intptr_t)"elev-000-02",
  (intptr_t)"elev-000-01",
};

const ASN1CType asn1_type_j2735Elevation[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xfffff000,
  0xefff,
};

const ASN1CType asn1_type_j2735Extent[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  16,
  (intptr_t)"useInstantlyOnly",
  (intptr_t)"useFor3meters",
  (intptr_t)"useFor10meters",
  (intptr_t)"useFor50meters",
  (intptr_t)"useFor100meters",
  (intptr_t)"useFor500meters",
  (intptr_t)"useFor1000meters",
  (intptr_t)"useFor5000meters",
  (intptr_t)"useFor10000meters",
  (intptr_t)"useFor50000meters",
  (intptr_t)"useFor100000meters",
  (intptr_t)"useFor500000meters",
  (intptr_t)"useFor1000000meters",
  (intptr_t)"useFor5000000meters",
  (intptr_t)"useFor10000000meters",
  (intptr_t)"forever",
};

const ASN1CType asn1_type_j2735ExteriorLights[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x3,
  0x9,
  0x9,
};

const ASN1CType asn1_type_j2735FuelType[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xf,
};

const ASN1CType asn1_type_j2735FurtherInfoID[] = {
  (ASN1_CTYPE_OCTET_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x4,
  0x2,
  0x2,
};

const ASN1CType asn1_type_j2735GNSSstatus[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x3,
  0x8,
  0x8,
};

const ASN1CType asn1_type_j2735GrossDistance[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x3ff,
};

const ASN1CType asn1_type_j2735GrossSpeed[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x1f,
};

const ASN1CType asn1_type_j2735HeadingConfidence[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  8,
  (intptr_t)"unavailable",
  (intptr_t)"prec10deg",
  (intptr_t)"prec05deg",
  (intptr_t)"prec01deg",
  (intptr_t)"prec0-1deg",
  (intptr_t)"prec0-05deg",
  (intptr_t)"prec0-01deg",
  (intptr_t)"prec0-0125deg",
};

const ASN1CType asn1_type_j2735Heading[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x7080,
};

const ASN1CType asn1_type_j2735HeadingSlice[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x3,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735IntersectionID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735IntersectionStatusObject[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735IsDolly[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
};

const ASN1CType asn1_type_j2735Iso3833VehicleType[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x64,
};

const ASN1CType asn1_type_j2735ITIStextPhrase[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  ASN1_CSTR_IA5String,
  0x1,
  0x10,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735AxleLocation[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735AxleWeight[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xfaff,
};

const ASN1CType asn1_type_j2735CargoWeight[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x0,
  0xfaff,
};

const ASN1CType asn1_type_j2735DriveAxleLiftAirPressure[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x0,
  0x3e8,
};

const ASN1CType asn1_type_j2735DriveAxleLocation[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735DriveAxleLubePressure[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x0,
  0xfa,
};

const ASN1CType asn1_type_j2735DriveAxleTemperature[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0xffffffd8,
  0xd2,
};

const ASN1CType asn1_type_j2735SteeringAxleLubePressure[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100009,
  0x0,
  0xfa,
};

const ASN1CType asn1_type_j2735SteeringAxleTemperature[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0xffffffd8,
  0xd2,
};

const ASN1CType asn1_type_j2735TireLeakageRate[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x0,
  0xfaff,
};

const ASN1CType asn1_type_j2735TireLocation[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735TirePressureThresholdDetection[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  8,
  (intptr_t)"noData",
  (intptr_t)"overPressure",
  (intptr_t)"noWarningPressure",
  (intptr_t)"underPressure",
  (intptr_t)"extremeUnderPressure",
  (intptr_t)"undefined",
  (intptr_t)"errorIndicator",
  (intptr_t)"notAvailable",
};

const ASN1CType asn1_type_j2735TirePressure[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xfa,
};

const ASN1CType asn1_type_j2735TireTemp[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0xffffdde0,
  0xd8df,
};

const ASN1CType asn1_type_j2735TrailerWeight[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xfaff,
};

const ASN1CType asn1_type_j2735WheelEndElectFault[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  4,
  (intptr_t)"isOk",
  (intptr_t)"isNotDefined",
  (intptr_t)"isError",
  (intptr_t)"isNotSupported",
};

const ASN1CType asn1_type_j2735WheelSensorStatus[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  4,
  (intptr_t)"off",
  (intptr_t)"on",
  (intptr_t)"notDefined",
  (intptr_t)"notSupported",
};

const ASN1CType asn1_type_j2735LaneAttributes_Barrier[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735LaneAttributes_Bike[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735LaneAttributes_Crosswalk[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735LaneAttributes_Parking[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735LaneAttributes_Sidewalk[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735LaneAttributes_Striping[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735LaneAttributes_TrackedVehicle[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735LaneAttributes_Vehicle[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x100000,
  0x8,
  0x8,
};

const ASN1CType asn1_type_j2735LaneConnectionID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735LaneDirection[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x2,
  0x2,
};

const ASN1CType asn1_type_j2735LaneID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735LaneSharing[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0xa,
  0xa,
};

const ASN1CType asn1_type_j2735LaneWidth[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x7fff,
};

const ASN1CType asn1_type_j2735Latitude[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xca5b1700,
  0x35a4e901,
};

const ASN1CType asn1_type_j2735LayerID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x0,
  0x64,
};

const ASN1CType asn1_type_j2735LayerType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  8,
  0,
  (intptr_t)"none",
  (intptr_t)"mixedContent",
  (intptr_t)"generalMapData",
  (intptr_t)"intersectionData",
  (intptr_t)"curveData",
  (intptr_t)"roadwaySectionData",
  (intptr_t)"parkingAreaData",
  (intptr_t)"sharedLaneData",
};

const ASN1CType asn1_type_j2735LightbarInUse[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  8,
  (intptr_t)"unavailable",
  (intptr_t)"notInUse",
  (intptr_t)"inUse",
  (intptr_t)"yellowCautionLights",
  (intptr_t)"schooldBusLights",
  (intptr_t)"arrowSignsActive",
  (intptr_t)"slowMovingVehicle",
  (intptr_t)"freqStops",
};

const ASN1CType asn1_type_j2735Longitude[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x94b62e01,
  0x6b49d201,
};

const ASN1CType asn1_type_j2735MergeDivergeNodeAngle[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0xffffff4c,
  0xb4,
};

const ASN1CType asn1_type_j2735MinuteOfTheYear[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x80ac0,
};

const ASN1CType asn1_type_j2735MinutesDuration[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x0,
  0x7d00,
};

const ASN1CType asn1_type_j2735MotorizedPropelledType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  6,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"otherTypes",
  (intptr_t)"wheelChair",
  (intptr_t)"bicycle",
  (intptr_t)"scooter",
  (intptr_t)"selfBalancingDevice",
};

const ASN1CType asn1_type_j2735MovementPhaseState[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  10,
  (intptr_t)"unavailable",
  (intptr_t)"dark",
  (intptr_t)"stop-Then-Proceed",
  (intptr_t)"stop-And-Remain",
  (intptr_t)"pre-Movement",
  (intptr_t)"permissive-Movement-Allowed",
  (intptr_t)"protected-Movement-Allowed",
  (intptr_t)"permissive-clearance",
  (intptr_t)"protected-clearance",
  (intptr_t)"caution-Conflicting-Traffic",
};

const ASN1CType asn1_type_j2735MsgCount[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735MsgCRC[] = {
  (ASN1_CTYPE_OCTET_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x2,
  0x2,
};

const ASN1CType asn1_type_j2735MultiVehicleResponse[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  4,
  (intptr_t)"unavailable",
  (intptr_t)"singleVehicle",
  (intptr_t)"multiVehicle",
  (intptr_t)"reserved",
};

const ASN1CType asn1_type_j2735MUTCDCode[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  7,
  0,
  (intptr_t)"none",
  (intptr_t)"regulatory",
  (intptr_t)"warning",
  (intptr_t)"maintenance",
  (intptr_t)"motoristService",
  (intptr_t)"guide",
  (intptr_t)"rec",
};

const ASN1CType asn1_type_j2735NMEA_MsgType[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x0,
  0x7fff,
};

const ASN1CType asn1_type_j2735NMEA_Payload[] = {
  (ASN1_CTYPE_OCTET_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x1,
  0x3ff,
};

const ASN1CType asn1_type_j2735NMEA_Revision[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  7,
  0,
  (intptr_t)"unknown",
  (intptr_t)"reserved",
  (intptr_t)"rev1",
  (intptr_t)"rev2",
  (intptr_t)"rev3",
  (intptr_t)"rev4",
  (intptr_t)"rev5",
};

const ASN1CType asn1_type_j2735NodeAttributeLL[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  12,
  0,
  (intptr_t)"reserved",
  (intptr_t)"stopLine",
  (intptr_t)"roundedCapStyleA",
  (intptr_t)"roundedCapStyleB",
  (intptr_t)"mergePoint",
  (intptr_t)"divergePoint",
  (intptr_t)"downstreamStopLine",
  (intptr_t)"downstreamStartNode",
  (intptr_t)"closedToTraffic",
  (intptr_t)"safeIsland",
  (intptr_t)"curbPresentAtStepOff",
  (intptr_t)"hydrantPresent",
};

const ASN1CType asn1_type_j2735NodeAttributeXY[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  12,
  0,
  (intptr_t)"reserved",
  (intptr_t)"stopLine",
  (intptr_t)"roundedCapStyleA",
  (intptr_t)"roundedCapStyleB",
  (intptr_t)"mergePoint",
  (intptr_t)"divergePoint",
  (intptr_t)"downstreamStopLine",
  (intptr_t)"downstreamStartNode",
  (intptr_t)"closedToTraffic",
  (intptr_t)"safeIsland",
  (intptr_t)"curbPresentAtStepOff",
  (intptr_t)"hydrantPresent",
};

const ASN1CType asn1_type_j2735NumberOfParticipantsInCluster[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10000f,
  4,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"small",
  (intptr_t)"medium",
  (intptr_t)"large",
};

const ASN1CType asn1_type_j2735ObjectCount[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x0,
  0x3ff,
};

const ASN1CType asn1_type_j2735ObstacleDirection[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Angle,
};

const ASN1CType asn1_type_j2735ObstacleDistance[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x7fff,
};

const ASN1CType asn1_type_j2735Offset_B09[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0xffffff00,
  0xff,
};

const ASN1CType asn1_type_j2735Offset_B10[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xfffffe00,
  0x1ff,
};

const ASN1CType asn1_type_j2735Offset_B11[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xfffffc00,
  0x3ff,
};

const ASN1CType asn1_type_j2735Offset_B12[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xfffff800,
  0x7ff,
};

const ASN1CType asn1_type_j2735Offset_B13[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xfffff000,
  0xfff,
};

const ASN1CType asn1_type_j2735Offset_B14[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xffffe000,
  0x1fff,
};

const ASN1CType asn1_type_j2735Offset_B16[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xffff8000,
  0x7fff,
};

const ASN1CType asn1_type_j2735OffsetLL_B12[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xfffff800,
  0x7ff,
};

const ASN1CType asn1_type_j2735OffsetLL_B14[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xffffe000,
  0x1fff,
};

const ASN1CType asn1_type_j2735OffsetLL_B16[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xffff8000,
  0x7fff,
};

const ASN1CType asn1_type_j2735OffsetLL_B18[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xfffe0000,
  0x1ffff,
};

const ASN1CType asn1_type_j2735OffsetLL_B22[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xffe00000,
  0x1fffff,
};

const ASN1CType asn1_type_j2735OffsetLL_B24[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xff800000,
  0x7fffff,
};

const ASN1CType asn1_type_j2735PedestrianBicycleDetect[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
};

const ASN1CType asn1_type_j2735HumanPropelledType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  6,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"otherTypes",
  (intptr_t)"onFoot",
  (intptr_t)"skateboard",
  (intptr_t)"pushOrKickScooter",
  (intptr_t)"wheelchair",
};

const ASN1CType asn1_type_j2735PersonalAssistive[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x100014,
  0x6,
  0x6,
};

const ASN1CType asn1_type_j2735PersonalClusterRadius[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100010,
  0x0,
  0x64,
};

const ASN1CType asn1_type_j2735PersonalCrossingInProgress[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000e,
};

const ASN1CType asn1_type_j2735PersonalCrossingRequest[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000d,
};

const ASN1CType asn1_type_j2735PersonalDeviceUsageState[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x10000c,
  0x9,
  0x9,
};

const ASN1CType asn1_type_j2735PersonalDeviceUserType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  5,
  9,
  (intptr_t)"unavailable",
  (intptr_t)"aPEDESTRIAN",
  (intptr_t)"aPEDALCYCLIST",
  (intptr_t)"aPUBLICSAFETYWORKER",
  (intptr_t)"anANIMAL",
  (intptr_t)"aWheelchair",
  (intptr_t)"aSelfBalancingDevice",
  (intptr_t)"aStandingScooter",
  (intptr_t)"aSeatedScooter",
  (intptr_t)"aSkates",
  (intptr_t)"anOneWheelTransporter",
  (intptr_t)"aCargoBike",
  (intptr_t)"aConstructionWorker",
  (intptr_t)"anOther",
};

const ASN1CType asn1_type_j2735PivotingAllowed[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
};

const ASN1CType asn1_type_j2735PositionConfidence[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  16,
  (intptr_t)"unavailable",
  (intptr_t)"a500m",
  (intptr_t)"a200m",
  (intptr_t)"a100m",
  (intptr_t)"a50m",
  (intptr_t)"a20m",
  (intptr_t)"a10m",
  (intptr_t)"a5m",
  (intptr_t)"a2m",
  (intptr_t)"a1m",
  (intptr_t)"a50cm",
  (intptr_t)"a20cm",
  (intptr_t)"a10cm",
  (intptr_t)"a5cm",
  (intptr_t)"a2cm",
  (intptr_t)"a1cm",
};

const ASN1CType asn1_type_j2735PrioritizationResponseStatus[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100006,
  8,
  0,
  (intptr_t)"unknown",
  (intptr_t)"requested",
  (intptr_t)"processing",
  (intptr_t)"watchOtherTraffic",
  (intptr_t)"granted",
  (intptr_t)"rejected",
  (intptr_t)"maxPresence",
  (intptr_t)"reserviceLocked",
};

const ASN1CType asn1_type_j2735Priority[] = {
  (ASN1_CTYPE_OCTET_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x4,
  0x1,
  0x1,
};

const ASN1CType asn1_type_j2735PriorityRequestType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  4,
  0,
  (intptr_t)"priorityRequestTypeReserved",
  (intptr_t)"priorityRequest",
  (intptr_t)"priorityRequestUpdate",
  (intptr_t)"priorityCancellation",
};

const ASN1CType asn1_type_j2735PrivilegedEventFlags[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x10,
  0x10,
};

const ASN1CType asn1_type_j2735ProbeSegmentNumber[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0x7fff,
};

const ASN1CType asn1_type_j2735PublicSafetyAndRoadWorkerActivity[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x100012,
  0x6,
  0x6,
};

const ASN1CType asn1_type_j2735PublicSafetyDirectingTrafficSubType[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x100013,
  0x7,
  0x7,
};

const ASN1CType asn1_type_j2735PublicSafetyEventResponderWorkerType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100011,
  8,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"towOperater",
  (intptr_t)"fireAndEMSWorker",
  (intptr_t)"aDOTWorker",
  (intptr_t)"lawEnforcement",
  (intptr_t)"hazmatResponder",
  (intptr_t)"animalControlWorker",
  (intptr_t)"otherPersonnel",
};

const ASN1CType asn1_type_j2735RadiusOfCurvature[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735Radius_B12[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xfff,
};

const ASN1CType asn1_type_j2735RainSensor[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  8,
  (intptr_t)"none",
  (intptr_t)"lightMist",
  (intptr_t)"heavyMist",
  (intptr_t)"lightRainOrDrizzle",
  (intptr_t)"rain",
  (intptr_t)"moderateRain",
  (intptr_t)"heavyRain",
  (intptr_t)"heavyDownpour",
};

const ASN1CType asn1_type_j2735RegionId[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735RequestedItem[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  17,
  1,
  (intptr_t)"reserved",
  (intptr_t)"itemA",
  (intptr_t)"itemB",
  (intptr_t)"itemC",
  (intptr_t)"itemD",
  (intptr_t)"itemE",
  (intptr_t)"itemF",
  (intptr_t)"itemG",
  (intptr_t)"itemI",
  (intptr_t)"itemJ",
  (intptr_t)"itemK",
  (intptr_t)"itemL",
  (intptr_t)"itemM",
  (intptr_t)"itemN",
  (intptr_t)"itemO",
  (intptr_t)"itemP",
  (intptr_t)"itemQ",
  (intptr_t)"itemR",
};

const ASN1CType asn1_type_j2735RequestID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735RequestImportanceLevel[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  16,
  (intptr_t)"requestImportanceLevelUnKnown",
  (intptr_t)"requestImportanceLevel1",
  (intptr_t)"requestImportanceLevel2",
  (intptr_t)"requestImportanceLevel3",
  (intptr_t)"requestImportanceLevel4",
  (intptr_t)"requestImportanceLevel5",
  (intptr_t)"requestImportanceLevel6",
  (intptr_t)"requestImportanceLevel7",
  (intptr_t)"requestImportanceLevel8",
  (intptr_t)"requestImportanceLevel9",
  (intptr_t)"requestImportanceLevel10",
  (intptr_t)"requestImportanceLevel11",
  (intptr_t)"requestImportanceLevel12",
  (intptr_t)"requestImportanceLevel13",
  (intptr_t)"requestImportanceLevel14",
  (intptr_t)"requestImportanceReserved",
};

const ASN1CType asn1_type_j2735RequestSubRole[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  16,
  (intptr_t)"requestSubRoleUnKnown",
  (intptr_t)"requestSubRole1",
  (intptr_t)"requestSubRole2",
  (intptr_t)"requestSubRole3",
  (intptr_t)"requestSubRole4",
  (intptr_t)"requestSubRole5",
  (intptr_t)"requestSubRole6",
  (intptr_t)"requestSubRole7",
  (intptr_t)"requestSubRole8",
  (intptr_t)"requestSubRole9",
  (intptr_t)"requestSubRole10",
  (intptr_t)"requestSubRole11",
  (intptr_t)"requestSubRole12",
  (intptr_t)"requestSubRole13",
  (intptr_t)"requestSubRole14",
  (intptr_t)"requestSubRoleReserved",
};

const ASN1CType asn1_type_j2735ResponseType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  7,
  0,
  (intptr_t)"notInUseOrNotEquipped",
  (intptr_t)"emergency",
  (intptr_t)"nonEmergency",
  (intptr_t)"pursuit",
  (intptr_t)"stationary",
  (intptr_t)"slowMoving",
  (intptr_t)"stopAndGoMovement",
};

const ASN1CType asn1_type_j2735RestrictionAppliesTo[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  14,
  0,
  (intptr_t)"none",
  (intptr_t)"equippedTransit",
  (intptr_t)"equippedTaxis",
  (intptr_t)"equippedOther",
  (intptr_t)"emissionCompliant",
  (intptr_t)"equippedBicycle",
  (intptr_t)"weightCompliant",
  (intptr_t)"heightCompliant",
  (intptr_t)"pedestrians",
  (intptr_t)"slowMovingPersons",
  (intptr_t)"wheelchairUsers",
  (intptr_t)"visualDisabilities",
  (intptr_t)"audioDisabilities",
  (intptr_t)"otherUnknownDisabilities",
};

const ASN1CType asn1_type_j2735RestrictionClassID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735RoadRegulatorID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735RoadSegmentID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735RoadwayCrownAngle[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xffffff80,
  0x7f,
};

const ASN1CType asn1_type_j2735RTCM_Revision[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  4,
  0,
  (intptr_t)"unknown",
  (intptr_t)"rtcmRev2",
  (intptr_t)"rtcmRev3",
  (intptr_t)"reserved",
};

const ASN1CType asn1_type_j2735RTCMmessage[] = {
  (ASN1_CTYPE_OCTET_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x4,
  0x1,
  0x3ff,
};

const ASN1CType asn1_type_j2735Scale_B12[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xfffff800,
  0x7ff,
};

const ASN1CType asn1_type_j2735SecondOfTime[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x3d,
};

const ASN1CType asn1_type_j2735SegmentAttributeLL[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  38,
  0,
  (intptr_t)"reserved",
  (intptr_t)"doNotBlock",
  (intptr_t)"whiteLine",
  (intptr_t)"mergingLaneLeft",
  (intptr_t)"mergingLaneRight",
  (intptr_t)"curbOnLeft",
  (intptr_t)"curbOnRight",
  (intptr_t)"loadingzoneOnLeft",
  (intptr_t)"loadingzoneOnRight",
  (intptr_t)"turnOutPointOnLeft",
  (intptr_t)"turnOutPointOnRight",
  (intptr_t)"adjacentParkingOnLeft",
  (intptr_t)"adjacentParkingOnRight",
  (intptr_t)"adjacentBikeLaneOnLeft",
  (intptr_t)"adjacentBikeLaneOnRight",
  (intptr_t)"sharedBikeLane",
  (intptr_t)"bikeBoxInFront",
  (intptr_t)"transitStopOnLeft",
  (intptr_t)"transitStopOnRight",
  (intptr_t)"transitStopInLane",
  (intptr_t)"sharedWithTrackedVehicle",
  (intptr_t)"safeIsland",
  (intptr_t)"lowCurbsPresent",
  (intptr_t)"rumbleStripPresent",
  (intptr_t)"audibleSignalingPresent",
  (intptr_t)"adaptiveTimingPresent",
  (intptr_t)"rfSignalRequestPresent",
  (intptr_t)"partialCurbIntrusion",
  (intptr_t)"taperToLeft",
  (intptr_t)"taperToRight",
  (intptr_t)"taperToCenterLine",
  (intptr_t)"parallelParking",
  (intptr_t)"headInParking",
  (intptr_t)"freeParking",
  (intptr_t)"timeRestrictionsOnParking",
  (intptr_t)"costToPark",
  (intptr_t)"midBlockCurbPresent",
  (intptr_t)"unEvenPavementPresent",
};

const ASN1CType asn1_type_j2735SegmentAttributeXY[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  38,
  0,
  (intptr_t)"reserved",
  (intptr_t)"doNotBlock",
  (intptr_t)"whiteLine",
  (intptr_t)"mergingLaneLeft",
  (intptr_t)"mergingLaneRight",
  (intptr_t)"curbOnLeft",
  (intptr_t)"curbOnRight",
  (intptr_t)"loadingzoneOnLeft",
  (intptr_t)"loadingzoneOnRight",
  (intptr_t)"turnOutPointOnLeft",
  (intptr_t)"turnOutPointOnRight",
  (intptr_t)"adjacentParkingOnLeft",
  (intptr_t)"adjacentParkingOnRight",
  (intptr_t)"adjacentBikeLaneOnLeft",
  (intptr_t)"adjacentBikeLaneOnRight",
  (intptr_t)"sharedBikeLane",
  (intptr_t)"bikeBoxInFront",
  (intptr_t)"transitStopOnLeft",
  (intptr_t)"transitStopOnRight",
  (intptr_t)"transitStopInLane",
  (intptr_t)"sharedWithTrackedVehicle",
  (intptr_t)"safeIsland",
  (intptr_t)"lowCurbsPresent",
  (intptr_t)"rumbleStripPresent",
  (intptr_t)"audibleSignalingPresent",
  (intptr_t)"adaptiveTimingPresent",
  (intptr_t)"rfSignalRequestPresent",
  (intptr_t)"partialCurbIntrusion",
  (intptr_t)"taperToLeft",
  (intptr_t)"taperToRight",
  (intptr_t)"taperToCenterLine",
  (intptr_t)"parallelParking",
  (intptr_t)"headInParking",
  (intptr_t)"freeParking",
  (intptr_t)"timeRestrictionsOnParking",
  (intptr_t)"costToPark",
  (intptr_t)"midBlockCurbPresent",
  (intptr_t)"unEvenPavementPresent",
};

const ASN1CType asn1_type_j2735SemiMajorAxisAccuracy[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735SemiMajorAxisOrientation[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735SemiMinorAxisAccuracy[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735SignalGroupID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735SignPrority[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x0,
  0x7,
};

const ASN1CType asn1_type_j2735SirenInUse[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  4,
  (intptr_t)"unavailable",
  (intptr_t)"notInUse",
  (intptr_t)"inUse",
  (intptr_t)"reserved",
};

const ASN1CType asn1_type_j2735SpeedAdvice[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0x1f4,
};

const ASN1CType asn1_type_j2735SpeedConfidence[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  8,
  (intptr_t)"unavailable",
  (intptr_t)"prec100ms",
  (intptr_t)"prec10ms",
  (intptr_t)"prec5ms",
  (intptr_t)"prec1ms",
  (intptr_t)"prec0-1ms",
  (intptr_t)"prec0-05ms",
  (intptr_t)"prec0-01ms",
};

const ASN1CType asn1_type_j2735SpeedLimitType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  13,
  0,
  (intptr_t)"unknown",
  (intptr_t)"maxSpeedInSchoolZone",
  (intptr_t)"maxSpeedInSchoolZoneWhenChildrenArePresent",
  (intptr_t)"maxSpeedInConstructionZone",
  (intptr_t)"vehicleMinSpeed",
  (intptr_t)"vehicleMaxSpeed",
  (intptr_t)"vehicleNightMaxSpeed",
  (intptr_t)"truckMinSpeed",
  (intptr_t)"truckMaxSpeed",
  (intptr_t)"truckNightMaxSpeed",
  (intptr_t)"vehiclesWithTrailersMinSpeed",
  (intptr_t)"vehiclesWithTrailersMaxSpeed",
  (intptr_t)"vehiclesWithTrailersNightMaxSpeed",
};

const ASN1CType asn1_type_j2735Speed[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x1fff,
};

const ASN1CType asn1_type_j2735SSPindex[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x1f,
};

const ASN1CType asn1_type_j2735StabilityControlStatus[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  4,
  (intptr_t)"unavailable",
  (intptr_t)"off",
  (intptr_t)"on",
  (intptr_t)"engaged",
};

const ASN1CType asn1_type_j2735StationID[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xffffffff,
};

const ASN1CType asn1_type_j2735SteeringWheelAngleConfidence[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  4,
  (intptr_t)"unavailable",
  (intptr_t)"prec2deg",
  (intptr_t)"prec1deg",
  (intptr_t)"prec0-02deg",
};

const ASN1CType asn1_type_j2735SteeringWheelAngleRateOfChange[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0xffffff81,
  0x7f,
};

const ASN1CType asn1_type_j2735SteeringWheelAngle[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xffffff82,
  0x7f,
};

const ASN1CType asn1_type_j2735SunSensor[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x0,
  0x3e8,
};

const ASN1CType asn1_type_j2735TemporaryID[] = {
  (ASN1_CTYPE_OCTET_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x4,
  0x4,
  0x4,
};

const ASN1CType asn1_type_j2735TermDistance[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x7530,
};

const ASN1CType asn1_type_j2735TermTime[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x1,
  0x708,
};

const ASN1CType asn1_type_j2735ThrottleConfidence[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  4,
  (intptr_t)"unavailable",
  (intptr_t)"prec10percent",
  (intptr_t)"prec1percent",
  (intptr_t)"prec0-5percent",
};

const ASN1CType asn1_type_j2735ThrottlePosition[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10000e,
  0x0,
  0xc8,
};

const ASN1CType asn1_type_j2735TimeConfidence[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  40,
  (intptr_t)"unavailable",
  (intptr_t)"time-100-000",
  (intptr_t)"time-050-000",
  (intptr_t)"time-020-000",
  (intptr_t)"time-010-000",
  (intptr_t)"time-002-000",
  (intptr_t)"time-001-000",
  (intptr_t)"time-000-500",
  (intptr_t)"time-000-200",
  (intptr_t)"time-000-100",
  (intptr_t)"time-000-050",
  (intptr_t)"time-000-020",
  (intptr_t)"time-000-010",
  (intptr_t)"time-000-005",
  (intptr_t)"time-000-002",
  (intptr_t)"time-000-001",
  (intptr_t)"time-000-000-5",
  (intptr_t)"time-000-000-2",
  (intptr_t)"time-000-000-1",
  (intptr_t)"time-000-000-05",
  (intptr_t)"time-000-000-02",
  (intptr_t)"time-000-000-01",
  (intptr_t)"time-000-000-005",
  (intptr_t)"time-000-000-002",
  (intptr_t)"time-000-000-001",
  (intptr_t)"time-000-000-000-5",
  (intptr_t)"time-000-000-000-2",
  (intptr_t)"time-000-000-000-1",
  (intptr_t)"time-000-000-000-05",
  (intptr_t)"time-000-000-000-02",
  (intptr_t)"time-000-000-000-01",
  (intptr_t)"time-000-000-000-005",
  (intptr_t)"time-000-000-000-002",
  (intptr_t)"time-000-000-000-001",
  (intptr_t)"time-000-000-000-000-5",
  (intptr_t)"time-000-000-000-000-2",
  (intptr_t)"time-000-000-000-000-1",
  (intptr_t)"time-000-000-000-000-05",
  (intptr_t)"time-000-000-000-000-02",
  (intptr_t)"time-000-000-000-000-01",
};

const ASN1CType asn1_type_j2735TimeIntervalConfidence[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x0,
  0xf,
};

const ASN1CType asn1_type_j2735TimeMark[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x8d0f,
};

const ASN1CType asn1_type_j2735TimeOffset[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x1,
  0xffff,
};

const ASN1CType asn1_type_j2735TractionControlStatus[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  4,
  (intptr_t)"unavailable",
  (intptr_t)"off",
  (intptr_t)"on",
  (intptr_t)"engaged",
};

const ASN1CType asn1_type_j2735TrailerMass[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735TransitVehicleOccupancy[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  8,
  (intptr_t)"occupancyUnknown",
  (intptr_t)"occupancyEmpty",
  (intptr_t)"occupancyVeryLow",
  (intptr_t)"occupancyLow",
  (intptr_t)"occupancyMed",
  (intptr_t)"occupancyHigh",
  (intptr_t)"occupancyNearlyFull",
  (intptr_t)"occupancyFull",
};

const ASN1CType asn1_type_j2735TransitVehicleStatus[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0x8,
  0x8,
};

const ASN1CType asn1_type_j2735TransmissionState[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  8,
  (intptr_t)"neutral",
  (intptr_t)"park",
  (intptr_t)"forwardGears",
  (intptr_t)"reverseGears",
  (intptr_t)"reserved1",
  (intptr_t)"reserved2",
  (intptr_t)"reserved3",
  (intptr_t)"unavailable",
};

const ASN1CType asn1_type_j2735TravelerInfoType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  4,
  0,
  (intptr_t)"unknown",
  (intptr_t)"advisory",
  (intptr_t)"roadSignage",
  (intptr_t)"commercialSignage",
};

const ASN1CType asn1_type_j2735UniqueMSGID[] = {
  (ASN1_CTYPE_OCTET_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x9,
  0x9,
};

const ASN1CType asn1_type_j2735URL_Base[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  ASN1_CSTR_IA5String,
  0x1,
  0x2d,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735URL_Short[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10000c,
  ASN1_CSTR_IA5String,
  0x1,
  0xf,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735UserSizeAndBehaviour[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x100015,
  0x5,
  0x5,
};

const ASN1CType asn1_type_j2735VehicleEventFlags[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x3,
  0xd,
  0xd,
};

const ASN1CType asn1_type_j2735VehicleHeight[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735VehicleLength[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xfff,
};

const ASN1CType asn1_type_j2735VehicleMass[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735VehicleStatusDeviceTypeTag[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  29,
  0,
  (intptr_t)"unknown",
  (intptr_t)"lights",
  (intptr_t)"wipers",
  (intptr_t)"brakes",
  (intptr_t)"stab",
  (intptr_t)"trac",
  (intptr_t)"abs",
  (intptr_t)"sunS",
  (intptr_t)"rainS",
  (intptr_t)"airTemp",
  (intptr_t)"steering",
  (intptr_t)"vertAccelThres",
  (intptr_t)"vertAccel",
  (intptr_t)"hozAccelLong",
  (intptr_t)"hozAccelLat",
  (intptr_t)"hozAccelCon",
  (intptr_t)"accel4way",
  (intptr_t)"confidenceSet",
  (intptr_t)"obDist",
  (intptr_t)"obDirect",
  (intptr_t)"yaw",
  (intptr_t)"yawRateCon",
  (intptr_t)"dateTime",
  (intptr_t)"fullPos",
  (intptr_t)"position2D",
  (intptr_t)"position3D",
  (intptr_t)"vehicle",
  (intptr_t)"speedHeadC",
  (intptr_t)"speedC",
};

const ASN1CType asn1_type_j2735VehicleType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  16,
  0,
  (intptr_t)"none",
  (intptr_t)"unknown",
  (intptr_t)"special",
  (intptr_t)"moto",
  (intptr_t)"car",
  (intptr_t)"carOther",
  (intptr_t)"bus",
  (intptr_t)"axleCnt2",
  (intptr_t)"axleCnt3",
  (intptr_t)"axleCnt4",
  (intptr_t)"axleCnt4Trailer",
  (intptr_t)"axleCnt5Trailer",
  (intptr_t)"axleCnt6Trailer",
  (intptr_t)"axleCnt5MultiTrailer",
  (intptr_t)"axleCnt6MultiTrailer",
  (intptr_t)"axleCnt7MultiTrailer",
};

const ASN1CType asn1_type_j2735VehicleWidth[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x3ff,
};

const ASN1CType asn1_type_j2735Velocity[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x1fff,
};

const ASN1CType asn1_type_j2735VerticalAccelerationThreshold[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x3,
  0x5,
  0x5,
};

const ASN1CType asn1_type_j2735VerticalAcceleration[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0xffffff81,
  0x7f,
};

const ASN1CType asn1_type_j2735VertOffset_B07[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0xffffffc0,
  0x3f,
};

const ASN1CType asn1_type_j2735VertOffset_B12[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0xfffff800,
  0x7ff,
};

const ASN1CType asn1_type_j2735VINstring[] = {
  (ASN1_CTYPE_OCTET_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x11,
};

const ASN1CType asn1_type_j2735WaitOnStopline[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
};

const ASN1CType asn1_type_j2735WiperRate[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735WiperStatus[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  7,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"off",
  (intptr_t)"intermittent",
  (intptr_t)"low",
  (intptr_t)"high",
  (intptr_t)"washerInUse",
  (intptr_t)"automaticPresent",
};

const ASN1CType asn1_type_j2735YawRateConfidence[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0xa,
  8,
  (intptr_t)"unavailable",
  (intptr_t)"degSec-100-00",
  (intptr_t)"degSec-010-00",
  (intptr_t)"degSec-005-00",
  (intptr_t)"degSec-001-00",
  (intptr_t)"degSec-000-10",
  (intptr_t)"degSec-000-05",
  (intptr_t)"degSec-000-01",
};

const ASN1CType asn1_type_j2735YawRate[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735ZoneLength[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x2710,
};

const ASN1CType asn1_type_j2735Zoom[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xf,
};

const ASN1CType asn1_type_j2735ITIScodes_1[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x20b,
  0x21d,
};

const ASN1CType asn1_type_j2735ITIScodes_2[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x20b,
  0x21d,
};

const ASN1CType asn1_type_j2735_local_602[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735_local_603[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735LaneID,
};

const ASN1CType asn1_type_j2735_local_604[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735RoadSegmentReferenceID,
};

const ASN1CType asn1_type_j2735_local_605[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735DectLane_KOR,
};

const ASN1CType asn1_type_j2735BasicSafetyMessage_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  11,
  sizeof(j2735BasicSafetyMessage_KOR),

  offsetof(j2735BasicSafetyMessage_KOR, adas) | 0x8000000,
  (intptr_t)asn1_type_j2735BasicSafetyMessage_KOR_1,
  offsetof(j2735BasicSafetyMessage_KOR, adas_option),
  (intptr_t)"adas",

  offsetof(j2735BasicSafetyMessage_KOR, typeEvent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_602,
  offsetof(j2735BasicSafetyMessage_KOR, typeEvent_option),
  (intptr_t)"typeEvent",

  offsetof(j2735BasicSafetyMessage_KOR, laneID) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_603,
  offsetof(j2735BasicSafetyMessage_KOR, laneID_option),
  (intptr_t)"laneID",

  offsetof(j2735BasicSafetyMessage_KOR, roadSegmentRef) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_604,
  offsetof(j2735BasicSafetyMessage_KOR, roadSegmentRef_option),
  (intptr_t)"roadSegmentRef",

  offsetof(j2735BasicSafetyMessage_KOR, dectLane) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_605,
  offsetof(j2735BasicSafetyMessage_KOR, dectLane_option),
  (intptr_t)"dectLane",

  offsetof(j2735BasicSafetyMessage_KOR, verticalHeading) | 0x8000000,
  (intptr_t)asn1_type_j2735VerticalHeading_KOR,
  offsetof(j2735BasicSafetyMessage_KOR, verticalHeading_option),
  (intptr_t)"verticalHeading",

  offsetof(j2735BasicSafetyMessage_KOR, brakePedalCmd) | 0x8000000,
  (intptr_t)asn1_type_j2735BrakePedalCmd_KOR,
  offsetof(j2735BasicSafetyMessage_KOR, brakePedalCmd_option),
  (intptr_t)"brakePedalCmd",

  offsetof(j2735BasicSafetyMessage_KOR, brakePedalPressure) | 0x8000000,
  (intptr_t)asn1_type_j2735BrakePedalPressure_KOR,
  offsetof(j2735BasicSafetyMessage_KOR, brakePedalPressure_option),
  (intptr_t)"brakePedalPressure",

  offsetof(j2735BasicSafetyMessage_KOR, eventFlags) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleEventFlags_KOR,
  offsetof(j2735BasicSafetyMessage_KOR, eventFlags_option),
  (intptr_t)"eventFlags",

  offsetof(j2735BasicSafetyMessage_KOR, wheelAngle) | 0x8000000,
  (intptr_t)asn1_type_j2735WheelAngles_KOR,
  offsetof(j2735BasicSafetyMessage_KOR, wheelAngle_option),
  (intptr_t)"wheelAngle",

  offsetof(j2735BasicSafetyMessage_KOR, responseType) | 0x8000000,
  (intptr_t)asn1_type_j2735ResponseType_KOR,
  offsetof(j2735BasicSafetyMessage_KOR, responseType_option),
  (intptr_t)"responseType",

};

const ASN1CType asn1_type_j2735BasicSafetyMessage_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0xc,
  sizeof(j2735CITSADAS_KOR),
  (intptr_t)asn1_type_j2735CITSADAS_KOR,
  0,
};

const ASN1CType asn1_type_j2735_local_606[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735ObstacleDistance,
};

const ASN1CType asn1_type_j2735_local_607[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735DSecond,
};

const ASN1CType asn1_type_j2735_local_608[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735Longitude,
};

const ASN1CType asn1_type_j2735_local_609[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735Latitude,
};

const ASN1CType asn1_type_j2735_local_610[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735PositionConfidence,
};

const ASN1CType asn1_type_j2735CITSADAS_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735CITSADAS_KOR),

  offsetof(j2735CITSADAS_KOR, event) | 0x0,
  (intptr_t)asn1_type_j2735ADASEventType_KOR,
  0,
  (intptr_t)"event",

  offsetof(j2735CITSADAS_KOR, distance) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_606,
  offsetof(j2735CITSADAS_KOR, distance_option),
  (intptr_t)"distance",

  offsetof(j2735CITSADAS_KOR, time) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_607,
  offsetof(j2735CITSADAS_KOR, time_option),
  (intptr_t)"time",

  offsetof(j2735CITSADAS_KOR, Long) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_608,
  offsetof(j2735CITSADAS_KOR, Long_option),
  (intptr_t)"long",

  offsetof(j2735CITSADAS_KOR, lat) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_609,
  offsetof(j2735CITSADAS_KOR, lat_option),
  (intptr_t)"lat",

  offsetof(j2735CITSADAS_KOR, confidence) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_610,
  offsetof(j2735CITSADAS_KOR, confidence_option),
  (intptr_t)"confidence",

};

const ASN1CType asn1_type_j2735ConsecutiveTrafficLight[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  3,
  0,
  (intptr_t)"none",
  (intptr_t)"first",
  (intptr_t)"second",
};

const ASN1CType asn1_type_j2735_local_611[] = {
  (ASN1_CTYPE_OCTET_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x1,
  0x40,
};

const ASN1CType asn1_type_j2735_local_612[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735_local_613[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735FullPositionVector,
};

const ASN1CType asn1_type_j2735DetectedObject_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  3,
  sizeof(j2735DetectedObject_KOR),

  offsetof(j2735DetectedObject_KOR, objectID) | 0x0,
  (intptr_t)asn1_type_j2735_local_611,
  0,
  (intptr_t)"objectID",

  offsetof(j2735DetectedObject_KOR, objectType) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_612,
  offsetof(j2735DetectedObject_KOR, objectType_option),
  (intptr_t)"objectType",

  offsetof(j2735DetectedObject_KOR, objectLocation) | 0x0,
  (intptr_t)asn1_type_j2735_local_613,
  0,
  (intptr_t)"objectLocation",

};

const ASN1CType asn1_type_j2735_local_614[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735DetectedObjectInfo_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100004,
  2,
  sizeof(j2735DetectedObjectInfo_KOR),

  offsetof(j2735DetectedObjectInfo_KOR, detectorID) | 0x0,
  (intptr_t)asn1_type_j2735_local_614,
  0,
  (intptr_t)"detectorID",

  offsetof(j2735DetectedObjectInfo_KOR, objects) | 0x0,
  (intptr_t)asn1_type_j2735DetectedObjectList_KOR,
  0,
  (intptr_t)"objects",

};

const ASN1CType asn1_type_j2735DetectedObjectList_KOR[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x20,
  sizeof(j2735DetectedObject_KOR),
  (intptr_t)asn1_type_j2735DetectedObject_KOR,
  0,
};

const ASN1CType asn1_type_j2735_local_615[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITISCodes_KOR,
};

const ASN1CType asn1_type_j2735_local_616[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735Heading,
};

const ASN1CType asn1_type_j2735_local_617[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735Extent,
};

const ASN1CType asn1_type_j2735_local_618[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735FullPositionVector,
};

const ASN1CType asn1_type_j2735EmergencyVehicleAlert_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  4,
  sizeof(j2735EmergencyVehicleAlert_KOR),

  offsetof(j2735EmergencyVehicleAlert_KOR, typeEvent) | 0x0,
  (intptr_t)asn1_type_j2735_local_615,
  0,
  (intptr_t)"typeEvent",

  offsetof(j2735EmergencyVehicleAlert_KOR, heading) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_616,
  offsetof(j2735EmergencyVehicleAlert_KOR, heading_option),
  (intptr_t)"heading",

  offsetof(j2735EmergencyVehicleAlert_KOR, extent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_617,
  offsetof(j2735EmergencyVehicleAlert_KOR, extent_option),
  (intptr_t)"extent",

  offsetof(j2735EmergencyVehicleAlert_KOR, position) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_618,
  offsetof(j2735EmergencyVehicleAlert_KOR, position_option),
  (intptr_t)"position",

};

const ASN1CType asn1_type_j2735_local_619[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITISCodes_KOR,
};

const ASN1CType asn1_type_j2735EventDescription_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  1,
  sizeof(j2735EventDescription_KOR),

  offsetof(j2735EventDescription_KOR, eventType) | 0x0,
  (intptr_t)asn1_type_j2735_local_619,
  0,
  (intptr_t)"eventType",

};

const ASN1CType asn1_type_j2735_local_620[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735_local_621[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735ITIScodesAndText,
};

const ASN1CType asn1_type_j2735EventOrSvcInfo_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  4,
  sizeof(j2735EventOrSvcInfo_KOR),

  offsetof(j2735EventOrSvcInfo_KOR, typeEvent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_620,
  offsetof(j2735EventOrSvcInfo_KOR, typeEvent_option),
  (intptr_t)"typeEvent",

  offsetof(j2735EventOrSvcInfo_KOR, description) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_621,
  offsetof(j2735EventOrSvcInfo_KOR, description_option),
  (intptr_t)"description",

  offsetof(j2735EventOrSvcInfo_KOR, text) | 0x8000000,
  (intptr_t)asn1_type_j2735EventOrSvcInfo_KOR_1,
  offsetof(j2735EventOrSvcInfo_KOR, text_option),
  (intptr_t)"text",

  offsetof(j2735EventOrSvcInfo_KOR, subtext) | 0x8000000,
  (intptr_t)asn1_type_j2735EventOrSvcInfo_KOR_2,
  offsetof(j2735EventOrSvcInfo_KOR, subtext_option),
  (intptr_t)"subtext",

};

const ASN1CType asn1_type_j2735_local_622[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0xc,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735EventOrSvcInfo_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x8,
  sizeof(ASN1String),
  (intptr_t)asn1_type_j2735_local_622,
  0,
};

const ASN1CType asn1_type_j2735_local_623[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0xc,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735EventOrSvcInfo_KOR_2[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x1,
  0x8,
  sizeof(ASN1String),
  (intptr_t)asn1_type_j2735_local_623,
  0,
};

const ASN1CType asn1_type_j2735_local_624[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x400000 | 0x100000,
};

const ASN1CType asn1_type_j2735_local_625[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x400000 | 0x100001,
};

const ASN1CType asn1_type_j2735FuelEfficiency_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  2,
  sizeof(j2735FuelEfficiency_KOR),

  offsetof(j2735FuelEfficiency_KOR, vehicleMileage) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_624,
  offsetof(j2735FuelEfficiency_KOR, vehicleMileage_option),
  (intptr_t)"vehicleMileage",

  offsetof(j2735FuelEfficiency_KOR, evMileage) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_625,
  offsetof(j2735FuelEfficiency_KOR, evMileage_option),
  (intptr_t)"evMileage",

};

const ASN1CType asn1_type_j2735_local_626[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0x64,
};

const ASN1CType asn1_type_j2735_local_627[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0x64,
};

const ASN1CType asn1_type_j2735FuelResidualQuantity_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  2,
  sizeof(j2735FuelResidualQuantity_KOR),

  offsetof(j2735FuelResidualQuantity_KOR, vehiclePercent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_626,
  offsetof(j2735FuelResidualQuantity_KOR, vehiclePercent_option),
  (intptr_t)"vehiclePercent",

  offsetof(j2735FuelResidualQuantity_KOR, evPercent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_627,
  offsetof(j2735FuelResidualQuantity_KOR, evPercent_option),
  (intptr_t)"evPercent",

};

const ASN1CType asn1_type_j2735FuelStatus_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10000c,
  3,
  sizeof(j2735FuelStatus_KOR),

  offsetof(j2735FuelStatus_KOR, efficiency) | 0x8000000,
  (intptr_t)asn1_type_j2735FuelEfficiency_KOR,
  offsetof(j2735FuelStatus_KOR, efficiency_option),
  (intptr_t)"efficiency",

  offsetof(j2735FuelStatus_KOR, residualQuantity) | 0x8000000,
  (intptr_t)asn1_type_j2735FuelResidualQuantity_KOR,
  offsetof(j2735FuelStatus_KOR, residualQuantity_option),
  (intptr_t)"residualQuantity",

  offsetof(j2735FuelStatus_KOR, tankCapacity) | 0x8000000,
  (intptr_t)asn1_type_j2735FuelTankCapacity_KOR,
  offsetof(j2735FuelStatus_KOR, tankCapacity_option),
  (intptr_t)"tankCapacity",

};

const ASN1CType asn1_type_j2735_local_628[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735_local_629[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735FuelTankCapacity_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  2,
  sizeof(j2735FuelTankCapacity_KOR),

  offsetof(j2735FuelTankCapacity_KOR, fuelCapacity) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_628,
  offsetof(j2735FuelTankCapacity_KOR, fuelCapacity_option),
  (intptr_t)"fuelCapacity",

  offsetof(j2735FuelTankCapacity_KOR, evCapacity) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_629,
  offsetof(j2735FuelTankCapacity_KOR, evCapacity_option),
  (intptr_t)"evCapacity",

};

const ASN1CType asn1_type_j2735GeographicalPath_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  1,
  sizeof(j2735GeographicalPath_KOR),

  offsetof(j2735GeographicalPath_KOR, regions) | 0x0,
  (intptr_t)asn1_type_j2735GeographicalPath_KOR_1,
  0,
  (intptr_t)"regions",

};

const ASN1CType asn1_type_j2735GeographicalPath_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x1,
  0x20,
  sizeof(j2735RegionDescribedByID_KOR),
  (intptr_t)asn1_type_j2735RegionDescribedByID_KOR,
  0,
};

const ASN1CType asn1_type_j2735_local_630[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_631[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_632[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_633[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_634[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_635[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_636[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_637[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735_local_638[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0xffff8001,
  0x7fff,
};

const ASN1CType asn1_type_j2735GyroscopeValue_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  9,
  sizeof(j2735GyroscopeValue_KOR),

  offsetof(j2735GyroscopeValue_KOR, xmax) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_630,
  offsetof(j2735GyroscopeValue_KOR, xmax_option),
  (intptr_t)"xmax",

  offsetof(j2735GyroscopeValue_KOR, xmin) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_631,
  offsetof(j2735GyroscopeValue_KOR, xmin_option),
  (intptr_t)"xmin",

  offsetof(j2735GyroscopeValue_KOR, xavg) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_632,
  offsetof(j2735GyroscopeValue_KOR, xavg_option),
  (intptr_t)"xavg",

  offsetof(j2735GyroscopeValue_KOR, ymax) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_633,
  offsetof(j2735GyroscopeValue_KOR, ymax_option),
  (intptr_t)"ymax",

  offsetof(j2735GyroscopeValue_KOR, ymin) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_634,
  offsetof(j2735GyroscopeValue_KOR, ymin_option),
  (intptr_t)"ymin",

  offsetof(j2735GyroscopeValue_KOR, yavg) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_635,
  offsetof(j2735GyroscopeValue_KOR, yavg_option),
  (intptr_t)"yavg",

  offsetof(j2735GyroscopeValue_KOR, zmax) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_636,
  offsetof(j2735GyroscopeValue_KOR, zmax_option),
  (intptr_t)"zmax",

  offsetof(j2735GyroscopeValue_KOR, zmin) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_637,
  offsetof(j2735GyroscopeValue_KOR, zmin_option),
  (intptr_t)"zmin",

  offsetof(j2735GyroscopeValue_KOR, zavg) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_638,
  offsetof(j2735GyroscopeValue_KOR, zavg_option),
  (intptr_t)"zavg",

};

const ASN1CType asn1_type_j2735IntersectionCollisionAvoidance_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  1,
  sizeof(j2735IntersectionCollisionAvoidance_KOR),

  offsetof(j2735IntersectionCollisionAvoidance_KOR, path) | 0x8000000,
  (intptr_t)asn1_type_j2735PathPlanned_KOR,
  offsetof(j2735IntersectionCollisionAvoidance_KOR, path_option),
  (intptr_t)"path",

};

const ASN1CType asn1_type_j2735_local_639[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735FurtherInfoID,
};

const ASN1CType asn1_type_j2735_local_640[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735ITISCodes_KOR,
};

const ASN1CType asn1_type_j2735_local_641[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735TemporaryID,
};

const ASN1CType asn1_type_j2735_local_642[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735DDateTime,
};

const ASN1CType asn1_type_j2735_local_643[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735MsgDisplayLocation_KOR,
};

const ASN1CType asn1_type_j2735_local_644[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  (intptr_t)asn1_type_j2735DDateTime,
};

const ASN1CType asn1_type_j2735MsgLog_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  10,
  sizeof(j2735MsgLog_KOR),

  offsetof(j2735MsgLog_KOR, furtherInfoID) | 0x0,
  (intptr_t)asn1_type_j2735_local_639,
  0,
  (intptr_t)"furtherInfoID",

  offsetof(j2735MsgLog_KOR, msgType) | 0x0,
  (intptr_t)asn1_type_j2735MsgType_KOR,
  0,
  (intptr_t)"msgType",

  offsetof(j2735MsgLog_KOR, msgEvent) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_640,
  offsetof(j2735MsgLog_KOR, msgEvent_option),
  (intptr_t)"msgEvent",

  offsetof(j2735MsgLog_KOR, msgID) | 0x0,
  (intptr_t)asn1_type_j2735_local_641,
  0,
  (intptr_t)"msgID",

  offsetof(j2735MsgLog_KOR, msgDate) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_642,
  offsetof(j2735MsgLog_KOR, msgDate_option),
  (intptr_t)"msgDate",

  offsetof(j2735MsgLog_KOR, msgCommType) | 0x8000000,
  (intptr_t)asn1_type_j2735MsgCommType_KOR,
  offsetof(j2735MsgLog_KOR, msgCommType_option),
  (intptr_t)"msgCommType",

  offsetof(j2735MsgLog_KOR, msgChannel) | 0x8000000,
  (intptr_t)asn1_type_j2735MsgChannel_KOR,
  offsetof(j2735MsgLog_KOR, msgChannel_option),
  (intptr_t)"msgChannel",

  offsetof(j2735MsgLog_KOR, msgDeviceType) | 0x0,
  (intptr_t)asn1_type_j2735MsgDeviceType_KOR,
  0,
  (intptr_t)"msgDeviceType",

  offsetof(j2735MsgLog_KOR, msgDisplayLocation) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_643,
  offsetof(j2735MsgLog_KOR, msgDisplayLocation_option),
  (intptr_t)"msgDisplayLocation",

  offsetof(j2735MsgLog_KOR, msgDisplayTime) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_644,
  offsetof(j2735MsgLog_KOR, msgDisplayTime_option),
  (intptr_t)"msgDisplayTime",

};

const ASN1CType asn1_type_j2735MsgLogs_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100009,
  2,
  sizeof(j2735MsgLogs_KOR),

  offsetof(j2735MsgLogs_KOR, rcvMsgs) | 0x8000000,
  (intptr_t)asn1_type_j2735MsgLogs_KOR_1,
  offsetof(j2735MsgLogs_KOR, rcvMsgs_option),
  (intptr_t)"rcvMsgs",

  offsetof(j2735MsgLogs_KOR, sndMsgs) | 0x8000000,
  (intptr_t)asn1_type_j2735MsgLogs_KOR_2,
  offsetof(j2735MsgLogs_KOR, sndMsgs_option),
  (intptr_t)"sndMsgs",

};

const ASN1CType asn1_type_j2735MsgLogs_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x1,
  0x14,
  sizeof(j2735MsgLog_KOR),
  (intptr_t)asn1_type_j2735MsgLog_KOR,
  0,
};

const ASN1CType asn1_type_j2735MsgLogs_KOR_2[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x1e,
  sizeof(j2735MsgLog_KOR),
  (intptr_t)asn1_type_j2735MsgLog_KOR,
  0,
};

const ASN1CType asn1_type_j2735_local_645[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x400000 | 0x100001,
};

const ASN1CType asn1_type_j2735_local_646[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x400000 | 0x100002,
};

const ASN1CType asn1_type_j2735ParkingCapacity_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  3,
  sizeof(j2735ParkingCapacity_KOR),

  offsetof(j2735ParkingCapacity_KOR, type) | 0x0,
  (intptr_t)asn1_type_j2735ParkingType_KOR,
  0,
  (intptr_t)"type",

  offsetof(j2735ParkingCapacity_KOR, numberOfParkingSpace) | 0x0,
  (intptr_t)asn1_type_j2735_local_645,
  0,
  (intptr_t)"numberOfParkingSpace",

  offsetof(j2735ParkingCapacity_KOR, maxParkingSpace) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_646,
  offsetof(j2735ParkingCapacity_KOR, maxParkingSpace_option),
  (intptr_t)"maxParkingSpace",

};

const ASN1CType asn1_type_j2735_local_647[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735_local_648[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735ParkingZoneInformation_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  3,
  sizeof(j2735ParkingZoneInformation_KOR),

  offsetof(j2735ParkingZoneInformation_KOR, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_647,
  offsetof(j2735ParkingZoneInformation_KOR, name_option),
  (intptr_t)"name",

  offsetof(j2735ParkingZoneInformation_KOR, id) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_648,
  offsetof(j2735ParkingZoneInformation_KOR, id_option),
  (intptr_t)"id",

  offsetof(j2735ParkingZoneInformation_KOR, capacity) | 0x0,
  (intptr_t)asn1_type_j2735ParkingZoneInformation_KOR_1,
  0,
  (intptr_t)"capacity",

};

const ASN1CType asn1_type_j2735ParkingZoneInformation_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x0,
  0x10,
  sizeof(j2735ParkingCapacity_KOR),
  (intptr_t)asn1_type_j2735ParkingCapacity_KOR,
  0,
};

const ASN1CType asn1_type_j2735PersonalSafetyMessage_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  1,
  sizeof(j2735PersonalSafetyMessage_KOR),

  offsetof(j2735PersonalSafetyMessage_KOR, path) | 0x8000000,
  (intptr_t)asn1_type_j2735PathPlanned_KOR,
  offsetof(j2735PersonalSafetyMessage_KOR, path_option),
  (intptr_t)"path",

};

const ASN1CType asn1_type_j2735_local_649[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735FullPositionVector,
};

const ASN1CType asn1_type_j2735_local_650[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735GNSSstatus,
};

const ASN1CType asn1_type_j2735PathPlanned_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  3,
  sizeof(j2735PathPlanned_KOR),

  offsetof(j2735PathPlanned_KOR, initialPosition) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_649,
  offsetof(j2735PathPlanned_KOR, initialPosition_option),
  (intptr_t)"initialPosition",

  offsetof(j2735PathPlanned_KOR, currGNSSstatue) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_650,
  offsetof(j2735PathPlanned_KOR, currGNSSstatue_option),
  (intptr_t)"currGNSSstatue",

  offsetof(j2735PathPlanned_KOR, pathPlanned) | 0x0,
  (intptr_t)asn1_type_j2735PathPlannedPointList_KOR,
  0,
  (intptr_t)"pathPlanned",

};

const ASN1CType asn1_type_j2735_local_651[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735OffsetLL_B18,
};

const ASN1CType asn1_type_j2735_local_652[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735OffsetLL_B18,
};

const ASN1CType asn1_type_j2735_local_653[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735TimeOffset,
};

const ASN1CType asn1_type_j2735_local_654[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  (intptr_t)asn1_type_j2735PositionalAccuracy,
};

const ASN1CType asn1_type_j2735_local_655[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735CoarseHeading,
};

const ASN1CType asn1_type_j2735PathPlannedPoint_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735PathPlannedPoint_KOR),

  offsetof(j2735PathPlannedPoint_KOR, latOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_651,
  0,
  (intptr_t)"latOffset",

  offsetof(j2735PathPlannedPoint_KOR, lonOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_652,
  0,
  (intptr_t)"lonOffset",

  offsetof(j2735PathPlannedPoint_KOR, elevationOffset) | 0x0,
  (intptr_t)asn1_type_j2735VertOffset_B12,
  0,
  (intptr_t)"elevationOffset",

  offsetof(j2735PathPlannedPoint_KOR, timeOffset) | 0x0,
  (intptr_t)asn1_type_j2735_local_653,
  0,
  (intptr_t)"timeOffset",

  offsetof(j2735PathPlannedPoint_KOR, positionalAccuracy) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_654,
  offsetof(j2735PathPlannedPoint_KOR, positionalAccuracy_option),
  (intptr_t)"positionalAccuracy",

  offsetof(j2735PathPlannedPoint_KOR, heading) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_655,
  offsetof(j2735PathPlannedPoint_KOR, heading_option),
  (intptr_t)"heading",

};

const ASN1CType asn1_type_j2735PathPlannedPointList_KOR[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x20,
  sizeof(j2735PathPlannedPoint_KOR),
  (intptr_t)asn1_type_j2735PathPlannedPoint_KOR,
  0,
};

const ASN1CType asn1_type_j2735_local_656[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735_local_657[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735DectLane_KOR,
};

const ASN1CType asn1_type_j2735_local_658[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735CITSADAS_KOR,
};

const ASN1CType asn1_type_j2735_local_659[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735EventOrSvcInfo_KOR,
};

const ASN1CType asn1_type_j2735ProbeVehicleData_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  14,
  sizeof(j2735ProbeVehicleData_KOR),

  offsetof(j2735ProbeVehicleData_KOR, msgID) | 0x0,
  (intptr_t)asn1_type_j2735_local_656,
  0,
  (intptr_t)"msgID",

  offsetof(j2735ProbeVehicleData_KOR, curlane) | 0x0,
  (intptr_t)asn1_type_j2735_local_657,
  0,
  (intptr_t)"curlane",

  offsetof(j2735ProbeVehicleData_KOR, adas) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_658,
  offsetof(j2735ProbeVehicleData_KOR, adas_option),
  (intptr_t)"adas",

  offsetof(j2735ProbeVehicleData_KOR, driverStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735HumanDriverStatus_KOR,
  offsetof(j2735ProbeVehicleData_KOR, driverStatus_option),
  (intptr_t)"driverStatus",

  offsetof(j2735ProbeVehicleData_KOR, deviceStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735DeviceStatus_KOR,
  offsetof(j2735ProbeVehicleData_KOR, deviceStatus_option),
  (intptr_t)"deviceStatus",

  offsetof(j2735ProbeVehicleData_KOR, transitStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735TransitVehicleStatus,
  offsetof(j2735ProbeVehicleData_KOR, transitStatus_option),
  (intptr_t)"transitStatus",

  offsetof(j2735ProbeVehicleData_KOR, transitOccupancy) | 0x8000000,
  (intptr_t)asn1_type_j2735TransitVehicleOccupancy,
  offsetof(j2735ProbeVehicleData_KOR, transitOccupancy_option),
  (intptr_t)"transitOccupancy",

  offsetof(j2735ProbeVehicleData_KOR, roadWorkInfo) | 0x8000000,
  (intptr_t)asn1_type_j2735RoadWorkInfo_KOR,
  offsetof(j2735ProbeVehicleData_KOR, roadWorkInfo_option),
  (intptr_t)"roadWorkInfo",

  offsetof(j2735ProbeVehicleData_KOR, eventOrSvcInfo) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_659,
  offsetof(j2735ProbeVehicleData_KOR, eventOrSvcInfo_option),
  (intptr_t)"eventOrSvcInfo",

  offsetof(j2735ProbeVehicleData_KOR, msgLogs) | 0x8000000,
  (intptr_t)asn1_type_j2735MsgLogs_KOR,
  offsetof(j2735ProbeVehicleData_KOR, msgLogs_option),
  (intptr_t)"msgLogs",

  offsetof(j2735ProbeVehicleData_KOR, vehicleControlInfo) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleControlInfo_KOR,
  offsetof(j2735ProbeVehicleData_KOR, vehicleControlInfo_option),
  (intptr_t)"vehicleControlInfo",

  offsetof(j2735ProbeVehicleData_KOR, drivingStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735DrivingStatus_KOR,
  offsetof(j2735ProbeVehicleData_KOR, drivingStatus_option),
  (intptr_t)"drivingStatus",

  offsetof(j2735ProbeVehicleData_KOR, fuelStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735FuelStatus_KOR,
  offsetof(j2735ProbeVehicleData_KOR, fuelStatus_option),
  (intptr_t)"fuelStatus",

  offsetof(j2735ProbeVehicleData_KOR, sensorValues) | 0x8000000,
  (intptr_t)asn1_type_j2735SensorValuesRegardingDriving_KOR,
  offsetof(j2735ProbeVehicleData_KOR, sensorValues_option),
  (intptr_t)"sensorValues",

};

const ASN1CType asn1_type_j2735_local_660[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735LaneID,
};

const ASN1CType asn1_type_j2735_local_661[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735_local_662[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735DectLane_KOR,
};

const ASN1CType asn1_type_j2735_local_663[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735_local_664[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100005,
  (intptr_t)asn1_type_j2735IntersectionID,
};

const ASN1CType asn1_type_j2735RegionDescribedByID_KOR[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x0,
  6,
  0,
  sizeof(j2735RegionDescribedByID_KOR),
  offsetof(j2735RegionDescribedByID_KOR, choice),
  offsetof(j2735RegionDescribedByID_KOR, u),
  (intptr_t)asn1_type_j2735_local_660,
  (intptr_t)"laneID",
  (intptr_t)asn1_type_j2735RoadSegmentID,
  (intptr_t)"segmentID",
  (intptr_t)asn1_type_j2735_local_661,
  (intptr_t)"linkID",
  (intptr_t)asn1_type_j2735_local_662,
  (intptr_t)"detectLane",
  (intptr_t)asn1_type_j2735_local_663,
  (intptr_t)"areaID",
  (intptr_t)asn1_type_j2735_local_664,
  (intptr_t)"intersections",
};

const ASN1CType asn1_type_j2735ReservedBit[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x10000a,
};

const ASN1CType asn1_type_j2735_local_665[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735_local_666[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
};

const ASN1CType asn1_type_j2735RidingNotification_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  3,
  sizeof(j2735RidingNotification_KOR),

  offsetof(j2735RidingNotification_KOR, stopID) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_665,
  offsetof(j2735RidingNotification_KOR, stopID_option),
  (intptr_t)"stopID",

  offsetof(j2735RidingNotification_KOR, disabled) | 0x8000000,
  (intptr_t)asn1_type_j2735Disabled_KOR,
  offsetof(j2735RidingNotification_KOR, disabled_option),
  (intptr_t)"disabled",

  offsetof(j2735RidingNotification_KOR, boarding) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_666,
  offsetof(j2735RidingNotification_KOR, boarding_option),
  (intptr_t)"boarding",

};

const ASN1CType asn1_type_j2735_local_667[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735_local_668[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735EventOrSvcInfo_KOR,
};

const ASN1CType asn1_type_j2735_local_669[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735MovementEvent,
};

const ASN1CType asn1_type_j2735RoadSideAlert_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  5,
  sizeof(j2735RoadSideAlert_KOR),

  offsetof(j2735RoadSideAlert_KOR, msgID) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_667,
  offsetof(j2735RoadSideAlert_KOR, msgID_option),
  (intptr_t)"msgID",

  offsetof(j2735RoadSideAlert_KOR, regions) | 0x8000000,
  (intptr_t)asn1_type_j2735RoadSideAlert_KOR_1,
  offsetof(j2735RoadSideAlert_KOR, regions_option),
  (intptr_t)"regions",

  offsetof(j2735RoadSideAlert_KOR, events) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_668,
  offsetof(j2735RoadSideAlert_KOR, events_option),
  (intptr_t)"events",

  offsetof(j2735RoadSideAlert_KOR, state) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_669,
  offsetof(j2735RoadSideAlert_KOR, state_option),
  (intptr_t)"state",

  offsetof(j2735RoadSideAlert_KOR, detectedObject) | 0x8000000,
  (intptr_t)asn1_type_j2735DetectedObjectInfo_KOR,
  offsetof(j2735RoadSideAlert_KOR, detectedObject_option),
  (intptr_t)"detectedObject",

};

const ASN1CType asn1_type_j2735RoadSideAlert_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1000000 | 0x100001,
  0x0,
  sizeof(j2735RegionDescribedByID_KOR),
  (intptr_t)asn1_type_j2735RegionDescribedByID_KOR,
  0,
};

const ASN1CType asn1_type_j2735_local_670[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735DDateTime,
};

const ASN1CType asn1_type_j2735_local_671[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735DDateTime,
};

const ASN1CType asn1_type_j2735RoadWorkInfo_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100007,
  3,
  sizeof(j2735RoadWorkInfo_KOR),

  offsetof(j2735RoadWorkInfo_KOR, worklanes) | 0x8000000,
  (intptr_t)asn1_type_j2735RoadWorkInfo_KOR_1,
  offsetof(j2735RoadWorkInfo_KOR, worklanes_option),
  (intptr_t)"worklanes",

  offsetof(j2735RoadWorkInfo_KOR, workstart) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_670,
  offsetof(j2735RoadWorkInfo_KOR, workstart_option),
  (intptr_t)"workstart",

  offsetof(j2735RoadWorkInfo_KOR, workend) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_671,
  offsetof(j2735RoadWorkInfo_KOR, workend_option),
  (intptr_t)"workend",

};

const ASN1CType asn1_type_j2735RoadWorkInfo_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x1,
  0x10,
  sizeof(j2735LaneID),
  (intptr_t)asn1_type_j2735LaneID,
  0,
};

const ASN1CType asn1_type_j2735_local_672[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735_local_673[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735_local_674[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735HighwayDirection_KOR,
};

const ASN1CType asn1_type_j2735ServiceAreaInformation_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100004,
  5,
  sizeof(j2735ServiceAreaInformation_KOR),

  offsetof(j2735ServiceAreaInformation_KOR, name) | 0x0,
  (intptr_t)asn1_type_j2735_local_672,
  0,
  (intptr_t)"name",

  offsetof(j2735ServiceAreaInformation_KOR, id) | 0x0,
  (intptr_t)asn1_type_j2735_local_673,
  0,
  (intptr_t)"id",

  offsetof(j2735ServiceAreaInformation_KOR, direction) | 0x0,
  (intptr_t)asn1_type_j2735_local_674,
  0,
  (intptr_t)"direction",

  offsetof(j2735ServiceAreaInformation_KOR, stationInfo) | 0x8000000,
  (intptr_t)asn1_type_j2735ServiceAreaInformation_KOR_1,
  offsetof(j2735ServiceAreaInformation_KOR, stationInfo_option),
  (intptr_t)"stationInfo",

  offsetof(j2735ServiceAreaInformation_KOR, parkingInfo) | 0x8000000,
  (intptr_t)asn1_type_j2735ServiceAreaInformation_KOR_2,
  offsetof(j2735ServiceAreaInformation_KOR, parkingInfo_option),
  (intptr_t)"parkingInfo",

};

const ASN1CType asn1_type_j2735ServiceAreaInformation_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1000000 | 0x100003,
  0x0,
  sizeof(j2735StationInformation_KOR),
  (intptr_t)asn1_type_j2735StationInformation_KOR,
  0,
};

const ASN1CType asn1_type_j2735ServiceAreaInformation_KOR_2[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1000000 | 0x100004,
  0x0,
  sizeof(j2735ParkingZoneInformation_KOR),
  (intptr_t)asn1_type_j2735ParkingZoneInformation_KOR,
  0,
};

const ASN1CType asn1_type_j2735_local_675[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735TransmissionState,
};

const ASN1CType asn1_type_j2735SensorValuesRegardingDriving_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10000d,
  3,
  sizeof(j2735SensorValuesRegardingDriving_KOR),

  offsetof(j2735SensorValuesRegardingDriving_KOR, rpm) | 0x8000000,
  (intptr_t)asn1_type_j2735Rpm_KOR,
  offsetof(j2735SensorValuesRegardingDriving_KOR, rpm_option),
  (intptr_t)"rpm",

  offsetof(j2735SensorValuesRegardingDriving_KOR, gears) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_675,
  offsetof(j2735SensorValuesRegardingDriving_KOR, gears_option),
  (intptr_t)"gears",

  offsetof(j2735SensorValuesRegardingDriving_KOR, gyro) | 0x8000000,
  (intptr_t)asn1_type_j2735GyroscopeValue_KOR,
  offsetof(j2735SensorValuesRegardingDriving_KOR, gyro_option),
  (intptr_t)"gyro",

};

const ASN1CType asn1_type_j2735_local_676[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0x100,
};

const ASN1CType asn1_type_j2735_local_677[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x0,
  0x100,
};

const ASN1CType asn1_type_j2735_local_678[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x400000 | 0x100003,
};

const ASN1CType asn1_type_j2735StationInformation_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  4,
  sizeof(j2735StationInformation_KOR),

  offsetof(j2735StationInformation_KOR, type) | 0x0,
  (intptr_t)asn1_type_j2735FuelType_KOR,
  0,
  (intptr_t)"type",

  offsetof(j2735StationInformation_KOR, capacity) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_676,
  offsetof(j2735StationInformation_KOR, capacity_option),
  (intptr_t)"capacity",

  offsetof(j2735StationInformation_KOR, maxcapacity) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_677,
  offsetof(j2735StationInformation_KOR, maxcapacity_option),
  (intptr_t)"maxcapacity",

  offsetof(j2735StationInformation_KOR, price) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_678,
  offsetof(j2735StationInformation_KOR, price_option),
  (intptr_t)"price",

};

const ASN1CType asn1_type_j2735SupplementalVehicleExtensions_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  1,
  sizeof(j2735SupplementalVehicleExtensions_KOR),

  offsetof(j2735SupplementalVehicleExtensions_KOR, vehicleClass) | 0x0,
  (intptr_t)asn1_type_j2735BasicVehicleClass_KOR,
  0,
  (intptr_t)"vehicleClass",

};

const ASN1CType asn1_type_j2735_local_679[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DectLane_KOR,
};

const ASN1CType asn1_type_j2735_local_680[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735LaneID,
};

const ASN1CType asn1_type_j2735_local_681[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
};

const ASN1CType asn1_type_j2735TollgateLane_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  4,
  sizeof(j2735TollgateLane_KOR),

  offsetof(j2735TollgateLane_KOR, laneNo) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_679,
  offsetof(j2735TollgateLane_KOR, laneNo_option),
  (intptr_t)"laneNo",

  offsetof(j2735TollgateLane_KOR, linkID) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_680,
  offsetof(j2735TollgateLane_KOR, linkID_option),
  (intptr_t)"linkID",

  offsetof(j2735TollgateLane_KOR, type) | 0x8000000,
  (intptr_t)asn1_type_j2735TollgateType_KOR,
  offsetof(j2735TollgateLane_KOR, type_option),
  (intptr_t)"type",

  offsetof(j2735TollgateLane_KOR, open) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_681,
  offsetof(j2735TollgateLane_KOR, open_option),
  (intptr_t)"open",

};

const ASN1CType asn1_type_j2735_local_682[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  ASN1_CSTR_UTF8String,
};

const ASN1CType asn1_type_j2735_local_683[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735HighwayDirection_KOR,
};

const ASN1CType asn1_type_j2735TollgateLaneInformation_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100005,
  3,
  sizeof(j2735TollgateLaneInformation_KOR),

  offsetof(j2735TollgateLaneInformation_KOR, name) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_682,
  offsetof(j2735TollgateLaneInformation_KOR, name_option),
  (intptr_t)"name",

  offsetof(j2735TollgateLaneInformation_KOR, heading) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_683,
  offsetof(j2735TollgateLaneInformation_KOR, heading_option),
  (intptr_t)"heading",

  offsetof(j2735TollgateLaneInformation_KOR, lanes) | 0x8000000,
  (intptr_t)asn1_type_j2735TollgateLaneInformation_KOR_1,
  offsetof(j2735TollgateLaneInformation_KOR, lanes_option),
  (intptr_t)"lanes",

};

const ASN1CType asn1_type_j2735TollgateLaneInformation_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x1,
  0x20,
  sizeof(j2735TollgateLane_KOR),
  (intptr_t)asn1_type_j2735TollgateLane_KOR,
  0,
};

const ASN1CType asn1_type_j2735_local_684[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100007,
  (intptr_t)asn1_type_j2735TimeInSecond_B8,
};

const ASN1CType asn1_type_j2735_local_685[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100008,
  (intptr_t)asn1_type_j2735TimeInSecond_B8,
};

const ASN1CType asn1_type_j2735_local_686[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  (intptr_t)asn1_type_j2735TrafficLightDirectionCode,
};

const ASN1CType asn1_type_j2735TrafficLightStatus[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  11,
  sizeof(j2735TrafficLightStatus),

  offsetof(j2735TrafficLightStatus, trafficLightType) | 0x0,
  (intptr_t)asn1_type_j2735TrafficLightType,
  0,
  (intptr_t)"trafficLightType",

  offsetof(j2735TrafficLightStatus, consecutiveTrafficLight) | 0x0,
  (intptr_t)asn1_type_j2735ConsecutiveTrafficLight,
  0,
  (intptr_t)"consecutiveTrafficLight",

  offsetof(j2735TrafficLightStatus, trafficLightIntervalType) | 0x0,
  (intptr_t)asn1_type_j2735TrafficLightIntervalType,
  0,
  (intptr_t)"trafficLightIntervalType",

  offsetof(j2735TrafficLightStatus, pedestrianCall) | 0x0,
  (intptr_t)asn1_type_j2735PedestrianCall,
  0,
  (intptr_t)"pedestrianCall",

  offsetof(j2735TrafficLightStatus, actuatedinterval) | 0x0,
  (intptr_t)asn1_type_j2735ActuatedInterval,
  0,
  (intptr_t)"actuatedinterval",

  offsetof(j2735TrafficLightStatus, permissiveNonProtected) | 0x0,
  (intptr_t)asn1_type_j2735PermissiveNonProtected,
  0,
  (intptr_t)"permissiveNonProtected",

  offsetof(j2735TrafficLightStatus, lightingStatus) | 0x0,
  (intptr_t)asn1_type_j2735TrafficLightingStatus,
  0,
  (intptr_t)"lightingStatus",

  offsetof(j2735TrafficLightStatus, maxIntervalLength) | 0x0,
  (intptr_t)asn1_type_j2735_local_684,
  0,
  (intptr_t)"maxIntervalLength",

  offsetof(j2735TrafficLightStatus, remainingTime) | 0x0,
  (intptr_t)asn1_type_j2735_local_685,
  0,
  (intptr_t)"remainingTime",

  offsetof(j2735TrafficLightStatus, directionCode) | 0x0,
  (intptr_t)asn1_type_j2735_local_686,
  0,
  (intptr_t)"directionCode",

  offsetof(j2735TrafficLightStatus, reserved) | 0x0,
  (intptr_t)asn1_type_j2735ReservedBit,
  0,
  (intptr_t)"reserved",

};

const ASN1CType asn1_type_j2735TrafficLightStatusList[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x1,
  0xff,
  sizeof(j2735TrafficLightStatus),
  (intptr_t)asn1_type_j2735TrafficLightStatus,
  0,
};

const ASN1CType asn1_type_j2735_local_687[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0x8,
};

const ASN1CType asn1_type_j2735_local_688[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735EventOrSvcInfo_KOR,
};

const ASN1CType asn1_type_j2735TravelerDataFrameAdditionalInformation_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  6,
  sizeof(j2735TravelerDataFrameAdditionalInformation_KOR),

  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, dataFrameIndex) | 0x0,
  (intptr_t)asn1_type_j2735_local_687,
  0,
  (intptr_t)"dataFrameIndex",

  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, events) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_688,
  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, events_option),
  (intptr_t)"events",

  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, vehicleRidingStatus) | 0x8000000,
  (intptr_t)asn1_type_j2735RidingNotification_KOR,
  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, vehicleRidingStatus_option),
  (intptr_t)"vehicleRidingStatus",

  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, weatherInfo) | 0x8000000,
  (intptr_t)asn1_type_j2735WeatherInfo_KOR,
  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, weatherInfo_option),
  (intptr_t)"weatherInfo",

  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, serviceArea) | 0x8000000,
  (intptr_t)asn1_type_j2735ServiceAreaInformation_KOR,
  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, serviceArea_option),
  (intptr_t)"serviceArea",

  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, tollgate) | 0x8000000,
  (intptr_t)asn1_type_j2735TollgateLaneInformation_KOR,
  offsetof(j2735TravelerDataFrameAdditionalInformation_KOR, tollgate_option),
  (intptr_t)"tollgate",

};

const ASN1CType asn1_type_j2735_local_689[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x400000 | 0x100000,
};

const ASN1CType asn1_type_j2735TravelerInformation_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  2,
  sizeof(j2735TravelerInformation_KOR),

  offsetof(j2735TravelerInformation_KOR, msgID) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_689,
  offsetof(j2735TravelerInformation_KOR, msgID_option),
  (intptr_t)"msgID",

  offsetof(j2735TravelerInformation_KOR, additionalInfo) | 0x8000000,
  (intptr_t)asn1_type_j2735TravelerInformation_KOR_1,
  offsetof(j2735TravelerInformation_KOR, additionalInfo_option),
  (intptr_t)"additionalInfo",

};

const ASN1CType asn1_type_j2735TravelerInformation_KOR_1[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x1,
  0x8,
  sizeof(j2735TravelerDataFrameAdditionalInformation_KOR),
  (intptr_t)asn1_type_j2735TravelerDataFrameAdditionalInformation_KOR,
  0,
};

const ASN1CType asn1_type_j2735VehicleClassification_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10,
  1,
  sizeof(j2735VehicleClassification_KOR),

  offsetof(j2735VehicleClassification_KOR, role) | 0x8000000,
  (intptr_t)asn1_type_j2735BasicVehicleRole_KOR,
  offsetof(j2735VehicleClassification_KOR, role_option),
  (intptr_t)"role",

};

const ASN1CType asn1_type_j2735_local_690[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735DrivingWheelAngle,
};

const ASN1CType asn1_type_j2735VehicleControlInfo_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10000a,
  3,
  sizeof(j2735VehicleControlInfo_KOR),

  offsetof(j2735VehicleControlInfo_KOR, mode) | 0x8000000,
  (intptr_t)asn1_type_j2735VehicleControlMode_KOR,
  offsetof(j2735VehicleControlInfo_KOR, mode_option),
  (intptr_t)"mode",

  offsetof(j2735VehicleControlInfo_KOR, turning) | 0x8000000,
  (intptr_t)asn1_type_j2735PossibleTurn_KOR,
  offsetof(j2735VehicleControlInfo_KOR, turning_option),
  (intptr_t)"turning",

  offsetof(j2735VehicleControlInfo_KOR, angle) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_690,
  offsetof(j2735VehicleControlInfo_KOR, angle_option),
  (intptr_t)"angle",

};

const ASN1CType asn1_type_j2735_local_691[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735AmbientAirTemperature,
};

const ASN1CType asn1_type_j2735_local_692[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100002,
  0x0,
  0x64,
};

const ASN1CType asn1_type_j2735_local_693[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735HeadingSlice,
};

const ASN1CType asn1_type_j2735_local_694[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x0,
  0x2710,
};

const ASN1CType asn1_type_j2735WeatherInfo_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100003,
  5,
  sizeof(j2735WeatherInfo_KOR),

  offsetof(j2735WeatherInfo_KOR, temp) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_691,
  offsetof(j2735WeatherInfo_KOR, temp_option),
  (intptr_t)"temp",

  offsetof(j2735WeatherInfo_KOR, rainfall) | 0x8000000,
  (intptr_t)asn1_type_j2735EssPrecipRate,
  offsetof(j2735WeatherInfo_KOR, rainfall_option),
  (intptr_t)"rainfall",

  offsetof(j2735WeatherInfo_KOR, humidity) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_692,
  offsetof(j2735WeatherInfo_KOR, humidity_option),
  (intptr_t)"humidity",

  offsetof(j2735WeatherInfo_KOR, windDir) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_693,
  offsetof(j2735WeatherInfo_KOR, windDir_option),
  (intptr_t)"windDir",

  offsetof(j2735WeatherInfo_KOR, windSpd) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_694,
  offsetof(j2735WeatherInfo_KOR, windSpd_option),
  (intptr_t)"windSpd",

};

const ASN1CType asn1_type_j2735_local_695[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735DrivingWheelAngle,
};

const ASN1CType asn1_type_j2735_local_696[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
  (intptr_t)asn1_type_j2735DrivingWheelAngle,
};

const ASN1CType asn1_type_j2735_local_697[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100002,
  (intptr_t)asn1_type_j2735DrivingWheelAngle,
};

const ASN1CType asn1_type_j2735_local_698[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
  (intptr_t)asn1_type_j2735DrivingWheelAngle,
};

const ASN1CType asn1_type_j2735WheelAngles_KOR[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100009,
  4,
  sizeof(j2735WheelAngles_KOR),

  offsetof(j2735WheelAngles_KOR, flWheelAngle) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_695,
  offsetof(j2735WheelAngles_KOR, flWheelAngle_option),
  (intptr_t)"flWheelAngle",

  offsetof(j2735WheelAngles_KOR, frWheelAngle) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_696,
  offsetof(j2735WheelAngles_KOR, frWheelAngle_option),
  (intptr_t)"frWheelAngle",

  offsetof(j2735WheelAngles_KOR, rlWheelAngle) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_697,
  offsetof(j2735WheelAngles_KOR, rlWheelAngle_option),
  (intptr_t)"rlWheelAngle",

  offsetof(j2735WheelAngles_KOR, rrWheelAngle) | 0x8000000,
  (intptr_t)asn1_type_j2735_local_698,
  offsetof(j2735WheelAngles_KOR, rrWheelAngle_option),
  (intptr_t)"rrWheelAngle",

};

const ASN1CType asn1_type_j2735ActuatedInterval[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100004,
};

const ASN1CType asn1_type_j2735ADASEventType_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  13,
  0,
  (intptr_t)"nowarning",
  (intptr_t)"bus",
  (intptr_t)"car",
  (intptr_t)"truck",
  (intptr_t)"motors",
  (intptr_t)"special",
  (intptr_t)"bicycle",
  (intptr_t)"pedestrian",
  (intptr_t)"pothole",
  (intptr_t)"rubbercon",
  (intptr_t)"ldwsLeft",
  (intptr_t)"ldwsRight",
  (intptr_t)"ldwsWarning",
};

const ASN1CType asn1_type_j2735BasicVehicleClass_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  42,
  (intptr_t)"unknown",
  (intptr_t)"passenger-vehicle-tiny",
  (intptr_t)"passenger-vehicle-mini",
  (intptr_t)"passenger-vehicle-small",
  (intptr_t)"passenger-vehicle-medium",
  (intptr_t)"passenger-vehicle-fullsize",
  (intptr_t)"van-mini",
  (intptr_t)"van-small",
  (intptr_t)"van-medium",
  (intptr_t)"van-fullsize",
  (intptr_t)"truck-tiny",
  (intptr_t)"truck-mini",
  (intptr_t)"truck-small-A",
  (intptr_t)"truck-small-B",
  (intptr_t)"truck-medium-A",
  (intptr_t)"truck-medium-B",
  (intptr_t)"truck-medium-C",
  (intptr_t)"truck-fullsize-A",
  (intptr_t)"truck-fullsize-B",
  (intptr_t)"truck-fullsize-C",
  (intptr_t)"truck-fullsize-D",
  (intptr_t)"truck-fullsize-E",
  (intptr_t)"special-vehicle-mini",
  (intptr_t)"special-vehicle-small",
  (intptr_t)"special-vehicle-medium",
  (intptr_t)"special-vehicle-fullsize",
  (intptr_t)"motorcycle-mini",
  (intptr_t)"motorcycle-small",
  (intptr_t)"motorcycle-medium",
  (intptr_t)"motorcycle-fullsize",
  (intptr_t)"heavy-equipment",
  (intptr_t)"motorcycle-tiny",
  (intptr_t)"motorized-bicycle",
  (intptr_t)"personal-mobility-kickboard",
  (intptr_t)"personal-mobility-segway",
  (intptr_t)"personal-mobility-throttle-bicycle",
  (intptr_t)"bicycle-general",
  (intptr_t)"bicycle-pas",
  (intptr_t)"horse-cattle",
  (intptr_t)"tram",
  (intptr_t)"pedestrian",
  (intptr_t)"etc",
};

const ASN1CType asn1_type_j2735BasicVehicleRole_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  35,
  (intptr_t)"unknown",
  (intptr_t)"general-passenger-vehicle",
  (intptr_t)"general-school-bus",
  (intptr_t)"general-bicycle",
  (intptr_t)"general-motorcycle",
  (intptr_t)"general-heavy-equipment",
  (intptr_t)"general-vehicle-with-trailer",
  (intptr_t)"general-pedestrian",
  (intptr_t)"general-personal-mobility",
  (intptr_t)"general-farming-vehicle",
  (intptr_t)"general-recreational-vehicle",
  (intptr_t)"general-etc",
  (intptr_t)"general-unknown",
  (intptr_t)"emergency-police",
  (intptr_t)"emergency-fire",
  (intptr_t)"emergency-ambulance",
  (intptr_t)"emergency-hazmat",
  (intptr_t)"emergency-military",
  (intptr_t)"emergency-prosecutor",
  (intptr_t)"emergency-prison",
  (intptr_t)"emergency-gov-affairs",
  (intptr_t)"emergency-pub-affairs",
  (intptr_t)"emergency-civil-defense",
  (intptr_t)"emergency-post",
  (intptr_t)"emerency-etc",
  (intptr_t)"public-transport-bus",
  (intptr_t)"public-transport-taxi",
  (intptr_t)"public-transport-tram",
  (intptr_t)"roadmgmt-construction-vehicle",
  (intptr_t)"roadmgmt-snow-plow",
  (intptr_t)"roadmgmt-brush-vehicle",
  (intptr_t)"roadmgmt-road-rescue",
  (intptr_t)"truck",
  (intptr_t)"oversize-truck",
  (intptr_t)"overweight-truck",
};

const ASN1CType asn1_type_j2735BrakePedalPressure_KOR[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100007,
  0x0,
  0x65,
};

const ASN1CType asn1_type_j2735BrakePedalCmd_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100006,
  2,
  (intptr_t)"off",
  (intptr_t)"on",
};

const ASN1CType asn1_type_j2735DectLane_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x3000000 | 0xa,
  31,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"busOnlyLane",
  (intptr_t)"ctrBusLane",
  (intptr_t)"rdSdBusLane",
  (intptr_t)"mergingLane",
  (intptr_t)"accelerationLane",
  (intptr_t)"decelrationLane",
  (intptr_t)"rampLane",
  (intptr_t)"shoulderLane",
  (intptr_t)"carLane",
  (intptr_t)"car-1stLane",
  (intptr_t)"car-2ndLane",
  (intptr_t)"car-3rdLane",
  (intptr_t)"car-4thLane",
  (intptr_t)"car-5thLane",
  (intptr_t)"car-6thLane",
  (intptr_t)"car-7thLane",
  (intptr_t)"car-8thLane",
  (intptr_t)"car-9thLane",
  (intptr_t)"car-10thLane",
  (intptr_t)"car-11thLane",
  (intptr_t)"car-12thLane",
  (intptr_t)"car-13thLane",
  (intptr_t)"car-14thLane",
  (intptr_t)"car-15thLane",
  (intptr_t)"car-16thLane",
  (intptr_t)"car-17thLane",
  (intptr_t)"car-18thLane",
  (intptr_t)"car-19thLane",
  (intptr_t)"car-20thLane",
  (intptr_t)"etcLane",
  0,
  10,
  11,
  12,
  13,
  14,
  15,
  16,
  17,
  20,
  21,
  22,
  23,
  24,
  25,
  26,
  27,
  28,
  29,
  30,
  31,
  32,
  33,
  34,
  35,
  36,
  37,
  38,
  39,
  40,
  99,
  0,
  0,
  10,
  1,
  11,
  2,
  12,
  3,
  13,
  4,
  14,
  5,
  15,
  6,
  16,
  7,
  17,
  8,
  20,
  9,
  21,
  10,
  22,
  11,
  23,
  12,
  24,
  13,
  25,
  14,
  26,
  15,
  27,
  16,
  28,
  17,
  29,
  18,
  30,
  19,
  31,
  20,
  32,
  21,
  33,
  22,
  34,
  23,
  35,
  24,
  36,
  25,
  37,
  26,
  38,
  27,
  39,
  28,
  40,
  29,
  99,
  30,
};

const ASN1CType asn1_type_j2735DeviceStatus_KOR[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x8,
  0x8,
};

const ASN1CType asn1_type_j2735Disabled_KOR[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
};

const ASN1CType asn1_type_j2735DrivingStatus_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10000b,
  7,
  0,
  (intptr_t)"stop",
  (intptr_t)"forward-moving",
  (intptr_t)"backward-moving",
  (intptr_t)"straight",
  (intptr_t)"left-turning",
  (intptr_t)"right-turning",
  (intptr_t)"parking",
};

const ASN1CType asn1_type_j2735FuelType_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  8,
  0,
  (intptr_t)"gasoline",
  (intptr_t)"diesel",
  (intptr_t)"lpg",
  (intptr_t)"premium",
  (intptr_t)"hydrogen",
  (intptr_t)"normalCharging",
  (intptr_t)"quickCharging",
  (intptr_t)"supercharging",
};

const ASN1CType asn1_type_j2735HighwayDirection_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0xa,
  3,
  0,
  (intptr_t)"up",
  (intptr_t)"down",
  (intptr_t)"upAndDown",
};

const ASN1CType asn1_type_j2735HumanDriverStatus_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100003,
  7,
  0,
  (intptr_t)"frontEyeOut",
  (intptr_t)"drowsyLight",
  (intptr_t)"drowsySerious",
  (intptr_t)"faceOut",
  (intptr_t)"smoking",
  (intptr_t)"phone",
  (intptr_t)"yawn",
};

const ASN1CType asn1_type_j2735ITISCodes_KOR[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735MsgChannel_KOR[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100006,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735MsgCommType_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100005,
  11,
  0,
  (intptr_t)"unavailable",
  (intptr_t)"wave",
  (intptr_t)"lte-4G",
  (intptr_t)"lte-5G",
  (intptr_t)"lte-v2x-rel14",
  (intptr_t)"lte-v2x-rel15",
  (intptr_t)"lte-v2x-rel16",
  (intptr_t)"lte-v2x-rel17",
  (intptr_t)"lte-v2x-rel18",
  (intptr_t)"lte-v2x-rel19",
  (intptr_t)"lte-v2x-rel20",
};

const ASN1CType asn1_type_j2735MsgDeviceType_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100007,
  5,
  0,
  (intptr_t)"other",
  (intptr_t)"communication",
  (intptr_t)"display",
  (intptr_t)"adas",
  (intptr_t)"drowsinessDetector",
};

const ASN1CType asn1_type_j2735MsgDisplayLocation_KOR[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x80,
};

const ASN1CType asn1_type_j2735MsgType_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  13,
  0,
  (intptr_t)"bsm",
  (intptr_t)"spat",
  (intptr_t)"tls",
  (intptr_t)"map",
  (intptr_t)"nmeacorrection",
  (intptr_t)"rtcmcorrection",
  (intptr_t)"pvd",
  (intptr_t)"pdm",
  (intptr_t)"rsa",
  (intptr_t)"tim",
  (intptr_t)"adas",
  (intptr_t)"psm",
  (intptr_t)"drowsiness",
};

const ASN1CType asn1_type_j2735ParkingType_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  7,
  0,
  (intptr_t)"normal",
  (intptr_t)"disable",
  (intptr_t)"big",
  (intptr_t)"compact",
  (intptr_t)"emergency",
  (intptr_t)"women",
  (intptr_t)"etc",
};

const ASN1CType asn1_type_j2735PedestrianCall[] = {
  (ASN1_CTYPE_BOOLEAN << ASN1_CTYPE_SHIFT) | 0x0 | 0x100003,
};

const ASN1CType asn1_type_j2735PermissiveNonProtected[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100005,
  2,
  0,
  (intptr_t)"none",
  (intptr_t)"permissive",
};

const ASN1CType asn1_type_j2735PossibleTurn_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100001,
  4,
  0,
  (intptr_t)"none",
  (intptr_t)"left-turn",
  (intptr_t)"right-turn",
  (intptr_t)"both-turn",
};

const ASN1CType asn1_type_j2735ResponseType_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x10000a,
  8,
  0,
  (intptr_t)"not-in-use",
  (intptr_t)"emergency",
  (intptr_t)"non-emergency",
  (intptr_t)"emergency-in-drill",
  (intptr_t)"emergency-pursuit",
  (intptr_t)"emergency-stationary",
  (intptr_t)"emergency-slow-moving",
  (intptr_t)"emergency-stop-go",
};

const ASN1CType asn1_type_j2735Rpm_KOR[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100000,
  0x0,
  0x3fff,
};

const ASN1CType asn1_type_j2735TimeInSecond_B16[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735TimeInSecond_B8[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xff,
};

const ASN1CType asn1_type_j2735TollgateType_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x3000000 | 0x100002,
  12,
  0,
  (intptr_t)"general",
  (intptr_t)"hipass",
  (intptr_t)"truck-hipass",
  (intptr_t)"multi-hipass-2",
  (intptr_t)"multi-hipass-3",
  (intptr_t)"multi-hipass-4",
  (intptr_t)"multi-hipass-5",
  (intptr_t)"multi-hipass-6",
  (intptr_t)"multi-hipass-7",
  (intptr_t)"multi-hipass-8",
  (intptr_t)"multi-hipass-9",
  (intptr_t)"multi-hipass-10",
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  1,
  0,
  2,
  1,
  3,
  2,
  4,
  3,
  5,
  4,
  6,
  5,
  7,
  6,
  8,
  7,
  9,
  8,
  10,
  9,
  11,
  10,
  12,
  11,
};

const ASN1CType asn1_type_j2735TrafficLightControllerStatus[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x100004,
  0x8,
  0x8,
};

const ASN1CType asn1_type_j2735TrafficLightDirectionCode[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0x167,
};

const ASN1CType asn1_type_j2735_local_699[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x400000 | 0x100000,
};

const ASN1CType asn1_type_j2735_local_700[] = {
  (ASN1_CTYPE_OBJECT_IDENTIFIER << ASN1_CTYPE_SHIFT) | 0x0 | 0x100001,
};

const ASN1CType asn1_type_j2735TrafficLightID[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735TrafficLightID),
  offsetof(j2735TrafficLightID, choice),
  offsetof(j2735TrafficLightID, u),
  (intptr_t)asn1_type_j2735_local_699,
  (intptr_t)"id",
  (intptr_t)asn1_type_j2735_local_700,
  (intptr_t)"oid",
};

const ASN1CType asn1_type_j2735TrafficLightingStatus[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100006,
  7,
  0,
  (intptr_t)"off",
  (intptr_t)"red-on",
  (intptr_t)"yellow-on",
  (intptr_t)"green-on",
  (intptr_t)"red-blinking",
  (intptr_t)"yellow-blinking",
  (intptr_t)"green-blinking",
};

const ASN1CType asn1_type_j2735TrafficLightIntervalType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100002,
  2,
  0,
  (intptr_t)"static",
  (intptr_t)"variable",
};

const ASN1CType asn1_type_j2735TrafficLightOperationStatus[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x3800000 | 0x100003,
  0x8,
  0x8,
};

const ASN1CType asn1_type_j2735TrafficLightType[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  8,
  0,
  (intptr_t)"reserved",
  (intptr_t)"straight",
  (intptr_t)"left-turn",
  (intptr_t)"pedestrian",
  (intptr_t)"bicycle",
  (intptr_t)"right-turn",
  (intptr_t)"bus",
  (intptr_t)"u-turn",
};

const ASN1CType asn1_type_j2735VehicleControlMode_KOR[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x2000000 | 0x100000,
  2,
  0,
  (intptr_t)"manual",
  (intptr_t)"auto",
};

const ASN1CType asn1_type_j2735VehicleEventFlags_KOR[] = {
  (ASN1_CTYPE_BIT_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100008,
  0x8,
  0x8,
};

const ASN1CType asn1_type_j2735VerticalHeading_KOR[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100005,
  0xffffff9b,
  0x64,
};

const ASN1CType asn1_type_j2735EssMobileFriction[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100004,
  0x0,
  0x65,
};

const ASN1CType asn1_type_j2735EssPrecipRate[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735EssPrecipSituation[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x1000000 | 0x100002,
  15,
  (intptr_t)"other",
  (intptr_t)"unknown",
  (intptr_t)"noPrecipitation",
  (intptr_t)"unidentifiedSlight",
  (intptr_t)"unidentifiedModerate",
  (intptr_t)"unidentifiedHeavy",
  (intptr_t)"snowSlight",
  (intptr_t)"snowModerate",
  (intptr_t)"snowHeavy",
  (intptr_t)"rainSlight",
  (intptr_t)"rainModerate",
  (intptr_t)"rainHeavy",
  (intptr_t)"frozenPrecipitationSlight",
  (intptr_t)"frozenPrecipitationModerate",
  (intptr_t)"frozenPrecipitationHeavy",
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  13,
  14,
  15,
  1,
  0,
  2,
  1,
  3,
  2,
  4,
  3,
  5,
  4,
  6,
  5,
  7,
  6,
  8,
  7,
  9,
  8,
  10,
  9,
  11,
  10,
  12,
  11,
  13,
  12,
  14,
  13,
  15,
  14,
};

const ASN1CType asn1_type_j2735EssPrecipYesNo[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x1000000 | 0x100000,
  3,
  (intptr_t)"precip",
  (intptr_t)"noPrecip",
  (intptr_t)"error",
  1,
  2,
  3,
  1,
  0,
  2,
  1,
  3,
  2,
};

const ASN1CType asn1_type_j2735EssSolarRadiation[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100003,
  0x0,
  0xffff,
};

const ASN1CType asn1_type_j2735GenericLocations[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x3000000 | 0xa,
  96,
  0,
  (intptr_t)"on-bridges",
  (intptr_t)"in-tunnels",
  (intptr_t)"entering-or-leaving-tunnels",
  (intptr_t)"on-ramps",
  (intptr_t)"in-road-construction-area",
  (intptr_t)"around-a-curve",
  (intptr_t)"on-curve",
  (intptr_t)"on-tracks",
  (intptr_t)"in-street",
  (intptr_t)"shoulder",
  (intptr_t)"on-minor-roads",
  (intptr_t)"in-the-opposing-lanes",
  (intptr_t)"adjacent-to-roadway",
  (intptr_t)"across-tracks",
  (intptr_t)"on-bend",
  (intptr_t)"intersection",
  (intptr_t)"entire-intersection",
  (intptr_t)"in-the-median",
  (intptr_t)"moved-to-side-of-road",
  (intptr_t)"moved-to-shoulder",
  (intptr_t)"on-the-roadway",
  (intptr_t)"dip",
  (intptr_t)"traffic-circle",
  (intptr_t)"crossover",
  (intptr_t)"cross-road",
  (intptr_t)"side-road",
  (intptr_t)"to",
  (intptr_t)"by",
  (intptr_t)"through",
  (intptr_t)"area-of",
  (intptr_t)"under",
  (intptr_t)"over",
  (intptr_t)"from",
  (intptr_t)"approaching",
  (intptr_t)"entering-at",
  (intptr_t)"exiting-at",
  (intptr_t)"in-shaded-areas",
  (intptr_t)"in-low-lying-areas",
  (intptr_t)"in-the-downtown-area",
  (intptr_t)"in-the-inner-city-area",
  (intptr_t)"in-parts",
  (intptr_t)"in-some-places",
  (intptr_t)"in-the-ditch",
  (intptr_t)"in-the-valley",
  (intptr_t)"on-hill-top",
  (intptr_t)"near-the-foothills",
  (intptr_t)"at-high-altitudes",
  (intptr_t)"near-the-lake",
  (intptr_t)"near-the-shore",
  (intptr_t)"nearby-basin",
  (intptr_t)"over-the-crest-of-a-hill",
  (intptr_t)"other-than-on-the-roadway",
  (intptr_t)"near-the-beach",
  (intptr_t)"near-beach-access-point",
  (intptr_t)"mountain-pass",
  (intptr_t)"lower-level",
  (intptr_t)"upper-level",
  (intptr_t)"airport",
  (intptr_t)"concourse",
  (intptr_t)"gate",
  (intptr_t)"baggage-claim",
  (intptr_t)"customs-point",
  (intptr_t)"reservation-center",
  (intptr_t)"station",
  (intptr_t)"platform",
  (intptr_t)"dock",
  (intptr_t)"depot",
  (intptr_t)"ev-charging-point",
  (intptr_t)"information-welcome-point",
  (intptr_t)"at-rest-area",
  (intptr_t)"at-service-area",
  (intptr_t)"at-weigh-station",
  (intptr_t)"roadside-park",
  (intptr_t)"picnic-areas",
  (intptr_t)"rest-area",
  (intptr_t)"service-stations",
  (intptr_t)"toilets",
  (intptr_t)"bus-stop",
  (intptr_t)"park-and-ride-lot",
  (intptr_t)"on-the-right",
  (intptr_t)"on-the-left",
  (intptr_t)"in-the-center",
  (intptr_t)"in-the-opposite-direction",
  (intptr_t)"cross-traffic",
  (intptr_t)"northbound-traffic",
  (intptr_t)"eastbound-traffic",
  (intptr_t)"southbound-traffic",
  (intptr_t)"westbound-traffic",
  (intptr_t)"north",
  (intptr_t)"south",
  (intptr_t)"east",
  (intptr_t)"west",
  (intptr_t)"northeast",
  (intptr_t)"northwest",
  (intptr_t)"southeast",
  (intptr_t)"southwest",
  7937,
  7938,
  7939,
  7940,
  7941,
  7942,
  8026,
  8009,
  8025,
  8027,
  7943,
  7944,
  7945,
  8024,
  7946,
  8032,
  7947,
  7948,
  7949,
  7950,
  7951,
  8010,
  8011,
  8028,
  8029,
  8030,
  8014,
  8015,
  8016,
  8017,
  8018,
  8019,
  8020,
  8021,
  8022,
  8023,
  7952,
  7953,
  7954,
  7955,
  7956,
  7957,
  7958,
  7959,
  7960,
  7961,
  7962,
  7963,
  7964,
  8008,
  7965,
  7966,
  7967,
  7968,
  8006,
  7969,
  7970,
  7971,
  7972,
  7973,
  7974,
  7975,
  8007,
  7976,
  7977,
  7978,
  7979,
  7980,
  7981,
  7982,
  7983,
  7984,
  8033,
  7985,
  7986,
  7987,
  7988,
  8031,
  8012,
  7989,
  7990,
  7991,
  7992,
  7993,
  7994,
  7995,
  7996,
  7997,
  7998,
  7999,
  8000,
  8001,
  8002,
  8003,
  8004,
  8005,
  7937,
  0,
  7938,
  1,
  7939,
  2,
  7940,
  3,
  7941,
  4,
  7942,
  5,
  7943,
  10,
  7944,
  11,
  7945,
  12,
  7946,
  14,
  7947,
  16,
  7948,
  17,
  7949,
  18,
  7950,
  19,
  7951,
  20,
  7952,
  36,
  7953,
  37,
  7954,
  38,
  7955,
  39,
  7956,
  40,
  7957,
  41,
  7958,
  42,
  7959,
  43,
  7960,
  44,
  7961,
  45,
  7962,
  46,
  7963,
  47,
  7964,
  48,
  7965,
  50,
  7966,
  51,
  7967,
  52,
  7968,
  53,
  7969,
  55,
  7970,
  56,
  7971,
  57,
  7972,
  58,
  7973,
  59,
  7974,
  60,
  7975,
  61,
  7976,
  63,
  7977,
  64,
  7978,
  65,
  7979,
  66,
  7980,
  67,
  7981,
  68,
  7982,
  69,
  7983,
  70,
  7984,
  71,
  7985,
  73,
  7986,
  74,
  7987,
  75,
  7988,
  76,
  7989,
  79,
  7990,
  80,
  7991,
  81,
  7992,
  82,
  7993,
  83,
  7994,
  84,
  7995,
  85,
  7996,
  86,
  7997,
  87,
  7998,
  88,
  7999,
  89,
  8000,
  90,
  8001,
  91,
  8002,
  92,
  8003,
  93,
  8004,
  94,
  8005,
  95,
  8006,
  54,
  8007,
  62,
  8008,
  49,
  8009,
  7,
  8010,
  21,
  8011,
  22,
  8012,
  78,
  8014,
  26,
  8015,
  27,
  8016,
  28,
  8017,
  29,
  8018,
  30,
  8019,
  31,
  8020,
  32,
  8021,
  33,
  8022,
  34,
  8023,
  35,
  8024,
  13,
  8025,
  8,
  8026,
  6,
  8027,
  9,
  8028,
  23,
  8029,
  24,
  8030,
  25,
  8031,
  77,
  8032,
  15,
  8033,
  72,
};

const ASN1CType asn1_type_j2735IncidentResponseEquipment[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x3000000 | 0xa,
  72,
  0,
  (intptr_t)"ground-fire-suppression",
  (intptr_t)"heavy-ground-equipment",
  (intptr_t)"aircraft",
  (intptr_t)"marine-equipment",
  (intptr_t)"support-equipment",
  (intptr_t)"medical-rescue-unit",
  (intptr_t)"other",
  (intptr_t)"ground-fire-suppression-other",
  (intptr_t)"engine",
  (intptr_t)"truck-or-aerial",
  (intptr_t)"quint",
  (intptr_t)"tanker-pumper-combination",
  (intptr_t)"brush-truck",
  (intptr_t)"aircraft-rescue-firefighting",
  (intptr_t)"heavy-ground-equipment-other",
  (intptr_t)"dozer-or-plow",
  (intptr_t)"tractor",
  (intptr_t)"tanker-or-tender",
  (intptr_t)"aircraft-other",
  (intptr_t)"aircraft-fixed-wing-tanker",
  (intptr_t)"helitanker",
  (intptr_t)"helicopter",
  (intptr_t)"marine-equipment-other",
  (intptr_t)"fire-boat-with-pump",
  (intptr_t)"boat-no-pump",
  (intptr_t)"support-apparatus-other",
  (intptr_t)"breathing-apparatus-support",
  (intptr_t)"light-and-air-unit",
  (intptr_t)"medical-rescue-unit-other",
  (intptr_t)"rescue-unit",
  (intptr_t)"urban-search-rescue-unit",
  (intptr_t)"high-angle-rescue",
  (intptr_t)"crash-fire-rescue",
  (intptr_t)"bLS-unit",
  (intptr_t)"aLS-unit",
  (intptr_t)"mobile-command-post",
  (intptr_t)"chief-officer-car",
  (intptr_t)"hAZMAT-unit",
  (intptr_t)"type-i-hand-crew",
  (intptr_t)"type-ii-hand-crew",
  (intptr_t)"privately-owned-vehicle",
  (intptr_t)"other-apparatus-resource",
  (intptr_t)"ambulance",
  (intptr_t)"bomb-squad-van",
  (intptr_t)"combine-harvester",
  (intptr_t)"construction-vehicle",
  (intptr_t)"farm-tractor",
  (intptr_t)"grass-cutting-machines",
  (intptr_t)"hAZMAT-containment-tow",
  (intptr_t)"heavy-tow",
  (intptr_t)"light-tow",
  (intptr_t)"flatbed-tow",
  (intptr_t)"hedge-cutting-machines",
  (intptr_t)"mobile-crane",
  (intptr_t)"refuse-collection-vehicle",
  (intptr_t)"resurfacing-vehicle",
  (intptr_t)"road-sweeper",
  (intptr_t)"roadside-litter-collection-crews",
  (intptr_t)"salvage-vehicle",
  (intptr_t)"sand-truck",
  (intptr_t)"snowplow",
  (intptr_t)"steam-roller",
  (intptr_t)"swat-team-van",
  (intptr_t)"track-laying-vehicle",
  (intptr_t)"unknown-vehicle",
  (intptr_t)"white-lining-vehicle",
  (intptr_t)"dump-truck",
  (intptr_t)"supervisor-vehicle",
  (intptr_t)"snow-blower",
  (intptr_t)"rotary-snow-blower",
  (intptr_t)"road-grader",
  (intptr_t)"steam-truck",
  9985,
  9986,
  9988,
  9989,
  9990,
  9991,
  9993,
  9994,
  9995,
  9996,
  9997,
  9998,
  10000,
  10001,
  10004,
  10005,
  10006,
  10008,
  10024,
  10025,
  10026,
  10027,
  10034,
  10035,
  10036,
  10044,
  10045,
  10046,
  10054,
  10055,
  10056,
  10057,
  10058,
  10059,
  10060,
  10075,
  10076,
  10077,
  10078,
  10079,
  10083,
  10084,
  10085,
  10086,
  10087,
  10088,
  10089,
  10090,
  10091,
  10092,
  10094,
  10114,
  10093,
  10095,
  10096,
  10097,
  10098,
  10099,
  10100,
  10101,
  10102,
  10103,
  10104,
  10105,
  10106,
  10107,
  10108,
  10109,
  10110,
  10111,
  10112,
  10113,
  9985,
  0,
  9986,
  1,
  9988,
  2,
  9989,
  3,
  9990,
  4,
  9991,
  5,
  9993,
  6,
  9994,
  7,
  9995,
  8,
  9996,
  9,
  9997,
  10,
  9998,
  11,
  10000,
  12,
  10001,
  13,
  10004,
  14,
  10005,
  15,
  10006,
  16,
  10008,
  17,
  10024,
  18,
  10025,
  19,
  10026,
  20,
  10027,
  21,
  10034,
  22,
  10035,
  23,
  10036,
  24,
  10044,
  25,
  10045,
  26,
  10046,
  27,
  10054,
  28,
  10055,
  29,
  10056,
  30,
  10057,
  31,
  10058,
  32,
  10059,
  33,
  10060,
  34,
  10075,
  35,
  10076,
  36,
  10077,
  37,
  10078,
  38,
  10079,
  39,
  10083,
  40,
  10084,
  41,
  10085,
  42,
  10086,
  43,
  10087,
  44,
  10088,
  45,
  10089,
  46,
  10090,
  47,
  10091,
  48,
  10092,
  49,
  10093,
  52,
  10094,
  50,
  10095,
  53,
  10096,
  54,
  10097,
  55,
  10098,
  56,
  10099,
  57,
  10100,
  58,
  10101,
  59,
  10102,
  60,
  10103,
  61,
  10104,
  62,
  10105,
  63,
  10106,
  64,
  10107,
  65,
  10108,
  66,
  10109,
  67,
  10110,
  68,
  10111,
  69,
  10112,
  70,
  10113,
  71,
  10114,
  51,
};

const ASN1CType asn1_type_j2735ITIStext[] = {
  (ASN1_CTYPE_CHAR_STRING << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x100001,
  ASN1_CSTR_IA5String,
  0x1,
  0x1f4,
  1,
  0x0,
  0x7f,
};

const ASN1CType asn1_type_j2735ResponderGroupAffected[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x3000000 | 0xa,
  14,
  0,
  (intptr_t)"emergency-vehicle-units",
  (intptr_t)"federal-law-enforcement-units",
  (intptr_t)"state-police-units",
  (intptr_t)"county-police-units",
  (intptr_t)"local-police-units",
  (intptr_t)"ambulance-units",
  (intptr_t)"rescue-units",
  (intptr_t)"fire-units",
  (intptr_t)"hAZMAT-units",
  (intptr_t)"light-tow-unit",
  (intptr_t)"heavy-tow-unit",
  (intptr_t)"freeway-service-patrols",
  (intptr_t)"transportation-response-units",
  (intptr_t)"private-contractor-response-units",
  9729,
  9730,
  9731,
  9732,
  9733,
  9734,
  9735,
  9736,
  9737,
  9738,
  9739,
  9740,
  9741,
  9742,
  9729,
  0,
  9730,
  1,
  9731,
  2,
  9732,
  3,
  9733,
  4,
  9734,
  5,
  9735,
  6,
  9736,
  7,
  9737,
  8,
  9738,
  9,
  9739,
  10,
  9740,
  11,
  9741,
  12,
  9742,
  13,
};

const ASN1CType asn1_type_j2735VehicleGroupAffected[] = {
  (ASN1_CTYPE_ENUMERATED << ASN1_CTYPE_SHIFT) | 0x3000000 | 0xa,
  35,
  0,
  (intptr_t)"all-vehicles",
  (intptr_t)"bicycles",
  (intptr_t)"motorcycles",
  (intptr_t)"cars",
  (intptr_t)"light-vehicles",
  (intptr_t)"cars-and-light-vehicles",
  (intptr_t)"cars-with-trailers",
  (intptr_t)"cars-with-recreational-trailers",
  (intptr_t)"vehicles-with-trailers",
  (intptr_t)"heavy-vehicles",
  (intptr_t)"trucks",
  (intptr_t)"buses",
  (intptr_t)"articulated-buses",
  (intptr_t)"school-buses",
  (intptr_t)"vehicles-with-semi-trailers",
  (intptr_t)"vehicles-with-double-trailers",
  (intptr_t)"high-profile-vehicles",
  (intptr_t)"wide-vehicles",
  (intptr_t)"long-vehicles",
  (intptr_t)"hazardous-loads",
  (intptr_t)"exceptional-loads",
  (intptr_t)"abnormal-loads",
  (intptr_t)"convoys",
  (intptr_t)"maintenance-vehicles",
  (intptr_t)"delivery-vehicles",
  (intptr_t)"vehicles-with-even-numbered-license-plates",
  (intptr_t)"vehicles-with-odd-numbered-license-plates",
  (intptr_t)"vehicles-with-parking-permits",
  (intptr_t)"vehicles-with-catalytic-converters",
  (intptr_t)"vehicles-without-catalytic-converters",
  (intptr_t)"gas-powered-vehicles",
  (intptr_t)"diesel-powered-vehicles",
  (intptr_t)"lPG-vehicles",
  (intptr_t)"military-convoys",
  (intptr_t)"military-vehicles",
  9217,
  9218,
  9219,
  9220,
  9221,
  9222,
  9223,
  9224,
  9225,
  9226,
  9227,
  9228,
  9229,
  9230,
  9231,
  9232,
  9233,
  9234,
  9235,
  9236,
  9237,
  9238,
  9239,
  9240,
  9241,
  9242,
  9243,
  9244,
  9245,
  9246,
  9247,
  9248,
  9249,
  9250,
  9251,
  9217,
  0,
  9218,
  1,
  9219,
  2,
  9220,
  3,
  9221,
  4,
  9222,
  5,
  9223,
  6,
  9224,
  7,
  9225,
  8,
  9226,
  9,
  9227,
  10,
  9228,
  11,
  9229,
  12,
  9230,
  13,
  9231,
  14,
  9232,
  15,
  9233,
  16,
  9234,
  17,
  9235,
  18,
  9236,
  19,
  9237,
  20,
  9238,
  21,
  9239,
  22,
  9240,
  23,
  9241,
  24,
  9242,
  25,
  9243,
  26,
  9244,
  27,
  9245,
  28,
  9246,
  29,
  9247,
  30,
  9248,
  31,
  9249,
  32,
  9250,
  33,
  9251,
  34,
};

const ASN1CType asn1_type_j2735ITIScodesAndText[] = {
  (ASN1_CTYPE_SEQUENCE_OF << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x10,
  0x1,
  0x64,
  sizeof(j2735ITIScodesAndText_2),
  (intptr_t)asn1_type_j2735ITIScodesAndText_2,
  0,
};

const ASN1CType asn1_type_j2735_local_701[] = {
  (ASN1_CTYPE_TAGGED << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  (intptr_t)asn1_type_j2735ITIScodes,
};

const ASN1CType asn1_type_j2735ITIScodesAndText_1[] = {
  (ASN1_CTYPE_CHOICE << ASN1_CTYPE_SHIFT) | 0x0 | 0x100000,
  2,
  sizeof(j2735ITIScodesAndText_1),
  offsetof(j2735ITIScodesAndText_1, choice),
  offsetof(j2735ITIScodesAndText_1, u),
  (intptr_t)asn1_type_j2735_local_701,
  (intptr_t)"itis",
  (intptr_t)asn1_type_j2735ITIStext,
  (intptr_t)"text",
};

const ASN1CType asn1_type_j2735ITIScodesAndText_2[] = {
  (ASN1_CTYPE_SEQUENCE << ASN1_CTYPE_SHIFT) | 0x0 | 0x10,
  1,
  sizeof(j2735ITIScodesAndText_2),

  offsetof(j2735ITIScodesAndText_2, item) | 0x0,
  (intptr_t)asn1_type_j2735ITIScodesAndText_1,
  0,
  (intptr_t)"item",

};

const ASN1CType asn1_type_j2735ITIScodes[] = {
  (ASN1_CTYPE_INTEGER << ASN1_CTYPE_SHIFT) | 0x1800000 | 0x2,
  0x0,
  0xffff,
};

