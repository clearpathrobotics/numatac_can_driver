#ifndef BT_RTCAN_H_
#define BT_RTCAN_H_

#include "bt_circular_buffer.h"

//#define DEFAULT_TIMER
#define MACHINE_TIMER

#define MAX_BIOTACS_FOR_JACO              3             // should be 3 for BarrettHand BH8-280

//=========================================================================
// DEFAULT CONSTANTS
//=========================================================================
#define BT_SPI_BITRATE_KHZ_DEFAULT            3000
#define BT_AFTERSAMPLE_DELAY_DEFAULT          50000     /* Delay after sampling command */
#define BT_INTERWORD_DELAY_DEFAULT            10000   /* Delay between words in communication */
#define BT_SAMPLE_RATE_HZ_DEFAULT           3000
#define BT_FRAMES_IN_BATCH_DEFAULT            1
#define BT_BATCH_MS_DEFAULT               1/BT_SAMPLE_RATE_HZ_DEFAULT*1000
#define BT_FRAME_STRUCTURE_DEFAULT            {\
                            BT_E01_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_E02_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_E03_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_E04_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_E05_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_E06_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_E07_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_E08_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_E09_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_E10_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_E11_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_E12_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_E13_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_E14_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_E15_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_E16_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_E17_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_E18_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_E19_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_PDC_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, \
                            BT_TAC_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, BT_TDC_SAMPLING_COMMAND, BT_PAC_SAMPLING_COMMAND, '\0'}
#ifndef BOOL
typedef int                       BOOL;
#endif
#define PARITY_GOOD                   (BOOL)0
#define PARITY_BAD                    (BOOL)!PARITY_GOOD

//=========================================================================
// TYPEDEFS
//=========================================================================

#include <stdint.h>
typedef uint8_t   u08;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint64_t  u64;
typedef int8_t    s08;
typedef int16_t   s16;
typedef int32_t   s32;
typedef int64_t   s64;

//=========================================================================
// DEFAULT CONSTANTS
//=========================================================================
#ifndef BOOL
typedef int                       BOOL;
#endif
#define PARITY_GOOD                   (BOOL)0
#define PARITY_BAD                    (BOOL)!PARITY_GOOD
#define PARITY_NOT_AVALIBLE               -1

//=========================================================================
// BioTac commands define
//=========================================================================
#define BT_PAC_SAMPLING                 0           // command index: 00
#define BT_PDC_SAMPLING                 1           // command index: 01
#define BT_TAC_SAMPLING                 2           // command index: 02
#define BT_TDC_SAMPLING                 3           // command index: 03

#define BT_E01_SAMPLING                 17            // command index: 17
#define BT_E02_SAMPLING                 18            // command index: 18
#define BT_E03_SAMPLING                 19            // command index: 19
#define BT_E04_SAMPLING                 20            // command index: 20
#define BT_E05_SAMPLING                 21            // command index: 21
#define BT_E06_SAMPLING                 22            // command index: 22
#define BT_E07_SAMPLING                 23            // command index: 23
#define BT_E08_SAMPLING                 24            // command index: 24
#define BT_E09_SAMPLING                 25            // command index: 25
#define BT_E10_SAMPLING                 26            // command index: 26
#define BT_E11_SAMPLING                 27            // command index: 27
#define BT_E12_SAMPLING                 28            // command index: 28
#define BT_E13_SAMPLING                 29            // command index: 29
#define BT_E14_SAMPLING                 30            // command index: 30
#define BT_E15_SAMPLING                 31            // command index: 31
#define BT_E16_SAMPLING                 32            // command index: 32
#define BT_E17_SAMPLING                 33            // command index: 33
#define BT_E18_SAMPLING                 34            // command index: 34
#define BT_E19_SAMPLING                 35            // command index: 35

#define BT_SAMPLING_PATTERN_READ_LENGTH         44
#define BT_CPU_SPEED_READ_LENGTH            2

//=========================================================================
// Error codes definitions
//=========================================================================
#define BT_OK                     0
#define BT_WRONG_MAX_BIOTAC_NUMBER            -3
#define BT_RTCAN_WRONG_ARGUMENT             -10

//=========================================================================
// channel id mode definitions
//=========================================================================
#define INTEGER                     1
#define STRING                      2

//=========================================================================
// time mode definitions
//=========================================================================
#define ZERO                      0
#define ELAPSED                     1
#define ABSOLUTE_TIME                 2

//=========================================================================
// Data structure definition
//=========================================================================
typedef int BioTac;

typedef struct
{
  unsigned long long index;
  double time;
  double frame_index;
  double batch_index;
  u08 channel_id;
  union
  {
    u16 word;
    u08 byte[2];
  } d[MAX_BIOTACS_FOR_JACO];
  s08 bt_parity[MAX_BIOTACS_FOR_JACO];
} bt_data;

typedef struct
{
  int spi_clock_speed;
  int number_of_biotacs;
  int sample_rate_Hz;
  struct
  {
    int frame_type;
    int frame_size;
    char frame_structure[100];
  } frame;
  struct
  {
    int batch_frame_count;
    int batch_ms;
  } batch;
} bt_info;

//=========================================================================
// CONSTANTS
//=========================================================================
static const unsigned char parity_values[] = \
{
  0x01, 0x02, 0x04, 0x07, 0x08, 0x0B, 0x0D, 0x0E, \
  0x10, 0x13, 0x15, 0x16, 0x19, 0x1A, 0x1C, 0x1F, \
  0x20, 0x23, 0x25, 0x26, 0x29, 0x2A, 0x2C, 0x2F, \
  0x31, 0x32, 0x34, 0x37, 0x38, 0x3B, 0x3D, 0x3E, \
  0x40, 0x43, 0x45, 0x46, 0x49, 0x4A, 0x4C, 0x4F, \
  0x51, 0x52, 0x54, 0x57, 0x58, 0x5B, 0x5D, 0x5E, \
  0x61, 0x62, 0x64, 0x67, 0x68, 0x6B, 0x6D, 0x6E, \
  0x70, 0x73, 0x75, 0x76, 0x79, 0x7A, 0x7C, 0x7F, \
  0x80, 0x83, 0x85, 0x86, 0x89, 0x8A, 0x8C, 0x8F, \
  0x91, 0x92, 0x94, 0x97, 0x98, 0x9B, 0x9D, 0x9E, \
  0xA1, 0xA2, 0xA4, 0xA7, 0xA8, 0xAB, 0xAD, 0xAE, \
  0xB0, 0xB3, 0xB5, 0xB6, 0xB9, 0xBA, 0xBC, 0xBF, \
  0xC1, 0xC2, 0xC4, 0xC7, 0xC8, 0xCB, 0xCD, 0xCE, \
  0xD0, 0xD3, 0xD5, 0xD6, 0xD9, 0xDA, 0xDC, 0xDF, \
  0xE0, 0xE3, 0xE5, 0xE6, 0xE9, 0xEA, 0xEC, 0xEF, \
  0xF1, 0xF2, 0xF4, 0xF7, 0xF8, 0xFB, 0xFD, 0xFE
};

static const char *command_name[] = \
{
  "PAC", "PDC", "TAC", "TDC", "   ", "   ", "   ", "   ", "   ", "   ", \
  "   ", "   ", "   ", "   ", "   ", "HAL", "REV", "E01", "E02", "E03", \
  "E04", "E05", "E06", "E07", "E08", "E09", "E10", "E11", "E12", "E13", \
  "E14", "E15", "E16", "E17", "E18", "E19", "   ", "   ", "   ", "   ", \
  "   ", "   ", "   ", "   ", "   ", "   ", "   ", "   ", "   ", "   ", \
  "   ", "   ", "   ", "   ", "   ", "   ", "   ", "   ", "   ", "   ", \
  "   ", "   ", "   ", "   "
};

#endif /* BIOTAC_CAN_H_ */
