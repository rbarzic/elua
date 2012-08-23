#include <stdint.h>

#define SETTING_FORMAT "/wo/._%s"

enum {
  PARAM_NIL=0,
  PARAM_NUMBER,
  PARAM_BOOLEAN,
  PARAM_STRING,
};

// Generic set & get functions
int32_t set_param( uint8_t * name, uint8_t type, uint8_t * value, uint32_t length ); // returns bytes written for type (not including header)
int32_t get_param( uint8_t * name, uint8_t type, uint8_t * value, uint32_t length ); // returns bytes read, <0 = error
int32_t get_param_type( uint8_t * name); // returns type, <0 = error

// Set & get functions for signed 32-bit integer
int32_t set_param_s32( uint8_t * name, int32_t value ); // returns bytes written for type (not including header)
int32_t get_param_s32( uint8_t * name, int32_t *value ); // returns bytes read, <0 = error

// Set & get functions for strings
int32_t set_param_string( uint8_t * name, uint8_t * value ); // returns bytes written for type (not including header)
int32_t get_param_string_len( uint8_t * name ); // returns byte length, <0 = error
int32_t get_param_string( uint8_t * name, uint8_t *value, uint32_t max_len ); // returns bytes read, <0 = error