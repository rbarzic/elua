#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdint.h>

#include "param.h"

int32_t set_param( uint8_t * name, uint8_t type, uint8_t * value, uint32_t length )
{
    FILE *fp;
    char fname[20];
    int n;

    sprintf( fname, SETTING_FORMAT, name );
    fp = fopen(fname, "w+");

    // bail if we can't write out setting
    if (fp == NULL)
        return -1;

    n = fputc(type, fp);
    n = fwrite( value, 1, length, fp);
    fclose(fp);
    return n;
}

int32_t get_param( uint8_t * name, uint8_t type, uint8_t * value, uint32_t length )
{
    FILE *fp;
    char fname[20];
    int n;

    sprintf( fname, SETTING_FORMAT, name );
    fp = fopen(fname, "r");

    // bail if we can't get stored setting
    if (fp == NULL)
        return -1;

    // bail if type isn't what we expected
    if( fgetc( fp ) != type)
        return -2;

    n = fread(value, 1, length, fp);
    fclose(fp);
    return n; // return length of item actually read
}

int32_t get_param_type( uint8_t * name)
{
    FILE *fp;
    char fname[20];
    int n;

    sprintf( fname, SETTING_FORMAT, name );
    fp = fopen(fname, "r");

    if (fp == NULL)
        return -1;

    return fgetc( fp );
}

// store 32-bit integer parameter
int32_t set_param_s32( uint8_t * name, int32_t value )
{
    return set_param( name, PARAM_INTEGER, ( uint8_t * )&value, 4 );
}

// get 32-bit integer parameter, return number of bytes read
int32_t get_param_s32( uint8_t * name, int32_t *value )
{
    uint8_t b[4];
    int32_t ret;

    ret = get_param( name, PARAM_INTEGER, b, 4);
    
    if( ret < 0 )
        return ret;

    *value = *( int32_t * )b;
    return 1;
}

// store string parameter
int32_t set_param_string( uint8_t * name, uint8_t * value )
{
    return set_param( name, PARAM_STRING, value, strlen( ( const char * )value ) );
}

// return string parameter length
int32_t get_param_string_len( uint8_t * name )
{
    FILE *fp;
    char fname[20];
    int n = 0;

    sprintf( fname, SETTING_FORMAT, name );
    fp = fopen(fname, "r");
    if( fp == NULL )
        return -1;

    while( fgetc( fp ) != EOF )
        n++;

    fclose(fp);
    return n-1; // return position minus header length
}


// get string parameter, return length
int32_t get_param_string( uint8_t * name, uint8_t *value, uint32_t max_len  )
{
    int32_t len;

    len = get_param_string_len( name );

    // If an error was encountered, pass it up
    if( len < 0 )
        return len;

    if( len <= max_len )
        len = get_param( name, PARAM_STRING, value, len);
    else
        return -3;

    // Null terminate string
    value[ len ] = 0;

    return len;
}