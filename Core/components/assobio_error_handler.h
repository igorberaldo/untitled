/**
 * @file assobio_error_handler.h
 * @author Jaime Albuquerque (jaime.albq@gmail.com)
 * @brief Handle simple errors from the features
 * @version 0.1
 * @date 2023-01-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

typedef enum {
        ASSOBIO_OK = 0,
        ASSOBIO_NOT_ENABLED,
        ASSOBIO_NOT_DISABLED,
        ASSOBIO_NO_PARAMETER,
        ASSOBIO_COMMUNICATION_ERROR,
        ASSOBIO_UNKOWN_ERROR
} assobio_error_t;

typedef enum {
        ASSOBIO_DISABLE = 0,
        ASSOBIO_ENABLE = 1
} assobio_enabling_t;