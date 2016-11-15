/*
 * dw1000, driver for decawave dw1000 UVB chip
 *  Copyright (C) 2014  Kim Albertsson
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser Public License for more details.
 *  
 *  You should have received a copy of the GNU Lesser Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @brief DMA callback pointer.
 */
#define RECEIVE_BUFFER_SIZE 123

/**
 * @brief DMA callback pointer.
 */
typedef enum
{
    /**
     * @brief DMA callback pointer.
     */
    DW_UNINITIALIZED = 0,
    /**
     * @brief DMA callback pointer.
     */
    DW_READY,
    /**
     * @brief DMA callback pointer.
     */
    DW_TANSMITING,
    /**
     * @brief DMA callback pointer.
     */
    DW_RECEIVING,
    /**
     * @brief DMA callback pointer.
     */
    DW_ERROR
} DW1000_base_states;

typedef struct
{
    
} DW1000Config;

/**
 * @brief DW1000 base driver structure.
 */
typedef struct
{
    /**
     * @brief DMA callback pointer.
     */
    void (* dma_cb)(void);

    /**
     * @brief DW1000 receive buffer.
     */
    uint8_t receive_buffer[RECEIVE_BUFFER_SIZE];

    /**
     * @brief DW1000 driver states.
     */
    DW1000States state;

    /**
     * @brief DW1000 error conditions.
     */
    uint32_t error_no;

    /**
     * @brief DW1000 configuration structure.
     */
    const DW1000Config *cfg;
} DW1000BaseDriver;


/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param[in/out] a [description]
 * @param[in] b [description]
 * 
 * @return [description]
 */
int asd(int *a, int b)
{
    *a = *a + b;

    return *a - 1;
}