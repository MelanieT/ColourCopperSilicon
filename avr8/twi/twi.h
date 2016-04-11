/*
 *  Copyright Melanie Thielker and Leonie Gaertner
 *  Portions copyright (c) 2008 Atmel Corporation
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  The API was styled to follow the Atmel XMEGA twi library to allow
 *  easy porting of source code between MEGA and XMEGA processors.
 *
 *  The original license for the API and constants can be found in the
 *  LICENSES file in this directory.
 */

#ifndef TWI_H_
#define TWI_H_

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

typedef volatile uint8_t register8_t;

#define TWI_BAUD(F_SYS, F_TWI) ((F_SYS / F_TWI - 16) / 2)

/*! Buffer size defines */
#ifndef TWIM_WRITE_BUFFER_SIZE
#define TWIM_WRITE_BUFFER_SIZE          8
#endif
#ifndef TWIM_READ_BUFFER_SIZE
#define TWIM_READ_BUFFER_SIZE           8
#endif
#ifndef TWIS_RECEIVE_BUFFER_SIZE
#define TWIS_RECEIVE_BUFFER_SIZE        8
#endif
#ifndef TWIS_SEND_BUFFER_SIZE
#define TWIS_SEND_BUFFER_SIZE           8
#endif

/*! Transaction status defines. */
#define TWIM_STATUS_READY               0
#define TWIM_STATUS_BUSY                1
#define TWIS_STATUS_READY               0
#define TWIS_STATUS_BUSY                1


/*! Transaction result enumeration. */
typedef enum TWIM_RESULT_enum {
    TWIM_RESULT_UNKNOWN             = (0x00<<0),
    TWIM_RESULT_OK                  = (0x01<<0),
    TWIM_RESULT_BUFFER_OVERFLOW     = (0x02<<0),
    TWIM_RESULT_ARBITRATION_LOST    = (0x03<<0),
    TWIM_RESULT_BUS_ERROR           = (0x04<<0),
    TWIM_RESULT_NACK_RECEIVED       = (0x05<<0),
    TWIM_RESULT_FAIL                = (0x06<<0),
} TWIM_RESULT_t;

/* Transaction result enumeration */
typedef enum TWIS_RESULT_enum {
    TWIS_RESULT_UNKNOWN             = (0x00<<0),
    TWIS_RESULT_OK                  = (0x01<<0),
    TWIS_RESULT_BUFFER_OVERFLOW     = (0x02<<0),
    TWIS_RESULT_TRANSMIT_COLLISION  = (0x03<<0),
    TWIS_RESULT_BUS_ERROR           = (0x04<<0),
    TWIS_RESULT_FAIL                = (0x05<<0),
    TWIS_RESULT_ABORTED             = (0x06<<0),
} TWIS_RESULT_t;

/*! \brief TWI slave driver struct.
 *
 *  TWI slave struct. Holds pointer data processing routine,
 *  buffers and necessary variables.
 */
typedef struct TWI_Slave {
    void (*Process_Data) (struct TWI_Slave *, char);                    /*!< Pointer to process data function*/
    register8_t receivedData[TWIS_RECEIVE_BUFFER_SIZE]; /*!< Read data*/
    register8_t sendData[TWIS_SEND_BUFFER_SIZE];        /*!< Data to write*/
    register8_t bytesReceived;                          /*!< Number of bytes received*/
    register8_t bytesSent;                              /*!< Number of bytes sent*/
    register8_t status;                                 /*!< Status of transaction*/
    register8_t result;                                 /*!< Result of transaction*/
    bool abort;                                     /*!< Strobe to abort*/
} TWI_Slave_t;

/*! \brief TWI master driver struct
 *
 *  TWI master struct. Holds pointer to TWI module,
 *  buffers and necessary varibles.
 */
typedef struct TWI_Master {
    register8_t address;                            /*!< Slave address */
    register8_t writeData[TWIM_WRITE_BUFFER_SIZE];  /*!< Data to write */
    register8_t readData[TWIM_READ_BUFFER_SIZE];    /*!< Read data */
    register8_t bytesToWrite;                       /*!< Number of bytes to write */
    register8_t bytesToRead;                        /*!< Number of bytes to read */
    register8_t bytesWritten;                       /*!< Number of bytes written */
    register8_t bytesRead;                          /*!< Number of bytes read */
    register8_t status;                             /*!< Status of transaction */
    register8_t result;                             /*!< Result of transaction */
}TWI_Master_t;



void TWI_SlaveInitializeDriver(TWI_Slave_t *twi,
                               void (*processDataFunction) (TWI_Slave_t *, char));

void TWI_SlaveInitializeModule(TWI_Slave_t *twi,
                               uint8_t address);

void TWI_MasterInit(TWI_Master_t *twi,
                    uint8_t baudRateRegisterSetting);

void TWI_InterruptHandler(TWI_Slave_t *twis, TWI_Master_t *twim);

bool TWI_MasterWrite(TWI_Master_t *twi,
                     uint8_t address,
                     uint8_t * writeData,
                     uint16_t bytesToWrite);

bool TWI_MasterRead(TWI_Master_t *twi,
                    uint8_t address,
                    uint8_t bytesToRead);

bool TWI_MasterWriteRead(TWI_Master_t *twi,
                         uint8_t address,
                         uint8_t *writeData,
                         uint16_t bytesToWrite,
                         uint8_t bytesToRead);
#endif /* TWI_H_ */
