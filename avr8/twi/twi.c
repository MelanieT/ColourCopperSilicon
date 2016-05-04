/*
 *  Copyright Melanie Thielker and Leonie Gaertner
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
 */

#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "twi.h"

// Curently addressed as slave
static volatile uint8_t addressed = 0;
// If in a slave transaction, this queues a master transaction
static volatile uint8_t send_pending = 0;

// This is used in the below macro to set TWEA in those cases
// where it means to ack slave requests.
static volatile uint8_t slave_active = 0;

// The flags that we always need to set
#define TWI_ON ((1 << TWEN) | (1 << TWIE) | (1 << TWINT))
// The flag we need to add when a slave is active.
// Will do nothing if no slave is configured.
#define TWI_SLAVE_ON (slave_active << TWEA)
 
// Initialize the slave data structure.
// This doesn't touch any hardware.
void TWI_SlaveInitializeDriver(TWI_Slave_t *twi, void (*processDataFunction) (TWI_Slave_t *, char))
{
    twi->Process_Data = processDataFunction;
    twi->bytesReceived = 0;
    twi->bytesSent = 0;
    twi->status = TWIS_STATUS_READY;
    twi->result = TWIS_RESULT_UNKNOWN;
    twi->abort = false;
}

// Start the TWI and enable slave operation
void TWI_SlaveInitializeModule(TWI_Slave_t *twi, uint8_t address)
{
	// Set the address we will be answering to
    TWAR = address << 1;
    slave_active = 1;

	// Enable TWI interrupt, enable acknowledge,
	// clear interrupt flag and enable receiver
	TWCR = TWI_SLAVE_ON | TWI_ON;
}

// Set up TWI to allow master operation
void TWI_MasterInit(TWI_Master_t *twi, uint8_t baudRateRegisterSetting)
{
    TWBR = baudRateRegisterSetting;

	TWCR = TWI_SLAVE_ON | TWI_ON;
}

bool TWI_MasterWrite(TWI_Master_t *twi,
                     uint8_t address,
                     uint8_t * writeData,
                     uint16_t bytesToWrite)
{
    return TWI_MasterWriteRead(twi, address, writeData, bytesToWrite, 0);
}

bool TWI_MasterRead(TWI_Master_t *twi,
                    uint8_t address,
                    uint8_t bytesToRead)
{
    return TWI_MasterWriteRead(twi, address, 0, 0, bytesToRead);
}

bool TWI_MasterWriteRead(TWI_Master_t *twi,
                         uint8_t address,
                         uint8_t *writeData,
                         uint16_t bytesToWrite,
                         uint8_t bytesToRead)
{
    if (bytesToWrite > TWIM_WRITE_BUFFER_SIZE)
        return false;

    twi->address = address;
    twi->bytesWritten = 0;
    twi->bytesRead = 0;
    twi->bytesToRead = bytesToRead;
    twi->bytesToWrite = bytesToWrite;
    twi->result = TWIM_RESULT_UNKNOWN;
    twi->status = TWIM_STATUS_BUSY;

    if (bytesToWrite && writeData)
    {
        for (uint8_t i = 0 ; i < bytesToWrite ; i++)
            twi->writeData[i] = writeData[i];
    }

    if (!addressed)
        TWCR = (1 << TWSTA) | TWI_SLAVE_ON | TWI_ON;
    else
        send_pending = 1;

    return true;
}

void TWI_InterruptHandler(TWI_Slave_t *twis, TWI_Master_t *twim)
{
    register uint8_t twsr = TWSR & 0xf8;

	if (twsr == TW_SR_STOP)
	{
        // This is Evil with a capital 'E'!
        // Resetting addressed here allows a race condition where
        // code trying to send could possibly clear the interrupt
        // flag while setting TWSTA. That would lead to a lost
        // interrupt and possibly a bus hang.
        // I don't expect this to ever happen (har har).
        addressed = 0;

        // If we have received bytes, call the main program.
        // This is also evil in that the main program will need
        // to return here in about 80 clock cycles.
        // There is just no better way to do it since we can't stop
        // the TWI in this state.
        if (twis && twis->bytesReceived)
        {
            twis->Process_Data(twis, 0);
            twis->bytesReceived = 0;
            twis->status = TWIS_STATUS_READY;
        }

		TWCR = TWI_SLAVE_ON | TWI_ON;
        return;
	}
    else if (twsr == TW_BUS_ERROR)
    {
        // Set all transactions as done and canceled
        if (twim)
        {
            twim->result = TWIM_RESULT_BUS_ERROR;
            twim->status = TWIM_STATUS_READY;
        }
        if (twis)
        {
            twis->result = TWIS_RESULT_BUS_ERROR;
            twis->status = TWIS_STATUS_READY;
            twis->bytesReceived = 0;
        }

        TWCR = (1 << TWSTO) | TWI_SLAVE_ON | TWI_ON;
        return;
    }
	else if (twsr == TW_SR_SLA_ACK || twsr == TW_SR_ARB_LOST_SLA_ACK)
	{
		// We have seen our slave address
		// Make all look pretty
        if (twis)
        {
            twis->result = TWIS_RESULT_UNKNOWN;
            twis->status = TWIS_STATUS_BUSY;
            twis->bytesReceived = 0;
        }
        else
        {
            // This will cause a slave NAK
            TWCR = TWI_ON;
            return;
        }

        // Set up as addressed so master transactions are made to wait
		addressed = 1;
		
        // Same line as above but with twis set to something meaningful
        // we can send ACK instead.
		TWCR = TWI_SLAVE_ON | TWI_ON;

		return;
	}
	else if (twsr == TW_SR_DATA_ACK)
	{
        if (twis)
        {
            register uint8_t data = TWDR;

            twis->result = TWIS_RESULT_OK;

            // Store it if there is room
            if (twis->bytesReceived < TWIS_RECEIVE_BUFFER_SIZE)
                twis->receivedData[twis->bytesReceived++] = data;
            else
                twis->result = TWIM_RESULT_BUFFER_OVERFLOW;
            
            // If there is only one byte left to go, we do want to
            // NAK the next byte as it's the last one that fits.
            if (twis->bytesReceived >= TWIS_RECEIVE_BUFFER_SIZE - 1)
                TWCR = TWI_ON;
            else
                TWCR = (1 << TWEA) | TWI_ON;

            return;
        }

        TWCR = TWI_ON;
	}
	else if (twsr == TW_SR_DATA_NACK)
	{
        // This is one of the places where we can start a master
        // transaction. If one is pending, we set TWSTA here.
	    if (send_pending)
	        TWCR = (1 << TWSTA) | TWI_SLAVE_ON | TWI_ON;
	    else
	        TWCR = TWI_SLAVE_ON | TWI_ON;

        return;
	}
	else if (twsr == TW_ST_SLA_ACK || twsr == TW_ST_DATA_ACK || twsr == TW_ST_ARB_LOST_SLA_ACK)
	{
        if (twis)
        {
            twis->result = TWIS_RESULT_OK;
            twis->status = TWIS_STATUS_BUSY; 

            // If this is the first byte, reset buffer pointer
            if (twsr == TW_ST_SLA_ACK)
            {
                addressed = 1;

                twis->bytesSent = 0;

                // Get the data to return from main program. Will hold the bus
                // for the duration so this should return quickly.
                twis->Process_Data(twis, 1);
            }
            
            // Send next buffer byte if we have one, else send zero
            if (twis->bytesSent < TWIS_SEND_BUFFER_SIZE)
                TWDR = twis->sendData[twis->bytesSent++];
            else
                TWDR = 0;
            
            // If no more bytes, NAK it
            if (twis->bytesSent >= TWIS_SEND_BUFFER_SIZE)
                TWCR = TWI_ON;
            else
                TWCR = (1 << TWEA) | TWI_ON;
            
            return;
        }

        TWCR = TWI_ON;

        return;
	}
    else if (twsr == TW_ST_DATA_NACK)
    {
        // We have sent the whole buffer. This
        // ends addressed mode for us.
        if (twis)
        {
            twis->status = TWIS_STATUS_READY; 
        }

        addressed = 0;

        // If a master send is waiting, start it now.
        if (send_pending)
        {
            TWCR = (1 << TWSTA) | TWI_SLAVE_ON | TWI_ON;
            return;
        }

        TWCR = TWI_SLAVE_ON | TWI_ON;

        return;
    }
    else if (twsr == TW_START || twsr == TW_REP_START)
    {
        send_pending = 0;

        if (twim)
        {
            if (twim->bytesWritten == 0 && twim->bytesToWrite > 0)
            {
                // This is a start and we're going to write
                // Address the slave for writing.
                TWDR = twim->address << 1;
            }
            else if (twim->bytesRead == 0 && twim->bytesToRead > 0)
            {
                // Reading is requested, address for reading
                TWDR = (twim->address << 1) | 1;
            }
            else
            {
                // WTF are we doing here? Empty messages are not
                // permitted in the TWI spec!
                twim->status = TWIM_STATUS_READY;
                twim->result = TWIM_RESULT_FAIL; // Epic
                TWCR = (1 << TWSTO) | TWI_SLAVE_ON | TWI_ON;
                return;
            }

            twim->result = TWIM_RESULT_UNKNOWN;
        }

        // In any case, the show must go on.
        TWCR = TWI_SLAVE_ON | TWI_ON;

        return;
    }
    else if (twsr == TW_MT_SLA_NACK)
    {
        // The slave has sent a NAK. I know of no condition
        // where this happens in real life but the possibility
        // exists, so here is the code for it.
        if (twim)
        {
            twim->status = TWIM_STATUS_READY;
            twim->result = TWIM_RESULT_FAIL;
        }

        // Send stop and roll on
        TWCR = (1 << TWSTO) | TWI_SLAVE_ON | TWI_ON;

        return;
    }
    else if (twsr == TW_MT_DATA_NACK)
    {
        // We should go to read mode here if there is anything to read
        if (twim)
        {
            if(twim->bytesToRead)
            {
                // Send repeated start
                TWCR = (1 << TWSTA) | TWI_SLAVE_ON | TWI_ON;
                return;
            }
            else
            {
                twim->status = TWIM_STATUS_READY;
                TWCR = (1 << TWSTO) | TWI_SLAVE_ON | TWI_ON;
                return;
            }
        }
        TWCR = (1 << TWSTO) | TWI_SLAVE_ON | TWI_ON;
        return;
    }
    else if (twsr == TW_MT_SLA_ACK || twsr == TW_MT_DATA_ACK)
    {
        if (twim)
        {
            twim->result = TWIM_RESULT_OK;

            // If we have written our entire data payload and
            // wsnt to get a reply, we should switch here.
            if (twim->bytesWritten >= twim->bytesToWrite)
            {
                if(twim->bytesToRead)
                {
                    // Send repeated start
                    TWCR = (1 << TWSTA) | TWI_SLAVE_ON | TWI_ON;
                    return;
                }

                // Nothing to read, stop.
                twim->status = TWIM_STATUS_READY;
                TWCR = (1 << TWSTO) | (1 << TWEA) | TWI_ON;

                return;
            }

            // Send next data byte
            TWDR = twim->writeData[twim->bytesWritten++];
        }

        TWCR = TWI_SLAVE_ON | TWI_ON;
    }
    else if (twsr == TW_MT_ARB_LOST)
    {
        // We lost arbitration but we may yet not be addressed
        // so set the start flag here.
        send_pending = 1;

        TWCR = (1 << TWSTA) | TWI_SLAVE_ON | TWI_ON;

        return;
    }
    else if (twsr == TW_MR_SLA_ACK)
    {
        // We want to receive.
        // If we want more than one byte, ACK here
        if (twim)
        {
            if (twim->bytesToRead > 1)
                TWCR = (1 << TWEA) | TWI_ON;
            else
                TWCR = TWI_ON;
            return;
        }

        TWCR = TWI_ON;

        return;
    }
    else if (twsr == TW_MR_SLA_NACK)
    {
        // Slave NAKed the address.
        if (twim)
        {
            twim->status = TWIM_STATUS_READY;
            twim->result = TWIM_RESULT_FAIL;
        }

        // Send stop and roll on
        TWCR = (1 << TWSTO) | TWI_SLAVE_ON | TWI_ON;
        return;
    }
    else if (twsr == TW_MR_DATA_ACK)
    {
        uint8_t data = TWDR;
        if (twim)
        {
            // If there is room for this byte, store it.
            if (twim->bytesRead < twim->bytesToRead)
                twim->readData[twim->bytesRead++] = data;

            // If there is only room for one more, set
            // the chip to NAK the next byte.
            if (twim->bytesRead < twim->bytesToRead - 1)
                TWCR = (1 << TWEA) | TWI_ON;
            else
                TWCR = TWI_ON;

            return;
        }

        TWCR = TWI_ON;
        return;
    }
    else if (twsr == TW_MR_DATA_NACK)
    {
        uint8_t data = TWDR;
        if (twim)
        {
            // If there is room for this byte, store it.
            if (twim->bytesRead < twim->bytesToRead)
                twim->readData[twim->bytesRead++] = data;
        }

        // We already NAKed the byte so let's do a stop
        TWCR = (1 << TWSTO) | TWI_SLAVE_ON | TWI_ON;

        if (twim)
        {
            twim->status = TWIM_STATUS_READY;
            twim->result = TWIM_RESULT_OK;
        }

        return;
    }
	else
	{
		// Let the bus roll on
		TWCR = TWI_SLAVE_ON | TWI_ON;
        return;
	}
}
