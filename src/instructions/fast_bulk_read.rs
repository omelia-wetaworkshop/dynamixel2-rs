use super::{instruction_id, packet_id, BulkReadData};
use crate::endian::{write_u16_le, write_u8_le};
use crate::serial_port::SerialPort;
use crate::{Bus, MotorError, Response, StatusPacket, TransferError};

#[cfg(feature = "alloc")]
use alloc::{vec::Vec, borrow::ToOwned};
use crate::bus::message_transfer_time;
use crate::packet::Packet;

impl<ReadBuffer, WriteBuffer, T> Bus<ReadBuffer, WriteBuffer, T>
where
    ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
    WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
    T: SerialPort,
{
    /// Synchronously read arbitrary data ranges from multiple motors in one command.
    ///
    /// Unlike the sync read instruction, a bulk read can be used to read a different amount of data from a different address for each motor.
    ///
    /// The data for multi-byte registers is received in little-endian format.
    ///
    /// The `on_response` function is called for the reply from each motor.
    /// If the function fails to write the instruction, an error is returned and the function is not called.
    ///
    /// # Panics
    /// The protocol forbids specifying the same motor ID multiple times.
    /// This function panics if the same motor ID is used for more than one read.
    pub fn fast_bulk_read_cb<Read, F>(&mut self, reads: &[Read], on_response: F) -> Result<(), TransferError<T::Error>>
    where
        Read: AsRef<BulkReadData>,
        F: FnMut(Result<Response<&[u8]>, MotorError>),
    {
        for i in 0..reads.len() {
            for j in i + 1..reads.len() {
                if reads[i].as_ref().motor_id == reads[j].as_ref().motor_id {
                    panic!(
                        "bulk_read_cb: motor ID {} used multiple at index {} and {}",
                        reads[i].as_ref().motor_id,
                        i,
                        j
                    )
                }
            }
        }

        self.write_instruction(packet_id::BROADCAST, instruction_id::FAST_BULK_READ, 5 * reads.len(), |buffer| {
            for (i, read) in reads.iter().enumerate() {
                let read = read.as_ref();
                let buffer = &mut buffer[i..][..5];
                write_u8_le(&mut buffer[0..], read.motor_id);
                write_u16_le(&mut buffer[1..], read.address);
                write_u16_le(&mut buffer[3..], read.count);
            }
        })?;

        // -1 as the first ERR byte is included in the count + 4
        let total_count = reads.iter().fold(0, |acc, read| acc + read.as_ref().count + 4) + StatusPacket::HEADER_SIZE as u16 - 1;
        let timeout = message_transfer_time(total_count as u32, self.messenger.baud_rate);
        let response: StatusPacket = self.messenger.read_packet_response_timeout(timeout)?;

        crate::InvalidInstruction::check(response.instruction_id(), instruction_id::STATUS)?;
        crate::InvalidPacketId::check(response.packet_id(), 0xfe)?;
        crate::InvalidParameterCount::check(response.parameters().len(), total_count as usize)?;

        let data = &response.data[StatusPacket::HEADER_SIZE - 1..];

        let mut curr_index = 0;
        reads.iter().map(|read| {
            let read = read.as_ref();
            let count = read.count as usize;
            let data = &data[curr_index..count + 4];
            curr_index += count + 4;

            let error = data[0];
            MotorError::check(error)?;

            Ok(Response {
                motor_id: data[1],
                alert: error & 0x80 != 0,
                data:  &data[2..count + 2],
            })
        }).for_each(on_response);

        Ok(())
    }

    /// Synchronously read arbitrary data ranges from multiple motors in one command.
    ///
    /// Unlike the sync read instruction, a bulk read can be used to read a different amount of data from a different address for each motor.
    ///
    /// If this function fails to get the data from any of the motors, the entire function retrns an error.
    /// If you need access to the data from other motors, or if you want acces to the error for each motor, see [`Self::bulk_read_cb`].
    ///
    /// # Panics
    /// The protocol forbids specifying the same motor ID multiple times.
    /// This function panics if the same motor ID is used for more than one read.
    #[cfg(any(feature = "alloc", feature = "std"))]
    pub fn fast_bulk_read<Read>(&mut self, reads: &[Read]) -> Result<Vec<Result<Response<Vec<u8>>, MotorError>>, TransferError<T::Error>>
    where
        Read: AsRef<BulkReadData>,
    {
        let mut responses = Vec::with_capacity(reads.len());

        self.fast_bulk_read_cb(reads, |response| {
             responses.push(response.map(|response| Response {
                motor_id: response.motor_id,
                alert: response.alert,
                data: response.data.to_owned(),
            }))
        })?;

        Ok(responses)
    }
}
