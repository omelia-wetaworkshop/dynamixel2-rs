use super::{instruction_id, packet_id};
use crate::endian::{write_u16_le};
use crate::serial_port::SerialPort;
use crate::{Bus, MotorError, Response, StatusPacket, TransferError};

use crate::packet::Packet;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;
use crate::bus::message_transfer_time;

impl<ReadBuffer, WriteBuffer, T> Bus<ReadBuffer, WriteBuffer, T>
where
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
	T: SerialPort,
{
	/// Synchronously read an arbitrary number of bytes from multiple motors in one command.
	///
	/// The `on_response` function is called for the reply from each motor.
	/// If the function fails to write the instruction, an error is returned and the function is not called.
	pub fn fast_sync_read_cb<'a, F>(
		&'a mut self,
		motor_ids: &'a [u8],
		address: u16,
		count: u16,
		on_response: F,
	) -> Result<(), TransferError<T::Error>>
	where
		F: FnMut(Result<Response<&[u8]>, MotorError>),
	{
		self.write_instruction(packet_id::BROADCAST, instruction_id::FAST_SYNC_READ, 4 + motor_ids.len(), |buffer| {
			write_u16_le(&mut buffer[0..], address);
			write_u16_le(&mut buffer[2..], count);
			buffer[4..].copy_from_slice(motor_ids);
		})?;
		// -1 as the first ERR byte is included in the count + 4
		let total_count = (((count + 4) as usize * motor_ids.len()) + StatusPacket::HEADER_SIZE - 1 ) as u32;
		let timeout = message_transfer_time(total_count, self.messenger.baud_rate);
		let response: StatusPacket = self.messenger.read_packet_response_timeout(timeout)?;

		crate::InvalidInstruction::check(response.instruction_id(), instruction_id::STATUS)?;
		crate::InvalidPacketId::check(response.packet_id(), 0xfe)?;
		crate::InvalidParameterCount::check(response.parameters().len(), total_count as usize)?;

		response.parameters().chunks(count as usize + 4).map(|data| {
			let error = data[0];
			MotorError::check(error)?;

			Ok(Response {
				motor_id: data[1],
				alert: error & 0x80 != 0,
				data:  &data[2..(count + 2) as usize],
			})
		}).for_each(on_response);
		Ok(())
	}

	/// Synchronously read an arbitrary number of bytes from multiple motors in one command.
	///
	/// If this function fails to get the data from any of the motors, the entire function retrns an error.
	/// If you need access to the data from other motors, or if you want acces to the error for each motor, see [`Self::sync_read_cb`].
	#[cfg(any(feature = "alloc", feature = "std"))]
	pub fn fast_sync_read<'a>(
		&'a mut self,
		motor_ids: &'a [u8],
		address: u16,
		count: u16,
	) -> Result<Vec<Result<Response<Vec<u8>>, MotorError>>, TransferError<T::Error>> {
		let mut responses = Vec::with_capacity(motor_ids.len());
		self.fast_sync_read_cb(motor_ids, address, count, |response| {
             responses.push(response.map(|response| Response {
                motor_id: response.motor_id,
                alert: response.alert,
                data: response.data.to_owned(),
            }))
        })?;
		Ok(responses)
	}

}
