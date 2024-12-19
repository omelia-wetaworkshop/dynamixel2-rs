use super::{instruction_id, packet_id};
use crate::endian::{write_u16_le};
use crate::serial_port::SerialPort;
use crate::{Bus, MotorError, Read, ReadError, Response, StatusPacket, TransferError};

use crate::packet::Packet;
use std::marker::PhantomData;
use crate::bus::message_transfer_time;

pub struct FastSyncRead<'a, Data, T> {
	parameters: &'a [u8],
	phantom_data: PhantomData<(Data, T)>,

}
impl<'a, Data, T> FastSyncRead<'a, Data, T>  where Data: Read, T: SerialPort{
	fn parse_response(data: &[u8]) -> Result<Response<Data>,  ReadError<T::Error>> {
		let error = data[0];
		MotorError::check(error)?;

		Ok(Response {
			motor_id: data[1],
			alert: error & 0x80 != 0,
			data:  Data::try_from_bytes(&data[2..])?,
		})
	}

	pub fn next(&mut self) -> Option<Result<Response<Data>, ReadError<T::Error>>> {
		let (data, remaining) = self.parameters.split_at_checked(Data::R_COUNT as usize + 4)?;
		self.parameters = remaining;

		Some(Self::parse_response(data))
	}
}

impl<Data, T> Iterator for FastSyncRead<'_, Data, T> where Data: Read, T: SerialPort {
	type Item = Result<Response<Data>, ReadError<T::Error>>;

	fn next(&mut self) -> Option<Self::Item> {
		self.next()
	}
}
impl<ReadBuffer, WriteBuffer, T> Bus<ReadBuffer, WriteBuffer, T>
where
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
	T: SerialPort,
{
	/// Synchronously read an arbitrary number of bytes from multiple motors in one command using the fast read protocol.
	pub fn fast_sync_read_new<Data>(&mut self, motor_ids: &[u8], address: u16) -> Result<FastSyncRead<Data, T>, TransferError<T::Error>>
	where Data: Read {
		self.write_instruction(packet_id::BROADCAST, instruction_id::FAST_SYNC_READ, 4 + motor_ids.len(), |buffer| {
			write_u16_le(&mut buffer[0..], address);
			write_u16_le(&mut buffer[2..], Data::R_COUNT);
			buffer[4..].copy_from_slice(motor_ids);
		})?;
		// -1 as the first ERR byte is included in the count + 4
		let total_count = (((Data::R_COUNT + 4) as usize * motor_ids.len()) + StatusPacket::HEADER_SIZE - 1 ) as u32;
		let timeout = message_transfer_time(total_count, self.messenger.baud_rate);
		let response: StatusPacket = self.messenger.read_packet_response_timeout(timeout)?;

		crate::InvalidInstruction::check(response.instruction_id(), instruction_id::STATUS)?;
		crate::InvalidPacketId::check(response.packet_id(), 0xfe)?;
		crate::InvalidParameterCount::check(response.parameters().len(), total_count as usize)?;

		Ok(FastSyncRead {
			parameters: &response.data[StatusPacket::HEADER_SIZE - 1..],
			phantom_data: PhantomData,
		})
	}
}
