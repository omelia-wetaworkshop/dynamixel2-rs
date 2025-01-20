use assert2::{assert, let_assert};
use dynamixel2::instructions::BulkReadData;
use test_log::test;

pub mod common;
use common::run;

#[test]
fn test_read() {
	run(|ids, mut client| {
		let_assert!(Ok(response) = client.read::<u32>(ids[0], 132));
		assert!(response.motor_id == ids[0]);
	})
}

#[test]
fn test_read_bytes() {
	run(|ids, mut client| {
		let_assert!(Ok(response) = client.read_bytes::<Vec<u8>>(ids[0], 132, 4));
		assert!(response.motor_id == ids[0]);
	})
}

#[test]
fn test_write() {
	run(|ids, mut client| {
		let_assert!(Ok(response) = client.write(ids[0], 65, &1u8));
		assert!(response.motor_id == ids[0]);
		let _ = client.write(ids[0], 65, &0u8);
	})
}

#[test]
fn test_reg_write() {
	run(|ids, mut client| {
		let_assert!(Ok(response) = client.reg_write(ids[0], 65, &1u8));
		assert!(response.motor_id == ids[0]);
		let_assert!(Ok(response) = client.action(ids[0]));
		assert!(response.motor_id == ids[0]);
		let _ = client.write(ids[0], 65, &0u8);
	})
}

#[test]
#[cfg_attr(not(feature = "integration_test"), should_panic)] // test panics as Mock Serial doesn't currently support sync_read
fn test_sync_read() {
	run(|ids, mut client| {
		let response = client.sync_read::<u32>(ids, 132).unwrap();
		for (r, id) in response.zip(ids) {
			match r {
				Err(e) => panic!("id {id} {e}"),
				Ok(r) => {
					assert!(r.motor_id == *id)
				},
			}
		}
	})
}

#[test]
#[cfg_attr(not(feature = "integration_test"), should_panic)] // test panics as Mock Serial doesn't currently support sync_read
fn test_sync_read_bytes() {
	run(|ids, mut client| {
		let response = client.sync_read_bytes::<Vec<u8>>(ids, 132, 4).unwrap();
		for (r, id) in response.zip(ids) {
			match r {
				Err(e) => panic!("id {id} {e}"),
				Ok(r) => {
					assert!(r.motor_id == *id)
				},
			}
		}
	})
}

#[test]
#[cfg_attr(not(feature = "integration_test"), should_panic)] // test panics as Mock Serial doesn't currently support bulk_read
fn test_bulk_read_bytes() {
	run(|ids, mut client| {
		let bulk_read_data: Vec<_> = ids
			.iter()
			.map(|id| BulkReadData {
				motor_id: *id,
				address: 132,
				count: 4,
			})
			.collect();
		let response = client.bulk_read_bytes::<Vec<u8>>(&bulk_read_data).unwrap();
		for (r, id) in response.zip(ids) {
			match r {
				Err(e) => panic!("id {id} {e}"),
				Ok(r) => {
					assert!(r.motor_id == *id)
				},
			}
		}
	})
}

#[test]
fn test_ping() {
	run(|ids, mut client| {
		let response = client.scan().unwrap();
		assert!(response.into_iter().count() == ids.len(), "missing motor ping");
	})
}