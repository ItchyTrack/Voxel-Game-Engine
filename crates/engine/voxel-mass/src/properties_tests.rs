use super::*;

fn part(mass: u64, x: Fixed) -> MassProperties {
	MassProperties { mass: Mass(mass), center_of_mass: CenterOfMass(FixedVec3::X * x), ..MassProperties::ZERO }
}

#[test]
fn center_uses_wide_integer_moments() {
	let mass = (1u64 << 54) + 1;
	let x = Fixed::from_num(1_000_000_000i64);
	let result = MassProperties::sum([part(mass, x), part(mass, -x)]);
	assert_eq!(result.mass, Mass(2 * mass));
	assert_eq!(result.center_of_mass.0, FixedVec3::ZERO);
}

#[test]
fn center_rounds_halfway_to_even_for_both_signs() {
	for (a, b, expected) in [(0, 1, 0), (1, 2, 2), (0, -1, 0), (-1, -2, -2)] {
		let result = MassProperties::sum([part(1, Fixed::from_bits(a)), part(1, Fixed::from_bits(b))]);
		assert_eq!(result.center_of_mass.0.x, Fixed::from_bits(expected));
	}
}

#[test]
fn tracked_additions_do_not_accumulate_center_rounding() {
	let parts = [part(2, Fixed::ZERO), part(1, Fixed::EPSILON), part(1, Fixed::EPSILON), part(1, Fixed::EPSILON)];
	let mut total = MassProperties::ZERO;
	let mut moment = [0; 3];
	for part in parts {
		total = total.replaced_tracking([(MassProperties::ZERO, part)], &mut moment);
	}
	assert_eq!(total.center_of_mass, MassProperties::sum(parts).center_of_mass);
	assert_eq!(moment, [3, 0, 0]);
	assert_eq!(total.center_of_mass.0.x, Fixed::EPSILON);
}

#[test]
fn empty_aggregate_discards_old_rounding_remainder() {
	let mut moment = [0; 3];
	let total = MassProperties::ZERO.replaced_tracking([
		(MassProperties::ZERO, part(2, Fixed::ZERO)),
		(MassProperties::ZERO, part(1, Fixed::EPSILON)),
	], &mut moment);
	let empty = total.replaced_tracking([(total, MassProperties::ZERO)], &mut moment);
	assert_eq!(empty, MassProperties::ZERO);
	assert_eq!(moment, [0; 3]);
	let next = part(1, Fixed::from_bits(10));
	assert_eq!(empty.replaced_tracking([(MassProperties::ZERO, next)], &mut moment), next);
}
