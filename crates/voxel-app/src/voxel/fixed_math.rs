use voxel_math::Fixed;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct FixedVec2 {
	pub x: Fixed,
	pub y: Fixed,
}

impl FixedVec2 {
	pub const fn new(x: Fixed, y: Fixed) -> Self { Self { x, y } }
	pub const fn splat(value: Fixed) -> Self { Self::new(value, value) }
	pub fn dot(self, rhs: Self) -> Fixed { self.x * rhs.x + self.y * rhs.y }
	pub fn min(self, rhs: Self) -> Self { Self::new(self.x.min(rhs.x), self.y.min(rhs.y)) }
	pub fn max(self, rhs: Self) -> Self { Self::new(self.x.max(rhs.x), self.y.max(rhs.y)) }
	pub fn lerp(self, rhs: Self, t: Fixed) -> Self { Self::new(self.x + (rhs.x - self.x) * t, self.y + (rhs.y - self.y) * t) }
}

impl std::ops::Sub for FixedVec2 {
	type Output = Self;
	fn sub(self, rhs: Self) -> Self { Self::new(self.x - rhs.x, self.y - rhs.y) }
}
impl std::ops::Neg for FixedVec2 {
	type Output = Self;
	fn neg(self) -> Self { Self::new(-self.x, -self.y) }
}

pub(super) const PI: Fixed = Fixed::from_bits(52_707_179);
const HALF_PI: Fixed = Fixed::from_bits(26_353_589);
const TAU: Fixed = Fixed::from_bits(105_414_357);
const LN_2: Fixed = Fixed::from_bits(11_629_080);

/// Fixed-point approximations for procedural shapes.
pub(super) trait ShapeMath {
	fn powi(self, exponent: i32) -> Self;
	fn powf(self, exponent: Self) -> Self;
	fn sin(self) -> Self;
	fn cos(self) -> Self;
	fn atan2(self, x: Self) -> Self;
	fn acos(self) -> Self;
	fn ln(self) -> Self;
}

impl ShapeMath for Fixed {
	fn powi(self, exponent: i32) -> Self {
		let mut power = exponent.unsigned_abs();
		let mut base = if exponent < 0 { self.recip() } else { self };
		let mut result = Fixed::ONE;
		while power != 0 {
			if power & 1 != 0 { result *= base; }
			power >>= 1;
			if power != 0 { base *= base; }
		}
		result
	}

	fn powf(self, exponent: Self) -> Self {
		if exponent == Fixed::ZERO { return Fixed::ONE; }
		if exponent == Fixed::from_num(0.5) { return self.sqrt(); }
		if exponent.fract() == Fixed::ZERO { return self.powi(exponent.to_num()); }
		if self == Fixed::ZERO && exponent > Fixed::ZERO { return Fixed::ZERO; }
		exp(self.ln() * exponent)
	}

	fn sin(self) -> Self {
		let mut x = self % TAU;
		if x > PI { x -= TAU; }
		if x < -PI { x += TAU; }
		if x > HALF_PI { x = PI - x; }
		if x < -HALF_PI { x = -PI - x; }
		let square = x * x;
		let mut term = x;
		let mut sum = term;
		for n in 1..=7 {
			term = -term * square / Fixed::from_num((2 * n) * (2 * n + 1));
			sum += term;
		}
		sum.clamp(Fixed::NEG_ONE, Fixed::ONE)
	}

	fn cos(self) -> Self { (self % TAU + HALF_PI).sin() }

	fn atan2(self, x: Self) -> Self {
		if self == Fixed::ZERO {
			return if x < Fixed::ZERO { PI } else { Fixed::ZERO };
		}
		let ax = x.abs();
		let ay = self.abs();
		let mut angle = if ay > ax { HALF_PI - atan_unit(ax / ay) } else { atan_unit(ay / ax) };
		if x < Fixed::ZERO { angle = PI - angle; }
		if self < Fixed::ZERO { -angle } else { angle }
	}

	fn acos(self) -> Self {
		let x = self.clamp(Fixed::NEG_ONE, Fixed::ONE);
		(Fixed::ONE - x * x).max(Fixed::ZERO).sqrt().atan2(x)
	}

	fn ln(self) -> Self {
		assert!(self > Fixed::ZERO, "shape logarithm requires a positive input");
		let mut x = self;
		let mut power = 0i32;
		let two = Fixed::from_num(2);
		while x >= two { x /= two; power += 1; }
		while x < Fixed::ONE { x *= two; power -= 1; }
		let z = (x - Fixed::ONE) / (x + Fixed::ONE);
		let square = z * z;
		let mut term = z;
		let mut sum = z;
		for n in 1..=12 {
			term *= square;
			sum += term / Fixed::from_num(2 * n + 1);
		}
		two * sum + Fixed::from_num(power) * LN_2
	}
}

fn atan_unit(x: Fixed) -> Fixed {
	let reduced = x / (Fixed::ONE + (Fixed::ONE + x * x).sqrt());
	let square = reduced * reduced;
	let mut term = reduced;
	let mut sum = term;
	for n in 1..=12 {
		term *= -square;
		sum += term / Fixed::from_num(2 * n + 1);
	}
	Fixed::from_num(2) * sum
}

fn exp(value: Fixed) -> Fixed {
	let power = (value / LN_2).floor().to_num::<i32>();
	let remainder = value - Fixed::from_num(power) * LN_2;
	let mut term = Fixed::ONE;
	let mut sum = term;
	for n in 1..=14 {
		term = term * remainder / Fixed::from_num(n);
		sum += term;
	}
	if power >= 0 {
		for _ in 0..power { sum *= Fixed::from_num(2); }
	} else {
		for _ in power..0 { sum /= Fixed::from_num(2); }
	}
	sum
}
