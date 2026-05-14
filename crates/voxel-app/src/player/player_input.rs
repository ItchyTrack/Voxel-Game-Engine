use winit::keyboard::KeyCode;

#[derive(Clone)]
pub struct KeyState {
	pub is_pressed: bool,
	pub just_pressed: bool,
	pub just_released: bool,
}

pub struct PlayerInput {
	keys: Vec<KeyState>,
	updated: Vec<KeyCode>,
}

impl PlayerInput {
	pub fn new() -> Self {
		PlayerInput {
			keys: vec![KeyState {
				is_pressed: false,
				just_pressed: false,
				just_released: false,
			} ; KeyCode::F35 as usize + 1],
			updated: vec![],
		}
	}
	pub fn key(&self, key_code: KeyCode) -> &KeyState {
		&self.keys[key_code as usize]
	}
	pub fn set_state(&mut self, key_code: KeyCode, is_pressed: bool) {
		if self.keys[key_code as usize].is_pressed == is_pressed { return; }
		let key_state = &mut self.keys[key_code as usize];
		if !(key_state.just_pressed || key_state.just_released) {
			self.updated.push(key_code);
		}
		if is_pressed {
			key_state.is_pressed = true;
			key_state.just_pressed = true;
		} else {
			key_state.is_pressed = false;
			key_state.just_released = true;
		}
	}
	pub fn end_frame(&mut self) {
		for key_code in self.updated.iter() {
			let key_state = &mut self.keys[*key_code as usize];
			key_state.just_pressed = false;
			key_state.just_released = false;
		}
		self.updated = vec![];
	}
	pub fn updated_keys(&self) -> &Vec<KeyCode> {
		&self.updated
	}
}
