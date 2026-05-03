use super::entity_component_system::EntityComponentSystem;

pub struct System {
    name: String,
    func: Box<dyn FnMut(&mut EntityComponentSystem)>,
}

impl System {
    pub fn new<F>(name: String, func: F) -> Self
    where
        F: FnMut(&mut EntityComponentSystem) + 'static,
    {
        Self {
            name: name,
            func: Box::new(func),
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn run(&mut self, ecs: &mut EntityComponentSystem) {
        (self.func)(ecs);
    }
}