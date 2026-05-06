use super::entity_component_system::EntityComponentSystem;

pub struct System<D: 'static> {
    name: String,
    func: Box<dyn FnMut(&mut EntityComponentSystem<D>, &D)>,
}

impl<D: 'static> System<D> {
    pub fn new<F>(name: String, func: F) -> Self
    where
        F: FnMut(&mut EntityComponentSystem<D>, &D) + 'static,
    {
        Self { name: name, func: Box::new(func) }
    }

    pub fn name(&self) -> &str { &self.name }

    pub fn run(&mut self, ecs: &mut EntityComponentSystem<D>, data: &D) { (self.func)(ecs, data); }
}
