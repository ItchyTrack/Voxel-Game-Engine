#[allow(unused)]
#[macro_export] macro_rules! make_system_manager {
    ($name:ident, ($($param:ident : $type:ty),*)) => {
        pub struct $name {
            systems: std::sync::Mutex<std::collections::HashMap<String, std::sync::Arc<dyn Fn($($type),*)>>>,
        }

        impl $name {
            pub fn new() -> Self {
                Self { systems: std::sync::Mutex::new(std::collections::HashMap::new()) }
            }

            pub fn add_system(&self, name: impl Into<String>, func: impl Fn($($type),*) + 'static) {
                self.systems.lock().unwrap().insert(name.into(), std::sync::Arc::new(func));
            }

            pub fn run_system(&self, name: &str, $($param: $type),*) {
                if let Some(system) = { self.systems.lock().unwrap().get(name).cloned() } {
                    system($($param),*);
                }
            }
        }
    };
}
