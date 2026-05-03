use std::any::{Any, TypeId};
use std::collections::VecDeque;

trait MessageQueueDyn {
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn held_type_id(&self) -> TypeId;
    fn clear(&mut self);
    fn len(&self) -> usize;
}

struct MessageQueueImpl<T> {
    queue: VecDeque<T>,
}

impl<T: 'static> MessageQueueDyn for MessageQueueImpl<T> {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn held_type_id(&self) -> TypeId { TypeId::of::<T>() }
    fn clear(&mut self) { self.queue.clear(); }
    fn len(&self) -> usize { self.queue.len() }
}

pub struct MessageQueue {
    internal: Box<dyn MessageQueueDyn>,
}

impl MessageQueue {
    pub fn new<T: 'static>() -> Self {
        Self {
            internal: Box::new(MessageQueueImpl::<T> {
                queue: VecDeque::new(),
            }),
        }
    }

    pub fn push<T: 'static>(&mut self, message: T) {
        assert!(TypeId::of::<T>() == self.internal.held_type_id());
        self.internal
            .as_any_mut()
            .downcast_mut::<MessageQueueImpl<T>>()
            .unwrap()
            .queue
            .push_back(message);
    }

    pub fn pop<T: 'static>(&mut self) -> Option<T> {
        assert!(TypeId::of::<T>() == self.internal.held_type_id());
        self.internal
            .as_any_mut()
            .downcast_mut::<MessageQueueImpl<T>>()
            .unwrap()
            .queue
            .pop_front()
    }

    pub fn iter<T: 'static>(&self) -> impl Iterator<Item = &T> {
        assert!(TypeId::of::<T>() == self.internal.held_type_id());
        self.internal
            .as_any()
            .downcast_ref::<MessageQueueImpl<T>>()
            .unwrap()
            .queue
            .iter()
    }

    pub fn clear(&mut self) {
        self.internal.clear();
    }

    pub fn len(&self) -> usize {
        self.internal.len()
    }

    pub fn is_empty(&self) -> bool {
        self.internal.len() == 0
    }

    pub fn held_type_id(&self) -> TypeId {
        self.internal.held_type_id()
    }
}