use std::collections::HashMap;

pub trait SparseSetKey: std::hash::Hash { }

pub type SparseSet<K: SparseSetKey, T> = HashMap<K, T>;
