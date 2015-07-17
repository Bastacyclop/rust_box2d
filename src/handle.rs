use std::marker::PhantomData;
use std::ops::{Index, IndexMut};
use std::iter::{Iterator, DoubleEndedIterator};
use std::collections::VecMap;
use std::collections::vec_map;
use std::mem;

const INITIAL_CAPACITY: usize = 10;

#[derive(PartialOrd, Ord, Hash)]
pub struct TypedHandle<T> {
    index: usize,
    version: usize,
    phantom: PhantomData<T>
}

impl<T> TypedHandle<T> {
    pub fn new(index: usize, version: usize) -> TypedHandle<T> {
        TypedHandle {
            index: index,
            version: version,
            phantom: PhantomData
        }
    }

    #[inline]
    pub fn index(&self) -> usize {
        self.index
    }

    #[inline]
    pub fn version(&self) -> usize {
        self.version
    }
}

impl<T> Copy for TypedHandle<T> {}
impl<T> Clone for TypedHandle<T> {
    fn clone(&self) -> TypedHandle<T> { *self }
}


impl<T> Eq for TypedHandle<T> {}
impl<T> PartialEq for TypedHandle<T> {
    fn eq(&self, rhs: &TypedHandle<T>) -> bool {
        return self.index == rhs.index && self.version == rhs.version;
    }
}


struct HandleEntry<T> {
    pub version: usize,
    pub inner: Option<T>
}

impl<T> HandleEntry<T> {
    fn new() -> HandleEntry<T> {
        HandleEntry {
            version: 0,
            inner: None
        }
    }
}

pub struct HandleMap<T> {
    next_index: usize,
    availables: Vec<usize>,
    entries: VecMap<HandleEntry<T>>
}

impl<T> HandleMap<T> {
    pub fn new() -> HandleMap<T> {
        HandleMap {
            next_index: 0,
            availables: Vec::with_capacity(INITIAL_CAPACITY),
            entries: VecMap::new()
        }
    }

    pub fn with_capacity(capacity: usize) -> HandleMap<T> {
        HandleMap {
            next_index: 0,
            availables: Vec::with_capacity(INITIAL_CAPACITY),
            entries: VecMap::with_capacity(capacity)
        }
    }

    pub fn insert(&mut self, value: T) -> TypedHandle<T> {
        let index = match self.availables.pop() {
            Some(index) => index,
            None => {
                let next = self.next_index + 1;
                mem::replace(&mut self.next_index, next)
            }
        };

        if self.entries.len() <= index {
            self.entries.insert(index, HandleEntry::new());
        }

        let entry = &mut self.entries[index];
        entry.inner = Some(value);
        TypedHandle::new(index, entry.version)
    }

    pub fn remove(&mut self, handle: TypedHandle<T>) -> Option<T> {
        let index = handle.index;
        let entry = &mut self.entries[index];
        assert_eq!(entry.version, handle.version);
        assert!(entry.inner.is_some());

        entry.version += 1;
        self.availables.push(index);
        entry.inner.take()
    }

    pub fn clear(&mut self) {
        self.next_index = 0;
        self.availables.clear();
        self.entries.clear();
    }

    pub fn capacity(&self) -> usize {
        self.entries.capacity()
    }

    pub fn is_valid(&self, handle: TypedHandle<T>) -> bool {
        let entry = &self.entries[handle.index];

        entry.version == handle.version && entry.inner.is_some()
    }

    pub fn get(&self, handle: TypedHandle<T>) -> Option<&T> {
        let entry = &self.entries[handle.index];

        if entry.version == handle.version {
            return entry.inner.as_ref();
        }
        None
    }

    pub fn get_mut(&mut self, handle: TypedHandle<T>) -> Option<&mut T> {
        let entry = &mut self.entries[handle.index];

        if entry.version == handle.version {
            return entry.inner.as_mut();
        }
        None
    }

    pub fn iter<'a>(&'a self) -> HandleIter<'a, T> {
        HandleIter {
            iter: self.entries.iter()
        }
    }

    pub fn iter_mut<'a>(&'a mut self) -> HandleIterMut<'a, T> {
        HandleIterMut {
            iter: self.entries.iter_mut()
        }
    }
}

impl<T> Index<TypedHandle<T>> for HandleMap<T> {
    type Output = T;

    #[inline]
    fn index(&self, handle: TypedHandle<T>) -> &Self::Output {
        self.get(handle).unwrap()
    }
}

impl<T> IndexMut<TypedHandle<T>> for HandleMap<T> {
    #[inline]
    fn index_mut(&mut self, handle: TypedHandle<T>) -> &mut Self::Output {
        self.get_mut(handle).unwrap()
    }
}

impl<'a, T> IntoIterator for &'a HandleMap<T> {
    type Item = (TypedHandle<T>, &'a T);
    type IntoIter = HandleIter<'a, T>;

    fn into_iter(self) -> HandleIter<'a, T> {
        self.iter()
    }
}

impl<'a, T> IntoIterator for &'a mut HandleMap<T> {
    type Item = (TypedHandle<T>, &'a mut T);
    type IntoIter = HandleIterMut<'a, T>;

    fn into_iter(self) -> HandleIterMut<'a, T> {
        self.iter_mut()
    }
}

macro_rules! iterator {
    (impl $name:ident -> $elem:ty, $($getter:ident),+) => {
        impl<'a, T> Iterator for $name<'a, T> {
            type Item = $elem;

            #[inline]
            fn next(&mut self) -> Option<$elem> {
                while let Some((index, entry)) = self.iter.next() {
                    if entry.inner.is_some() {
                        return Some((TypedHandle::new(index, entry.version),
                                entry.inner$(. $getter())+.unwrap()));
                    }
                }
                None
            }

            #[inline]
            fn size_hint(&self) -> (usize, Option<usize>) {
                self.iter.size_hint()
            }
        }
    }
}

macro_rules! double_ended_iterator {
    (impl $name:ident -> $elem:ty, $($getter:ident),+) => {
        impl<'a, T> DoubleEndedIterator for $name<'a, T> {
            #[inline]
            fn next_back(&mut self) -> Option<$elem> {
                while let Some((index, entry)) = self.iter.next_back() {
                    if entry.inner.is_some() {
                        return Some((TypedHandle::new(index, entry.version),
                                entry.inner$(. $getter())+.unwrap()));
                    }
                }
                None
            }
        }
    }
}

pub struct HandleIter<'a, T: 'a> {
    iter: vec_map::Iter<'a, HandleEntry<T>>
}

iterator! { impl HandleIter -> (TypedHandle<T>, &'a T), as_ref }
double_ended_iterator! { impl HandleIter -> (TypedHandle<T>, &'a T), as_ref }

pub struct HandleIterMut<'a, T: 'a> {
    iter: vec_map::IterMut<'a, HandleEntry<T>>
}

iterator! { impl HandleIterMut -> (TypedHandle<T>, &'a mut T), as_mut }
double_ended_iterator! { impl HandleIterMut -> (TypedHandle<T>, &'a mut T), as_mut }

#[cfg(test)]
mod test {
    use super::*;

    const DUMMY_VALUE: usize = 5;

    #[test]
    fn insert() {
        let mut map = HandleMap::new();

        let handle = map.insert(DUMMY_VALUE);

        assert_eq!(DUMMY_VALUE, map[handle]);
    }

    #[test]
    fn remove() {
        let mut map = HandleMap::new();

        let handle = map.insert(DUMMY_VALUE);
        map.remove(handle);

        assert_eq!(map.is_valid(handle), false);
    }

    #[test]
    fn access_after_remove() {
        let mut map = HandleMap::new();

        let handle = map.insert(DUMMY_VALUE);
        map.remove(handle);

        assert_eq!(map.get(handle).is_none(), true)
    }

    #[test]
    fn access_with_old_handle() {
        let mut map = HandleMap::new();

        let handle = map.insert(DUMMY_VALUE);
        map.remove(handle);
        map.insert(DUMMY_VALUE);

        assert_eq!(map.get(handle).is_none(), true)
    }

    #[test]
    #[should_panic]
    fn remove_with_old_handle() {
        let mut map = HandleMap::new();
        let handle = map.insert(DUMMY_VALUE);
        map.remove(handle);
        map.remove(handle);
    }
}
