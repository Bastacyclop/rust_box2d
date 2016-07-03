use std::cell::{RefCell, Ref, RefMut};
use std::marker::PhantomData;
use std::iter::{Iterator, DoubleEndedIterator};
use vec_map::{self, VecMap};
use std::mem;
use std::fmt;

#[derive(PartialOrd, Ord, Hash)]
pub struct TypedHandle<T: ?Sized> {
    index: usize,
    version: usize,
    phantom: PhantomData<T>,
}

impl<T: ?Sized> TypedHandle<T> {
    #[doc(hidden)]
    pub fn new(index: usize, version: usize) -> TypedHandle<T> {
        TypedHandle {
            index: index,
            version: version,
            phantom: PhantomData,
        }
    }

    #[inline]
    #[doc(hidden)]
    pub fn index(&self) -> usize {
        self.index
    }

    #[inline]
    #[doc(hidden)]
    pub fn version(&self) -> usize {
        self.version
    }
}

impl<T: ?Sized> Copy for TypedHandle<T> {}
impl<T: ?Sized> Clone for TypedHandle<T> {
    fn clone(&self) -> TypedHandle<T> {
        *self
    }
}

unsafe impl<T: ?Sized> Send for TypedHandle<T> {}
unsafe impl<T: ?Sized> Sync for TypedHandle<T> {}

impl<T: ?Sized> Eq for TypedHandle<T> {}
impl<T: ?Sized> PartialEq for TypedHandle<T> {
    fn eq(&self, rhs: &TypedHandle<T>) -> bool {
        return self.index == rhs.index && self.version == rhs.version;
    }
}

impl<T: ?Sized> fmt::Debug for TypedHandle<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
               "TypedHandle {{ index: {}, version: {} }}",
               self.index,
               self.version)
    }
}

struct HandleEntry<E> {
    pub version: usize,
    pub inner: Option<RefCell<E>>,
}

impl<E> HandleEntry<E> {
    fn new() -> HandleEntry<E> {
        HandleEntry {
            version: 0,
            inner: None,
        }
    }
}

#[doc(hidden)]
pub struct HandleMap<E, T: ?Sized = ()> {
    next_index: usize,
    availables: Vec<usize>,
    entries: VecMap<HandleEntry<E>>,
    phantom: PhantomData<T>,
}

impl<T: ?Sized, E> HandleMap<E, T> {
    pub fn new() -> HandleMap<E, T> {
        HandleMap {
            next_index: 0,
            availables: Vec::new(),
            entries: VecMap::new(),
            phantom: PhantomData,
        }
    }

    pub fn with_capacities(availables: usize, entries: usize) -> HandleMap<E, T> {
        HandleMap {
            next_index: 0,
            availables: Vec::with_capacity(availables),
            entries: VecMap::with_capacity(entries),
            phantom: PhantomData,
        }
    }

    pub fn insert(&mut self, value: E) -> TypedHandle<T> {
        let index = self.find_available();

        let entry = &mut self.entries[index];
        entry.inner = Some(RefCell::new(value));
        TypedHandle::new(index, entry.version)
    }

    pub fn insert_with<F>(&mut self, f: F) -> TypedHandle<T>
        where F: FnOnce(TypedHandle<T>) -> E
    {

        let index = self.find_available();

        let entry = &mut self.entries[index];
        let handle = TypedHandle::new(index, entry.version);
        entry.inner = Some(RefCell::new(f(handle)));
        handle
    }

    fn find_available(&mut self) -> usize {
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

        index
    }

    pub fn remove(&mut self, handle: TypedHandle<T>) -> E {
        let index = handle.index;
        let entry = &mut self.entries[index];
        assert_eq!(entry.version, handle.version);
        assert!(entry.inner.is_some());

        entry.version += 1;
        self.availables.push(index);
        entry.inner.take().unwrap().into_inner()
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

    #[inline]
    fn get_inner(&self, handle: TypedHandle<T>) -> Option<&RefCell<E>> {
        let entry = &self.entries[handle.index];

        if entry.version == handle.version {
            entry.inner.as_ref()
        } else {
            None
        }
    }

    pub fn get(&self, handle: TypedHandle<T>) -> Option<Ref<E>> {
        self.get_inner(handle).map(|e| e.borrow())
    }

    pub fn get_mut(&self, handle: TypedHandle<T>) -> Option<RefMut<E>> {
        self.get_inner(handle).map(|e| e.borrow_mut())
    }

    pub fn iter<'a>(&'a self) -> HandleIter<'a, T, E> {
        HandleIter {
            iter: self.entries.iter(),
            phantom: PhantomData,
        }
    }
}

impl<'a, T: ?Sized, E> IntoIterator for &'a HandleMap<E, T> {
    type Item = (TypedHandle<T>, &'a RefCell<E>);
    type IntoIter = HandleIter<'a, T, E>;

    fn into_iter(self) -> HandleIter<'a, T, E> {
        self.iter()
    }
}

macro_rules! iterator {
    (impl $name:ident -> $elem:ty, $($getter:ident),+ and then $($modifier:ident),*) => {
        impl<'a, T: ?Sized, E> Iterator for $name<'a, T, E> {
            type Item = $elem;

            #[inline]
            fn next(&mut self) -> Option<$elem> {
                while let Some((index, entry)) = self.iter.next() {
                    if entry.inner.is_some() {
                        return Some((
                            TypedHandle::new(index, entry.version),
                            entry.inner$(. $getter())+.unwrap()$(. $modifier())*
                        ));
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
    (impl $name:ident -> $elem:ty, $($getter:ident),+ and then $($modifier:ident),*) => {
        impl<'a, T: ?Sized, E> DoubleEndedIterator for $name<'a, T, E> {
            #[inline]
            fn next_back(&mut self) -> Option<$elem> {
                while let Some((index, entry)) = self.iter.next_back() {
                    if entry.inner.is_some() {
                        return Some((
                            TypedHandle::new(index, entry.version),
                            entry.inner$(. $getter())+.unwrap()$(. $modifier())*
                        ));
                    }
                }
                None
            }
        }
    }
}

pub struct HandleIter<'a, T: ?Sized, E: 'a> {
    iter: vec_map::Iter<'a, HandleEntry<E>>,
    phantom: PhantomData<T>,
}

iterator! {
    impl HandleIter -> (TypedHandle<T>, &'a RefCell<E>),
    as_ref and then
}

double_ended_iterator! {
    impl HandleIter -> (TypedHandle<T>, &'a RefCell<E>),
    as_ref and then
}

#[cfg(test)]
mod test {
    use super::*;

    const DUMMY_VALUE: usize = 5;

    #[test]
    fn insert() {
        let mut map = HandleMap::<usize>::new();

        let handle = map.insert(DUMMY_VALUE);

        assert_eq!(*map.get(handle).unwrap(), DUMMY_VALUE);
    }

    #[test]
    fn remove() {
        let mut map = HandleMap::<usize>::new();

        let handle = map.insert(DUMMY_VALUE);
        map.remove(handle);

        assert!(!map.is_valid(handle));
    }

    #[test]
    fn access_after_remove() {
        let mut map = HandleMap::<usize>::new();

        let handle = map.insert(DUMMY_VALUE);
        map.remove(handle);

        assert!(map.get(handle).is_none());
    }

    #[test]
    fn access_with_old_handle() {
        let mut map = HandleMap::<usize>::new();

        let handle = map.insert(DUMMY_VALUE);
        map.remove(handle);
        map.insert(DUMMY_VALUE);

        assert!(map.get(handle).is_none())
    }

    #[test]
    #[should_panic]
    fn remove_with_old_handle() {
        let mut map = HandleMap::<usize>::new();
        let handle = map.insert(DUMMY_VALUE);
        map.remove(handle);
        map.remove(handle);
    }
}
