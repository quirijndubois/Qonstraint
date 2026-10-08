//! A world's constraint or force with an on/off switch (the editor's
//! per-connection toggle). It derefs to the item, so reading code is
//! unchanged; the integrators, contacts and drawing skip items that are off.

use std::ops::{Deref, DerefMut};

pub struct Slot<T: ?Sized> {
    item: Box<T>,
    /// Taking part in the simulation. Off, a constraint holds nothing and a
    /// force pushes nothing, but both stay in place (and saved) to turn back on.
    pub on: bool,
}

impl<T: ?Sized> Slot<T> {
    pub fn new(item: Box<T>) -> Self { Self { item, on: true } }
}

impl<T: ?Sized> Deref for Slot<T> {
    type Target = T;
    fn deref(&self) -> &T { &self.item }
}

impl<T: ?Sized> DerefMut for Slot<T> {
    fn deref_mut(&mut self) -> &mut T { &mut self.item }
}
