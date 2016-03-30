use std::mem;
use std::marker::PhantomData;
use common::math::Sweep;
use collision::distance::{Proxy, RawProxy};

#[repr(C)]
#[doc(hidden)]
pub struct RawInput {
    proxy_a: RawProxy,
    proxy_b: RawProxy,
    sweep_a: Sweep,
    sweep_b: Sweep,
    t_max: f32,
}

pub struct Input<'a> {
    raw: RawInput,
    phantom: PhantomData<&'a ()>,
}

impl<'a> Input<'a> {
    pub fn new(proxy_a: Proxy<'a>,
               proxy_b: Proxy<'a>,
               sweep_a: Sweep,
               sweep_b: Sweep,
               t_max: f32)
               -> Input<'a> {
        Input {
            raw: RawInput {
                proxy_a: proxy_a.raw,
                proxy_b: proxy_b.raw,
                sweep_a: sweep_a,
                sweep_b: sweep_b,
                t_max: t_max,
            },
            phantom: PhantomData,
        }
    }

    pub fn query(&self) -> Output {
        unsafe {
            let mut out = mem::zeroed();
            ffi::time_of_impact(&mut out, &self.raw);
            out
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum State {
    Unknown,
    Failed,
    Overlapped,
    Touching,
    Separated,
}

#[repr(C)]
pub struct Output {
    pub state: State,
    pub t: f32,
}

#[doc(hidden)]
pub mod ffi {
    use super::{RawInput, Output};

    extern "C" {
        pub fn time_of_impact(output: *mut Output, input: *const RawInput);
    }
}
