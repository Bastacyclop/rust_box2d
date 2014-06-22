use std::iter::range_step;

use ffi;
use math::Vec2;
use dynamics::body;
use dynamics::Body;
use dynamics::joint;

pub struct World {
    ptr: *mut ffi::World,
}

impl World {
    pub unsafe fn from_ptr(ptr: *mut ffi::World) -> World {
        assert!(!ptr.is_null());
        World { ptr: ptr }
    }
    pub unsafe fn get_ptr(&self) -> *ffi::World {
        self.ptr as *ffi::World
    }
    pub unsafe fn get_mut_ptr(&mut self) -> *mut ffi::World {
        self.ptr
    }
    
    pub fn new(gravity: Vec2) -> World {
        unsafe {
            World::from_ptr(&mut ffi::World_new(&gravity))
        }
    }
    pub fn create_body<'l>(&'l mut self, def: body::Def) -> Body<'l> {
        unsafe {
            Body::from_ptr(
                ffi::World_create_body(self.get_mut_ptr(), &def)
                )
        }
    }
    pub fn destroy_body(&mut self, body: Body) {
        unsafe {
            let mut body = body;
            ffi::World_destroy_body(self.get_mut_ptr(), body.get_mut_ptr())
        }
    }
    pub fn create_joint<'l>(&'l mut self,
                            def: joint::Def) -> joint::Unknown<'l> {
        unsafe {
            joint::Unknown::from_ptr(
                ffi::World_create_joint(self.get_mut_ptr(), &def)
                )
        }
    }
    pub fn destroy_joint(&mut self, joint: joint::Unknown) {
        unsafe {
            let mut joint = joint;
            ffi::World_destroy_joint(self.get_mut_ptr(), joint.get_mut_ptr())
        }
    }
    pub fn step(&mut self,
                time_step: f32,
                velocity_iterations: f32,
                position_iterations: f32) {
        unsafe {
            ffi::World_step(self.get_mut_ptr(),
                            time_step,
                            velocity_iterations,
                            position_iterations)
        }
    }
    pub fn clear_forces(&mut self) {
        unsafe {
            ffi::World_clear_forces(self.get_mut_ptr())
        }
    }
    pub fn draw_debug_data(&mut self) {
        unsafe {
            ffi::World_draw_debug_data(self.get_mut_ptr())
        }
    }
    pub fn get_mut_body_list<'l>(&'l mut self) -> Vec<Body<'l>> {
        unsafe {
            let base = ffi::World_get_body_list(self.get_mut_ptr()) as uint;
            let count = self.get_body_count();
            let size = ffi::SIZEOF_BODY as uint;
            
            let mut vec = Vec::with_capacity(count);
            for ptr in range_step(base, base+(count*size), size) {
                vec.push(Body::from_ptr(ptr as *mut ffi::Body));
            }
            vec
        }
    }
    pub fn set_allow_sleeping(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_allow_sleeping(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_allow_sleeping(&self) -> bool {
        unsafe {
            ffi::World_get_allow_sleeping(self.get_ptr())
        }
    }
    pub fn set_warm_starting(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_warm_starting(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_warm_starting(&self) -> bool {
        unsafe {
            ffi::World_get_warm_starting(self.get_ptr())
        }
    }
    pub fn set_continuous_physics(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_continuous_physics(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_continuous_physics(&self) -> bool {
        unsafe {
            ffi::World_get_continuous_physics(self.get_ptr())
        }
    }
    pub fn set_sub_stepping(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_sub_stepping(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_sub_stepping(&self) -> bool {
        unsafe {
            ffi::World_get_sub_stepping(self.get_ptr())
        }
    }
    pub fn get_proxy_count(&self) -> uint {
        unsafe {
            ffi::World_get_proxy_count(self.get_ptr()) as uint
        }
    }
    pub fn get_body_count(&self) -> uint {
        unsafe {
            ffi::World_get_body_count(self.get_ptr()) as uint
        }
    }
    pub fn get_joint_count(&self) -> uint {
        unsafe {
            ffi::World_get_joint_count(self.get_ptr()) as uint
        }
    }
    pub fn get_contact_count(&self) -> uint {
        unsafe {
            ffi::World_get_contact_count(self.get_ptr()) as uint
        }
    }
    pub fn get_tree_height(&self) -> i32 {
        unsafe {
            ffi::World_get_tree_height(self.get_ptr())
        }
    }
    pub fn get_tree_balance(&self) -> i32 {
        unsafe {
            ffi::World_get_tree_balance(self.get_ptr())
        }
    }
    pub fn get_tree_quality(&self) -> f32 {
        unsafe {
            ffi::World_get_tree_quality(self.get_ptr())
        }
    }
    pub fn set_gravity(&mut self, gravity: Vec2) {
        unsafe {
            ffi::World_set_gravity(self.get_mut_ptr(), &gravity)
        }
    }
    pub fn get_gravity(&self) -> Vec2 {
        unsafe {
            ffi::World_get_gravity(self.get_ptr())
        }
    }
    pub fn is_locked(&self) -> bool {
        unsafe {
            ffi::World_is_locked(self.get_ptr())
        }
    }
    pub fn set_auto_clear_forces(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_auto_clear_forces(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_auto_clear_forces(&self) -> bool {
        unsafe {
            ffi::World_get_auto_clear_forces(self.get_ptr())
        }
    }
    pub fn shift_origin(&mut self, origin: Vec2) {
        unsafe {
            ffi::World_shift_origin(self.get_mut_ptr(), &origin)
        }
    }
    pub fn dump(&mut self) {
        unsafe {
            ffi::World_dump(self.get_mut_ptr())
        }
    }
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe {
            ffi::World_drop(self.get_mut_ptr())
        }
    }
}
