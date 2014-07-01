use ffi;
use super::{WrappedJoint, Joint, JointDef};

joint_def!(DistanceJointDef
    local_anchor_a: Vec2,
    local_anchor_b: Vec2,
    pub length: f32,
    pub frequency: f32,
    pub damping_ratio: f32
)

impl<'l> DistanceJointDef<'l> {    
    pub fn new<'m>(body_a: &'m mut Body,
                   body_b: &'m mut Body,
                   anchor_a: &Vec2,
                   anchor_b: &Vec2) -> DistanceJointDef<'m> {
        unsafe {
            let mut joint = ffi::DistanceJoint_default();
            ffi::DistanceJoint_initialize(&mut joint,
                                          body_a.get_mut_ptr(),
                                          body_b.get_mut_ptr(),
                                          anchor_a, anchor_b);
            joint
        }
    }
}

wrap!(ffi::DistanceJoint into DistanceJoint)

impl DistanceJoint {
    pub fn get_local_anchor_a(&self) -> Vec2 {
        unsafe {
            ffi::DistanceJoint_get_local_anchor_a(self.get_ptr()).clone()
        }
    }
    pub fn get_local_anchor_b(&self) -> Vec2 {
        unsafe {
            ffi::DistanceJoint_get_local_anchor_b(self.get_ptr()).clone()
        }
    }
    pub fn set_length(&mut self, length: f32) {
        unsafe {
            ffi::DistanceJoint_set_length(self.get_mut_ptr(), length)
        }
    }
    pub fn get_length(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_length(self.get_ptr())
        }
    }
    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::DistanceJoint_set_frequency(self.get_mut_ptr(), frequency)
        }
    }
    pub fn get_frequency(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_frequency(self.get_ptr())
        }
    }
    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::DistanceJoint_set_damping_ratio(self.get_mut_ptr(), ratio)
        }
    }
    pub fn get_damping_ratio(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_damping_ratio(self.get_ptr())
        }
    }
}

impl_joint!(for DistanceJoint
    << ffi::Joint_as_distance_joint
    >> ffi::DistanceJoint_as_joint
    )
