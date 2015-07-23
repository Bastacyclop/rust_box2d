use wrap::*;
use dynamics::world::{ World, JointHandle };
use dynamics::joints::{
    Joint, JointType, JointDefBase, RawJointDefBase, JointDef,
    ToRaw
};

joint_def! {
    RawGearJointDef => GearJointDef (JointType::Gear) {
        joint_a: *mut ffi::Joint => JointHandle,
        joint_b: *mut ffi::Joint => JointHandle,
        ratio: f32 => f32
    }
}

impl GearJointDef {
    pub fn new(joint_a: JointHandle,
               joint_b: JointHandle) -> GearJointDef {
        GearJointDef {
            base: JointDefBase::new(),
            joint_a: joint_a,
            joint_b: joint_b,
            ratio: 1.
        }
    }

    fn assert_well_formed(&self) { }
}

wrap_joint! {
    ffi::GearJoint => GearJoint (JointType::Gear)
    < ffi::GearJoint_as_joint
    > ffi::Joint_as_gear_joint
}

impl GearJoint {
    pub fn ratio(&self) -> f32 {
        unsafe {
            ffi::GearJoint_get_ratio(self.ptr())
        }
    }

    pub fn joint_1(&self) -> usize {
        unsafe {
            ffi::GearJoint_get_joint_1(self.ptr() as *mut ffi::GearJoint) as usize
        }
    }

    pub fn joint_2(&self) -> usize {
        unsafe {
            ffi::GearJoint_get_joint_2(self.ptr() as *mut ffi::GearJoint) as usize
        }
    }

    pub fn set_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::GearJoint_set_ratio(self.mut_ptr(), ratio)
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::joints::ffi::Joint;

    #[repr(C)] pub struct GearJoint;

    extern {
        pub fn GearJoint_as_joint(slf: *mut GearJoint) -> *mut Joint;
        pub fn Joint_as_gear_joint(slf: *mut Joint) -> *mut GearJoint;
        pub fn GearJoint_get_joint_1(slf: *mut GearJoint) -> *mut Joint;
        pub fn GearJoint_get_joint_2(slf: *mut GearJoint) -> *mut Joint;
        pub fn GearJoint_set_ratio(slf: *mut GearJoint, ratio: f32);
        pub fn GearJoint_get_ratio(slf: *const GearJoint) -> f32;
    }
}
