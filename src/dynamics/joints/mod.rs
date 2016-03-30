macro_rules! wrap_joint {
    {
        $wrapped:ty => $wrap:ident ($joint_type:path)
        < $as_base:path
        > $base_as:path
    } => {
        wrap! {
            ffi::Joint: $wrapped => pub $wrap
            < $as_base
            > $base_as
        }

        impl Joint for $wrap {
            fn assumed_type() -> JointType { $joint_type }
        }
    };
}

pub mod distance;
pub mod friction;
pub mod gear;
pub mod motor;
pub mod mouse;
pub mod prismatic;
pub mod pulley;
pub mod revolute;
pub mod rope;
pub mod weld;
pub mod wheel;

pub use self::distance::{ DistanceJoint, DistanceJointDef };
pub use self::friction::{ FrictionJoint, FrictionJointDef };
pub use self::gear::{ GearJoint, GearJointDef };
pub use self::motor::{ MotorJoint, MotorJointDef };
pub use self::mouse::{ MouseJoint, MouseJointDef };
pub use self::prismatic::{ PrismaticJoint, PrismaticJointDef };
pub use self::pulley::{ PulleyJoint, PulleyJointDef };
pub use self::revolute::{ RevoluteJoint, RevoluteJointDef };
pub use self::rope::{ RopeJoint, RopeJointDef };
pub use self::weld::{ WeldJoint, WeldJointDef };
pub use self::wheel::{ WheelJoint, WheelJointDef };


use std::ops::{ Deref, DerefMut };
use std::any::Any;
use wrap::*;
use common::math::Vec2;
use dynamics::world::{ World, BodyHandle, JointHandle };
use dynamics::user_data::{ UserData, RawUserData, RawUserDataMut, InternalUserData };

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum JointType {
    Unknown,
    Revolute,
    Prismatic,
    Distance,
    Pulley,
    Mouse,
    Gear,
    Wheel,
    Weld,
    Friction,
    Rope,
    Motor
}

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum LimitState {
    Inactive,
    Lower,
    Upper,
    Equal
}

pub trait JointDef {
    fn joint_type() -> JointType where Self: Sized;

    #[doc(hidden)]
    unsafe fn create(&self, world: &mut World) -> *mut ffi::Joint;
}

pub struct MetaJoint {
    joint: UnknownJoint,
    user_data: Box<InternalUserData<MetaJoint>>
}

impl MetaJoint {
    #[doc(hidden)]
    pub unsafe fn new(ptr: *mut ffi::Joint, handle: JointHandle) -> MetaJoint {
        let mut j = MetaJoint {
            joint: UnknownJoint::from_ffi(ptr),
            user_data: Box::new(InternalUserData {
                handle: handle,
                custom: None
            })
        };
        j.mut_base_ptr().set_internal_user_data(&mut *j.user_data);
        j
    }
}

impl UserData for MetaJoint {
    fn get_user_data(&self) -> &Option<Box<Any>> {
        &self.user_data.custom
    }

    fn get_user_data_mut(&mut self) -> &mut Option<Box<Any>> {
        &mut self.user_data.custom
    }
}

impl Deref for MetaJoint {
    type Target = UnknownJoint;

    fn deref(&self) -> &UnknownJoint { &self.joint }
}

impl DerefMut for MetaJoint {
    fn deref_mut(&mut self) -> &mut UnknownJoint { &mut self.joint }
}

pub trait Joint: WrappedBase<ffi::Joint> + FromFFI<ffi::Joint> {
    fn assumed_type() -> JointType where Self: Sized;

    fn get_type(&self) -> JointType {
        unsafe {
            ffi::Joint_get_type(self.base_ptr())
        }
    }

    fn anchor_a(&self) -> Vec2 {
        unsafe  {
            ffi::Joint_get_anchor_a_virtual(self.base_ptr())
        }
    }

    fn anchor_b(&self) -> Vec2 {
        unsafe {
            ffi::Joint_get_anchor_b_virtual(self.base_ptr())
        }
    }

    fn reaction_force(&self) -> Vec2 {
        unsafe {
            ffi::Joint_get_reaction_force_virtual(self.base_ptr())
        }
    }

    fn reaction_torque(&self) -> f32 {
        unsafe {
            ffi::Joint_get_reaction_torque_virtual(self.base_ptr())
        }
    }

    fn is_active(&self) -> bool {
        unsafe {
            ffi::Joint_is_active(self.base_ptr())
        }
    }

    fn body_a(&mut self) -> BodyHandle {
        unsafe {
            ffi::Joint_get_body_a(self.mut_base_ptr()).get_handle()
        }
    }

    fn body_b(&mut self) -> BodyHandle {
        unsafe {
            ffi::Joint_get_body_b(self.mut_base_ptr()).get_handle()
        }
    }

    fn dump(&mut self) {
        unsafe {
            ffi::Joint_dump_virtual(self.mut_base_ptr())
        }
    }

    fn shift_origin(&mut self, origin: &Vec2) {
        unsafe {
            ffi::Joint_shift_origin_virtual(self.mut_base_ptr(), origin)
        }
    }
}

#[repr(C)]
pub struct JointEdge {
    other: *mut ffi::Body,
    joint: *mut ffi::Joint,
    prev: *mut JointEdge,
    next: *mut JointEdge
}

impl JointEdge {
    pub fn other(&self) -> BodyHandle {
        unsafe {
            self.other.get_handle()
        }
    }

    pub fn joint(&self) -> JointHandle {
        unsafe {
            self.joint.get_handle()
        }
    }

    pub fn prev_mut(&mut self) -> Option<&mut JointEdge> {
        unsafe {
            self.prev.as_mut()
        }
    }

    pub fn prev(&self) -> Option<&JointEdge> {
        unsafe {
            self.prev.as_ref()
        }
    }

    pub fn next_mut(&mut self) -> Option<&mut JointEdge> {
        unsafe {
            self.next.as_mut()
        }
    }

    pub fn next(&self) -> Option<&JointEdge> {
        unsafe {
            self.next.as_ref()
        }
    }
}

pub enum UnknownJoint {
    Unknown,
    Revolute(RevoluteJoint),
    Prismatic(PrismaticJoint),
    Distance(DistanceJoint),
    Pulley(PulleyJoint),
    Mouse(MouseJoint),
    Gear(GearJoint),
    Wheel(WheelJoint),
    Weld(WeldJoint),
    Friction(FrictionJoint),
    Rope(RopeJoint),
    Motor(MotorJoint)
}

impl WrappedBase<ffi::Joint> for UnknownJoint {
    unsafe fn base_ptr(&self) -> *const ffi::Joint {
        use self::UnknownJoint::*;
        match self {
            &Distance(ref x) => x.base_ptr(),
            &Friction(ref x) => x.base_ptr(),
            &Gear(ref x) => x.base_ptr(),
            &Motor(ref x) => x.base_ptr(),
            &Mouse(ref x) => x.base_ptr(),
            &Prismatic(ref x) => x.base_ptr(),
            &Pulley(ref x) => x.base_ptr(),
            &Revolute(ref x) => x.base_ptr(),
            &Rope(ref x) => x.base_ptr(),
            &Weld(ref x) => x.base_ptr(),
            &Wheel(ref x) => x.base_ptr(),
            _ => panic!("Truly unknown joint")
        }
    }

    unsafe fn mut_base_ptr(&mut self) -> *mut ffi::Joint {
        use self::UnknownJoint::*;
        match self {
            &mut Distance(ref mut x) => x.mut_base_ptr(),
            &mut Friction(ref mut x) => x.mut_base_ptr(),
            &mut Gear(ref mut x) => x.mut_base_ptr(),
            &mut Motor(ref mut x) => x.mut_base_ptr(),
            &mut Mouse(ref mut x) => x.mut_base_ptr(),
            &mut Prismatic(ref mut x) => x.mut_base_ptr(),
            &mut Pulley(ref mut x) => x.mut_base_ptr(),
            &mut Revolute(ref mut x) => x.mut_base_ptr(),
            &mut Rope(ref mut x) => x.mut_base_ptr(),
            &mut Weld(ref mut x) => x.mut_base_ptr(),
            &mut Wheel(ref mut x) => x.mut_base_ptr(),
            _ => panic!("Truly unknown joint")
        }
    }
}

impl FromFFI<ffi::Joint> for UnknownJoint {
    unsafe fn from_ffi(ptr: *mut ffi::Joint) -> UnknownJoint {
        use self::UnknownJoint::*;
        assert!(!ptr.is_null());
        let joint_type = ffi::Joint_get_type(ptr as *const ffi::Joint);
        match joint_type {
            JointType::Revolute => Revolute(RevoluteJoint::from_ffi(ptr)),
            JointType::Prismatic => Prismatic(PrismaticJoint::from_ffi(ptr)),
            JointType::Distance => Distance(DistanceJoint::from_ffi(ptr)),
            JointType::Pulley => Pulley(PulleyJoint::from_ffi(ptr)),
            JointType::Mouse => Mouse(MouseJoint::from_ffi(ptr)),
            JointType::Gear => Gear(GearJoint::from_ffi(ptr)),
            JointType::Wheel => Wheel(WheelJoint::from_ffi(ptr)),
            JointType::Weld => Weld(WeldJoint::from_ffi(ptr)),
            JointType::Friction => Friction(FrictionJoint::from_ffi(ptr)),
            JointType::Rope => Rope(RopeJoint::from_ffi(ptr)),
            JointType::Motor => Motor(MotorJoint::from_ffi(ptr)),
            _ => Unknown
        }
    }
}

impl Joint for UnknownJoint {
    fn assumed_type() -> JointType { JointType::Unknown }
}


#[doc(hidden)]
pub mod ffi {
    pub use ffi::Any;
    pub use dynamics::body::ffi::Body;
    use common::math::Vec2;
    use super::JointType;

    pub enum Joint {}

    extern {
        pub fn Joint_get_type(slf: *const Joint) -> JointType;
        pub fn Joint_get_body_a(slf: *mut Joint) -> *mut Body;
        pub fn Joint_get_body_b(slf: *mut Joint) -> *mut Body;
        pub fn Joint_get_anchor_a_virtual(slf: *const Joint) -> Vec2;
        pub fn Joint_get_anchor_b_virtual(slf: *const Joint) -> Vec2;
        pub fn Joint_get_reaction_force_virtual(slf: *const Joint) -> Vec2;
        pub fn Joint_get_reaction_torque_virtual(slf: *const Joint) -> f32;
        //pub fn Joint_get_next(slf: *mut Joint) -> *mut Joint;
        //pub fn Joint_get_next_const(slf: *const Joint) -> *const Joint;
        pub fn Joint_is_active(slf: *const Joint) -> bool;
        pub fn Joint_dump_virtual(slf: *mut Joint);
        pub fn Joint_shift_origin_virtual(slf: *mut Joint, origin: *const Vec2);
    }
}
