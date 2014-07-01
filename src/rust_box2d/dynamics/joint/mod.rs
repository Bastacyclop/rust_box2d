use ffi;
use math::Vec2;

pub use self::distance::{DistanceJointDef, DistanceJoint};
pub use self::friction::{FrictionJointDef, FrictionJoint};
pub use self::gear::{GearJointDef, GearJoint};
pub use self::motor::{MotorJointDef, MotorJoint};
pub use self::mouse::{MouseJointDef, MouseJoint};
pub use self::prismatic::{PrismaticJointDef, PrismaticJoint};
pub use self::pulley::{PulleyJointDef, PulleyJoint};
pub use self::revolute::{RevoluteJointDef, RevoluteJoint};
pub use self::weld::{WeldJointDef, WeldJoint};
pub use self::wheel::{WheelJointDef, WheelJoint};

#[macro_export]
macro_rules! impl_joint(
    (for $wrap:ty << $joint_as:path >> $as_joint:path) => (
        impl WrappedJoint for $wrap {
            unsafe fn from_joint_ptr(ptr: *mut ffi::Shape) -> $wrap {
                Wrapped::from_ptr($joint_as(ptr))
            }
            unsafe fn get_joint_ptr(&self) -> *ffi::Shape {
                $as_joint(self.ptr) as *ffi::Shape
            }
            unsafe fn get_mut_joint_ptr(&mut self) -> *mut ffi::Shape {
                $as_joint(self.ptr)
            }
        }
        
        impl Joint for $wrap {}
    );
)

#[macro_export]
macro_rules! joint_def(
    ($name:ident $fields:tt) => (
        #[packed]
        pub struct $name<'l> {
            JointDefBase<'l>
            $fields
        }
        
        impl JointDef for $name {
            unsafe fn from_joint_def_ptr(ptr: *mut JointDefBase) -> $name {
                *(ptr as *mut $name)
            }
            unsafe fn get_joint_def_ptr(&self) -> *JointDefBase {
                self as *JointDefBase
            }
            unsafe fn get_mut_joint_def_ptr(&mut self) -> *mut JointDefBase {
                self as *mut JointDefBase
            }
        }
    );
)

pub mod distance;
pub mod friction;
pub mod gear;
pub mod motor;
pub mod mouse;
pub mod prismatic;
pub mod pulley;
pub mod revolute;
pub mod weld;
pub mod wheel;

c_enum!(JointType with
    UNKNOWN = 0,
    REVOLUTE = 1,
    PRISMATIC = 2,
    DISTANCE = 3,
    PULLEY = 4,
    MOUSE = 5,
    GEAR = 6,
    WHEEL = 7,
    WELD = 8,
    FRICTION = 9,
    ROPE = 10,
    MOTOR = 11
)

c_enum!(LimitState with
    INACTIVE_LIMIT = 0,
    LOWER_LIMIT = 1,
    UPPER_LIMIT = 2,
    EQUAL_LIMITS = 3
)

#[allow(dead_code)]
#[packed]
pub struct JointDefBase<'l> {
    pub joint_type: JointType,
    user_data: ffi::UserData,
    body_a: *mut ffi::Body,
    body_b: *mut ffi::Body,
    pub collide_connected: bool,
}

trait JointDef {
    unsafe fn from_joint_def_ptr(ptr: *mut JointDefBase) -> Self
    unsafe fn get_joint_def_ptr(&self) -> *JointDefBase;
    unsafe fn get_mut_joint_def_ptr(&mut self) -> *mut JointDefBase;
}

trait WrappedJoint {
    unsafe fn from_joint_ptr(ptr: *mut ffi::Joint) -> Self;
    unsafe fn get_joint_ptr(&self) -> *ffi::Joint;
    unsafe fn get_mut_joint_ptr(&mut self) -> *mut ffi::Joint;
}

pub trait Joint: WrappedJoint {
    fn get_type(&self) -> JointType {
        unsafe {
            ffi::Joint_get_type(self.get_joint_ptr())
        }
    }
    /*unsafe fn get_body_a(&mut self) -> Body {
        unsafe {
            Wrapped::from_ptr(
                ffi::Joint_get_body_a(self.get_mut_joint_ptr())
                )
        }
    }
    unsafe fn get_body_b(&mut self) -> Body {
        unsafe {
            Wrapped::from_ptr(
                ffi::Joint_get_body_b(self.get_mut_joint_ptr())
                )
        }
    }*/
    fn get_anchor_a(&self) -> Vec2 {
        unsafe  {
            ffi::Joint_get_anchor_a_virtual(self.get_joint_ptr())
        }
    }
    fn get_anchor_b(&self) -> Vec2 {
        unsafe {
            ffi::Joint_get_anchor_b_virtual(self.get_joint_ptr())
        }
    }
    fn get_reaction_force(&self) -> Vec2 {
        unsafe {
            ffi::Joint_get_reaction_force_virtual(self.get_joint_ptr())
        }
    }
    fn get_reaction_torque(&self) -> f32 {
        unsafe {
            ffi::Joint_get_reaction_torque_virtual(self.get_joint_ptr())
        }
    }
    /*unsafe fn get_mut_next(&mut self) -> &mut UnknownJoint {
        unsafe {
            Wrapped::from_ptr(
                ffi::Joint_get_next(self.get_mut_joint_ptr())
                )
        }
    }
    unsafe fn get_next(&self) -> &UnknownJoint {
        unsafe {
            Wrapped::from_ptr(
                ffi::Joint_get_next_const(self.get_joint_ptr())
                )
        }
    }*/
    fn is_active(&self) -> bool {
        unsafe {
            ffi::Joint_is_active(self.get_joint_ptr())
        }
    }
    fn dump(&mut self) {
        unsafe {
            ffi::Joint_dump_virtual(self.get_mut_joint_ptr())
        }
    }
    fn shift_origin(&mut self, origin: &Vec2) {
        unsafe {
            ffi::Joint_shift_origin_virtual(self.get_mut_joint_ptr(),
                                            origin)
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

impl WrappedJoint for UnknownJoint {
    unsafe fn from_joint_ptr(ptr: *mut ffi::Joint) -> UnknownJoint {
        assert!(!ptr.is_null())
        let joint_type = ffi::Joint_get_type(ptr as *ffi::Joint);
        match joint_type {
            REVOLUTE => Revolute (
                WrappedJoint::from_joint_ptr(ptr)
                ),
            PRISMATIC => Prismatic(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            DISTANCE => Distance(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            PULLEY => Pulley(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            MOUSE => Mouse(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            GEAR => Gear(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            WHEEL => Wheel(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            WELD => Weld(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            FRICTION => Friction(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            ROPE => Rope(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            MOTOR => Motor(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            _ => Unknown
        }
    }
    unsafe fn get_joint_ptr(&self) -> *ffi::Joint {
        match self {
            &Distance(x) => x.get_joint_ptr(),
            &Friction(x) => x.get_joint_ptr(),
            &Gear(x) => x.get_joint_ptr(),
            &Motor(x) => x.get_joint_ptr(),
            &Mouse(x) => x.get_joint_ptr(),
            &Prismatic(x) => x.get_joint_ptr(),
            &Pulley(x) => x.get_joint_ptr(),
            &Revolute(x) => x.get_joint_ptr(),
            &Rope(x) => x.get_joint_ptr(),
            &Weld(x) => x.get_joint_ptr(),
            &Wheel(x) => x.get_joint_ptr(),
            _ => fail!("Truly unknown joint")
        }
    }
    unsafe fn get_mut_joint_ptr(&mut self) -> *mut ffi::Joint {
        self.get_joint_ptr() as *mut ffi::Joint
    }
}

impl Joint for UnknownJoint {}

/*
impl Drop for UnknownJoint {
    fn drop(&mut self) {
    }
}*/
