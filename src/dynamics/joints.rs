use std::ptr;
use {ffi, Wrapped, clone_from_ptr};
use math::Vec2;
use dynamics::Body;
use self::private::{JointDef, WrappedJoint};

macro_rules! impl_joint(
    (for $wrap:ty ($typ:ident) << $joint_as:path >> $as_joint:path) => (
        impl WrappedJoint for $wrap {
            unsafe fn from_joint_ptr(ptr: *mut ffi::Joint) -> $wrap {
                Wrapped::from_ptr($joint_as(ptr))
            }
            unsafe fn joint_ptr(&self) -> *const ffi::Joint {
                $as_joint(self.ptr) as *const ffi::Joint
            }
            unsafe fn mut_joint_ptr(&mut self) -> *mut ffi::Joint {
                $as_joint(self.ptr)
            }
            fn joint_type(_: Option<*const $wrap>) -> JointType {
                $typ
            }
        }
        
        impl Joint for $wrap {}
    );
)

macro_rules! joint_def(
    ($name:ident $(($(($visibility:ident))*) $field:ident: $typ:ty),+) => (
        #[allow(dead_code)]
        pub struct $name {
            pub base: JointDefBase,
            $(
                $($visibility)* $field: $typ,
            )+
        }
        
        impl JointDef for $name {
            unsafe fn from_joint_def_ptr(ptr: *mut JointDefBase) -> $name {
                *(ptr as *mut $name)
            }
            unsafe fn joint_def_ptr(&self) -> *const JointDefBase {
                self as *const $name as *const JointDefBase
            }
            unsafe fn mut_joint_def_ptr(&mut self) -> *mut JointDefBase {
                self as *mut $name as *mut JointDefBase
            }
        }
    );
)

#[allow(visible_private_types)]
pub mod private {
    use ffi;
    use super::JointDefBase;
    use super::JointType;
    
    pub trait JointDef {
        unsafe fn from_joint_def_ptr(ptr: *mut JointDefBase) -> Self;
        unsafe fn joint_def_ptr(&self) -> *const JointDefBase;
        unsafe fn mut_joint_def_ptr(&mut self) -> *mut JointDefBase;
    }

    pub trait WrappedJoint {
        unsafe fn from_joint_ptr(ptr: *mut ffi::Joint) -> Self;
        unsafe fn joint_ptr(&self) -> *const ffi::Joint;
        unsafe fn mut_joint_ptr(&mut self) -> *mut ffi::Joint;
        fn joint_type(_: Option<*const Self>) -> JointType;
    }
}

c_enum!(JointType with
    UNKNOWN_JOINT = 0,
    REVOLUTE_JOINT = 1,
    PRISMATIC_JOINT = 2,
    DISTANCE_JOINT = 3,
    PULLEY_JOINT = 4,
    MOUSE_JOINT = 5,
    GEAR_JOINT = 6,
    WHEEL_JOINT = 7,
    WELD_JOINT = 8,
    FRICTION_JOINT = 9,
    ROPE_JOINT = 10,
    MOTOR_JOINT = 11
)

c_enum!(LimitState with
    INACTIVE_LIMIT = 0,
    LOWER_LIMIT = 1,
    UPPER_LIMIT = 2,
    EQUAL_LIMITS = 3
)

#[allow(dead_code)]
pub struct JointDefBase {
    pub joint_type: JointType,
    user_data: ffi::UserData,
    body_a: *mut ffi::Body,
    body_b: *mut ffi::Body,
    pub collide_connected: bool,
}

impl JointDefBase {
    fn new(typ: JointType) -> JointDefBase {
        JointDefBase {
            joint_type: typ,
            user_data: ptr::mut_null(),
            body_a: ptr::mut_null(),
            body_b: ptr::mut_null(),
            collide_connected: false
        }
    }
}

pub trait Joint: WrappedJoint {
    fn joint_type(&self) -> JointType {
        unsafe {
            ffi::Joint_get_type(self.joint_ptr())
        }
    }
    fn body_a(&mut self) -> Body {
        unsafe {
            Wrapped::from_ptr(
                ffi::Joint_get_body_a(self.mut_joint_ptr())
                )
        }
    }
    fn body_b(&mut self) -> Body {
        unsafe {
            Wrapped::from_ptr(
                ffi::Joint_get_body_b(self.mut_joint_ptr())
                )
        }
    }
    fn anchor_a(&self) -> Vec2 {
        unsafe  {
            ffi::Joint_get_anchor_a_virtual(self.joint_ptr())
        }
    }
    fn anchor_b(&self) -> Vec2 {
        unsafe {
            ffi::Joint_get_anchor_b_virtual(self.joint_ptr())
        }
    }
    fn reaction_force(&self) -> Vec2 {
        unsafe {
            ffi::Joint_get_reaction_force_virtual(self.joint_ptr()).clone()
        }
    }
    fn reaction_torque(&self) -> f32 {
        unsafe {
            ffi::Joint_get_reaction_torque_virtual(self.joint_ptr())
        }
    }
    fn mut_next(&mut self) -> UnknownJoint {
        unsafe {
            WrappedJoint::from_joint_ptr(
                ffi::Joint_get_next(self.mut_joint_ptr())
                )
        }
    }
    /*unsafe fn next(&self) -> &UnknownJoint {
        unsafe {
            Wrapped::from_ptr(
                ffi::Joint_get_next_const(self.joint_ptr())
                )
        }
    }*/
    fn is_active(&self) -> bool {
        unsafe {
            ffi::Joint_is_active(self.joint_ptr())
        }
    }
    fn dump(&mut self) {
        unsafe {
            ffi::Joint_dump_virtual(self.mut_joint_ptr())
        }
    }
    fn shift_origin(&mut self, origin: &Vec2) {
        unsafe {
            ffi::Joint_shift_origin_virtual(self.mut_joint_ptr(),
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
        let joint_type = ffi::Joint_get_type(ptr as *const ffi::Joint);
        match joint_type {
            REVOLUTE_JOINT => Revolute (
                WrappedJoint::from_joint_ptr(ptr)
                ),
            PRISMATIC_JOINT => Prismatic(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            DISTANCE_JOINT => Distance(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            PULLEY_JOINT => Pulley(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            MOUSE_JOINT => Mouse(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            GEAR_JOINT => Gear(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            WHEEL_JOINT => Wheel(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            WELD_JOINT => Weld(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            FRICTION_JOINT => Friction(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            ROPE_JOINT => Rope(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            MOTOR_JOINT => Motor(
                WrappedJoint::from_joint_ptr(ptr)
                ),
            _ => Unknown
        }
    }
    unsafe fn joint_ptr(&self) -> *const ffi::Joint {
        match self {
            &Distance(ref x) => x.joint_ptr(),
            &Friction(ref x) => x.joint_ptr(),
            &Gear(ref x) => x.joint_ptr(),
            &Motor(ref x) => x.joint_ptr(),
            &Mouse(ref x) => x.joint_ptr(),
            &Prismatic(ref x) => x.joint_ptr(),
            &Pulley(ref x) => x.joint_ptr(),
            &Revolute(ref x) => x.joint_ptr(),
            &Rope(ref x) => x.joint_ptr(),
            &Weld(ref x) => x.joint_ptr(),
            &Wheel(ref x) => x.joint_ptr(),
            _ => fail!("Truly unknown joint")
        }
    }
    unsafe fn mut_joint_ptr(&mut self) -> *mut ffi::Joint {
        match self {
            &Distance(ref mut x) => x.mut_joint_ptr(),
            &Friction(ref mut x) => x.mut_joint_ptr(),
            &Gear(ref mut x) => x.mut_joint_ptr(),
            &Motor(ref mut x) => x.mut_joint_ptr(),
            &Mouse(ref mut x) => x.mut_joint_ptr(),
            &Prismatic(ref mut x) => x.mut_joint_ptr(),
            &Pulley(ref mut x) => x.mut_joint_ptr(),
            &Revolute(ref mut x) => x.mut_joint_ptr(),
            &Rope(ref mut x) => x.mut_joint_ptr(),
            &Weld(ref mut x) => x.mut_joint_ptr(),
            &Wheel(ref mut x) => x.mut_joint_ptr(),
            _ => fail!("Truly unknown joint")
        }
    }
    fn joint_type(_: Option<*const UnknownJoint>) -> JointType {
        UNKNOWN_JOINT
    }
}

impl Joint for UnknownJoint {}

joint_def!(DistanceJointDef
    () local_anchor_a: Vec2,
    () local_anchor_b: Vec2,
    ((pub)) length: f32,
    ((pub)) frequency: f32,
    ((pub)) damping_ratio: f32
)

joint_def!(FrictionJointDef
    () local_anchor_a: Vec2,
    () local_anchor_b: Vec2,
    ((pub)) max_force: f32,
    ((pub)) max_torque: f32
)

joint_def!(GearJointDef
    () joint_a: *mut ffi::Joint,
    () joint_b: *mut ffi::Joint,
    ((pub)) ratio: f32
)

joint_def!(MotorJointDef
    ((pub)) linear_offset: Vec2,
    ((pub)) angular_offset: f32,
    ((pub)) max_force: f32,
    ((pub)) max_torque: f32,
    ((pub)) correction_factor: f32
)

joint_def!(MouseJointDef
    ((pub)) target: Vec2,
    ((pub)) max_force: f32,
    ((pub)) frequency: f32,
    ((pub)) damping_ratio: f32
)

joint_def!(PrismaticJointDef
    () local_anchor_a: Vec2,
    () local_anchor_b: Vec2,
    () local_axis_a: Vec2,
    ((pub)) reference_angle: f32,
    ((pub)) enable_limit: bool,
    ((pub)) lower_translation: f32,
    ((pub)) upper_translation: f32,
    ((pub)) enable_motor: bool,
    ((pub)) max_motor_force: f32,
    ((pub)) motor_speed: f32
)

joint_def!(PulleyJointDef
    () ground_anchor_a: Vec2,
    () ground_anchor_b: Vec2,
    () local_anchor_a: Vec2,
    () local_anchor_b: Vec2,
    () length_a: f32,
    () length_b: f32,
    ((pub)) ratio: f32
)

joint_def!(RevoluteJointDef
    () local_anchor_a: Vec2,
    () local_anchor_b: Vec2,
    ((pub)) reference_angle: f32,
    ((pub)) enable_limit: bool,
    ((pub)) lower_angle: f32,
    ((pub)) upper_angle: f32,
    ((pub)) enable_motor: bool,
    ((pub)) motor_speed: f32,
    ((pub)) max_motor_torque: f32
)

joint_def!(RopeJointDef
    () local_anchor_a: Vec2,
    () local_anchor_b: Vec2,
    ((pub)) max_length: f32
)

joint_def!(WeldJointDef
    () local_anchor_a: Vec2,
    () local_anchor_b: Vec2,
    ((pub)) reference_angle: f32,
    ((pub)) frequency: f32,
    ((pub)) damping_ratio: f32
)

joint_def!(WheelJointDef
    () local_anchor_a: Vec2,
    () local_anchor_b: Vec2,
    () local_axis_a: Vec2,
    ((pub)) enable_motor: bool,
    ((pub)) max_motor_torque: f32,
    ((pub)) motor_speed: f32,
    ((pub)) frequency: f32,
    ((pub)) damping_ratio: f32
)

impl DistanceJointDef {    
    pub fn new(body_a: &mut Body,
               body_b: &mut Body,
               anchor_a: &Vec2,
               anchor_b: &Vec2) -> DistanceJointDef {
        unsafe {
            let mut joint =
                DistanceJointDef {
                    base: JointDefBase::new(DISTANCE_JOINT),
                    local_anchor_a: Vec2 { x:0., y:0. },
                    local_anchor_b: Vec2 { x:0., y:0. },
                    length: 1.,
                    frequency: 0.,
                    damping_ratio: 0.
                };
            ffi::DistanceJointDef_initialize(&mut joint,
                                             body_a.mut_ptr(),
                                             body_b.mut_ptr(),
                                             anchor_a, anchor_b);
            joint
        }
    }
}

impl FrictionJointDef {
    pub fn new(body_a: &mut Body,
               body_b: &mut Body,
               anchor: &Vec2) -> FrictionJointDef {
        unsafe {
            let mut joint =
                FrictionJointDef {
                    base: JointDefBase::new(FRICTION_JOINT),
                    local_anchor_a: Vec2 { x:0., y:0. },
                    local_anchor_b: Vec2 { x:0., y:0. },
                    max_force: 0.,
                    max_torque: 0.
                };
            ffi::FrictionJointDef_initialize(&mut joint,
                                             body_a.mut_ptr(),
                                             body_b.mut_ptr(),
                                             anchor);
            joint
        }
    }
}

impl GearJointDef {
    pub fn new(joint_a: &mut Joint,
               joint_b: &mut Joint) -> GearJointDef {
        unsafe {
            GearJointDef {
                base: JointDefBase::new(GEAR_JOINT),
                joint_a: joint_a.mut_joint_ptr(),
                joint_b: joint_b.mut_joint_ptr(),
                ratio: 1.
            }
        }
    }
}

impl MotorJointDef {
    pub fn new(body_a: &mut Body,
               body_b: &mut Body) -> MotorJointDef {
        unsafe {
            let mut joint = 
                MotorJointDef {
                    base: JointDefBase::new(MOTOR_JOINT),
                    linear_offset: Vec2 { x:0., y:0. },
                    angular_offset: 0.,
                    max_force: 1.,
                    max_torque: 1.,
                    correction_factor: 0.3
                };
            ffi::MotorJointDef_initialize(&mut joint,
                                          body_a.mut_ptr(),
                                          body_b.mut_ptr());
            joint
        }
    }
}

impl MouseJointDef {
    pub fn new() -> MouseJointDef {
        MouseJointDef {
            base: JointDefBase::new(MOUSE_JOINT),
            target: Vec2 { x:0., y:0. },
            max_force: 0.,
            frequency: 5.,
            damping_ratio: 0.7
        }
    }
}

impl PrismaticJointDef {
    pub fn new(body_a: &mut Body,
               body_b: &mut Body,
               anchor: &Vec2,
               axis: &Vec2) -> PrismaticJointDef {
        unsafe {
            let mut joint =
                PrismaticJointDef {
                    base: JointDefBase::new(PRISMATIC_JOINT),
                    local_anchor_a: Vec2 { x:0., y:0. },
                    local_anchor_b: Vec2 { x:0., y:0. },
                    local_axis_a: Vec2 { x:1., y:0. },
                    reference_angle: 0.,
                    enable_limit: false,
                    lower_translation: 0.,
                    upper_translation: 0.,
                    enable_motor: false,
                    max_motor_force: 0.,
                    motor_speed: 0.
                };
            ffi::PrismaticJointDef_initialize(&mut joint,
                                              body_a.mut_ptr(),
                                              body_b.mut_ptr(),
                                              anchor, axis);
            joint
        }
    }
}

impl PulleyJointDef {
    pub fn new(body_a: &mut Body,
               body_b: &mut Body,
               ground_anchor_a: &Vec2,
               ground_anchor_b: &Vec2,
               anchor_a: &Vec2,
               anchor_b: &Vec2,
               ratio: f32) -> PulleyJointDef {
        unsafe {
            let mut joint =
                PulleyJointDef {
                    base: JointDefBase::new(PULLEY_JOINT),
                    ground_anchor_a: Vec2 { x:-1., y:1. },
                    ground_anchor_b: Vec2 { x:1., y:1. },
                    local_anchor_a: Vec2 { x:-1., y:0. },
                    local_anchor_b: Vec2 { x:1., y:0. },
                    length_a: 0.,
                    length_b: 0.,
                    ratio: 1.
                };
            joint.base.collide_connected = true;
            ffi::PulleyJointDef_initialize(&mut joint,
                                           body_a.mut_ptr(),
                                           body_b.mut_ptr(),
                                           ground_anchor_a,
                                           ground_anchor_b,
                                           anchor_a, anchor_b,
                                           ratio);
            joint
        }
    }
}

impl RevoluteJointDef {
    pub fn new(body_a: &mut Body,
               body_b: &mut Body,
               anchor: &Vec2) -> RevoluteJointDef {
        unsafe {
            let mut joint =
                RevoluteJointDef {
                    base: JointDefBase::new(REVOLUTE_JOINT),
                    local_anchor_a: Vec2 { x:0., y:0. },
                    local_anchor_b: Vec2 { x:0., y:0. },
                    reference_angle: 0.,
                    lower_angle: 0.,
                    upper_angle: 0.,
                    max_motor_torque: 0.,
                    motor_speed: 0.,
                    enable_limit: false,
                    enable_motor: false
                };
            ffi::RevoluteJointDef_initialize(&mut joint,
                                             body_a.mut_ptr(),
                                             body_b.mut_ptr(),
                                             anchor);
            joint
        }
    }
}

impl RopeJointDef {
    pub fn new() -> RopeJointDef {
        RopeJointDef {
            base: JointDefBase::new(ROPE_JOINT),
            local_anchor_a: Vec2 { x:-1., y:0. },
            local_anchor_b: Vec2 { x:1., y:0. },
            max_length: 0.
        }
    }
}

impl WeldJointDef {
    pub fn new(body_a: &mut Body,
               body_b: &mut Body,
               anchor: &Vec2) -> WeldJointDef {
        unsafe {
            let mut joint =
                WeldJointDef {
                    base: JointDefBase::new(WELD_JOINT),
                    local_anchor_a: Vec2 { x:0., y:0. },
                    local_anchor_b: Vec2 { x:0., y:0. },
                    reference_angle: 0.,
                    frequency: 0.,
                    damping_ratio: 0.
                };
            ffi::WeldJointDef_initialize(&mut joint,
                                         body_a.mut_ptr(),
                                         body_b.mut_ptr(),
                                         anchor);
            joint
        }
    }
}

impl WheelJointDef {
    pub fn new(body_a: &mut Body,
               body_b: &mut Body,
               anchor: &Vec2,
               axis: &Vec2) -> WheelJointDef {
        unsafe {
            let mut joint =
                WheelJointDef {
                    base: JointDefBase::new(WHEEL_JOINT),
                    local_anchor_a: Vec2 { x:0., y:0. },
                    local_anchor_b: Vec2 { x:0., y:0. },
                    local_axis_a: Vec2 { x:1., y:0. },
                    enable_motor: false,
                    max_motor_torque: 0.,
                    motor_speed: 0.,
                    frequency: 2.,
                    damping_ratio: 0.7
                };
            ffi::WheelJointDef_initialize(&mut joint,
                                          body_a.mut_ptr(),
                                          body_b.mut_ptr(),
                                          anchor, axis);
            joint
        }
    }
}

wrap!(ffi::DistanceJoint into DistanceJoint)
wrap!(ffi::FrictionJoint into FrictionJoint)
wrap!(ffi::GearJoint into GearJoint)
wrap!(ffi::MotorJoint into MotorJoint)
wrap!(ffi::MouseJoint into MouseJoint)
wrap!(ffi::PrismaticJoint into PrismaticJoint)
wrap!(ffi::PulleyJoint into PulleyJoint)
wrap!(ffi::RevoluteJoint into RevoluteJoint)
wrap!(ffi::RopeJoint into RopeJoint)
wrap!(ffi::WeldJoint into WeldJoint)
wrap!(ffi::WheelJoint into WheelJoint)

impl DistanceJoint {
    pub fn local_anchor_a(&self) -> Vec2 {
        unsafe {
            *ffi::DistanceJoint_get_local_anchor_a(self.ptr()).clone()
        }
    }
    pub fn local_anchor_b(&self) -> Vec2 {
        unsafe {
            *ffi::DistanceJoint_get_local_anchor_b(self.ptr()).clone()
        }
    }
    pub fn set_length(&mut self, length: f32) {
        unsafe {
            ffi::DistanceJoint_set_length(self.mut_ptr(), length)
        }
    }
    pub fn length(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_length(self.ptr())
        }
    }
    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::DistanceJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }
    pub fn frequency(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_frequency(self.ptr())
        }
    }
    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::DistanceJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
    pub fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_damping_ratio(self.ptr())
        }
    }
}

impl FrictionJoint {
    pub fn local_anchor_a(&self) -> Vec2 {
        unsafe {
            *ffi::FrictionJoint_get_local_anchor_a(self.ptr()).clone()
        }
    }
    pub fn local_anchor_b(&self) -> Vec2 {
        unsafe {
            *ffi::FrictionJoint_get_local_anchor_b(self.ptr()).clone()
        }
    }
    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_force(self.mut_ptr(), force)
        }
    }
    pub fn max_force(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_force(self.ptr())
        }
    }
    pub fn set_max_torque(&mut self, torque: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_torque(self.mut_ptr(), torque)
        }
    }
    pub fn max_torque(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_torque(self.ptr())
        }
    }
}

impl GearJoint {/*
    pub fn joint_a(&self) -> UnknownJoint {
        unsafe {
            WrappedJoint::from_joint_ptr(
                ffi::GearJoint_get_joint_1(self.ptr())
                )
        }
    }
    pub fn joint_b(&self) -> UnknownJoint {
        unsafe {
            WrappedJoint::from_joint_ptr(
                ffi::GearJoint_get_joint_2(self.ptr())
                )
        }
    }*/
    pub fn set_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::GearJoint_set_ratio(self.mut_ptr(), ratio)
        }
    }
    pub fn ratio(&self) -> f32 {
        unsafe {
            ffi::GearJoint_get_ratio(self.ptr())
        }
    }
}

impl MotorJoint {
    pub fn set_linear_offset(&mut self, offset: &Vec2) {
        unsafe {
            ffi::MotorJoint_set_linear_offset(self.mut_ptr(), offset)
        }
    }
    pub fn linear_offset(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::MotorJoint_get_linear_offset(self.ptr()))
        }
    }
    pub fn set_angular_offset(&mut self, offset: f32) {
        unsafe {
            ffi::MotorJoint_set_angular_offset(self.mut_ptr(), offset)
        }
    }
    pub fn angular_offset(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_angular_offset(self.ptr())
        }
    }
    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::MotorJoint_set_max_force(self.mut_ptr(), force)
        }
    }
    pub fn max_force(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_max_force(self.ptr())
        }
    }
    pub fn set_max_torque(&mut self, torque: f32) {
        unsafe {
            ffi::MotorJoint_set_max_torque(self.mut_ptr(), torque)
        }
    }
    pub fn max_torque(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_max_torque(self.ptr())
        }
    }
    pub fn set_correction_factor(&mut self, factor: f32) {
        unsafe {
            ffi::MotorJoint_set_correction_factor(self.mut_ptr(), factor)
        }
    }
    pub fn correction_factor(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_correction_factor(self.ptr())
        }
    }
}

impl MouseJoint {
    pub fn set_target(&mut self, target: &Vec2) {
        unsafe {
            ffi::MouseJoint_set_target(self.mut_ptr(), target)
        }
    }
    pub fn target(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::MouseJoint_get_target(self.ptr()))
        }
    }
    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::MouseJoint_set_max_force(self.mut_ptr(), force)
        }
    }
    pub fn max_force(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_max_force(self.ptr())
        }
    }
    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::MouseJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }
    pub fn frequency(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_frequency(self.ptr())
        }
    }
    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::MouseJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
    pub fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_damping_ratio(self.ptr())
        }
    }
}

impl PrismaticJoint {
    pub fn local_anchor_a(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::PrismaticJoint_get_local_anchor_a(self.ptr()))
        }
    }
    pub fn local_anchor_b(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::PrismaticJoint_get_local_anchor_b(self.ptr()))
        }
    }
    pub fn local_axis_a(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::PrismaticJoint_get_local_axis_a(self.ptr()))
        }
    }
    pub fn reference_angle(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_reference_angle(self.ptr())
        }
    }
    pub fn joint_translation(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_joint_translation(self.ptr())
        }
    }
    pub fn joint_speed(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_joint_speed(self.ptr())
        }
    }
    pub fn is_limit_enabled(&self) -> bool {
        unsafe {
            ffi::PrismaticJoint_is_limit_enabled(self.ptr())
        }
    }
    pub fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::PrismaticJoint_enable_limit(self.mut_ptr(), flag)
        }
    }
    pub fn limits(&self) -> (f32, f32) {
        unsafe {
            (ffi::PrismaticJoint_get_lower_limit(self.ptr()),
             ffi::PrismaticJoint_get_upper_limit(self.ptr()))
        }
    }
    pub fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::PrismaticJoint_set_limits(self.mut_ptr(), lower, upper)
        }
    }
    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::PrismaticJoint_is_motor_enabled(self.ptr())
        }
    }
    pub fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::PrismaticJoint_enable_motor(self.mut_ptr(), flag)
        }
    }
    pub fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::PrismaticJoint_set_motor_speed(self.mut_ptr(), speed)
        }
    }
    pub fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_motor_speed(self.ptr())
        }
    }
    pub fn set_max_motor_force(&mut self, force: f32) {
        unsafe {
            ffi::PrismaticJoint_set_max_motor_force(self.mut_ptr(), force)
        }
    }
    pub fn max_motor_force(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_max_motor_force(self.ptr())
        }
    }
    pub fn motor_force(&self, inv_dt: f32) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_motor_force(self.ptr(), inv_dt)
        }
    }
}

impl PulleyJoint {
    pub fn ground_anchor_a(&self) -> Vec2 {
        unsafe {
            ffi::PulleyJoint_get_ground_anchor_a(self.ptr())
        }
    }
    pub fn ground_anchor_b(&self) -> Vec2 {
        unsafe {
            ffi::PulleyJoint_get_ground_anchor_b(self.ptr())
        }
    }
    pub fn length_a(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_length_a(self.ptr())
        }
    }
    pub fn length_b(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_length_b(self.ptr())
        }
    }
    pub fn ratio(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_ratio(self.ptr())
        }
    }
    pub fn current_length_a(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_current_length_a(self.ptr())
        }
    }
    pub fn current_length_b(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_current_length_b(self.ptr())
        }
    }
}

impl RevoluteJoint {
    pub fn local_anchor_a(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::RevoluteJoint_get_local_anchor_a(self.ptr()))
        }
    }
    pub fn local_anchor_b(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::RevoluteJoint_get_local_anchor_b(self.ptr()))
        }
    }
    pub fn referance_angle(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_reference_angle(self.ptr())
        }
    }
    pub fn joint_angle(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_joint_angle(self.ptr())
        }
    }
    pub fn joint_speed(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_joint_speed(self.ptr())
        }
    }
    pub fn is_limit_enabled(&self) -> bool {
        unsafe {
            ffi::RevoluteJoint_is_limit_enabled(self.ptr())
        }
    }
    pub fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::RevoluteJoint_enable_limit(self.mut_ptr(), flag)
        }
    }
    pub fn limits(&self) -> (f32, f32) {
        unsafe {
            (ffi::RevoluteJoint_get_lower_limit(self.ptr()),
             ffi::RevoluteJoint_get_upper_limit(self.ptr()))
        }
    }
    pub fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::RevoluteJoint_set_limits(self.mut_ptr(), lower, upper)
        }
    }
    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::RevoluteJoint_is_motor_enabled(self.ptr())
        }
    }
    pub fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::RevoluteJoint_enable_motor(self.mut_ptr(), flag)
        }
    }
    pub fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::RevoluteJoint_set_motor_speed(self.mut_ptr(), speed)
        }
    }
    pub fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_motor_speed(self.ptr())
        }
    }
    pub fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::RevoluteJoint_set_max_motor_torque(self.mut_ptr(), torque)
        }
    }
    pub fn max_motor_torque(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_max_motor_torque(self.ptr())
        }
    }
    pub fn motor_torque(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_motor_torque(self.ptr())
        }
    }
}

impl RopeJoint {
    pub fn local_anchor_a(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::RopeJoint_get_local_anchor_a(self.ptr()))
        }
    }
    pub fn local_anchor_b(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::RopeJoint_get_local_anchor_b(self.ptr()))
        }
    }
    pub fn set_max_length(&mut self, length: f32) {
        unsafe {
            ffi::RopeJoint_set_max_length(self.mut_ptr(), length)
        }
    }
    pub fn max_length(&self) -> f32 {
        unsafe {
            ffi::RopeJoint_get_max_length(self.ptr())
        }
    }
    pub fn limit_state(&self) -> LimitState {
        unsafe {
            ffi::RopeJoint_get_limit_state(self.ptr())
        }
    }
}

impl WeldJoint {
    pub fn local_anchor_a(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::WeldJoint_get_local_anchor_a(self.ptr()))
        }
    }
    pub fn local_anchor_b(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::WeldJoint_get_local_anchor_b(self.ptr()))
        }
    }
    pub fn referance_angle(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_reference_angle(self.ptr())
        }
    }
    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WeldJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }
    pub fn frequency(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_frequency(self.ptr())
        }
    }
    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WeldJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
    pub fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_damping_ratio(self.ptr())
        }
    }
}

impl WheelJoint {
    pub fn local_anchor_a(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::WheelJoint_get_local_anchor_a(self.ptr()))
        }
    }
    pub fn local_anchor_b(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::WheelJoint_get_local_anchor_b(self.ptr()))
        }
    }
    pub fn local_axis_a(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::WheelJoint_get_local_axis_a(self.ptr()))
        }
    }
    pub fn joint_translation(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_joint_translation(self.ptr())
        }
    }
    pub fn joint_speed(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_joint_speed(self.ptr())
        }
    }
    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::WheelJoint_is_motor_enabled(self.ptr())
        }
    }
    pub fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::WheelJoint_enable_motor(self.mut_ptr(), flag)
        }
    }
    pub fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::WheelJoint_set_motor_speed(self.mut_ptr(), speed)
        }
    }
    pub fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_motor_speed(self.ptr())
        }
    }
    pub fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::WheelJoint_set_max_motor_torque(self.mut_ptr(), torque)
        }
    }
    pub fn max_motor_torque(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_max_motor_torque(self.ptr())
        }
    }
    pub fn motor_torque(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_motor_torque(self.ptr())
        }
    }
    pub fn set_spring_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_frequency(self.mut_ptr(), frequency)
        }
    }
    pub fn spring_frequency(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_frequency(self.ptr())
        }
    }
    pub fn set_spring_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_damping_ratio(self.mut_ptr(), ratio)
        }
    }
    pub fn spring_damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_damping_ratio(self.ptr())
        }
    }
}

impl_joint!(for DistanceJoint (DISTANCE_JOINT)
    << ffi::Joint_as_distance_joint
    >> ffi::DistanceJoint_as_joint
    )
impl_joint!(for FrictionJoint (FRICTION_JOINT)
    << ffi::Joint_as_friction_joint
    >> ffi::FrictionJoint_as_joint
    )
impl_joint!(for GearJoint (GEAR_JOINT)
    << ffi::Joint_as_gear_joint
    >> ffi::GearJoint_as_joint
    )
impl_joint!(for MotorJoint (MOTOR_JOINT)
    << ffi::Joint_as_motor_joint
    >> ffi::MotorJoint_as_joint
    )
impl_joint!(for MouseJoint (MOUSE_JOINT)
    << ffi::Joint_as_mouse_joint
    >> ffi::MouseJoint_as_joint
    )
impl_joint!(for PrismaticJoint (PRISMATIC_JOINT)
    << ffi::Joint_as_prismatic_joint
    >> ffi::PrismaticJoint_as_joint
    )
impl_joint!(for PulleyJoint (PULLEY_JOINT)
    << ffi::Joint_as_pulley_joint
    >> ffi::PulleyJoint_as_joint
    )
impl_joint!(for RevoluteJoint (REVOLUTE_JOINT)
    << ffi::Joint_as_revolute_joint
    >> ffi::RevoluteJoint_as_joint
    )
impl_joint!(for RopeJoint (ROPE_JOINT)
    << ffi::Joint_as_rope_joint
    >> ffi::RopeJoint_as_joint
    )
impl_joint!(for WeldJoint (WELD_JOINT)
    << ffi::Joint_as_weld_joint
    >> ffi::WeldJoint_as_joint
    )
impl_joint!(for WheelJoint (WHEEL_JOINT)
    << ffi::Joint_as_wheel_joint
    >> ffi::WheelJoint_as_joint
    )
