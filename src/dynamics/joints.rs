use std::ptr;
use {
    Wrapped, WrappedMut, WrappedConst,
    WrappedBase, WrappedMutBase, WrappedConstBase,
    ffi
};
use math::Vec2;
use dynamics::{BodyMutRef};
use self::private::{JointDef, WrappedJoint};

macro_rules! wrapped_joint(
    ($wrapped:ty into $wrap:ident, $const_wrap:ident ($typ:ident)
     << $base_as:path
     >> $as_base:path) => (
     
        wrapped!($wrapped into $wrap, $const_wrap
                 with base ffi::Joint
                 << $base_as
                 >> $as_base)
        
        impl<'l> WrappedJoint for $wrap<'l> {
            fn joint_type(_: Option<*const $wrap>) -> JointType {
                $typ
            }
        }               
        impl<'l> ConstJoint for $wrap<'l> {}
        impl<'l> MutJoint for $wrap<'l> {}
        
        impl<'l> WrappedJoint for $const_wrap<'l> {
            fn joint_type(_: Option<*const $const_wrap>) -> JointType {
                $typ
            }
        }
        impl<'l> ConstJoint for $const_wrap<'l> {}
    );
)

macro_rules! joint_def(
    ($name:ident $(($(($visibility:ident))*) $field:ident: $typ:ty),+) => (
        #[repr(C)]
        #[allow(dead_code)]
        pub struct $name {
            pub base: JointDefBase,
            $(
                $($visibility)* $field: $typ,
            )+
        }
        
        impl JointDef for $name {
            unsafe fn from_ptr(ptr: *mut JointDefBase) -> $name {
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
    use super::JointDefBase;
    use super::JointType;
    
    pub trait JointDef {
        unsafe fn from_ptr(ptr: *mut JointDefBase) -> Self;
        unsafe fn joint_def_ptr(&self) -> *const JointDefBase;
        unsafe fn mut_joint_def_ptr(&mut self) -> *mut JointDefBase;
    }

    pub trait WrappedJoint {
        fn joint_type(_: Option<*const Self>) -> JointType;
    }
}

#[repr(C)]
#[deriving(PartialEq, Show)]
pub enum JointType {
    UnknownJointType = 0,
    RevoluteJointType = 1,
    PrismaticJointType = 2,
    DistanceJointType = 3,
    PulleyJointType = 4,
    MouseJointType = 5,
    GearJointType = 6,
    WheelJointType = 7,
    WeldJointType = 8,
    FrictionJointType = 9,
    RopeJointType = 10,
    MotorJointType = 11
}

#[repr(C)]
#[deriving(PartialEq, Show)]
pub enum LimitState {
    InactiveLimitState = 0,
    LowerLimitState = 1,
    UpperLimitState = 2,
    EqualLimitStateS = 3
}

#[repr(C)]
#[allow(dead_code)]
pub struct JointDefBase {
    pub joint_type: JointType,
    user_data: ffi::Any,
    body_a: *mut ffi::Body,
    body_b: *mut ffi::Body,
    pub collide_connected: bool,
}

impl JointDefBase {
    fn new(joint_type: JointType) -> JointDefBase {
        JointDefBase {
            joint_type: joint_type,
            user_data: ptr::mut_null(),
            body_a: ptr::mut_null(),
            body_b: ptr::mut_null(),
            collide_connected: false
        }
    }
    
    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        self.user_data = data as ffi::Any
    }
}

#[allow(visible_private_types)]
pub trait ConstJoint: WrappedJoint+WrappedBase<ffi::Joint> {
    fn joint_type(&self) -> JointType {
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
            ffi::Joint_get_reaction_force_virtual(self.base_ptr()).clone()
        }
    }
    
    fn reaction_torque(&self) -> f32 {
        unsafe {
            ffi::Joint_get_reaction_torque_virtual(self.base_ptr())
        }
    }
    
    fn next<'a>(&'a self) -> UnknownJointRef<'a> {
        unsafe {
            WrappedConstBase::from_ptr(ffi::Joint_get_next_const(self.base_ptr()))
        }
    }
    
    fn is_active(&self) -> bool {
        unsafe {
            ffi::Joint_is_active(self.base_ptr())
        }
    }
    
    unsafe fn user_data<T>(&self) -> *mut T {
        ffi::Joint_get_user_data(self.base_ptr()) as *mut T
    } 
}

#[allow(visible_private_types)]
pub trait MutJoint: ConstJoint+WrappedMutBase<ffi::Joint> {
    fn body_a<'a>(&'a mut self) -> BodyMutRef<'a> {
        unsafe {
            WrappedMut::from_ptr(ffi::Joint_get_body_a(self.mut_base_ptr()))
        }
    }
    
    fn body_b<'a>(&'a mut self) -> BodyMutRef<'a> {
        unsafe {
            WrappedMut::from_ptr(ffi::Joint_get_body_b(self.mut_base_ptr()))
        }
    }
    
    fn mut_next<'a>(&'a mut self) -> UnknownJointMutRef<'a> {
        unsafe {
            WrappedMutBase::from_ptr(ffi::Joint_get_next(self.mut_base_ptr()))
        }
    }
    
    unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        ffi::Joint_set_user_data(self.mut_base_ptr(), data as ffi::Any)
    }
    
    fn dump(&mut self) {
        unsafe {
            ffi::Joint_dump_virtual(self.mut_base_ptr())
        }
    }
    
    fn shift_origin(&mut self, origin: &Vec2) {
        unsafe {
            ffi::Joint_shift_origin_virtual(self.mut_base_ptr(),
                                            origin)
        }
    } 
}

pub enum UnknownJointMutRef<'l> {
    UnknownMutRef,
    RevoluteMutRef(RevoluteJointMutRef<'l>),
    PrismaticMutRef(PrismaticJointMutRef<'l>),
    DistanceMutRef(DistanceJointMutRef<'l>),
    PulleyMutRef(PulleyJointMutRef<'l>),
    MouseMutRef(MouseJointMutRef<'l>),
    GearMutRef(GearJointMutRef<'l>),
    WheelMutRef(WheelJointMutRef<'l>),
    WeldMutRef(WeldJointMutRef<'l>),
    FrictionMutRef(FrictionJointMutRef<'l>),
    RopeMutRef(RopeJointMutRef<'l>),
    MotorMutRef(MotorJointMutRef<'l>)
}

pub enum UnknownJointRef<'l> {
    UnknownRef,
    RevoluteRef(RevoluteJointRef<'l>),
    PrismaticRef(PrismaticJointRef<'l>),
    DistanceRef(DistanceJointRef<'l>),
    PulleyRef(PulleyJointRef<'l>),
    MouseRef(MouseJointRef<'l>),
    GearRef(GearJointRef<'l>),
    WheelRef(WheelJointRef<'l>),
    WeldRef(WeldJointRef<'l>),
    FrictionRef(FrictionJointRef<'l>),
    RopeRef(RopeJointRef<'l>),
    MotorRef(MotorJointRef<'l>)
}

impl<'l> WrappedJoint for UnknownJointMutRef<'l> {
    fn joint_type(_: Option<*const UnknownJointMutRef>) -> JointType {
        UnknownJointType
    }
}

impl<'l> WrappedJoint for UnknownJointRef<'l> {
    fn joint_type(_: Option<*const UnknownJointRef>) -> JointType {
        UnknownJointType
    }
}

impl<'l> WrappedBase<ffi::Joint> for UnknownJointMutRef<'l> {
    unsafe fn base_ptr(&self) -> *const ffi::Joint {
        match self {
            &DistanceMutRef(ref x) => x.base_ptr(),
            &FrictionMutRef(ref x) => x.base_ptr(),
            &GearMutRef(ref x) => x.base_ptr(),
            &MotorMutRef(ref x) => x.base_ptr(),
            &MouseMutRef(ref x) => x.base_ptr(),
            &PrismaticMutRef(ref x) => x.base_ptr(),
            &PulleyMutRef(ref x) => x.base_ptr(),
            &RevoluteMutRef(ref x) => x.base_ptr(),
            &RopeMutRef(ref x) => x.base_ptr(),
            &WeldMutRef(ref x) => x.base_ptr(),
            &WheelMutRef(ref x) => x.base_ptr(),
            _ => fail!("Truly unknown joint")
        }
    }
}

impl<'l> WrappedBase<ffi::Joint> for UnknownJointRef<'l> {
    unsafe fn base_ptr(&self) -> *const ffi::Joint {
        match self {
            &DistanceRef(ref x) => x.base_ptr(),
            &FrictionRef(ref x) => x.base_ptr(),
            &GearRef(ref x) => x.base_ptr(),
            &MotorRef(ref x) => x.base_ptr(),
            &MouseRef(ref x) => x.base_ptr(),
            &PrismaticRef(ref x) => x.base_ptr(),
            &PulleyRef(ref x) => x.base_ptr(),
            &RevoluteRef(ref x) => x.base_ptr(),
            &RopeRef(ref x) => x.base_ptr(),
            &WeldRef(ref x) => x.base_ptr(),
            &WheelRef(ref x) => x.base_ptr(),
            _ => fail!("Truly unknown joint")
        }
    }
}

impl<'l> WrappedMutBase<ffi::Joint> for UnknownJointMutRef<'l> {
    unsafe fn from_ptr(ptr: *mut ffi::Joint) -> UnknownJointMutRef<'l> {
        assert!(!ptr.is_null())
        let joint_type = ffi::Joint_get_type(ptr as *const ffi::Joint);
        match joint_type {
            RevoluteJointType => RevoluteMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            PrismaticJointType => PrismaticMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            DistanceJointType => DistanceMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            PulleyJointType => PulleyMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            MouseJointType => MouseMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            GearJointType => GearMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            WheelJointType => WheelMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            WeldJointType => WeldMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            FrictionJointType => FrictionMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            RopeJointType => RopeMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            MotorJointType => MotorMutRef(
                WrappedMutBase::from_ptr(ptr)
                ),
            _ => UnknownMutRef
        }
    }
    
    unsafe fn mut_base_ptr(&mut self) -> *mut ffi::Joint {
        match self {
            &DistanceMutRef(ref mut x) => x.mut_base_ptr(),
            &FrictionMutRef(ref mut x) => x.mut_base_ptr(),
            &GearMutRef(ref mut x) => x.mut_base_ptr(),
            &MotorMutRef(ref mut x) => x.mut_base_ptr(),
            &MouseMutRef(ref mut x) => x.mut_base_ptr(),
            &PrismaticMutRef(ref mut x) => x.mut_base_ptr(),
            &PulleyMutRef(ref mut x) => x.mut_base_ptr(),
            &RevoluteMutRef(ref mut x) => x.mut_base_ptr(),
            &RopeMutRef(ref mut x) => x.mut_base_ptr(),
            &WeldMutRef(ref mut x) => x.mut_base_ptr(),
            &WheelMutRef(ref mut x) => x.mut_base_ptr(),
            _ => fail!("Truly unknown joint")
        }
    }
}

impl<'l> WrappedConstBase<ffi::Joint> for UnknownJointRef<'l> {
    unsafe fn from_ptr(ptr: *const ffi::Joint) -> UnknownJointRef<'l> {
        assert!(!ptr.is_null())
        let joint_type = ffi::Joint_get_type(ptr);
        match joint_type {
            RevoluteJointType => RevoluteRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            PrismaticJointType => PrismaticRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            DistanceJointType => DistanceRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            PulleyJointType => PulleyRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            MouseJointType => MouseRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            GearJointType => GearRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            WheelJointType => WheelRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            WeldJointType => WeldRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            FrictionJointType => FrictionRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            RopeJointType => RopeRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            MotorJointType => MotorRef(
                WrappedConstBase::from_ptr(ptr)
                ),
            _ => UnknownRef
        }
    }
}

impl<'l> ConstJoint for UnknownJointMutRef<'l> {}
impl<'l> MutJoint for UnknownJointMutRef<'l> {}
impl<'l> ConstJoint for UnknownJointRef<'l> {}

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
    pub fn new(body_a: BodyMutRef,
               body_b: BodyMutRef,
               anchor_a: &Vec2,
               anchor_b: &Vec2) -> DistanceJointDef {
        unsafe {
            let mut joint =
                DistanceJointDef {
                    base: JointDefBase::new(DistanceJointType),
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
    pub fn new(body_a: BodyMutRef,
               body_b: BodyMutRef,
               anchor: &Vec2) -> FrictionJointDef {
        unsafe {
            let mut joint =
                FrictionJointDef {
                    base: JointDefBase::new(FrictionJointType),
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
    pub fn new(joint_a: &mut MutJoint,
               joint_b: &mut MutJoint) -> GearJointDef {
        unsafe {
            GearJointDef {
                base: JointDefBase::new(GearJointType),
                joint_a: joint_a.mut_base_ptr(),
                joint_b: joint_b.mut_base_ptr(),
                ratio: 1.
            }
        }
    }
}

impl MotorJointDef {
    pub fn new(body_a: BodyMutRef,
               body_b: BodyMutRef) -> MotorJointDef {
        unsafe {
            let mut joint = 
                MotorJointDef {
                    base: JointDefBase::new(MotorJointType),
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
            base: JointDefBase::new(MouseJointType),
            target: Vec2 { x:0., y:0. },
            max_force: 0.,
            frequency: 5.,
            damping_ratio: 0.7
        }
    }
}

impl PrismaticJointDef {
    pub fn new(body_a: BodyMutRef,
               body_b: BodyMutRef,
               anchor: &Vec2,
               axis: &Vec2) -> PrismaticJointDef {
        unsafe {
            let mut joint =
                PrismaticJointDef {
                    base: JointDefBase::new(PrismaticJointType),
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
    pub fn new(body_a: BodyMutRef,
               body_b: BodyMutRef,
               ground_anchor_a: &Vec2,
               ground_anchor_b: &Vec2,
               anchor_a: &Vec2,
               anchor_b: &Vec2,
               ratio: f32) -> PulleyJointDef {
        unsafe {
            let mut joint =
                PulleyJointDef {
                    base: JointDefBase::new(PulleyJointType),
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
    pub fn new(body_a: BodyMutRef,
               body_b: BodyMutRef,
               anchor: &Vec2) -> RevoluteJointDef {
        unsafe {
            let mut joint =
                RevoluteJointDef {
                    base: JointDefBase::new(RevoluteJointType),
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
            base: JointDefBase::new(RopeJointType),
            local_anchor_a: Vec2 { x:-1., y:0. },
            local_anchor_b: Vec2 { x:1., y:0. },
            max_length: 0.
        }
    }
}

impl WeldJointDef {
    pub fn new(body_a: BodyMutRef,
               body_b: BodyMutRef,
               anchor: &Vec2) -> WeldJointDef {
        unsafe {
            let mut joint =
                WeldJointDef {
                    base: JointDefBase::new(WeldJointType),
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
    pub fn new(body_a: BodyMutRef,
               body_b: BodyMutRef,
               anchor: &Vec2,
               axis: &Vec2) -> WheelJointDef {
        unsafe {
            let mut joint =
                WheelJointDef {
                    base: JointDefBase::new(WheelJointType),
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

wrapped_joint!(ffi::DistanceJoint into DistanceJointMutRef, DistanceJointRef
               (DistanceJointType)
               << ffi::Joint_as_distance_joint
               >> ffi::DistanceJoint_as_joint
               )
wrapped_joint!(ffi::FrictionJoint into FrictionJointMutRef, FrictionJointRef
               (FrictionJointType)
               << ffi::Joint_as_friction_joint
               >> ffi::FrictionJoint_as_joint
               )
wrapped_joint!(ffi::GearJoint into GearJointMutRef, GearJointRef
               (GearJointType)
               << ffi::Joint_as_gear_joint
               >> ffi::GearJoint_as_joint
               )
wrapped_joint!(ffi::MotorJoint into MotorJointMutRef, MotorJointRef
               (MotorJointType)
               << ffi::Joint_as_motor_joint
               >> ffi::MotorJoint_as_joint
               )
wrapped_joint!(ffi::MouseJoint into MouseJointMutRef, MouseJointRef
               (MouseJointType)
               << ffi::Joint_as_mouse_joint
               >> ffi::MouseJoint_as_joint
               )
wrapped_joint!(ffi::PrismaticJoint into PrismaticJointMutRef, PrismaticJointRef
               (PrismaticJointType)
               << ffi::Joint_as_prismatic_joint
               >> ffi::PrismaticJoint_as_joint
               )
wrapped_joint!(ffi::PulleyJoint into PulleyJointMutRef, PulleyJointRef
               (PulleyJointType)
               << ffi::Joint_as_pulley_joint
               >> ffi::PulleyJoint_as_joint
               )
wrapped_joint!(ffi::RevoluteJoint into RevoluteJointMutRef, RevoluteJointRef
               (RevoluteJointType)
               << ffi::Joint_as_revolute_joint
               >> ffi::RevoluteJoint_as_joint
               )
wrapped_joint!(ffi::RopeJoint into RopeJointMutRef, RopeJointRef
               (RopeJointType)
               << ffi::Joint_as_rope_joint
               >> ffi::RopeJoint_as_joint
               )
wrapped_joint!(ffi::WeldJoint into WeldJointMutRef, WeldJointRef
               (WeldJointType)
               << ffi::Joint_as_weld_joint
               >> ffi::WeldJoint_as_joint
               )
wrapped_joint!(ffi::WheelJoint into WheelJointMutRef, WheelJointRef
               (WheelJointType)
               << ffi::Joint_as_wheel_joint
               >> ffi::WheelJoint_as_joint
               )

#[allow(visible_private_types)]
pub trait ConstDistanceJoint: Wrapped<ffi::DistanceJoint> {
    fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::DistanceJoint_get_local_anchor_a(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::DistanceJoint_get_local_anchor_b(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn length(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_length(self.ptr())
        }
    }
    
    fn frequency(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_frequency(self.ptr())
        }
    }
    
    fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_damping_ratio(self.ptr())
        }
    }
}

#[allow(visible_private_types)]
pub trait MutDistanceJoint: ConstDistanceJoint+WrappedMut<ffi::DistanceJoint> {
    fn set_length(&mut self, length: f32) {
        unsafe {
            ffi::DistanceJoint_set_length(self.mut_ptr(), length)
        }
    }
    
    fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::DistanceJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }
    
    fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::DistanceJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

impl<'l> ConstDistanceJoint for DistanceJointMutRef<'l> {}
impl<'l> MutDistanceJoint for DistanceJointMutRef<'l> {}
impl<'l> ConstDistanceJoint for DistanceJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstFrictionJoint: Wrapped<ffi::FrictionJoint> {
    fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::FrictionJoint_get_local_anchor_a(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::FrictionJoint_get_local_anchor_b(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn max_force(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_force(self.ptr())
        }
    }
    
    fn max_torque(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_torque(self.ptr())
        }
    }
}

#[allow(visible_private_types)]
pub trait MutFrictionJoint: ConstFrictionJoint+WrappedMut<ffi::FrictionJoint> {    
    fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_force(self.mut_ptr(), force)
        }
    }
    
    fn set_max_torque(&mut self, torque: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_torque(self.mut_ptr(), torque)
        }
    }
}

impl<'l> ConstFrictionJoint for FrictionJointMutRef<'l> {}
impl<'l> MutFrictionJoint for FrictionJointMutRef<'l> {}
impl<'l> ConstFrictionJoint for FrictionJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstGearJoint: Wrapped<ffi::GearJoint> {
    fn ratio(&self) -> f32 {
        unsafe {
            ffi::GearJoint_get_ratio(self.ptr())
        }
    }
}

#[allow(visible_private_types)]
pub trait MutGearJoint: ConstGearJoint+WrappedMut<ffi::GearJoint> {
    fn joint_a<'a>(&'a mut self) -> UnknownJointMutRef<'a> {
        unsafe {
            WrappedMutBase::from_ptr(ffi::GearJoint_get_joint_1(self.mut_ptr()))
        }
    }
    
    fn joint_b<'a>(&'a mut self) -> UnknownJointMutRef<'a> {
        unsafe {
            WrappedMutBase::from_ptr(ffi::GearJoint_get_joint_2(self.mut_ptr()))
        }
    }
    
    fn set_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::GearJoint_set_ratio(self.mut_ptr(), ratio)
        }
    }
}

impl<'l> ConstGearJoint for GearJointMutRef<'l> {}
impl<'l> MutGearJoint for GearJointMutRef<'l> {}
impl<'l> ConstGearJoint for GearJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstMotorJoint: Wrapped<ffi::MotorJoint> {
    fn linear_offset<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let offset = ffi::MotorJoint_get_linear_offset(self.ptr());
            assert!(!offset.is_null())
            &*offset
        }
    }
    
    fn angular_offset(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_angular_offset(self.ptr())
        }
    }
    
    fn max_force(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_max_force(self.ptr())
        }
    }
    
    fn max_torque(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_max_torque(self.ptr())
        }
    }
    
    fn correction_factor(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_correction_factor(self.ptr())
        }
    }
}

#[allow(visible_private_types)]
pub trait MutMotorJoint: ConstMotorJoint+WrappedMut<ffi::MotorJoint> {
    fn set_linear_offset(&mut self, offset: &Vec2) {
        unsafe {
            ffi::MotorJoint_set_linear_offset(self.mut_ptr(), offset)
        }
    }
    
    fn set_angular_offset(&mut self, offset: f32) {
        unsafe {
            ffi::MotorJoint_set_angular_offset(self.mut_ptr(), offset)
        }
    }
    
    fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::MotorJoint_set_max_force(self.mut_ptr(), force)
        }
    }
    
    fn set_max_torque(&mut self, torque: f32) {
        unsafe {
            ffi::MotorJoint_set_max_torque(self.mut_ptr(), torque)
        }
    }
    
    fn set_correction_factor(&mut self, factor: f32) {
        unsafe {
            ffi::MotorJoint_set_correction_factor(self.mut_ptr(), factor)
        }
    }
}

impl<'l> ConstMotorJoint for MotorJointMutRef<'l> {}
impl<'l> MutMotorJoint for MotorJointMutRef<'l> {}
impl<'l> ConstMotorJoint for MotorJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstMouseJoint: Wrapped<ffi::MouseJoint> {
    fn target<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let target = ffi::MouseJoint_get_target(self.ptr());
            assert!(!target.is_null())
            &*target
        }
    }
    
    fn max_force(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_max_force(self.ptr())
        }
    }
    
    fn frequency(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_frequency(self.ptr())
        }
    }
    
    fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_damping_ratio(self.ptr())
        }
    }
}

#[allow(visible_private_types)]
pub trait MutMouseJoint: ConstMouseJoint+WrappedMut<ffi::MouseJoint> {
    fn set_target(&mut self, target: &Vec2) {
        unsafe {
            ffi::MouseJoint_set_target(self.mut_ptr(), target)
        }
    }
    
    fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::MouseJoint_set_max_force(self.mut_ptr(), force)
        }
    }
    
    fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::MouseJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }
    
    fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::MouseJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

impl<'l> ConstMouseJoint for MouseJointMutRef<'l> {}
impl<'l> MutMouseJoint for MouseJointMutRef<'l> {}
impl<'l> ConstMouseJoint for MouseJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstPrismaticJoint: Wrapped<ffi::PrismaticJoint> {
    fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::PrismaticJoint_get_local_anchor_a(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::PrismaticJoint_get_local_anchor_b(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn local_axis_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let axis = ffi::PrismaticJoint_get_local_axis_a(self.ptr());
            assert!(!axis.is_null())
            &*axis
        }
    }
    
    fn reference_angle(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_reference_angle(self.ptr())
        }
    }
    
    fn joint_translation(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_joint_translation(self.ptr())
        }
    }
    
    fn joint_speed(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_joint_speed(self.ptr())
        }
    }
    
    fn is_limit_enabled(&self) -> bool {
        unsafe {
            ffi::PrismaticJoint_is_limit_enabled(self.ptr())
        }
    }
    
    fn limits(&self) -> (f32, f32) {
        unsafe {
            (ffi::PrismaticJoint_get_lower_limit(self.ptr()),
             ffi::PrismaticJoint_get_upper_limit(self.ptr()))
        }
    }
    
    fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::PrismaticJoint_is_motor_enabled(self.ptr())
        }
    }
    
    fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_motor_speed(self.ptr())
        }
    }
    
    fn max_motor_force(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_max_motor_force(self.ptr())
        }
    }
    
    fn motor_force(&self, inv_dt: f32) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_motor_force(self.ptr(), inv_dt)
        }
    }
}

#[allow(visible_private_types)]
pub trait MutPrismaticJoint: ConstPrismaticJoint+WrappedMut<ffi::PrismaticJoint> {
    fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::PrismaticJoint_enable_limit(self.mut_ptr(), flag)
        }
    }
    
    fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::PrismaticJoint_set_limits(self.mut_ptr(), lower, upper)
        }
    }
    
    fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::PrismaticJoint_enable_motor(self.mut_ptr(), flag)
        }
    }
    
    fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::PrismaticJoint_set_motor_speed(self.mut_ptr(), speed)
        }
    }
    
    fn set_max_motor_force(&mut self, force: f32) {
        unsafe {
            ffi::PrismaticJoint_set_max_motor_force(self.mut_ptr(), force)
        }
    }
}

impl<'l> ConstPrismaticJoint for PrismaticJointMutRef<'l> {}
impl<'l> MutPrismaticJoint for PrismaticJointMutRef<'l> {}
impl<'l> ConstPrismaticJoint for PrismaticJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstPulleyJoint: Wrapped<ffi::PulleyJoint> {
    fn ground_anchor_a(&self) -> Vec2 {
        unsafe {
            ffi::PulleyJoint_get_ground_anchor_a(self.ptr())
        }
    }
    
    fn ground_anchor_b(&self) -> Vec2 {
        unsafe {
            ffi::PulleyJoint_get_ground_anchor_b(self.ptr())
        }
    }
    
    fn length_a(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_length_a(self.ptr())
        }
    }
    
    fn length_b(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_length_b(self.ptr())
        }
    }
    
    fn ratio(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_ratio(self.ptr())
        }
    }
    
    fn current_length_a(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_current_length_a(self.ptr())
        }
    }
    
    fn current_length_b(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_current_length_b(self.ptr())
        }
    }
}

#[allow(visible_private_types)]
pub trait MutPulleyJoint: ConstPulleyJoint+WrappedMut<ffi::PulleyJoint> {

}

impl<'l> ConstPulleyJoint for PulleyJointMutRef<'l> {}
impl<'l> MutPulleyJoint for PulleyJointMutRef<'l> {}
impl<'l> ConstPulleyJoint for PulleyJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstRevoluteJoint: Wrapped<ffi::RevoluteJoint> {
    fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::RevoluteJoint_get_local_anchor_a(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::RevoluteJoint_get_local_anchor_b(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn referance_angle(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_reference_angle(self.ptr())
        }
    }
    
    fn joint_angle(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_joint_angle(self.ptr())
        }
    }
    
    fn joint_speed(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_joint_speed(self.ptr())
        }
    }
    
    fn is_limit_enabled(&self) -> bool {
        unsafe {
            ffi::RevoluteJoint_is_limit_enabled(self.ptr())
        }
    }
    
    fn limits(&self) -> (f32, f32) {
        unsafe {
            (ffi::RevoluteJoint_get_lower_limit(self.ptr()),
             ffi::RevoluteJoint_get_upper_limit(self.ptr()))
        }
    }
    
    fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::RevoluteJoint_is_motor_enabled(self.ptr())
        }
    }
    
    fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_motor_speed(self.ptr())
        }
    }
    
    fn max_motor_torque(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_max_motor_torque(self.ptr())
        }
    }
    
    fn motor_torque(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_motor_torque(self.ptr())
        }
    }
}

#[allow(visible_private_types)]
pub trait MutRevoluteJoint: ConstRevoluteJoint+WrappedMut<ffi::RevoluteJoint> {
    fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::RevoluteJoint_enable_limit(self.mut_ptr(), flag)
        }
    }
    
    fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::RevoluteJoint_set_limits(self.mut_ptr(), lower, upper)
        }
    }
    
    fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::RevoluteJoint_enable_motor(self.mut_ptr(), flag)
        }
    }
    
    fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::RevoluteJoint_set_motor_speed(self.mut_ptr(), speed)
        }
    }
    
    fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::RevoluteJoint_set_max_motor_torque(self.mut_ptr(), torque)
        }
    }
}

impl<'l> ConstRevoluteJoint for RevoluteJointMutRef<'l> {}
impl<'l> MutRevoluteJoint for RevoluteJointMutRef<'l> {}
impl<'l> ConstRevoluteJoint for RevoluteJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstRopeJoint: Wrapped<ffi::RopeJoint> {
    fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::RopeJoint_get_local_anchor_a(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let  anchor = ffi::RopeJoint_get_local_anchor_b(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn max_length(&self) -> f32 {
        unsafe {
            ffi::RopeJoint_get_max_length(self.ptr())
        }
    }
    
    fn limit_state(&self) -> LimitState {
        unsafe {
            ffi::RopeJoint_get_limit_state(self.ptr())
        }
    }
}

#[allow(visible_private_types)]
pub trait MutRopeJoint: ConstRopeJoint+WrappedMut<ffi::RopeJoint> {
    fn set_max_length(&mut self, length: f32) {
        unsafe {
            ffi::RopeJoint_set_max_length(self.mut_ptr(), length)
        }
    }
}

impl<'l> ConstRopeJoint for RopeJointMutRef<'l> {}
impl<'l> MutRopeJoint for RopeJointMutRef<'l> {}
impl<'l> ConstRopeJoint for RopeJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstWeldJoint: Wrapped<ffi::WeldJoint> {
    fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::WeldJoint_get_local_anchor_a(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::WeldJoint_get_local_anchor_b(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn referance_angle(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_reference_angle(self.ptr())
        }
    }
    
    fn frequency(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_frequency(self.ptr())
        }
    }
    
    fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_damping_ratio(self.ptr())
        }
    }
}

#[allow(visible_private_types)]
pub trait MutWeldJoint: ConstWeldJoint+WrappedMut<ffi::WeldJoint> {
    fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WeldJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }
    
    fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WeldJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

impl<'l> ConstWeldJoint for WeldJointMutRef<'l> {}
impl<'l> MutWeldJoint for WeldJointMutRef<'l> {}
impl<'l> ConstWeldJoint for WeldJointRef<'l> {}

#[allow(visible_private_types)]
pub trait ConstWheelJoint: Wrapped<ffi::WheelJoint> {
    fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::WheelJoint_get_local_anchor_a(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let anchor = ffi::WheelJoint_get_local_anchor_b(self.ptr());
            assert!(!anchor.is_null())
            &*anchor
        }
    }
    
    fn local_axis_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let axis = ffi::WheelJoint_get_local_axis_a(self.ptr());
            assert!(!axis.is_null())
            &*axis
        }
    }
    
    fn joint_translation(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_joint_translation(self.ptr())
        }
    }
    
    fn joint_speed(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_joint_speed(self.ptr())
        }
    }
    
    fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::WheelJoint_is_motor_enabled(self.ptr())
        }
    }
    
    fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_motor_speed(self.ptr())
        }
    }
    
    fn max_motor_torque(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_max_motor_torque(self.ptr())
        }
    }
    
    fn motor_torque(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_motor_torque(self.ptr())
        }
    }
    
    fn spring_frequency(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_frequency(self.ptr())
        }
    }
    
    fn spring_damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_damping_ratio(self.ptr())
        }
    }   
}

#[allow(visible_private_types)]
pub trait MutWheelJoint: ConstWheelJoint+WrappedMut<ffi::WheelJoint> {  
    fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::WheelJoint_enable_motor(self.mut_ptr(), flag)
        }
    }
    
    fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::WheelJoint_set_motor_speed(self.mut_ptr(), speed)
        }
    }
    
    fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::WheelJoint_set_max_motor_torque(self.mut_ptr(), torque)
        }
    }
    
    fn set_spring_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_frequency(self.mut_ptr(), frequency)
        }
    }
    
    fn set_spring_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

impl<'l> ConstWheelJoint for WheelJointMutRef<'l> {}
impl<'l> MutWheelJoint for WheelJointMutRef<'l> {}
impl<'l> ConstWheelJoint for WheelJointRef<'l> {}
