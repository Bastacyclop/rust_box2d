use ffi;
use math::Vec2;
use Wrapped;
use dynamics::Body;

macro_rules! impl_joint(
    (for $wrap:ty << $joint_as:path >> $as_joint:path) => (
        impl WrappedJoint for $wrap {
            unsafe fn from_joint_ptr(ptr: *mut ffi::Joint) -> $wrap {
                Wrapped::from_ptr($joint_as(ptr))
            }
            unsafe fn get_joint_ptr(&self) -> *const ffi::Joint {
                $as_joint(self.ptr) as *const ffi::Joint
            }
            unsafe fn get_mut_joint_ptr(&mut self) -> *mut ffi::Joint {
                $as_joint(self.ptr)
            }
        }
        
        impl Joint for $wrap {}
    );
)

macro_rules! joint_def(
    ($name:ident $(($(($visibility:ident))*) $field:ident: $typ:ty),+) => (
        #[packed]
        #[allow(dead_code)]
        pub struct $name<'l> {
            pub base: JointDefBase<'l>,
            $(
                $($visibility)* $field: $typ,
            )+
        }
        
        impl<'l> JointDef for $name<'l> {
            unsafe fn from_joint_def_ptr(ptr: *mut JointDefBase) -> $name {
                *(ptr as *mut $name)
            }
            unsafe fn get_joint_def_ptr(&self) -> *const JointDefBase {
                self as *const $name as *const JointDefBase
            }
            unsafe fn get_mut_joint_def_ptr(&mut self) -> *mut JointDefBase {
                self as *mut $name as *mut JointDefBase
            }
        }
    );
)

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

pub trait JointDef {
    unsafe fn from_joint_def_ptr(ptr: *mut JointDefBase) -> Self;
    unsafe fn get_joint_def_ptr(&self) -> *const JointDefBase;
    unsafe fn get_mut_joint_def_ptr(&mut self) -> *mut JointDefBase;
}

pub trait WrappedJoint {
    unsafe fn from_joint_ptr(ptr: *mut ffi::Joint) -> Self;
    unsafe fn get_joint_ptr(&self) -> *const ffi::Joint;
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
            ffi::Joint_get_reaction_force_virtual(self.get_joint_ptr()).clone()
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
        let joint_type = ffi::Joint_get_type(ptr as *const ffi::Joint);
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
    unsafe fn get_joint_ptr(&self) -> *const ffi::Joint {
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

impl<'l> DistanceJointDef<'l> {    
    pub fn new<'m>(body_a: &'m mut Body,
                   body_b: &'m mut Body,
                   anchor_a: &Vec2,
                   anchor_b: &Vec2) -> DistanceJointDef<'m> {
        unsafe {
            let mut joint = ffi::DistanceJointDef_default();
            ffi::DistanceJointDef_initialize(&mut joint,
                                          body_a.get_mut_ptr(),
                                          body_b.get_mut_ptr(),
                                          anchor_a, anchor_b);
            joint
        }
    }
}
impl<'l> FrictionJointDef<'l> {    
    pub fn new<'m>(body_a: &'m mut Body,
                   body_b: &'m mut Body,
                   anchor: &Vec2) -> FrictionJointDef<'m> {
        unsafe {
            let mut joint = ffi::FrictionJointDef_default();
            ffi::FrictionJointDef_initialize(&mut joint,
                                          body_a.get_mut_ptr(),
                                          body_b.get_mut_ptr(),
                                          anchor);
            joint
        }
    }
}
impl<'l> GearJointDef<'l> {
    pub fn new<'m>(joint_a: &'m mut Joint,
                   joint_b: &'m mut Joint) -> GearJointDef<'m> {
        unsafe {
            let mut joint = ffi::GearJointDef_default();
            joint.joint_a = joint_a.get_mut_joint_ptr();
            joint.joint_b = joint_b.get_mut_joint_ptr();
            joint
        }
    }
}
impl<'l> MotorJointDef<'l> {
    pub fn new<'m>(body_a: &'m mut Body,
                   body_b: &'m mut Body) -> MotorJointDef<'m> {
        unsafe {
            let mut joint = ffi::MotorJointDef_default();
            ffi::MotorJointDef_initialize(&mut joint,
                                          body_a.get_mut_ptr(),
                                          body_b.get_mut_ptr());
            joint
        }
    }
}
impl<'l> MouseJointDef<'l> {
    pub fn new<'m>() -> MouseJointDef<'m> {
        unsafe {
            ffi::MouseJointDef_default()
        }
    }
}
impl<'l> PrismaticJointDef<'l> {
    pub fn new<'m>(body_a: &'m mut Body,
                   body_b: &'m mut Body,
                   anchor: &Vec2,
                   axis: &Vec2) -> PrismaticJointDef<'m> {
        unsafe {
            let mut joint = ffi::PrismaticJointDef_default();
            ffi::PrismaticJointDef_initialize(&mut joint,
                                              body_a.get_mut_ptr(),
                                              body_b.get_mut_ptr(),
                                              anchor, axis);
            joint
        }
    }
}
impl<'l> PulleyJointDef<'l> {
    pub fn new<'m>(body_a: &'m mut Body,
                   body_b: &'m mut Body,
                   ground_anchor_a: &Vec2,
                   ground_anchor_b: &Vec2,
                   anchor_a: &Vec2,
                   anchor_b: &Vec2,
                   ratio: f32) -> PulleyJointDef<'m> {
        unsafe {
            let mut joint = ffi::PulleyJointDef_default();
            ffi::PulleyJointDef_initialize(&mut joint,
                                           body_a.get_mut_ptr(),
                                           body_b.get_mut_ptr(),
                                           ground_anchor_a,
                                           ground_anchor_b,
                                           anchor_a, anchor_b,
                                           ratio);
            joint
        }
    }
}
impl<'l> RevoluteJointDef<'l> {
    pub fn new<'m>(body_a: &'m mut Body,
                   body_b: &'m mut Body,
                   anchor: &Vec2) -> RevoluteJointDef<'m> {
        unsafe {
            let mut joint = ffi::RevoluteJointDef_default();
            ffi::RevoluteJointDef_initialize(&mut joint,
                                             body_a.get_mut_ptr(),
                                             body_b.get_mut_ptr(),
                                             anchor);
            joint
        }
    }
}
impl<'l> RopeJointDef<'l> {
    pub fn new<'m>() -> RopeJointDef<'m> {
        unsafe {
            ffi::RopeJointDef_default()
        }
    }
}
impl<'l> WeldJointDef<'l> {
    pub fn new<'m>(body_a: &'m mut Body,
                   body_b: &'m mut Body,
                   anchor: &Vec2) -> WeldJointDef<'m> {
        unsafe {
            let mut joint = ffi::WeldJointDef_default();
            ffi::WeldJointDef_initialize(&mut joint,
                                             body_a.get_mut_ptr(),
                                             body_b.get_mut_ptr(),
                                             anchor);
            joint
        }
    }
}
impl<'l> WheelJointDef<'l> {
    pub fn new<'m>(body_a: &'m mut Body,
                   body_b: &'m mut Body,
                   anchor: &Vec2,
                   axis: &Vec2) -> WheelJointDef<'m> {
        unsafe {
            let mut joint = ffi::WheelJointDef_default();
            ffi::WheelJointDef_initialize(&mut joint,
                                             body_a.get_mut_ptr(),
                                             body_b.get_mut_ptr(),
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
    pub fn get_local_anchor_a(&self) -> Vec2 {
        unsafe {
            *ffi::DistanceJoint_get_local_anchor_a(self.get_ptr()).clone()
        }
    }
    pub fn get_local_anchor_b(&self) -> Vec2 {
        unsafe {
            *ffi::DistanceJoint_get_local_anchor_b(self.get_ptr()).clone()
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
impl FrictionJoint {
    pub fn get_local_anchor_a(&self) -> Vec2 {
        unsafe {
            *ffi::FrictionJoint_get_local_anchor_a(self.get_ptr()).clone()
        }
    }
    pub fn get_local_anchor_b(&self) -> Vec2 {
        unsafe {
            *ffi::FrictionJoint_get_local_anchor_b(self.get_ptr()).clone()
        }
    }
    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_force(self.get_mut_ptr(), force)
        }
    }
    pub fn get_max_force(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_force(self.get_ptr())
        }
    }
    pub fn set_max_torque(&mut self, torque: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_torque(self.get_mut_ptr(), torque)
        }
    }
    pub fn get_max_torque(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_torque(self.get_ptr())
        }
    }
}
impl GearJoint {/*
    pub fn get_joint_a(&self) -> UnknownJoint {
        unsafe {
            WrappedJoint::from_joint_ptr(
                ffi::GearJoint_get_joint_1(self.get_ptr())
                )
        }
    }
    pub fn get_joint_b(&self) -> UnknownJoint {
        unsafe {
            WrappedJoint::from_joint_ptr(
                ffi::GearJoint_get_joint_2(self.get_ptr())
                )
        }
    }*/
    pub fn set_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::GearJoint_set_ratio(self.get_mut_ptr(), ratio)
        }
    }
    pub fn get_ratio(&self) -> f32 {
        unsafe {
            ffi::GearJoint_get_ratio(self.get_ptr())
        }
    }
}
impl MotorJoint {
    pub fn set_linear_offset(&mut self, offset: &Vec2) {
        unsafe {
            ffi::MotorJoint_set_linear_offset(self.get_mut_ptr(), offset)
        }
    }
    pub fn get_linear_offset(&self) -> Vec2 {
        unsafe {
            *ffi::MotorJoint_get_linear_offset(self.get_ptr()).clone()
        }
    }
    pub fn set_angular_offset(&mut self, offset: f32) {
        unsafe {
            ffi::MotorJoint_set_angular_offset(self.get_mut_ptr(), offset)
        }
    }
    pub fn get_angular_offset(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_angular_offset(self.get_ptr())
        }
    }
    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::MotorJoint_set_max_force(self.get_mut_ptr(), force)
        }
    }
    pub fn get_max_force(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_max_force(self.get_ptr())
        }
    }
    pub fn set_max_torque(&mut self, torque: f32) {
        unsafe {
            ffi::MotorJoint_set_max_torque(self.get_mut_ptr(), torque)
        }
    }
    pub fn get_max_torque(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_max_torque(self.get_ptr())
        }
    }
    pub fn set_correction_factor(&mut self, factor: f32) {
        unsafe {
            ffi::MotorJoint_set_correction_factor(self.get_mut_ptr(), factor)
        }
    }
    pub fn get_correction_factor(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_correction_factor(self.get_ptr())
        }
    }
}
impl MouseJoint {
    pub fn set_target(&mut self, target: &Vec2) {
        unsafe {
            ffi::MouseJoint_set_target(self.get_mut_ptr(), target)
        }
    }
    pub fn get_target(&self) -> Vec2 {
        unsafe {
            *ffi::MouseJoint_get_target(self.get_ptr()).clone()
        }
    }
    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::MouseJoint_set_max_force(self.get_mut_ptr(), force)
        }
    }
    pub fn get_max_force(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_max_force(self.get_ptr())
        }
    }
    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::MouseJoint_set_frequency(self.get_mut_ptr(), frequency)
        }
    }
    pub fn get_frequency(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_frequency(self.get_ptr())
        }
    }
    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::MouseJoint_set_damping_ratio(self.get_mut_ptr(), ratio)
        }
    }
    pub fn get_damping_ratio(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_damping_ratio(self.get_ptr())
        }
    }
}
impl PrismaticJoint {
    pub fn get_local_anchor_a(&self) -> Vec2 {
        unsafe {
            *ffi::PrismaticJoint_get_local_anchor_a(self.get_ptr()).clone()
        }
    }
    pub fn get_local_anchor_b(&self) -> Vec2 {
        unsafe {
            *ffi::PrismaticJoint_get_local_anchor_b(self.get_ptr()).clone()
        }
    }
    pub fn get_reference_angle(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_reference_angle(self.get_ptr())
        }
    }
    pub fn get_joint_translation(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_joint_translation(self.get_ptr())
        }
    }
    pub fn get_joint_speed(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_joint_speed(self.get_ptr())
        }
    }
    pub fn is_limit_enabled(&self) -> bool {
        unsafe {
            ffi::PrismaticJoint_is_limit_enabled(self.get_ptr())
        }
    }
    pub fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::PrismaticJoint_enable_limit(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_limits(&self) -> (f32, f32) {
        unsafe {
            (ffi::PrismaticJoint_get_lower_limit(self.get_ptr()),
             ffi::PrismaticJoint_get_upper_limit(self.get_ptr()))
        }
    }
    pub fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::PrismaticJoint_set_limits(self.get_mut_ptr(), lower, upper)
        }
    }
    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::PrismaticJoint_is_motor_enabled(self.get_ptr())
        }
    }
    pub fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::PrismaticJoint_enable_motor(self.get_mut_ptr(), flag)
        }
    }
    pub fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::PrismaticJoint_set_motor_speed(self.get_mut_ptr(), speed)
        }
    }
    pub fn get_motor_speed(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_motor_speed(self.get_ptr())
        }
    }
    pub fn set_max_motor_force(&mut self, force: f32) {
        unsafe {
            ffi::PrismaticJoint_set_max_motor_force(self.get_mut_ptr(), force)
        }
    }
    pub fn get_max_motor_force(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_max_motor_force(self.get_ptr())
        }
    }
    pub fn get_motor_force(&self, inv_dt: f32) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_motor_force(self.get_ptr(), inv_dt)
        }
    }
}
impl PulleyJoint {
    pub fn get_ground_anchor_a(&self) -> Vec2 {
        unsafe {
            ffi::PulleyJoint_get_ground_anchor_a(self.get_ptr())
        }
    }
    pub fn get_ground_anchor_b(&self) -> Vec2 {
        unsafe {
            ffi::PulleyJoint_get_ground_anchor_b(self.get_ptr())
        }
    }
    pub fn get_length_a(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_length_a(self.get_ptr())
        }
    }
    pub fn get_length_b(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_length_b(self.get_ptr())
        }
    }
    pub fn get_ratio(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_ratio(self.get_ptr())
        }
    }
    pub fn get_current_length_a(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_current_length_a(self.get_ptr())
        }
    }
    pub fn get_current_length_b(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_current_length_b(self.get_ptr())
        }
    }
}
impl RevoluteJoint {
    pub fn get_local_anchor_a(&self) -> Vec2 {
        unsafe {
            *ffi::RevoluteJoint_get_local_anchor_a(self.get_ptr()).clone()
        }
    }
    pub fn get_local_anchor_b(&self) -> Vec2 {
        unsafe {
            *ffi::RevoluteJoint_get_local_anchor_b(self.get_ptr()).clone()
        }
    }
    pub fn get_referance_angle(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_reference_angle(self.get_ptr())
        }
    }
    pub fn get_joint_angle(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_joint_angle(self.get_ptr())
        }
    }
    pub fn get_joint_speed(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_joint_speed(self.get_ptr())
        }
    }
    pub fn is_limit_enabled(&self) -> bool {
        unsafe {
            ffi::RevoluteJoint_is_limit_enabled(self.get_ptr())
        }
    }
    pub fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::RevoluteJoint_enable_limit(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_limits(&self) -> (f32, f32) {
        unsafe {
            (ffi::RevoluteJoint_get_lower_limit(self.get_ptr()),
             ffi::RevoluteJoint_get_lower_limit(self.get_ptr()))
        }
    }
    pub fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::RevoluteJoint_set_limits(self.get_mut_ptr(), lower, upper)
        }
    }
    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::RevoluteJoint_is_motor_enabled(self.get_ptr())
        }
    }
    pub fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::RevoluteJoint_enable_motor(self.get_mut_ptr(), flag)
        }
    }
    pub fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::RevoluteJoint_set_motor_speed(self.get_mut_ptr(), speed)
        }
    }
    pub fn get_motor_speed(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_motor_speed(self.get_ptr())
        }
    }
    pub fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::RevoluteJoint_set_max_motor_torque(self.get_mut_ptr(), torque)
        }
    }
    pub fn get_max_motor_torque(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_max_motor_torque(self.get_ptr())
        }
    }
    pub fn get_motor_torque(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_motor_torque(self.get_ptr())
        }
    }
}
impl RopeJoint {
    pub fn get_local_anchor_a(&self) -> Vec2 {
        unsafe {
            *ffi::RopeJoint_get_local_anchor_a(self.get_ptr()).clone()
        }
    }
    pub fn get_local_anchor_b(&self) -> Vec2 {
        unsafe {
            *ffi::RopeJoint_get_local_anchor_b(self.get_ptr()).clone()
        }
    }
    pub fn set_max_length(&mut self, length: f32) {
        unsafe {
            ffi::RopeJoint_set_max_length(self.get_mut_ptr(), length)
        }
    }
    pub fn get_max_length(&self) -> f32 {
        unsafe {
            ffi::RopeJoint_get_max_length(self.get_ptr())
        }
    }
    pub fn get_limit_state(&self) -> LimitState {
        unsafe {
            ffi::RopeJoint_get_limit_state(self.get_ptr())
        }
    }
}
impl WeldJoint {
    pub fn get_local_anchor_a(&self) -> Vec2 {
        unsafe {
            *ffi::WeldJoint_get_local_anchor_a(self.get_ptr()).clone()
        }
    }
    pub fn get_local_anchor_b(&self) -> Vec2 {
        unsafe {
            *ffi::WeldJoint_get_local_anchor_b(self.get_ptr()).clone()
        }
    }
    pub fn get_referance_angle(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_reference_angle(self.get_ptr())
        }
    }
    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WeldJoint_set_frequency(self.get_mut_ptr(), frequency)
        }
    }
    pub fn get_frequency(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_frequency(self.get_ptr())
        }
    }
    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WeldJoint_set_damping_ratio(self.get_mut_ptr(), ratio)
        }
    }
    pub fn get_damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_damping_ratio(self.get_ptr())
        }
    }
}
impl WheelJoint {
    pub fn get_local_anchor_a(&self) -> Vec2 {
        unsafe {
            *ffi::WheelJoint_get_local_anchor_a(self.get_ptr()).clone()
        }
    }
    pub fn get_local_anchor_b(&self) -> Vec2 {
        unsafe {
            *ffi::WheelJoint_get_local_anchor_b(self.get_ptr()).clone()
        }
    }
    pub fn get_local_axis_a(&self) -> Vec2 {
        unsafe {
            *ffi::WheelJoint_get_local_anchor_a(self.get_ptr()).clone()
        }
    }
    pub fn get_joint_translation(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_joint_translation(self.get_ptr())
        }
    }
    pub fn get_joint_speed(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_joint_speed(self.get_ptr())
        }
    }
    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::WheelJoint_is_motor_enabled(self.get_ptr())
        }
    }
    pub fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::WheelJoint_enable_motor(self.get_mut_ptr(), flag)
        }
    }
    pub fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::WheelJoint_set_motor_speed(self.get_mut_ptr(), speed)
        }
    }
    pub fn get_motor_speed(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_motor_speed(self.get_ptr())
        }
    }
    pub fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::WheelJoint_set_max_motor_torque(self.get_mut_ptr(), torque)
        }
    }
    pub fn get_max_motor_torque(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_max_motor_torque(self.get_ptr())
        }
    }
    pub fn get_motor_torque(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_motor_torque(self.get_ptr())
        }
    }
    pub fn set_spring_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_frequency(self.get_mut_ptr(), frequency)
        }
    }
    pub fn get_spring_frequency(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_frequency(self.get_ptr())
        }
    }
    pub fn set_spring_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_damping_ratio(self.get_mut_ptr(), ratio)
        }
    }
    pub fn get_spring_damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_damping_ratio(self.get_ptr())
        }
    }
}

impl_joint!(for DistanceJoint
    << ffi::Joint_as_distance_joint
    >> ffi::DistanceJoint_as_joint
    )
impl_joint!(for FrictionJoint
    << ffi::Joint_as_friction_joint
    >> ffi::FrictionJoint_as_joint
    )
impl_joint!(for GearJoint
    << ffi::Joint_as_gear_joint
    >> ffi::GearJoint_as_joint
    )
impl_joint!(for MotorJoint
    << ffi::Joint_as_motor_joint
    >> ffi::MotorJoint_as_joint
    )
impl_joint!(for MouseJoint
    << ffi::Joint_as_mouse_joint
    >> ffi::MouseJoint_as_joint
    )
impl_joint!(for PrismaticJoint
    << ffi::Joint_as_prismatic_joint
    >> ffi::PrismaticJoint_as_joint
    )
impl_joint!(for PulleyJoint
    << ffi::Joint_as_pulley_joint
    >> ffi::PulleyJoint_as_joint
    )
impl_joint!(for RevoluteJoint
    << ffi::Joint_as_revolute_joint
    >> ffi::RevoluteJoint_as_joint
    )
impl_joint!(for RopeJoint
    << ffi::Joint_as_rope_joint
    >> ffi::RopeJoint_as_joint
    )
impl_joint!(for WeldJoint
    << ffi::Joint_as_weld_joint
    >> ffi::WeldJoint_as_joint
    )
impl_joint!(for WheelJoint
    << ffi::Joint_as_wheel_joint
    >> ffi::WheelJoint_as_joint
    )
