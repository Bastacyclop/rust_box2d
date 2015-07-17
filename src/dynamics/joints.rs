use std::ptr;
use ffi;
use wrap::*;
use math::Vec2;
use dynamics::Body;

macro_rules! wrap_joint {
    ($wrapped:ty: $wrap:ident ($joint_type:path)
     > $as_base:path,
     < $base_as:path
    ) => (
        wrap! {
            $wrapped: $wrap with base ffi::Joint
            > $as_base,
            < $base_as
        }

        impl Joint for $wrap {
            fn assumed_joint_type() -> JointType { $joint_type }
        }
    );
}

macro_rules! joint_def {
    ($name:ident {
        $(($($visibility:ident)*) $field:ident: $typ:ty),+
    }) => {
        #[repr(C)]
        #[allow(dead_code)]
        pub struct $name {
            pub base: JointDefBase,
            $(
                $($visibility)* $field: $typ,
            )+
        }

        impl WrappedBase<JointDefBase> for $name {
            unsafe fn base_ptr(&self) -> *const JointDefBase {
                self as *const $name as *const JointDefBase
            }

            unsafe fn mut_base_ptr(&mut self) -> *mut JointDefBase {
                self as *mut $name as *mut JointDefBase
            }
        }

        impl JointDef for $name {}
    }
}

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum JointType {
    Unknown = 0,
    Revolute = 1,
    Prismatic = 2,
    Distance = 3,
    Pulley = 4,
    Mouse = 5,
    Gear = 6,
    Wheel = 7,
    Weld = 8,
    Friction = 9,
    Rope = 10,
    Motor = 11
}

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum LimitState {
    Inactive = 0,
    Lower = 1,
    Upper = 2,
    Equal = 3
}

pub trait JointDef: WrappedBase<JointDefBase> {
    fn joint_type(&self) -> JointType {
        unsafe {
            (*self.base_ptr()).joint_type
        }
    }
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
            user_data: ptr::null_mut(),
            body_a: ptr::null_mut(),
            body_b: ptr::null_mut(),
            collide_connected: false
        }
    }

    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        self.user_data = data as ffi::Any
    }
}

pub trait Joint: WrappedBase<ffi::Joint> + FromFFI<ffi::Joint> {
    fn assumed_joint_type() -> JointType where Self: Sized;

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

    fn body_a(&self) -> usize {
        unsafe {
            ffi::Joint_get_body_a(self.base_ptr() as *mut ffi::Joint) as usize
        }
    }

    fn body_b(&self) -> usize {
        unsafe {
            ffi::Joint_get_body_b(self.base_ptr() as *mut ffi::Joint) as usize
        }
    }

    unsafe fn user_data<T>(&self) -> *mut T {
        ffi::Joint_get_user_data(self.base_ptr()) as *mut T
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
    pub fn other(&self) -> usize {
        self.other as usize
    }

    pub fn joint(&self) -> usize {
        self.joint as usize
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
    fn assumed_joint_type() -> JointType { JointType::Unknown }
}

joint_def! {
    DistanceJointDef {
        () local_anchor_a: Vec2,
        () local_anchor_b: Vec2,
        (pub) length: f32,
        (pub) frequency: f32,
        (pub) damping_ratio: f32
    }
}

joint_def! {
    FrictionJointDef {
        () local_anchor_a: Vec2,
        () local_anchor_b: Vec2,
        (pub) max_force: f32,
        (pub) max_torque: f32
    }
}

joint_def! {
    GearJointDef {
        () joint_a: *mut ffi::Joint,
        () joint_b: *mut ffi::Joint,
        (pub) ratio: f32
    }
}

joint_def! {
    MotorJointDef {
        (pub) linear_offset: Vec2,
        (pub) angular_offset: f32,
        (pub) max_force: f32,
        (pub) max_torque: f32,
        (pub) correction_factor: f32
    }
}

joint_def! {
    MouseJointDef {
        (pub) target: Vec2,
        (pub) max_force: f32,
        (pub) frequency: f32,
        (pub) damping_ratio: f32
    }
}

joint_def! {
    PrismaticJointDef {
        () local_anchor_a: Vec2,
        () local_anchor_b: Vec2,
        () local_axis_a: Vec2,
        (pub) reference_angle: f32,
        (pub) enable_limit: bool,
        (pub) lower_translation: f32,
        (pub) upper_translation: f32,
        (pub) enable_motor: bool,
        (pub) max_motor_force: f32,
        (pub) motor_speed: f32
    }
}

joint_def! {
    PulleyJointDef {
        () ground_anchor_a: Vec2,
        () ground_anchor_b: Vec2,
        () local_anchor_a: Vec2,
        () local_anchor_b: Vec2,
        () length_a: f32,
        () length_b: f32,
        (pub) ratio: f32
    }
}

joint_def! {
    RevoluteJointDef {
        () local_anchor_a: Vec2,
        () local_anchor_b: Vec2,
        (pub) reference_angle: f32,
        (pub) enable_limit: bool,
        (pub) lower_angle: f32,
        (pub) upper_angle: f32,
        (pub) enable_motor: bool,
        (pub) motor_speed: f32,
        (pub) max_motor_torque: f32
    }
}

joint_def! {
    RopeJointDef {
        () local_anchor_a: Vec2,
        () local_anchor_b: Vec2,
        (pub) max_length: f32
    }
}

joint_def! {
    WeldJointDef {
        () local_anchor_a: Vec2,
        () local_anchor_b: Vec2,
        (pub) reference_angle: f32,
        (pub) frequency: f32,
        (pub) damping_ratio: f32
    }
}

joint_def! {
    WheelJointDef {
        () local_anchor_a: Vec2,
        () local_anchor_b: Vec2,
        () local_axis_a: Vec2,
        (pub) enable_motor: bool,
        (pub) max_motor_torque: f32,
        (pub) motor_speed: f32,
        (pub) frequency: f32,
        (pub) damping_ratio: f32
    }
}

impl DistanceJointDef {
    pub fn new(mut body_a: Body,
               mut body_b: Body,
               anchor_a: &Vec2,
               anchor_b: &Vec2) -> DistanceJointDef {
        unsafe {
            let mut joint =
                DistanceJointDef {
                    base: JointDefBase::new(JointType::Distance),
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
    pub fn new(mut body_a: Body,
               mut body_b: Body,
               anchor: &Vec2) -> FrictionJointDef {
        unsafe {
            let mut joint =
                FrictionJointDef {
                    base: JointDefBase::new(JointType::Friction),
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
    pub fn new<Ja: Joint, Jb: Joint>(mut joint_a: Ja,
                                     mut joint_b: Jb
                                     ) -> GearJointDef {
        unsafe {
            GearJointDef {
                base: JointDefBase::new(JointType::Gear),
                joint_a: joint_a.mut_base_ptr(),
                joint_b: joint_b.mut_base_ptr(),
                ratio: 1.
            }
        }
    }
}

impl MotorJointDef {
    pub fn new(mut body_a: Body,
               mut body_b: Body) -> MotorJointDef {
        unsafe {
            let mut joint =
                MotorJointDef {
                    base: JointDefBase::new(JointType::Motor),
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
            base: JointDefBase::new(JointType::Mouse),
            target: Vec2 { x:0., y:0. },
            max_force: 0.,
            frequency: 5.,
            damping_ratio: 0.7
        }
    }
}

impl PrismaticJointDef {
    pub fn new(mut body_a: Body,
               mut body_b: Body,
               anchor: &Vec2,
               axis: &Vec2) -> PrismaticJointDef {
        unsafe {
            let mut joint =
                PrismaticJointDef {
                    base: JointDefBase::new(JointType::Prismatic),
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
    pub fn new(mut body_a: Body,
               mut body_b: Body,
               ground_anchor_a: &Vec2,
               ground_anchor_b: &Vec2,
               anchor_a: &Vec2,
               anchor_b: &Vec2,
               ratio: f32) -> PulleyJointDef {
        unsafe {
            let mut joint =
                PulleyJointDef {
                    base: JointDefBase::new(JointType::Pulley),
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
    pub fn new(mut body_a: Body,
               mut body_b: Body,
               anchor: &Vec2) -> RevoluteJointDef {
        unsafe {
            let mut joint =
                RevoluteJointDef {
                    base: JointDefBase::new(JointType::Revolute),
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
            base: JointDefBase::new(JointType::Rope),
            local_anchor_a: Vec2 { x:-1., y:0. },
            local_anchor_b: Vec2 { x:1., y:0. },
            max_length: 0.
        }
    }
}

impl WeldJointDef {
    pub fn new(mut body_a: Body,
               mut body_b: Body,
               anchor: &Vec2) -> WeldJointDef {
        unsafe {
            let mut joint =
                WeldJointDef {
                    base: JointDefBase::new(JointType::Weld),
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
    pub fn new(mut body_a: Body,
               mut body_b: Body,
               anchor: &Vec2,
               axis: &Vec2) -> WheelJointDef {
        unsafe {
            let mut joint =
                WheelJointDef {
                    base: JointDefBase::new(JointType::Wheel),
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

wrap_joint! {
    ffi::DistanceJoint: DistanceJoint (JointType::Distance)
    > ffi::DistanceJoint_as_joint,
    < ffi::Joint_as_distance_joint
}

wrap_joint! {
    ffi::FrictionJoint: FrictionJoint (JointType::Friction)
    > ffi::FrictionJoint_as_joint,
    < ffi::Joint_as_friction_joint
}

wrap_joint! {
    ffi::GearJoint: GearJoint (JointType::Gear)
    > ffi::GearJoint_as_joint,
    < ffi::Joint_as_gear_joint
}

wrap_joint! {
    ffi::MotorJoint: MotorJoint (JointType::Motor)
    > ffi::MotorJoint_as_joint,
    < ffi::Joint_as_motor_joint
}

wrap_joint! {
    ffi::MouseJoint: MouseJoint (JointType::Mouse)
    > ffi::MouseJoint_as_joint,
    < ffi::Joint_as_mouse_joint
}

wrap_joint! {
    ffi::PrismaticJoint: PrismaticJoint (JointType::Prismatic)
    > ffi::PrismaticJoint_as_joint,
    < ffi::Joint_as_prismatic_joint
}

wrap_joint! {
    ffi::PulleyJoint: PulleyJoint (JointType::Pulley)
    > ffi::PulleyJoint_as_joint,
    < ffi::Joint_as_pulley_joint
}

wrap_joint! {
    ffi::RevoluteJoint: RevoluteJoint (JointType::Revolute)
    > ffi::RevoluteJoint_as_joint,
    < ffi::Joint_as_revolute_joint
}

wrap_joint! {
    ffi::RopeJoint: RopeJoint (JointType::Rope)
    > ffi::RopeJoint_as_joint,
    < ffi::Joint_as_rope_joint
}

wrap_joint! {
    ffi::WeldJoint: WeldJoint (JointType::Weld)
    > ffi::WeldJoint_as_joint,
    < ffi::Joint_as_weld_joint
}

wrap_joint! {
    ffi::WheelJoint: WheelJoint (JointType::Wheel)
    > ffi::WheelJoint_as_joint,
    < ffi::Joint_as_wheel_joint
}

impl DistanceJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::DistanceJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::DistanceJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn length(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_length(self.ptr())
        }
    }

    pub fn frequency(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_frequency(self.ptr())
        }
    }

    pub fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_damping_ratio(self.ptr())
        }
    }

    pub fn set_length(&mut self, length: f32) {
        unsafe {
            ffi::DistanceJoint_set_length(self.mut_ptr(), length)
        }
    }

    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::DistanceJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }

    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::DistanceJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

impl FrictionJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::FrictionJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::FrictionJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn max_force(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_force(self.ptr())
        }
    }

    pub fn max_torque(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_torque(self.ptr())
        }
    }

    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_force(self.mut_ptr(), force)
        }
    }

    pub fn set_max_torque(&mut self, torque: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_torque(self.mut_ptr(), torque)
        }
    }
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

impl MotorJoint {
    pub fn linear_offset<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::MotorJoint_get_linear_offset(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn angular_offset(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_angular_offset(self.ptr())
        }
    }

    pub fn max_force(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_max_force(self.ptr())
        }
    }

    pub fn max_torque(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_max_torque(self.ptr())
        }
    }

    pub fn correction_factor(&self) -> f32 {
        unsafe {
            ffi::MotorJoint_get_correction_factor(self.ptr())
        }
    }

    pub fn set_linear_offset(&mut self, offset: &Vec2) {
        unsafe {
            ffi::MotorJoint_set_linear_offset(self.mut_ptr(), offset)
        }
    }

    pub fn set_angular_offset(&mut self, offset: f32) {
        unsafe {
            ffi::MotorJoint_set_angular_offset(self.mut_ptr(), offset)
        }
    }

    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::MotorJoint_set_max_force(self.mut_ptr(), force)
        }
    }

    pub fn set_max_torque(&mut self, torque: f32) {
        unsafe {
            ffi::MotorJoint_set_max_torque(self.mut_ptr(), torque)
        }
    }

    pub fn set_correction_factor(&mut self, factor: f32) {
        unsafe {
            ffi::MotorJoint_set_correction_factor(self.mut_ptr(), factor)
        }
    }
}

impl MouseJoint {
    pub fn target<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::MouseJoint_get_target(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn max_force(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_max_force(self.ptr())
        }
    }

    pub fn frequency(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_frequency(self.ptr())
        }
    }

    pub fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::MouseJoint_get_damping_ratio(self.ptr())
        }
    }

    pub fn set_target(&mut self, target: &Vec2) {
        unsafe {
            ffi::MouseJoint_set_target(self.mut_ptr(), target)
        }
    }

    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::MouseJoint_set_max_force(self.mut_ptr(), force)
        }
    }

    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::MouseJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }

    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::MouseJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

impl PrismaticJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::PrismaticJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::PrismaticJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_axis_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::PrismaticJoint_get_local_axis_a(self.ptr()) // Comes from a C++ &
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

    pub fn lower_limit(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_lower_limit(self.ptr())
        }
    }

    pub fn upper_limit(&self) -> f32 {
        unsafe {
             ffi::PrismaticJoint_get_upper_limit(self.ptr())
        }
    }

    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::PrismaticJoint_is_motor_enabled(self.ptr())
        }
    }

    pub fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_motor_speed(self.ptr())
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

    pub fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::PrismaticJoint_enable_limit(self.mut_ptr(), flag)
        }
    }

    pub fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::PrismaticJoint_set_limits(self.mut_ptr(), lower, upper)
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

    pub fn set_max_motor_force(&mut self, force: f32) {
        unsafe {
            ffi::PrismaticJoint_set_max_motor_force(self.mut_ptr(), force)
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
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::RevoluteJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::RevoluteJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
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

    pub fn lower_limit(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_lower_limit(self.ptr())
        }
    }

    pub fn upper_limit(&self) -> f32 {
        unsafe {
             ffi::RevoluteJoint_get_upper_limit(self.ptr())
        }
    }

    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::RevoluteJoint_is_motor_enabled(self.ptr())
        }
    }

    pub fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_motor_speed(self.ptr())
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

    pub fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::RevoluteJoint_enable_limit(self.mut_ptr(), flag)
        }
    }

    pub fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::RevoluteJoint_set_limits(self.mut_ptr(), lower, upper)
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

    pub fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::RevoluteJoint_set_max_motor_torque(self.mut_ptr(), torque)
        }
    }
}

impl RopeJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::RopeJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::RopeJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
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

    pub fn set_max_length(&mut self, length: f32) {
        unsafe {
            ffi::RopeJoint_set_max_length(self.mut_ptr(), length)
        }
    }
}

impl WeldJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WeldJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WeldJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn referance_angle(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_reference_angle(self.ptr())
        }
    }

    pub fn frequency(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_frequency(self.ptr())
        }
    }

    pub fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_damping_ratio(self.ptr())
        }
    }

    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WeldJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }

    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WeldJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

impl WheelJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WheelJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WheelJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_axis_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WheelJoint_get_local_axis_a(self.ptr()) // Comes from a C++ &
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

    pub fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_motor_speed(self.ptr())
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

    pub fn spring_frequency(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_frequency(self.ptr())
        }
    }

    pub fn spring_damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_damping_ratio(self.ptr())
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

    pub fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::WheelJoint_set_max_motor_torque(self.mut_ptr(), torque)
        }
    }

    pub fn set_spring_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_frequency(self.mut_ptr(), frequency)
        }
    }

    pub fn set_spring_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}
