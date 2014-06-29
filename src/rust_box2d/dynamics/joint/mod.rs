use ffi;
use math::Vec2;
use Wrapped;

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

#[allow(dead_code)]
#[packed]
pub struct JointDef {
    pub joint_type: JointType,
    user_data: ffi::UserData,
    body_a: *mut ffi::Body,
    body_b: *mut ffi::Body,
    pub collide_connected: bool,
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

impl Drop for UnknownJoint {
    fn drop(&mut self) {
    }
}
