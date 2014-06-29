use ffi;
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

wrap!(ffi::Joint into UnknownJoint)

impl Drop for UnknownJoint {
    fn drop(&mut self) {
    }
}
