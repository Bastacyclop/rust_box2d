b2Joint* GearJoint_as_joint(b2GearJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2GearJoint* Joint_as_gear_joint(b2Joint* self) {
    return static_cast<b2GearJoint*>(self);
}

const b2Joint* GearJoint_get_joint_1(b2GearJoint* self) {
    return self->GetJoint1();
}
const b2Joint* GearJoint_get_joint_2(b2GearJoint* self) {
    return self->GetJoint2();
}
void GearJoint_set_ratio(b2GearJoint* self, f32 ratio) {
    self->SetRatio(ratio);
}
f32 GearJoint_get_ratio(const b2GearJoint* self) {
    return self->GetRatio();
}
