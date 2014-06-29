b2MotorJointDef MotorJointDef_default() {
    return b2MotorJointDef();
}
void MotorJointDef_initialize(b2MotorJointDef* self,
                              b2Body* body_a, b2Body* body_b) {
    self->Initialize(body_a, body_b);
}

b2Joint* MotorJoint_as_joint(b2MotorJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2MotorJoint* Joint_as_motor_joint(b2Joint* self) {
    return static_cast<b2MotorJoint*>(self);
}

void MotorJoint_set_linear_offset(b2MotorJoint* self,
                                  const b2Vec2* offset) {
    self->SetLinearOffset(*offset);
}
const b2Vec2* MotorJoint_get_linear_offset(const b2MotorJoint* self) {
    return &self->GetLinearOffset();
}
void MotorJoint_set_angular_offset(b2MotorJoint* self, f32 offset) {
    self->SetAngularOffset(offset);
}
f32 MotorJoint_get_angular_offset(const b2MotorJoint* self) {
    return self->GetAngularOffset();
}
void MotorJoint_set_max_force(b2MotorJoint* self, f32 force) {
    self->SetMaxForce(force);
}
f32 MotorJoint_get_max_force(const b2MotorJoint* self) {
    return self->GetMaxForce();
}
void MotorJoint_set_max_torque(b2MotorJoint* self, f32 torque) {
    self->SetMaxTorque(torque);
}
f32 MotorJoint_get_max_torque(const b2MotorJoint* self) {
    return self->GetMaxTorque();
}
void MotorJoint_set_correction_factor(b2MotorJoint* self, f32 factor) {
    self->SetCorrectionFactor(factor);
}
f32 MotorJoint_get_correction_factor(const b2MotorJoint* self) {
    return self->GetCorrectionFactor();
}
