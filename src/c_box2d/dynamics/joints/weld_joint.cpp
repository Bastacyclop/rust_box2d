b2WeldJointDef WeldJointDef_default() {
    return b2WeldJointDef();
}
void WeldJointDef_initialize(b2WeldJointDef* self,
                                 b2Body* body_a, b2Body* body_b,
                                 const b2Vec2* anchor) {
    self->Initialize(body_a, body_b, *anchor);
}

b2Joint* WeldJoint_as_joint(b2WeldJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2WeldJoint* Joint_as_weld_joint(b2Joint* self) {
    return static_cast<b2WeldJoint*>(self);
}

const b2Vec2* WeldJoint_get_local_anchor_a(const b2WeldJoint* self) {
    return &self->GetLocalAnchorA();
}
const b2Vec2* WeldJoint_get_local_anchor_b(const b2WeldJoint* self) {
    return &self->GetLocalAnchorB();
}
f32 WeldJoint_get_reference_angle(const b2WeldJoint* self) {
    return self->GetReferenceAngle();
}
void WeldJoint_set_frequency(b2WeldJoint* self, f32 frequency) {
    self->SetFrequency(frequency);
}
f32 WeldJoint_get_frequency(const b2WeldJoint* self) {
    return self->GetFrequency();
}
void WeldJoint_set_damping_ratio(b2WeldJoint* self, f32 ratio) {
    self->SetDampingRatio(ratio);
}
f32 WeldJoint_get_damping_ratio(const b2WeldJoint* self) {
    return self->GetDampingRatio();
}
