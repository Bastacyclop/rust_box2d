b2PulleyJointDef PulleyJointDef_default() {
    return b2PulleyJointDef();
}
void PulleyJointDef_initialize(b2PulleyJointDef* self,
                               b2Body* body_a, b2Body* body_b,
                               const b2Vec2* ground_anchor_a,
                               const b2Vec2* ground_anchor_b,
                               const b2Vec2* anchor_a,
                               const b2Vec2* anchor_b,
                               f32 ratio) {
    self->Initialize(body_a, body_b,
                     *ground_anchor_a, *ground_anchor_b,
                     *anchor_a, *anchor_b,
                     ratio);
}

b2Joint* PulleyJoint_as_joint(b2PulleyJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2PulleyJoint* Joint_as_pulley_joint(b2Joint* self) {
    return static_cast<b2PulleyJoint*>(self);
}

b2Vec2 PulleyJoint_get_ground_anchor_a(const b2PulleyJoint* self) {
    return self->GetGroundAnchorA();
}
b2Vec2 PulleyJoint_get_ground_anchor_b(const b2PulleyJoint* self) {
    return self->GetGroundAnchorB();
}
f32 PulleyJoint_get_length_a(const b2PulleyJoint* self) {
    return self->GetLengthA();
}
f32 PulleyJoint_get_length_b(const b2PulleyJoint* self) {
    return self->GetLengthB();
}
f32 PulleyJoint_get_ratio(const b2PulleyJoint* self) {
    return self->GetRatio();
}
f32 PulleyJoint_get_current_length_a(const b2PulleyJoint* self) {
    return self->GetCurrentLengthA();
}
f32 PulleyJoint_get_current_length_b(const b2PulleyJoint* self) {
    return self->GetCurrentLengthB();
}
