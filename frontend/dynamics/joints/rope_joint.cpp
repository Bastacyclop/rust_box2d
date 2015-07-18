b2Joint* RopeJoint_as_joint(b2RopeJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2RopeJoint* Joint_as_rope_joint(b2Joint* self) {
    return static_cast<b2RopeJoint*>(self);
}

const b2Vec2* RopeJoint_get_local_anchor_a(const b2RopeJoint* self) {
    return &self->GetLocalAnchorA();
}
const b2Vec2* RopeJoint_get_local_anchor_b(const b2RopeJoint* self) {
    return &self->GetLocalAnchorB();
}
void RopeJoint_set_max_length(b2RopeJoint* self, f32 length) {
    self->SetMaxLength(length);
}
f32 RopeJoint_get_max_length(const b2RopeJoint* self) {
    return self->GetMaxLength();
}
b2LimitState RopeJoint_get_limit_state(const b2RopeJoint* self) {
    return self->GetLimitState();
}
