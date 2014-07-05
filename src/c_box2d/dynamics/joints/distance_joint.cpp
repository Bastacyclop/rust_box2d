void DistanceJointDef_initialize(b2DistanceJointDef* self,
                                 b2Body* body_a, b2Body* body_b,
                                 const b2Vec2* anchor_a,
                                 const b2Vec2* anchor_b) {
    self->Initialize(body_a, body_b, *anchor_a, *anchor_b);
}

b2Joint* DistanceJoint_as_joint(b2DistanceJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2DistanceJoint* Joint_as_distance_joint(b2Joint* self) {
    return static_cast<b2DistanceJoint*>(self);
}

const b2Vec2* DistanceJoint_get_local_anchor_a(
                                        const b2DistanceJoint* self) {
    return &self->GetLocalAnchorA();
}
const b2Vec2* DistanceJoint_get_local_anchor_b(
                                        const b2DistanceJoint* self) {
    return &self->GetLocalAnchorB();
}
void DistanceJoint_set_length(b2DistanceJoint* self, f32 length) {
    self->SetLength(length);
}
f32 DistanceJoint_get_length(const b2DistanceJoint* self) {
    return self->GetLength();
}
void DistanceJoint_set_frequency(b2DistanceJoint* self, f32 hz) {
    self->SetFrequency(hz);
}
f32 DistanceJoint_get_frequency(const b2DistanceJoint* self) {
    return self->GetFrequency();
}
void DistanceJoint_set_damping_ratio(b2DistanceJoint* self,
                                     f32 ratio) {
    self->SetDampingRatio(ratio);
}
f32 DistanceJoint_get_damping_ratio(const b2DistanceJoint* self) {
    return self->GetDampingRatio();
}
