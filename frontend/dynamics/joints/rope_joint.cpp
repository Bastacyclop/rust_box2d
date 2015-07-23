b2Joint* World_create_rope_joint(
    b2World* world,
    b2Body* body_a,
    b2Body* body_b,
    bool collide_connected,
    b2Vec2 local_anchor_a,
    b2Vec2 local_anchor_b,
    f32 max_length
) {
    b2RopeJointDef def;
    def.bodyA = body_a;
    def.bodyB = body_b;
    def.collideConnected = collide_connected;
    def.localAnchorA = local_anchor_a;
    def.localAnchorB = local_anchor_b;
    def.maxLength = max_length;

    return world->CreateJoint(&def);
}

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
