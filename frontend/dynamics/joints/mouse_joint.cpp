b2Joint* World_create_mouse_joint(
    b2World* self,
    b2Body* body_a,
    b2Body* body_b,
    bool collide_connected,
    b2Vec2 target,
    f32 max_force,
    f32 frequency,
    f32 damping_ratio
) {
    b2MouseJointDef def;
    def.bodyA = body_a;
    def.bodyB = body_b;
    def.collideConnected = collide_connected;
    def.target = target;
    def.maxForce = max_force;
    def.frequencyHz = frequency;
    def.dampingRatio = damping_ratio;

    return self->CreateJoint(&def);
}

b2Joint* MouseJoint_as_joint(b2MouseJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2MouseJoint* Joint_as_mouse_joint(b2Joint* self) {
    return static_cast<b2MouseJoint*>(self);
}

void MouseJoint_set_target(b2MouseJoint* self, const b2Vec2* target) {
    self->SetTarget(*target);
}
const b2Vec2* MouseJoint_get_target(const b2MouseJoint* self) {
    return &self->GetTarget();
}
void MouseJoint_set_max_force(b2MouseJoint* self, f32 force) {
    self->SetMaxForce(force);
}
f32 MouseJoint_get_max_force(const b2MouseJoint* self) {
    return self->GetMaxForce();
}
void MouseJoint_set_frequency(b2MouseJoint* self, f32 hz) {
    self->SetFrequency(hz);
}
f32 MouseJoint_get_frequency(const b2MouseJoint* self) {
    return self->GetFrequency();
}
void MouseJoint_set_damping_ratio(b2MouseJoint* self, f32 ratio) {
    self->SetDampingRatio(ratio);
}
f32 MouseJoint_get_damping_ratio(const b2MouseJoint* self) {
    return self->GetDampingRatio();
}
