b2JointDef JointDef_default() {
    return b2JointDef();
}

i32 Joint_get_type(const b2Joint* self) {
    return self->GetType();
}
b2Body* Joint_get_body_a(b2Joint* self) {
    return self->GetBodyA();
}
b2Body* Joint_get_body_b(b2Joint* self) {
    return self->GetBodyB();
}
b2Vec2 Joint_get_anchor_a_virtual(const b2Joint* self) {
    return self->GetAnchorA();
}
b2Vec2 Joint_get_anchor_b_virtual(const b2Joint* self) {
    return self->GetAnchorB();
}
b2Vec2 Joint_get_reaction_force_virtual(const b2Joint* self,
                                        f32 inv_dt) {
    return self->GetReactionForce(inv_dt);
}
f32 Joint_get_reaction_torque_virtual(const b2Joint* self,
                                      f32 inv_dt) {
    return self->GetReactionTorque(inv_dt);
}
b2Joint* Joint_get_next(b2Joint* self) {
    return self->GetNext();
}
const b2Joint* Joint_get_next_const(const b2Joint* self) {
    return self->GetNext();
}
void* Joint_get_user_data(const b2Joint* self) {
    return self->GetUserData();
}
void Joint_set_user_data(b2Joint* self, void* data) {
    self->SetUserData(data);
}
bool Joint_is_active(const b2Joint* self) {
    return self->IsActive();
}

void Joint_dump_virtual(b2Joint* self) {
    self->Dump();
}
void Joint_shift_origin_virtual(b2Joint* self, const b2Vec2* origin) {
    self->ShiftOrigin(*origin);
}
