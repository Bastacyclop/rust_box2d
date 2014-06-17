b2FixtureDef FixtureDef_default() {
    return b2FixtureDef();
}

i32 Fixture_get_type(const b2Fixture* self) {
    return self->GetType();
}
b2Shape* Fixture_get_shape(b2Fixture* self) {
    return self->GetShape();
}
const b2Shape* Fixture_get_shape_const(const b2Fixture* self) {
    return self->GetShape();
}
void Fixture_set_sensor(b2Fixture* self, bool flag) {
    self->SetSensor(flag);
}
bool Fixture_is_sensor(const b2Fixture* self) {
    return self->IsSensor();
}
void Fixture_set_filter_data(b2Fixture* self, const b2Filter* filter) {
    self->SetFilterData(*filter);
}
const b2Filter* Fixture_get_filter_data(const b2Fixture* self) {
    return &self->GetFilterData();
}
void Fixture_refilter(b2Fixture* self) {
    self->Refilter();
}
b2Body* Fixture_get_body(b2Fixture* self) {
    return self->GetBody();
}
const b2Body* Fixture_get_body_const(const b2Fixture* self) {
    return self->GetBody();
}
b2Fixture* Fixture_get_next(b2Fixture* self) {
    return self->GetNext();
}
const b2Fixture* Fixture_get_next_const(const b2Fixture* self) {
    return self->GetNext();
}
void* Fixture_get_user_data(const b2Fixture* self) {
    return self->GetUserData();
}
void Fixture_set_user_data(b2Fixture* self, void* data) {
    self->SetUserData(data);
}
bool Fixture_test_point(const b2Fixture* self, const b2Vec2* p) {
    return self->TestPoint(*p);
}
bool Fixture_ray_cast(const b2Fixture* self,
                      b2RayCastOutput* output,
                      const b2RayCastInput* input,
                      i32 child_id) {
    return self->RayCast(output, *input, child_id);
}
void Fixture_get_mass_data(const b2Fixture* self, b2MassData* data) {
    self->GetMassData(data);
}
void Fixture_set_density(b2Fixture* self, f32 density) {
    self->SetDensity(density);
}
f32 Fixture_get_density(const b2Fixture* self) {
    return self->GetDensity();
}
f32 Fixture_get_friction(const b2Fixture* self) {
    return self->GetFriction();
}
void Fixture_set_friction(b2Fixture* self, f32 friction) {
    self->SetFriction(friction);
}
f32 Fixture_get_restitution(const b2Fixture* self) {
    return self->GetRestitution();
}
void Fixture_set_restitution(b2Fixture* self, f32 restitution) {
    self->SetRestitution(restitution);
}
const b2AABB* Fixture_get_aabb(const b2Fixture* self, i32 child_id) {
    return &self->GetAABB(child_id);
}

void Fixture_dump(b2Fixture* self, i32 body_id) {
    self->Dump(body_id);
}
