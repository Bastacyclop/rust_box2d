void WorldManifold_Initialize(b2WorldManifold* self,
                              const b2Manifold* manifold,
                              const b2Transform* xf_a, f32 radius_a,
                              const b2Transform* xf_b, f32 radius_b) {
    self->Initialize(manifold,
                     *xf_a, radius_a,
                     *xf_b, radius_b);
}

void get_point_states(b2PointState* s1, b2PointState* s2,
                      const b2Manifold* m1, const b2Manifold* m2) {
    b2GetPointStates(s1, s2, m1, m2);
}

bool test_overlap(const b2Shape* shape_a, i32 index_a,
                  const b2Shape* shape_b, i32 index_b,
                  const b2Transform* xf_a, const b2Transform* xf_b) {
    return b2TestOverlap(shape_a, index_a,
                         shape_b, index_b,
                         *xf_a, *xf_b);
}


void DistanceProxy_set(b2DistanceProxy* self,
                       const b2Shape* shape, i32 index) {
    self->Set(shape, index);
}

void distance(b2DistanceOutput* output,
              b2SimplexCache* cache,
              const b2DistanceInput* input) {
    b2Distance(output, cache, input);
}

void time_of_impact(b2TOIOutput* output, const b2TOIInput* input) {
    b2TimeOfImpact(output, input);
}


b2Manifold* Contact_get_manifold(b2Contact* self) {
    return self->GetManifold();
}

const b2Manifold* Contact_get_manifold_const(const b2Contact* self) {
    return self->GetManifold();
}

void Contact_get_world_manifold(const b2Contact* self,
                                b2WorldManifold* world_manifold) {
    self->GetWorldManifold(world_manifold);
}

bool Contact_is_touching(const b2Contact* self) {
    return self->IsTouching();
}

bool Contact_is_enabled(const b2Contact* self) {
    return self->IsEnabled();
}

b2Contact* Contact_get_next(b2Contact* self) {
    return self->GetNext();
}

const b2Contact* Contact_get_next_const(const b2Contact* self) {
    return self->GetNext();
}

b2Fixture* Contact_get_fixture_a(b2Contact* self) {
    return self->GetFixtureA();
}

const b2Fixture* Contact_get_fixture_a_const(const b2Contact* self) {
    return self->GetFixtureA();
}

i32 Contact_get_child_index_a(const b2Contact* self) {
    return self->GetChildIndexA();
}

b2Fixture* Contact_get_fixture_b(b2Contact* self) {
    return self->GetFixtureB();
}

const b2Fixture* Contact_get_fixture_b_const(const b2Contact* self) {
    return self->GetFixtureB();
}

i32 Contact_get_child_index_b(const b2Contact* self) {
    return self->GetChildIndexB();
}

void Contact_set_friction(b2Contact* self, f32 friction) {
    self->SetFriction(friction);
}

f32 Contact_get_friction(const b2Contact* self) {
    return self->GetFriction();
}

void Contact_reset_friction(b2Contact* self) {
    self->ResetFriction();
}

void Contact_set_restitution(b2Contact* self, f32 restitution) {
    self->SetRestitution(restitution);
}

f32 Contact_get_restitution(const b2Contact* self) {
    return self->GetRestitution();
}

void Contact_reset_restitution(b2Contact* self) {
    self->ResetRestitution();
}

void Contact_set_tangent_speed(b2Contact* self, f32 speed) {
    self->SetTangentSpeed(speed);
}

f32 Contact_get_tangent_speed(const b2Contact* self) {
    self->GetTangentSpeed();
}

void Contact_evaluate_virtual(b2Contact* self, b2Manifold* manifold,
                              const b2Transform* xf_a,
                              const b2Transform* xf_b) {
    self->Evaluate(manifold, *xf_a, *xf_b);
}
