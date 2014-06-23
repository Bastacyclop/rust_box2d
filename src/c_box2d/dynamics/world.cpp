extern "C"
{

b2World* World_new(const b2Vec2* gravity) {
    return new b2World(*gravity);
}
void World_drop(b2World* self) {
    delete self;
}

void World_set_destruction_listener(b2World* self,
                                    b2DestructionListener* listener) {
    self->SetDestructionListener(listener);
}
void World_set_contact_filter(b2World* self, b2ContactFilter* filter) {
    self->SetContactFilter(filter);
}
void World_set_contact_listener(b2World* self,
                                b2ContactListener* listener) {
    self->SetContactListener(listener);
}
void World_set_debug_draw(b2World* self, b2Draw* draw) {
    self->SetDebugDraw(draw);
}
b2Body* World_create_body(b2World* self, const b2BodyDef* def) {
    return self->CreateBody(def);
}
void World_destroy_body(b2World* self, b2Body* body) {
    self->DestroyBody(body);
}
b2Joint* World_create_joint(b2World* self, const b2JointDef* def) {
    return self->CreateJoint(def);
}
void World_destroy_joint(b2World* self, b2Joint* joint) {
    self->DestroyJoint(joint);
}
void World_step(b2World* self,
                f32 time_step,
                i32 velocity_iterations,
                i32 position_iterations) {
    self->Step(time_step, velocity_iterations, position_iterations);
}
void World_clear_forces(b2World* self) {
    self->ClearForces();
}
void World_draw_debug_data(b2World* self) {
    self->DrawDebugData();
}
void World_query_aabb(const b2World* self,
                      b2QueryCallback* callback,
                      const b2AABB* aabb) {
    self->QueryAABB(callback, *aabb);
}
void World_ray_cast(const b2World* self,
                    b2RayCastCallback* callback,
                    const b2Vec2* p1, const b2Vec2* p2) {
    self->RayCast(callback, *p1, *p2);
}
b2Body* World_get_body_list(b2World* self) {
    return self->GetBodyList();
}
const b2Body* World_get_body_list_const(const b2World* self) {
    return self->GetBodyList();
}
b2Joint* World_get_joint_list(b2World* self) {
    return self->GetJointList();
}
const b2Joint* World_get_joint_list_const(const b2World* self) {
    return self->GetJointList();
}
b2Contact* World_get_contact_list(b2World* self) {
    return self->GetContactList();
}
const b2Contact* World_get_contact_list_const(const b2World* self) {
    return self->GetContactList();
}
void World_set_allow_sleeping(b2World* self, bool flag) {
    self->SetAllowSleeping(flag);
}
bool World_get_allow_sleeping(const b2World* self) {
    return self->GetAllowSleeping();
}
void World_set_warm_starting(b2World* self, bool flag) {
    self->SetWarmStarting(flag);
}
bool World_get_warm_starting(const b2World* self) {
    return self->GetWarmStarting();
}
void World_set_continuous_physics(b2World* self, bool flag) {
    self->SetContinuousPhysics(flag);
}
bool World_get_continuous_physics(const b2World* self) {
    return self->GetContinuousPhysics();
}
void World_set_sub_stepping(b2World* self, bool flag) {
    self->SetSubStepping(flag);
}
bool World_get_sub_stepping(const b2World* self) {
    return self->GetSubStepping();
}
i32 World_get_proxy_count(const b2World* self) {
    return self->GetProxyCount();
}
i32 World_get_body_count(const b2World* self) {
    return self->GetBodyCount();
}
i32 World_get_joint_count(const b2World* self) {
    return self->GetJointCount();
}
i32 World_get_contact_count(const b2World* self) {
    return self->GetContactCount();
}
i32 World_get_tree_height(const b2World* self) {
    return self->GetTreeHeight();
}
i32 World_get_tree_balance(const b2World* self) {
    return self->GetTreeBalance();
}
f32 World_get_tree_quality(const b2World* self) {
    return self->GetTreeQuality();
}
void World_set_gravity(b2World* self, const b2Vec2* gravity) {
    self->SetGravity(*gravity);
}
b2Vec2 World_get_gravity(const b2World* self) {
    return self->GetGravity();
}
bool World_is_locked(const b2World* self) {
    return self->IsLocked();
}
void World_set_auto_clear_forces(b2World* self, bool flag) {
    self->SetAutoClearForces(flag);
}
bool World_get_auto_clear_forces(const b2World* self) {
    return self->GetAutoClearForces();
}
void World_shift_origin(b2World* self, const b2Vec2* origin) {
    self->ShiftOrigin(*origin);
}
const b2ContactManager* World_get_contact_manager(const b2World* self) {
    return &self->GetContactManager();
}
const b2Profile* World_get_profile(const b2World* self) {
    return &self->GetProfile();
}

void World_dump(b2World* self) {
    self->Dump();
}

}
