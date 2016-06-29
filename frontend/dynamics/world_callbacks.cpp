typedef void (*SayGoodbyeToJointCB)(RustObject, b2Joint*);
typedef void (*SayGoodbyeToFixtureCB)(RustObject, b2Fixture*);

struct DestructionListenerLink: public b2DestructionListener {
    DestructionListenerLink() {}
    ~DestructionListenerLink() {}

    void SayGoodbye(b2Joint* joint) {
        say_goodbye_to_joint(object, joint);
    }
    void SayGoodbye(b2Fixture* fixture) {
        say_goodbye_to_fixture(object, fixture);
    }

    RustObject object;
    SayGoodbyeToJointCB say_goodbye_to_joint;
    SayGoodbyeToFixtureCB say_goodbye_to_fixture;
};

DestructionListenerLink* DestructionListenerLink_alloc() {
    return new DestructionListenerLink();
}

void DestructionListenerLink_bind(DestructionListenerLink* self,
                                  RustObject o,                                                                                       SayGoodbyeToJointCB sgtj,
                                  SayGoodbyeToFixtureCB sgtf) {
    self->object = o;
    self->say_goodbye_to_joint = sgtj;
    self->say_goodbye_to_fixture = sgtf;
}

b2DestructionListener* DestructionListenerLink_as_base(DestructionListenerLink* self) {
    return static_cast<b2DestructionListener*>(self);
}

void DestructionListenerLink_drop(DestructionListenerLink* self) {
    delete self;
}

typedef bool (*ShouldCollideCB)(RustObject, b2Fixture*, b2Fixture*);

struct ContactFilterLink: public b2ContactFilter {
    ContactFilterLink() {}
    ~ContactFilterLink() {}

    bool ShouldCollide(b2Fixture* fixture_a, b2Fixture* fixture_b) {
        return should_collide(object, fixture_a, fixture_b);
    }

    RustObject object;
    ShouldCollideCB should_collide;
};

ContactFilterLink* ContactFilterLink_alloc() {
    return new ContactFilterLink();
}

void ContactFilterLink_bind(ContactFilterLink* self,
                            RustObject o,
                            ShouldCollideCB sc) {
    self->object = o;
    self->should_collide = sc;
}

b2ContactFilter* ContactFilterLink_as_base(ContactFilterLink* self) {
    return static_cast<b2ContactFilter*>(self);
}

void ContactFilterLink_drop(ContactFilterLink* self) {
    delete self;
}

typedef void (*BeginContactCB)(RustObject, b2Contact*);
typedef void (*EndContactCB)(RustObject, b2Contact*);
typedef void (*PreSolveCB)(RustObject, b2Contact*, const b2Manifold*);
typedef void (*PostSolveCB)(RustObject, b2Contact*, const b2ContactImpulse*);

struct ContactListenerLink: public b2ContactListener {
    ContactListenerLink() {}
    ~ContactListenerLink() {}

    void BeginContact(b2Contact* contact) {
        begin_contact(object, contact);
    }
    void EndContact(b2Contact* contact) {
        end_contact(object, contact);
    }
    void PreSolve(b2Contact* contact, const b2Manifold* old_manifold) {
        pre_solve(object, contact, old_manifold);
    }
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {
        post_solve(object, contact, impulse);
    }

    RustObject object;
    BeginContactCB begin_contact;
    EndContactCB end_contact;
    PreSolveCB pre_solve;
    PostSolveCB post_solve;
};

ContactListenerLink* ContactListenerLink_alloc() {
    return new ContactListenerLink();
}

void ContactListenerLink_bind(ContactListenerLink* self,
                              RustObject o,
                              BeginContactCB bc,
                              EndContactCB ec,
                              PreSolveCB pres,
                              PostSolveCB posts) {
    self->object = o;
    self->begin_contact = bc;
    self->end_contact = ec;
    self->pre_solve = pres;
    self->post_solve = posts;
}

b2ContactListener* ContactListenerLink_as_base(ContactListenerLink* self) {
    return static_cast<b2ContactListener*>(self);
}

void ContactListenerLink_drop(ContactListenerLink* self) {
    delete self;
}

typedef bool (*QCReportFixtureCB)(RustObject, b2Fixture*);

struct QueryCallbackLink: public b2QueryCallback {
    QueryCallbackLink() {}
    ~QueryCallbackLink() {}

    bool ReportFixture(b2Fixture* fixture) {
        return report_fixture(object, fixture);
    }

    RustObject object;
    QCReportFixtureCB report_fixture;
};

QueryCallbackLink* QueryCallbackLink_alloc() {
    return new QueryCallbackLink();
}

void QueryCallbackLink_bind(QueryCallbackLink* self,
                            RustObject object,
                            QCReportFixtureCB rf) {
    self->object = object;
    self->report_fixture = rf;
}

b2QueryCallback* QueryCallbackLink_as_base(QueryCallbackLink* self) {
    return static_cast<b2QueryCallback*>(self);
}

void QueryCallbackLink_drop(QueryCallbackLink* self) {
    delete self;
}

typedef f32 (*RCCReportFixtureCB)(RustObject, b2Fixture*,
                                  const b2Vec2*, const b2Vec2*, f32);

struct RayCastCallbackLink: public b2RayCastCallback {
    RayCastCallbackLink() {}
    ~RayCastCallbackLink() {}

    f32 ReportFixture(b2Fixture* fixture,
                      const b2Vec2& point,
                      const b2Vec2& normal,
                      f32 fraction) {
        return report_fixture(object, fixture, &point, &normal, fraction);
    }

    RustObject object;
    RCCReportFixtureCB report_fixture;
};

RayCastCallbackLink* RayCastCallbackLink_alloc() {
    return new RayCastCallbackLink();
}

void RayCastCallbackLink_bind(RayCastCallbackLink* self,
                              RustObject object,
                              RCCReportFixtureCB rf) {
    self->object = object;
    self->report_fixture = rf;
}

b2RayCastCallback* RayCastCallbackLink_as_base(RayCastCallbackLink* self) {
    return static_cast<b2RayCastCallback*>(self);
}

void RayCastCallbackLink_drop(RayCastCallbackLink* self) {
    delete self;
}
