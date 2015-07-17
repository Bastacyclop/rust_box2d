typedef void (*SayGoodbyeToJointCB)(RustFatObject, b2Joint*);
typedef void (*SayGoodbyeToFixtureCB)(RustFatObject, b2Fixture*);

struct DestructionListenerLink: public b2DestructionListener {
    DestructionListenerLink() {}
    ~DestructionListenerLink() {}
    
    void SayGoodbye(b2Joint* joint) {
        say_goodbye_to_joint(object, joint);
    }
    void SayGoodbye(b2Fixture* fixture) {
        say_goodbye_to_fixture(object, fixture);
    }
    
    RustFatObject object;
    SayGoodbyeToJointCB say_goodbye_to_joint;
    SayGoodbyeToFixtureCB say_goodbye_to_fixture;
};

DestructionListenerLink* DestructionListenerLink_new(RustFatObject o,
                                                     SayGoodbyeToJointCB sgtj,
                                                     SayGoodbyeToFixtureCB sgtf) {
    DestructionListenerLink* dl = new DestructionListenerLink();
    dl->object = o;
    dl->say_goodbye_to_joint = sgtj;
    dl->say_goodbye_to_fixture = sgtf;
    return dl;
}

void DestructionListenerLink_set_object(DestructionListenerLink* self, RustFatObject o) {
    self->object = o;
}

b2DestructionListener* DestructionListenerLink_as_base(DestructionListenerLink* self) {
    return static_cast<b2DestructionListener*>(self);
}

void DestructionListenerLink_drop(DestructionListenerLink* self) {
    delete self;
}

typedef bool (*ShouldCollideCB)(RustFatObject, b2Fixture*, b2Fixture*);

struct ContactFilterLink: public b2ContactFilter {
    ContactFilterLink() {}
    ~ContactFilterLink() {}
    
    bool ShouldCollide(b2Fixture* fixture_a, b2Fixture* fixture_b) {
        return should_collide(object, fixture_a, fixture_b);
    }
    
    RustFatObject object;
    ShouldCollideCB should_collide;
};

ContactFilterLink* ContactFilterLink_new(RustFatObject o, ShouldCollideCB sc) {
    ContactFilterLink* cf = new ContactFilterLink();
    cf->object = o;
    cf->should_collide = sc;
    return cf;
}

void ContactFilterLink_set_object(ContactFilterLink* self, RustFatObject o) {
    self->object = o;
}

b2ContactFilter* ContactFilterLink_as_base(ContactFilterLink* self) {
    return static_cast<b2ContactFilter*>(self);
}

void ContactFilterLink_drop(ContactFilterLink* self) {
    delete self;
}

typedef void (*BeginContactCB)(RustFatObject, b2Contact*);
typedef void (*EndContactCB)(RustFatObject, b2Contact*);
typedef void (*PreSolveCB)(RustFatObject, b2Contact*, const b2Manifold*);
typedef void (*PostSolveCB)(RustFatObject, b2Contact*, const b2ContactImpulse*);

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
    
    RustFatObject object;
    BeginContactCB begin_contact;
    EndContactCB end_contact;
    PreSolveCB pre_solve;
    PostSolveCB post_solve;
};

ContactListenerLink* ContactListenerLink_new(RustFatObject o,
                                             BeginContactCB bc,
                                             EndContactCB ec,
                                             PreSolveCB pres,
                                             PostSolveCB posts) {
    ContactListenerLink* cl = new ContactListenerLink();
    cl->object = o;
    cl->begin_contact = bc;
    cl->end_contact = ec;
    cl->pre_solve = pres;
    cl->post_solve = posts;
    return cl;
}

void ContactListenerLink_set_object(ContactListenerLink* self, RustFatObject o) {
    self->object = o;
}

b2ContactListener* ContactListenerLink_as_base(ContactListenerLink* self) {
    return static_cast<b2ContactListener*>(self);
}

void ContactListenerLink_drop(ContactListenerLink* self) {
    delete self;
}

typedef bool (*QCReportFixtureCB)(RustFatObject, b2Fixture*);

struct QueryCallbackLink: public b2QueryCallback {
    QueryCallbackLink() {}
    ~QueryCallbackLink() {}
    
    bool ReportFixture(b2Fixture* fixture) {
        return report_fixture(object, fixture);
    }
    
    RustFatObject object;
    QCReportFixtureCB report_fixture;
};

QueryCallbackLink* QueryCallbackLink_new(RustFatObject o, QCReportFixtureCB rf) {
    QueryCallbackLink* qc = new QueryCallbackLink();
    qc->object = o;
    qc->report_fixture = rf;
    return qc;
}

void QueryCallbackLink_set_object(QueryCallbackLink* self, RustFatObject o) {
    self->object = o;
}

b2QueryCallback* QueryCallbackLink_as_base(QueryCallbackLink* self) {
    return static_cast<b2QueryCallback*>(self);
}

void QueryCallbackLink_drop(QueryCallbackLink* self) {
    delete self;
}

typedef f32 (*RCCReportFixtureCB)(RustFatObject, b2Fixture*, const b2Vec2*,
                                  const b2Vec2*, f32);

struct RayCastCallbackLink: public b2RayCastCallback {
    RayCastCallbackLink() {}
    ~RayCastCallbackLink() {}
    
    f32 ReportFixture(b2Fixture* fixture,
                      const b2Vec2& point,
                      const b2Vec2& normal,
                      f32 fraction) {
        return report_fixture(object, fixture, &point, &normal, fraction);
    }
    
    RustFatObject object;
    RCCReportFixtureCB report_fixture;
};

RayCastCallbackLink* RayCastCallbackLink_new(RustFatObject o, RCCReportFixtureCB rf) {
    RayCastCallbackLink* rcc = new RayCastCallbackLink();
    rcc->object = o;
    rcc->report_fixture = rf;
    return rcc;
}

void RayCastCallbackLink_set_object(RayCastCallbackLink* self, RustFatObject o) {
    self->object = o;
}

b2RayCastCallback* RayCastCallbackLink_as_base(RayCastCallbackLink* self) {
    return static_cast<b2RayCastCallback*>(self);
}

void RayCastCallbackLink_drop(RayCastCallbackLink* self) {
    delete self;
}
