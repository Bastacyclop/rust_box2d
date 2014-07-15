typedef void (*SayGoodbyeToJointCB)(RustObject, b2Joint*);
typedef void (*SayGoodbyeToFixtureCB)(RustObject, b2Fixture*);

struct CDestructionListener: public b2DestructionListener {
    CDestructionListener() {}
    ~CDestructionListener() override {}
    
    void SayGoodbye(b2Joint* joint) override {
        say_goodbye_to_joint(object, joint);
    }
    void SayGoodbye(b2Fixture* fixture) override {
        say_goodbye_to_fixture(object, fixture);
    }
    
    RustObject object;
    SayGoodbyeToJointCB say_goodbye_to_joint;
    SayGoodbyeToFixtureCB say_goodbye_to_fixture;
};

CDestructionListener* CDestructionListener_new(RustObject o,
                                               SayGoodbyeToJointCB sgtj,
                                               SayGoodbyeToFixtureCB sgtf) {
    CDestructionListener* dl = new CDestructionListener();
    dl->object = o;
    dl->say_goodbye_to_joint = sgtj;
    dl->say_goodbye_to_fixture = sgtf;
    return dl;
}

b2DestructionListener* CDestructionListener_as_base(CDestructionListener* self) {
    return static_cast<b2DestructionListener*>(self);
}

void CDestructionListener_drop(CDestructionListener* self) {
    delete self;
}

typedef bool (*ShouldCollideCB)(RustObject, b2Fixture*, b2Fixture*);

struct CContactFilter: public b2ContactFilter {
    CContactFilter() {}
    ~CContactFilter() override {}
    
    bool ShouldCollide(b2Fixture* fixture_a, b2Fixture* fixture_b) override {
        return should_collide(object, fixture_a, fixture_b);
    }
    
    RustObject object;
    ShouldCollideCB should_collide;
};

CContactFilter* CContactFilter_new(RustObject o, ShouldCollideCB sc) {
    CContactFilter* cf = new CContactFilter();
    cf->object = o;
    cf->should_collide = sc;
    return cf;
}

b2ContactFilter* CContactFilter_as_base(CContactFilter* self) {
    return static_cast<b2ContactFilter*>(self);
}

void CContactFilter_drop(CContactFilter* self) {
    delete self;
}

typedef void (*BeginContactCB)(RustObject, b2Contact*);
typedef void (*EndContactCB)(RustObject, b2Contact*);
typedef void (*PreSolveCB)(RustObject, b2Contact*, const b2Manifold*);
typedef void (*PostSolveCB)(RustObject, b2Contact*, const b2ContactImpulse*);

struct CContactListener: public b2ContactListener {
    CContactListener() {}
    ~CContactListener() override {}
    
    void BeginContact(b2Contact* contact) override {
        begin_contact(object, contact);
    }
    void EndContact(b2Contact* contact) override {
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

CContactListener* CContactListener_new(RustObject o,
                                       BeginContactCB bc,
                                       EndContactCB ec,
                                       PreSolveCB pres,
                                       PostSolveCB posts) {
    CContactListener* cl = new CContactListener();
    cl->object = o;
    cl->begin_contact = bc;
    cl->end_contact = ec;
    cl->pre_solve = pres;
    cl->post_solve = posts;
    return cl;
}

b2ContactListener* CContactListener_as_base(CContactListener* self) {
    return static_cast<b2ContactListener*>(self);
}

void CContactListener_drop(CContactListener* self) {
    delete self;
}

typedef bool (*QCReportFixtureCB)(RustObject, b2Fixture*);

struct CQueryCallback: public b2QueryCallback {
    CQueryCallback() {}
    ~CQueryCallback() override {}
    
    bool ReportFixture(b2Fixture* fixture) override {
        return report_fixture(object, fixture);
    }
    
    RustObject object;
    QCReportFixtureCB report_fixture;
};

CQueryCallback* CQueryCallback_new(RustObject o, QCReportFixtureCB rf) {
    CQueryCallback* qc = new CQueryCallback();
    qc->object = o;
    qc->report_fixture = rf;
    return qc;
}

b2QueryCallback* CQueryCallback_as_base(CQueryCallback* self) {
    return static_cast<b2QueryCallback*>(self);
}

void CQueryCallback_drop(CQueryCallback* self) {
    delete self;
}

typedef f32 (*RCCReportFixtureCB)(RustObject, b2Fixture*, const b2Vec2*,
                                  const b2Vec2*, f32);

struct CRayCastCallback: public b2RayCastCallback {
    CRayCastCallback() {}
    ~CRayCastCallback() override {}
    
    f32 ReportFixture(b2Fixture* fixture,
                      const b2Vec2& point,
                      const b2Vec2& normal,
                      f32 fraction) override {
        return report_fixture(object, fixture, &point, &normal, fraction);
    }
    
    RustObject object;
    RCCReportFixtureCB report_fixture;
};

CRayCastCallback* CRayCastCallback_new(RustObject o, RCCReportFixtureCB rf) {
    CRayCastCallback* rcc = new CRayCastCallback();
    rcc->object = o;
    rcc->report_fixture = rf;
    return rcc;
}

b2RayCastCallback* CRayCastCallback_as_base(CRayCastCallback* self) {
    return static_cast<b2RayCastCallback*>(self);
}

void CRayCastCallback_drop(CRayCastCallback* self) {
    delete self;
}
