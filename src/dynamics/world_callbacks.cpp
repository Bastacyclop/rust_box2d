typedef void (*SayGoodbyeToJointCB)(b2Joint*, RustObject);
typedef void (*SayGoodbyeToFixtureCB)(b2Fixture*, RustObject);

struct CDestructionListener: public b2DestructionListener {
    CDestructionListener() {}
    ~CDestructionListener() override {}
    
    void SayGoodbye(b2Joint* joint) override {
        say_goodbye_to_joint(joint, say_goodbye_to_joint_object);
    }
    void SayGoodbye(b2Fixture* fixture) override {
        say_goodbye_to_fixture(fixture, say_goodbye_to_fixture_object);
    }
    
    SayGoodbyeToJointCB say_goodbye_to_joint;
    SayGoodbyeToFixtureCB say_goodbye_to_fixture;
    RustObject say_goodbye_to_joint_object;
    RustObject say_goodbye_to_fixture_object;
};

CDestructionListener* CDestructionListener_new(SayGoodbyeToJointCB sgtj,
                                               SayGoodbyeToFixtureCB sgtf,
                                               RustObject sgtj_o,
                                               RustObject sgtf_o) {
    CDestructionListener* dl = new CDestructionListener();
    dl->say_goodbye_to_joint = sgtj;
    dl->say_goodbye_to_fixture = sgtf;
    dl->say_goodbye_to_joint_object = sgtj_o;
    dl->say_goodbye_to_fixture_object = sgtf_o;
    return dl;
}

b2DestructionListener* CDestructionListener_as_base(CDestructionListener* self) {
    return static_cast<b2DestructionListener*>(self);
}

void CDestructionListener_drop(CDestructionListener* self) {
    delete self;
}

typedef bool (*ShouldCollideCB)(b2Fixture*, b2Fixture*, RustObject);

struct CContactFilter: public b2ContactFilter {
    CContactFilter() {}
    ~CContactFilter() override {}
    
    bool ShouldCollide(b2Fixture* fixture_a, b2Fixture* fixture_b) override {
        return should_collide(fixture_a, fixture_b, should_collide_object);
    }
    
    ShouldCollideCB should_collide;
    RustObject should_collide_object;
};

CContactFilter* CContactFilter_new(ShouldCollideCB sc, RustObject sc_o) {
    CContactFilter* cf = new CContactFilter();
    cf->should_collide = sc;
    cf->should_collide_object = sc_o;
    return cf;
}

b2ContactFilter* CContactFilter_as_base(CContactFilter* self) {
    return static_cast<b2ContactFilter*>(self);
}

void CContactFilter_drop(CContactFilter* self) {
    delete self;
}

typedef void (*BeginContactCB)(b2Contact*, RustObject);
typedef void (*EndContactCB)(b2Contact*, RustObject);
typedef void (*PreSolveCB)(b2Contact*, const b2Manifold*, RustObject);
typedef void (*PostSolveCB)(b2Contact*, const b2ContactImpulse*, RustObject);

struct CContactListener: public b2ContactListener {
    CContactListener() {}
    ~CContactListener() override {}
    
    void BeginContact(b2Contact* contact) override {
        begin_contact(contact, begin_contact_object);
    }
    void EndContact(b2Contact* contact) override {
        end_contact(contact, end_contact_object);
    }
    void PreSolve(b2Contact* contact, const b2Manifold* old_manifold) {
        pre_solve(contact, old_manifold, pre_solve_object);
    }
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {
        post_solve(contact, impulse, post_solve_object);
    }
    
    BeginContactCB begin_contact;
    EndContactCB end_contact;
    PreSolveCB pre_solve;
    PostSolveCB post_solve;
    RustObject begin_contact_object;
    RustObject end_contact_object;
    RustObject pre_solve_object;
    RustObject post_solve_object;
};

CContactListener* CContactListener_new(BeginContactCB bc,
                                       EndContactCB ec,
                                       PreSolveCB pres,
                                       PostSolveCB posts,
                                       RustObject bc_o,
                                       RustObject ec_o,
                                       RustObject pres_o,
                                       RustObject posts_o) {
    CContactListener* cl = new CContactListener();
    cl->begin_contact = bc;
    cl->end_contact = ec;
    cl->pre_solve = pres;
    cl->post_solve = posts;
    cl->begin_contact_object = bc_o;
    cl->end_contact_object = ec_o;
    cl->pre_solve_object = pres_o;
    cl->post_solve_object = posts_o;
    return cl;
}

b2ContactListener* CContactListener_as_base(CContactListener* self) {
    return static_cast<b2ContactListener*>(self);
}

void CContactListener_drop(CContactListener* self) {
    delete self;
}

typedef bool (*ReportFixtureCB)(b2Fixture*, RustObject);

struct CQueryCallback: public b2QueryCallback {
    CQueryCallback() {}
    ~CQueryCallback() override {}
    
    bool ReportFixture(b2Fixture* fixture) override {
        return report_fixture(fixture, report_fixture_object);
    }
    
    ReportFixtureCB report_fixture;
    RustObject report_fixture_object;
};

CQueryCallback* CQueryCallback_new(ReportFixtureCB rf,
                                   RustObject rf_o) {
    CQueryCallback* qc = new CQueryCallback();
    qc->report_fixture = rf;
    qc->report_fixture_object = rf_o;
    return qc;
}

b2QueryCallback* CQueryCallback_as_base(CQueryCallback* self) {
    return static_cast<b2QueryCallback*>(self);
}

void CQueryCallback_drop(CQueryCallback* self) {
    delete self;
}

typedef f32 (*HitFixtureCB)(b2Fixture*, const b2Vec2*, const b2Vec2*, f32, RustObject);

struct CRayCastCallback: public b2RayCastCallback {
    CRayCastCallback() {}
    ~CRayCastCallback() override {}
    
    f32 ReportFixture(b2Fixture* fixture,
                      const b2Vec2& point,
                      const b2Vec2& normal,
                      f32 fraction) override {
        return hit_fixture(fixture, &point, &normal, fraction, hit_fixture_object);
    }
    
    HitFixtureCB hit_fixture;
    RustObject hit_fixture_object;
};

CRayCastCallback* CRayCastCallback_new(HitFixtureCB hf,
                                       RustObject hf_o) {
    CRayCastCallback* rcc = new CRayCastCallback();
    rcc->hit_fixture = hf;
    rcc->hit_fixture_object = hf_o;
    return rcc;
}

b2RayCastCallback* CRayCastCallback_as_base(CRayCastCallback* self) {
    return static_cast<b2RayCastCallback*>(self);
}

void CRayCastCallback_drop(CRayCastCallback* self) {
    delete self;
}
