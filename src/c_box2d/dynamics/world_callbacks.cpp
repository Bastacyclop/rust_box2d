typedef void (*SayGoodbyeToJoint)(b2Joint*, i32);
typedef void (*SayGoodbyeToFixture)(b2Fixture*, i32);

struct DestructionCallbacks: public b2DestructionListener {
    DestructionCallbacks() {}
    virtual ~DestructionCallbacks() {}
    
    virtual void SayGoodbye(b2Joint* joint) {
        say_goodbye_to_joint(joint, joint_obj);
    }
    virtual void SayGoodbye(b2Fixture* fixture) {
        say_goodbye_to_fixture(fixture, fixture_obj);
    }
    
    SayGoodbyeToJoint say_goodbye_to_joint;
    SayGoodbyeToFixture say_goodbye_to_fixture;
    i32 joint_obj;
    i32 fixture_obj;
};

DestructionCallbacks* DestructionCallbacks_new(SayGoodbyeToJoint sgtj,
                                               SayGoodbyeToFixture sgtf,
                                               i32 jo, i32 fo) {
    DestructionCallbacks* dl = new DestructionCallbacks();
    dl->say_goodbye_to_joint = sgtj;
    dl->say_goodbye_to_fixture = sgtf;
    dl->joint_obj = jo;
    dl->fixture_obj = fo;
    return dl;
}

b2DestructionListener* DestructionCallbacks_as_listener(
                                    DestructionCallbacks* self) {
    return static_cast<b2DestructionListener*>(self);
}

void DestructionCallbacks_drop(DestructionCallbacks* self) {
    delete self;
}
